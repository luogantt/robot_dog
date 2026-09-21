#!/usr/bin/env python3
"""
ASDU UDP control script for CD1 robot.
Sequence: 起立 -> 楼梯步态 -> 定速前进 -> 停止 -> 趴下

Protocol: plain UDP port 30004 (encryption disabled)

【本脚本是客户端，不是服务端】
文档 §1.1.2："机器人本体为 TCP/UDP 服务端，外部板卡或系统为 TCP/UDP 客户端。"
所以：
    机器狗上  —— 什么都不用启动。厂商的 robotserve 开机即在跑，监听 30004。
    本脚本    —— 跑在你的电脑上（或机器狗主机上），主动连过去发指令。

【运行前必读】
1. 端口 30004 默认启用 DTLS 加密（文档 §1.1.2）。
   必须先在机器人上联系技术支持修改 robotserve 配置文件关闭加密，
   否则本脚本连不上（收不到任何回包，卡在等待状态上报）。
2. 本脚本的运动选型由 PROFILE 决定，两种路径不可混用（文档 §1.2.4）：
     PROFILE = "manual"  常规运动模式  Gait=0x1003 + 轴指令   0x00100002（比例量 [-1,1]）
     PROFILE = "nav"     导航运动模式  Gait=0x3003 + 真实轴指令 0x00110002（物理量 m/s）
   文档 §1.2.4 明确：二次开发做自主导航/自主爬梯时**必须**用导航模式专用步态配真实轴指令，
   且爬楼梯**必须在进入楼梯前**切到 0x3003，严禁进入楼梯后再切。
3. 文档未规定 §1.2.5 轴指令的下发频率，VELOCITY_HZ 是保守取值，上机前建议向技术支持确认。

参考：《软件开发指南》V1.0.1（20260831）
"""

import json
import socket
import struct
import threading
import time
import traceback
from datetime import datetime

# ============================ 配置 ============================

HOST = "10.21.33.103"
PORT = 30004

# 运动选型："manual"（常规运动模式，手动/试机）或 "nav"（导航运动模式，自主算法）
PROFILE = "manual"

PROFILES = {
    # 常规运动模式：楼梯步态 + 轴指令，X/Y/Z/Roll/Pitch/Yaw 为最大速度的比例 [-1,1]
    # 文档 §1.2.5：该指令仅支持在常规模式和辅助模式下执行，且不返回任何响应帧
    "manual": {
        "name": "楼梯（常规运动模式）",
        "gait": 0x1003,
        "cmd": 0x00100002,
        "unit": "%(比例)",
        "x": 0.12,  # 最大前进速度的 12%
        "modes": (0, 2),  # 需要机器人处于常规模式或辅助模式
    },
    # 导航运动模式：楼梯步态 + 真实轴指令，X/Y 为 m/s，Yaw 为 rad/s
    # 文档 §1.2.6：真实轴指令仅在导航模式下生效，数据原样下发不做速度限制
    "nav": {
        "name": "楼梯（导航运动模式）",
        "gait": 0x3003,
        "cmd": 0x00110002,
        "unit": "m/s",
        "x": 0.2,  # m/s
        "modes": (1,),  # 需要机器人处于导航模式
    },
}

VELOCITY_DURATION = 2.0  # 前进持续时间（秒）
VELOCITY_HZ = 20  # 轴指令下发频率。文档未规定该值，10~20Hz 为保守取值
VELOCITY_Y = 0.0  # 左右速度
VELOCITY_YAW = 0.0  # 偏航角速度

STOP_LINEAR_TH = 0.02  # 判定"已停止"的线速度阈值 m/s
STOP_ANGULAR_TH = 0.03  # 判定"已停止"的角速度阈值 rad/s
STOP_STABLE_TIME = 0.3  # 需连续低于阈值的时间（秒）
STOP_TIMEOUT = 5.0  # 等待停止超时

REQUIRE_MODE_CHECK = True  # True: 使用模式与所选 PROFILE 不符时拒绝下发轴指令
ABORT_SEVERITY = 5  # 故障严重等级 >= 此值时中断运动（3=WARN 4=ERROR 5=FATAL）
MAX_CONSEC_SEND_FAILURES = 10  # 连续发送失败次数达到此值时中断运动

HEARTBEAT_PERIOD = 0.9  # 心跳周期。文档 §1.2.1 要求不小于 1Hz，取 0.9s 留余量

# ========================== 协议常量 ==========================

HEADER_SYNC = b'\xeb\x91\xeb\x90'
HEADER_VERSION = 0x01
HEADER_FORMAT_JSON = 0x01
HEADER_LEN = 16

TYPE_HEARTBEAT = 0x00100064
TYPE_MOTION = 0x00100001
TYPE_STATUS_PUSH = 0x00F00000  # 所有主动上报状态帧的 Command 都是该值

CMD_HEARTBEAT = 0x00000005
CMD_MOTION_STATE = 0x00200002
CMD_GAIT_SWITCH = 0x00300002
CMD_AXIS = 0x00100002  # 文档 §1.2.5 轴指令（比例量）
CMD_AXIS_REAL = 0x00110002  # 文档 §1.2.6 真实轴指令（物理量，仅导航模式）

MOTION_STAND = 1
MOTION_CROUCH = 4
MOTION_RL_CONTROL = 17

# ========================== 输出工具 ==========================

_print_lock = threading.Lock()


def log(msg):
    """多线程安全的行输出。"""
    with _print_lock:
        print(msg, flush=True)


def now_str():
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


# ========================== 报文构造 ==========================

_lock = threading.Lock()
_packet_id = 0
_msg_id = 0


def build_apdu(body_dict, msg_id=None):
    """构造 APDU：16 字节协议头 + JSON ASDU（文档 §1.1.5）。

    msg_id 为 None 时自动分配并递增；返回 (apdu_bytes, 使用的 msgId)。
    """
    global _packet_id, _msg_id
    body_bytes = json.dumps(body_dict, separators=(',', ':')).encode('utf-8')
    with _lock:
        if msg_id is None:
            msg_id = _msg_id
            _msg_id = (_msg_id + 1) % 0x10000
        cur_pkt = _packet_id
        _packet_id = (_packet_id + 1) % 256

    header = struct.pack(
        '<4sHHBBB5s',
        HEADER_SYNC,
        len(body_bytes),
        msg_id,
        HEADER_FORMAT_JSON,
        cur_pkt,
        HEADER_VERSION,
        b'\x00' * 5,
    )
    return header + body_bytes, msg_id


def make_heartbeat():
    return {"PatrolDevice": {
        "Type": TYPE_HEARTBEAT, "Command": CMD_HEARTBEAT,
        "Time": now_str(), "Items": {}
    }}


def make_motion_state(motion_param):
    return {"PatrolDevice": {
        "Type": TYPE_MOTION, "Command": CMD_MOTION_STATE,
        "Time": now_str(), "Items": {"MotionParam": motion_param}
    }}


def make_gait_switch(gait_param):
    return {"PatrolDevice": {
        "Type": TYPE_MOTION, "Command": CMD_GAIT_SWITCH,
        "Time": now_str(), "Items": {"GaitParam": gait_param}
    }}


def make_velocity(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0,
                  command=CMD_AXIS):
    """轴指令。command=CMD_AXIS 为比例量 [-1,1]，command=CMD_AXIS_REAL 为物理量。"""
    return {"PatrolDevice": {
        "Type": TYPE_MOTION, "Command": command,
        "Time": now_str(),
        "Items": {"X": x, "Y": y, "Z": z, "Roll": roll, "Pitch": pitch, "Yaw": yaw}
    }}


# ========================== 报文解析 ==========================

def parse_apdu(data):
    """解析 APDU，返回 (info_dict, body_dict_or_None)。"""
    if len(data) < HEADER_LEN or data[0:4] != HEADER_SYNC:
        return None, None
    body_len, msg_id, fmt, pkt_id, ver = struct.unpack('<HHBBB', data[4:11])
    body_bytes = data[HEADER_LEN:HEADER_LEN + body_len]
    try:
        body = json.loads(body_bytes.decode('utf-8'))
    except Exception:
        body = None
    info = {'bodyLen': body_len, 'msgId': msg_id, 'format': fmt,
            'packetId': pkt_id, 'version': ver}
    return info, body


def _items(body):
    if not isinstance(body, dict):
        return {}
    items = body.get("PatrolDevice", {}).get("Items", {})
    return items if isinstance(items, dict) else {}


def extract_error(body):
    """提取通用协议接口调用状态响应（文档 §1.5）。

    返回 (ErrorCode, ErrorMessage)；不是该类响应时返回 (None, None)。
    """
    items = _items(body)
    if "ErrorCode" in items:
        return items.get("ErrorCode"), items.get("ErrorMessage")
    return None, None


def extract_faults(body):
    """提取异常状态上报的 ErrorList（文档 §1.3.1.4）。"""
    el = _items(body).get("ErrorList")
    if el is None:
        return []
    if isinstance(el, dict):
        el = [el]
    return [e for e in el if isinstance(e, dict)]


# ========================== 名称表 ==========================

MOTION_NAMES = {
    -2: "软急停", 0: "默认(运动未上报)", 1: "站立", 2: "关节阻尼",
    4: "趴下", 5: "标零", 17: "RL控制", 0x1001: "阻尼趴下",
}
GAIT_NAMES = {
    0: "无", 0x1001: "基础(常规)", 0x1002: "高台(常规)", 0x1003: "楼梯(常规)",
    0x3002: "平地(导航)", 0x3003: "楼梯(导航)",
    # 0x1002 不在厂商文档里，见 web_control.py 的 GAIT_CHOICES 注释。
    # 列在这里是为了状态栏能显示名字 —— 否则切到高台后界面会显示 "0x1002 ?"。
}
MODE_NAMES = {0: "常规模式", 1: "导航模式", 2: "辅助模式"}

# 常见故障码（节选自文档 §1.3.1.4 的错误码表；未列出的直接打印机器人上报的 Name）
FAULT_NAMES = {
    0x8006: "驱动器通信超时", 0x800A: "关节持续高速运转",
    0x8014: "关节位置超限", 0x8016: "关节数据无效", 0x8017: "关节数据更新错误",
    0x8101: "电池电量低", 0x8102: "电池电压过低",
    0x8108: "电池温度过高", 0x810A: "放电电流过大", 0x9401: "文件系统使用率过高",
}

SEVERITY_NAMES = {3: "WARN", 4: "ERROR", 5: "FATAL"}


def _fmt(v, table):
    return "?" if not isinstance(v, int) else table.get(v, "?")


def fmt_hex(v):
    return "?" if not isinstance(v, int) else f"0x{v:04X}"


# ========================== 客户端 ==========================

class RobotClient:
    """ASDU UDP 客户端：后台心跳 + 状态接收 + 命令收发。"""

    def __init__(self, host, port, profile):
        self.host = host
        self.port = port
        self.profile = profile
        self.target = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 显式 bind：不是为了修 bug（未 bind 的 socket 在首次 sendto 后也会被
        # 内核绑到 ephemeral 端口，且该端口在 socket 生命周期内不变），而是为了
        # 【可观测】—— 启动时就能打印出本机的 source endpoint，
        # 配合抓包可以确认"到机器人眼里的客户端身份"到底是什么。
        # 排查"Web 发送路径 vs 机器人响应"时这条信息是必需的，见 README 第 9 节。
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.5)
        log(f"[UDP] 本机 socket = {self.sock.getsockname()}  目标 = {self.target}")

        self._running = False
        self._closed = False
        self._hb_thread = None
        self._recv_thread = None

        # 状态缓存。BasicStatus(2Hz) 为权威来源，MotionStatus(10Hz) 补充速度类字段。
        self._status = {}
        self._status_lock = threading.Lock()
        self._basic_ts = 0.0
        self._prev_log = None

        # 请求-响应：msgId -> _Pending
        self._pending = {}
        self._pending_lock = threading.Lock()

        # 活动故障：Code -> info
        self._faults = {}

        self.motion_issued = False  # 是否下发过运动指令，决定退出时是否需要急停

    # ---------- 生命周期 ----------

    def start(self):
        self._running = True
        self._hb_thread = threading.Thread(target=self._heartbeat_loop,
                                           name="heartbeat", daemon=True)
        self._recv_thread = threading.Thread(target=self._recv_loop,
                                             name="recv", daemon=True)
        self._hb_thread.start()
        self._recv_thread.start()
        log(f"[INFO] 心跳线程已启动（周期 {HEARTBEAT_PERIOD}s，约 {1/HEARTBEAT_PERIOD:.1f}Hz），"
            f"监听 UDP {self.host}:{self.port}")

    def stop(self):
        self._running = False
        if self._hb_thread:
            self._hb_thread.join(timeout=2)
        if self._recv_thread:
            self._recv_thread.join(timeout=2)
        if not self._closed:
            self._closed = True
            try:
                self.sock.close()
            except Exception as e:
                log(f"[ERROR] 关闭 socket 失败: {e}")

    # ---------- 发送 ----------

    def _sendto(self, apdu):
        """发送一帧。失败时抛出 OSError，由调用方决定如何处置。"""
        if self._closed:
            raise OSError("socket 已关闭")
        self.sock.sendto(apdu, self.target)

    def local_endpoint(self):
        """本机 source endpoint。首次 sendto 后应变成 (真实IP, 端口)。

        排查用：这个 IP:端口 就是机器人口中的"客户端身份"，
        文档 §1.5 的 0xE006 判定依赖它。
        """
        try:
            return self.sock.getsockname()
        except OSError:
            return None

    def send(self, body_dict, label=""):
        """单向发送，不等响应。"""
        apdu, msg_id = build_apdu(body_dict)
        pd = body_dict.get("PatrolDevice", {})
        log(f"[SEND] {label}  Type=0x{pd['Type']:08x} Command=0x{pd['Command']:08x} "
            f"Items={pd.get('Items', {})} (msgId={msg_id})")
        self._sendto(apdu)

    def send_and_wait(self, body_dict, label="", timeout=3.0):
        """发送并等待同 msgId 的响应帧，返回响应 body；超时返回 None。

        注意：轴指令/真实轴指令按文档 §1.2.5、§1.2.6 **不返回任何响应帧**，
        对这两条指令调用本方法必然超时，应改用 send()。
        """
        apdu, msg_id = build_apdu(body_dict)
        pending = _Pending()
        with self._pending_lock:
            self._pending[msg_id] = pending
        pd = body_dict.get("PatrolDevice", {})
        log(f"[SEND] {label}  Type=0x{pd['Type']:08x} Command=0x{pd['Command']:08x} "
            f"Items={pd.get('Items', {})} (msgId={msg_id})")
        try:
            self._sendto(apdu)
            if pending.event.wait(timeout):
                return pending.body
            log(f"[WARN] {label or '命令'} 等待响应超时（msgId={msg_id}, {timeout}s）")
            return None
        finally:
            with self._pending_lock:
                self._pending.pop(msg_id, None)

    # ---------- 运动控制 ----------

    def stop_motion(self, cycles=10, hz=None):
        """按控制周期连续下发零速度。异常路径也会调用本方法，故自身必须不抛异常。"""
        hz = hz or VELOCITY_HZ
        period = 1.0 / hz
        cmd = self.profile["cmd"]
        ok = 0
        for _ in range(cycles):
            try:
                apdu, _ = build_apdu(make_velocity(0, 0, 0, 0, 0, 0, command=cmd))
                self._sendto(apdu)
                ok += 1
            except OSError as e:
                log(f"[ERROR] 速度归零发送失败: {e}")
                break
            except Exception as e:
                log(f"[ERROR] 速度归零异常: {e}")
                break
            time.sleep(period)
        return ok

    def move(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0,
             duration=1.0, hz=None, abort_on_fault=True):
        """以固定频率下发轴指令 duration 秒，返回 None 表示正常结束，否则返回中断原因。

        每帧重新构造 APDU（msgId / 包编号 / Time 逐帧更新）。
        无论正常结束还是异常/中断，都会在 finally 中下发零速度。
        """
        hz = hz or VELOCITY_HZ
        period = 1.0 / hz
        cmd = self.profile["cmd"]
        sent = failed = consec = 0
        aborted = None
        self.motion_issued = True
        try:
            deadline = time.monotonic() + duration
            next_t = time.monotonic()
            while time.monotonic() < deadline:
                if abort_on_fault and self.max_active_severity >= ABORT_SEVERITY:
                    aborted = f"检出严重故障（severity>={ABORT_SEVERITY}），已中断运动"
                    break
                try:
                    apdu, _ = build_apdu(make_velocity(x, y, z, roll, pitch, yaw,
                                                       command=cmd))
                    self._sendto(apdu)
                    sent += 1
                    consec = 0
                except OSError as e:
                    failed += 1
                    consec += 1
                    if consec >= MAX_CONSEC_SEND_FAILURES:
                        aborted = f"连续 {consec} 次发送失败: {e}"
                        break
                # 绝对时间调度，避免 time.sleep 漂移导致实际频率低于目标
                next_t += period
                if next_t <= time.monotonic():
                    while next_t <= time.monotonic():  # 落后过多则丢弃整周期重新对齐
                        next_t += period
                else:
                    time.sleep(next_t - time.monotonic())
        finally:
            self.stop_motion(hz=hz)

        log(f"[MOVE] X={x} Y={y} Yaw={yaw} ({self.profile['unit']})  "
            f"目标 {hz}Hz × {duration}s，实发 {sent} 帧，失败 {failed} 帧")
        if aborted:
            log(f"[MOVE] 已中断：{aborted}")
        return aborted

    # ---------- 接收 ----------

    def _heartbeat_loop(self):
        next_t = time.monotonic()
        while self._running:
            try:
                apdu, _ = build_apdu(make_heartbeat())
                self._sendto(apdu)
            except OSError as e:
                log(f"[ERROR] 心跳发送失败: {e}")
            except Exception as e:
                log(f"[ERROR] 心跳线程异常: {e}")
            next_t += HEARTBEAT_PERIOD
            if next_t <= time.monotonic():
                while next_t <= time.monotonic():
                    next_t += HEARTBEAT_PERIOD
            else:
                time.sleep(next_t - time.monotonic())

    def _recv_loop(self):
        while self._running:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if not self._running:
                    break
                continue
            try:
                self._handle_packet(data)
            except Exception as e:
                log(f"[ERROR] 报文处理异常: {e!r}")

    def _handle_packet(self, data):
        info, body = parse_apdu(data)
        if body is None:
            return
        pd = body.get("PatrolDevice", {})
        command = pd.get("Command")
        items = _items(body)

        # 异常状态上报（文档 §1.3.1.4）
        if "ErrorList" in items:
            self._update_faults(extract_faults(body))

        # 状态上报（文档 §1.3.1.1 / §1.3.1.2）
        if "BasicStatus" in items or "MotionStatus" in items:
            self._update_status(items)

        # 请求-响应帧：主动上报的 Command 恒为 0x00F00000，其余按 msgId 匹配
        if command != TYPE_STATUS_PUSH and info:
            with self._pending_lock:
                pending = self._pending.get(info['msgId'])
            if pending is not None:
                pending.body = body
                pending.event.set()
                return

        # 打印通用错误响应（文档 §1.5）
        code, msg = extract_error(body)
        if code is not None and code != 0:
            log(f"  [ERROR] 机器人返回 ErrorCode={fmt_hex(code)}  ErrorMessage={msg}")

    def _update_status(self, items):
        new = {}
        bs = items.get("BasicStatus")
        if isinstance(bs, dict):
            # BasicStatus 为权威来源：MotionState / Gait / ControlUsageMode 等
            for k in ("MotionState", "Gait", "ControlUsageMode", "HES",
                      "PowerManagement", "Sleep", "Model", "Version", "Charge"):
                if k in bs:
                    new[k] = bs[k]
            self._basic_ts = time.monotonic()
        ms = items.get("MotionStatus")
        if isinstance(ms, dict):
            # MotionStatus 补充速度类字段（等待"已停止"依赖这些字段）
            for k in ("LinearX", "LinearY", "AngularZ", "Height",
                      "Payload", "RemainMile", "AccX", "AccY", "AccZ"):
                if k in ms:
                    new[k] = ms[k]
            # MotionState 两个上报都有；仅在 BasicStatus 超过 1s 未更新时兜底写入，
            # 避免 10Hz 的 MotionStatus 覆盖掉权威值
            if "MotionState" in ms and time.monotonic() - self._basic_ts > 1.0:
                new["MotionState"] = ms["MotionState"]

        if not new:
            return
        with self._status_lock:
            self._status.update(new)

        # 状态变化时打印（仅 BasicStatus 包会同时带齐三个字段，2Hz，不会刷屏）
        snapshot = (new.get("MotionState"), new.get("Gait"), new.get("ControlUsageMode"))
        if all(v is not None for v in snapshot) and snapshot != self._prev_log:
            if self._prev_log is None or snapshot[:2] != self._prev_log[:2]:
                log(f"  [STATUS] MotionState={snapshot[0]}({_fmt(snapshot[0], MOTION_NAMES)}), "
                    f"Gait={fmt_hex(snapshot[1])}({_fmt(snapshot[1], GAIT_NAMES)}), "
                    f"Mode={snapshot[2]}({_fmt(snapshot[2], MODE_NAMES)})")
            self._prev_log = snapshot

    def _update_faults(self, faults):
        for f in faults:
            code = f.get("Code")
            if not isinstance(code, int):
                continue
            ftype = f.get("Type")
            name = FAULT_NAMES.get(code) or f.get("Name") or "未知故障"
            if ftype == 2:  # TYPE_STOP
                self._faults.pop(code, None)
                log(f"  [FAULT] 已解除 0x{code:04X} {name}")
                continue
            sev = max(f.get("Severities") or [0])
            resources = f.get("Resources") or []
            self._faults[code] = {"name": name, "severity": sev,
                                  "resources": resources, "details": f.get("Details", "")}
            log(f"  [FAULT] {SEVERITY_NAMES.get(sev, sev)} 0x{code:04X} {name} "
                f"部件={resources} {f.get('Details', '')}")

    # ---------- 状态读取 ----------

    @property
    def status(self):
        """返回状态快照副本，避免读到半更新状态。"""
        with self._status_lock:
            return dict(self._status)

    @property
    def max_active_severity(self):
        return max((f["severity"] for f in self._faults.values()), default=0)

    @property
    def active_faults(self):
        return dict(self._faults)


class _Pending:
    __slots__ = ("event", "body")

    def __init__(self):
        self.event = threading.Event()
        self.body = None


# ========================== 状态机辅助 ==========================

def _as_set(v):
    if v is None:
        return None
    if isinstance(v, int):
        return {v}
    return set(v)


def wait_for_state(client, motion=None, gait=None, timeout=15.0, poll=0.2):
    """等待 MotionState / Gait 进入目标集合（任一为空表示不关心）。"""
    motion_set = _as_set(motion)
    gait_set = _as_set(gait)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        st = client.status
        ms, g = st.get("MotionState"), st.get("Gait")
        if (motion_set is None or ms in motion_set) and (gait_set is None or g in gait_set):
            return True
        time.sleep(poll)
    return False


def wait_until_stopped(client, linear_th=STOP_LINEAR_TH, angular_th=STOP_ANGULAR_TH,
                       stable_time=STOP_STABLE_TIME, timeout=STOP_TIMEOUT):
    """基于 MotionStatus 的 LinearX/LinearY/AngularZ 判定机器人是否真的停住。

    文档 §1.2.4 要求步态切换必须在机器人完全停止后下发，
    仅凭 MotionState==17（已进入 RL 控制）不能代替"已停止"。
    """
    deadline = time.monotonic() + timeout
    stable_since = None
    while time.monotonic() < deadline:
        st = client.status
        vx, vy, wz = st.get("LinearX"), st.get("LinearY"), st.get("AngularZ")
        if vx is None or vy is None or wz is None:
            stable_since = None  # 尚无运控状态上报，无法判定
        elif abs(vx) < linear_th and abs(vy) < linear_th and abs(wz) < angular_th:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= stable_time:
                return True
        else:
            stable_since = None
        time.sleep(0.05)  # 需高于 MotionStatus 的 10Hz 上报频率
    return False


def wait_for_first_status(client, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if "MotionState" in client.status:
            return True
        time.sleep(0.1)
    return False


def describe_status(client):
    st = client.status
    ms, g, m = st.get("MotionState"), st.get("Gait"), st.get("ControlUsageMode")
    log(f"  MotionState={ms}({_fmt(ms, MOTION_NAMES)})  "
        f"Gait={fmt_hex(g)}({_fmt(g, GAIT_NAMES)})  "
        f"Mode={m}({_fmt(m, MODE_NAMES)})")
    log(f"  Model={st.get('Model')}  Version={st.get('Version')}  "
        f"Charge={st.get('Charge')}  HES={st.get('HES')}  "
        f"PowerManagement={st.get('PowerManagement')}")


# ========================== 主流程 ==========================

def run_sequence():
    prof = PROFILES[PROFILE]
    log(f"\n=== ASDU UDP Control  robot={HOST}:{PORT}  profile={PROFILE} ===")
    log(f"    运动选型: {prof['name']}  Gait=0x{prof['gait']:04X}  "
        f"Command=0x{prof['cmd']:08x}  单位={prof['unit']}\n")
    log("【提醒】请确认已在 robotserve 配置中关闭 30004 端口加密（文档 §1.1.2）")
    if PROFILE == "nav":
        log("【提醒】nav 模式：爬楼梯必须在进入楼梯前切到 0x3003，严禁进入后再切（文档 §1.2.4）")

    client = RobotClient(HOST, PORT, profile=prof)
    client.start()
    try:
        # --- 0. 等待状态上报 ---
        log("\n[0/6] 等待状态上报 ...")
        if not wait_for_first_status(client, timeout=5):
            log("  [FATAL] 5s 内未收到任何状态上报。请检查：")
            log("          1) robotserve 配置中 30004 端口加密是否已关闭")
            log("          2) HOST/PORT 是否正确、网络是否可达、防火墙是否放行")
            return
        describe_status(client)

        # --- 0b. 使用模式校验（文档 §1.2.2 / §1.2.5 / §1.2.6）---
        # 轴指令仅在常规模式和辅助模式下执行，真实轴指令仅在导航模式下生效。
        # 两者都不返回响应帧，被拒时拿不到任何错误反馈，所以必须在这里先自查。
        mode = client.status.get("ControlUsageMode")
        if REQUIRE_MODE_CHECK and mode not in prof["modes"]:
            want = "/".join(f"{m}({_fmt(m, MODE_NAMES)})" for m in prof["modes"])
            log(f"  [FATAL] 当前使用模式为 {mode}({_fmt(mode, MODE_NAMES)})，")
            log(f"          而 {prof['name']} 的 {'轴指令' if PROFILE == 'manual' else '真实轴指令'}"
                f" 要求使用模式为 {want}。")
            log(f"          请先用 App 或 Mode 指令切换使用模式，")
            log(f"          或将脚本中的 REQUIRE_MODE_CHECK 设为 False 强制继续。")
            return

        # --- 1. 起立 ---
        # 文档 §2.2.1：ASDU 下发站立指令后，完成起立会自动进入 RL 控制状态(17)，
        # 1 只是"正在起立"的中间态，因此这里等的是 17。
        ms = client.status.get("MotionState")
        log(f"\n[1/6] 起立 (MotionParam={MOTION_STAND})")
        if ms in (MOTION_STAND, MOTION_RL_CONTROL):
            log(f"  [SKIP] 已处于 {ms}({_fmt(ms, MOTION_NAMES)})")
        else:
            client.send_and_wait(make_motion_state(MOTION_STAND), "起立", timeout=2.0)
            if wait_for_state(client, motion=MOTION_RL_CONTROL, timeout=20):
                log("  [OK] 已起立并进入 RL 控制状态(17)")
            else:
                cur = client.status.get("MotionState")
                log(f"  [FATAL] 20s 内未进入 RL 控制状态，当前 "
                    f"MotionState={cur}({_fmt(cur, MOTION_NAMES)})")
                log("          未进入 RL 控制则无法接收轴指令，终止流程。")
                return

        # --- 2. 等待完全停止 ---
        # 文档 §1.2.4：步态切换需在机器人完全停止后下发，否则指令会被缓存。
        log("\n[2/6] 确认机器人已停止 ...")
        if wait_until_stopped(client, timeout=STOP_TIMEOUT):
            log("  [OK] 已确认停止 (LinearX/LinearY/AngularZ 均低于阈值)")
        else:
            st = client.status
            log(f"  [WARN] 未能确认停止（LinearX={st.get('LinearX')} "
                f"LinearY={st.get('LinearY')} AngularZ={st.get('AngularZ')}）。")
            log("         文档 §1.2.4 规定此时下发的步态切换会被缓存，")
            log("         继续执行，但随后的步态确认可能失败。")

        # --- 3. 步态切换 ---
        log(f"\n[3/6] 切换步态 GaitParam=0x{prof['gait']:04X} ({prof['name']})")
        resp = client.send_and_wait(make_gait_switch(prof["gait"]), "步态切换", timeout=2.0)
        code, emsg = extract_error(resp) if resp else (None, None)
        if code not in (None, 0):
            log(f"  [FATAL] 机器人拒绝步态切换: ErrorCode={fmt_hex(code)} {emsg}")
            return
        if wait_for_state(client, gait=prof["gait"], timeout=10):
            log(f"  [OK] 已切换到 {prof['name']}")
        else:
            g = client.status.get("Gait")
            log(f"  [FATAL] 10s 内未切换到目标步态，当前 Gait={fmt_hex(g)}"
                f"({_fmt(g, GAIT_NAMES)})，终止以免用错步态运动。")
            return

        # --- 4. 定速前进 ---
        log(f"\n[4/6] 前进 {VELOCITY_DURATION}s  "
            f"X={prof['x']} Y={VELOCITY_Y} Yaw={VELOCITY_YAW} ({prof['unit']}) "
            f"@{VELOCITY_HZ}Hz")
        aborted = client.move(x=prof["x"], y=VELOCITY_Y, yaw=VELOCITY_YAW,
                              duration=VELOCITY_DURATION)
        if aborted:
            log(f"  [!!] 运动被中断：{aborted}")
        else:
            log("  [OK] 运动结束，已下发零速度")

        log("\n[5/6] 确认停止 ...")
        if wait_until_stopped(client, timeout=STOP_TIMEOUT):
            log("  [OK] 已确认停止")
        else:
            log("  [WARN] 未能确认停止，仍继续执行趴下")

        # --- 6. 趴下 ---
        # 正常路径的终点，机器人此时位于平地，趴下是安全的。
        # 异常路径（finally）不强制趴下，原因见 finally 中的注释。
        log(f"\n[6/6] 趴下 (MotionParam={MOTION_CROUCH})")
        client.send_and_wait(make_motion_state(MOTION_CROUCH), "趴下", timeout=2.0)
        if wait_for_state(client, motion=MOTION_CROUCH, timeout=15):
            log("  [OK] 已趴下")
        else:
            cur = client.status.get("MotionState")
            log(f"  [!!] 超时，当前 MotionState={cur}({_fmt(cur, MOTION_NAMES)})")

    except KeyboardInterrupt:
        log("\n[INTERRUPT] 收到 Ctrl+C，正在执行安全停止 ...")
    except Exception as e:
        log(f"\n[FATAL] 未捕获异常: {e!r}")
        traceback.print_exc()
    finally:
        # 安全退出：只做"原地停止"，不强制趴下。
        # 此时机器人可能正站在楼梯上，强制趴下有跌落风险，站立原地交给人接管更安全。
        log("\n=== 安全退出 ===")
        try:
            if client.motion_issued:
                n = client.stop_motion()
                log(f"已下发零速度（{n} 帧）")
            else:
                # 本轮从未下发过运动指令，无需急停，也避免向不匹配的模式发送轴指令
                log("本轮未下发过运动指令，无需急停")
        except Exception as e:
            log(f"[ERROR] 安全停止失败: {e!r}")
        try:
            faults = client.active_faults
            if faults:
                log("退出时仍有活动故障：")
                for code, info in faults.items():
                    log(f"  0x{code:04X} {info['name']} "
                        f"severity={info['severity']} 部件={info['resources']}")
        except Exception as e:
            log(f"[ERROR] 读取故障列表失败: {e!r}")
        try:
            client.stop()
            log("已停止心跳并关闭 socket。")
        except Exception as e:
            log(f"[ERROR] 关闭客户端失败: {e!r}")


if __name__ == "__main__":
    run_sequence()
