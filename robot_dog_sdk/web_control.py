#!/usr/bin/env python3
"""
机器狗 Web 遥控后端。

架构（单进程，三线程 + uvicorn）：
    线程A  RobotClient 心跳(1Hz) + 接收状态        —— 复用 asdu_udp_control.py
    线程B  20Hz 控制循环：持续流式下发当前目标速度   —— 本文件核心
    线程C  动作队列：起立/趴下/切步态等耗时动作串行执行
    主线程 uvicorn：HTTP 静态页 + WebSocket

为什么控制循环必须"持续发"：
    文档 §1.2.5 的轴指令没有响应帧，且 §1.5 的 0xE006 要求"2s 内指令来自同一客户端"。
    持续下发既维持了控制权，也让浏览器端的任何故障都只表现为"指令停发"，
    进而被看门狗兜住 —— 而不是让机器狗保持最后一条速度指令跑飞。

四层安全闸：
    1. 看门狗    —— 400ms 内没收到浏览器意图 -> 归零
    2. 状态门    —— 收不到状态 / HES / 故障 / 非 RL 控制 -> 归零
    3. 限幅      —— 速度上限 + 每周期变化量限幅（避免瞬间满速）
    4. 时长限制  —— 连续运动超过 30s 强制归零，需重新按（防"按住不放人失能"）

指令码按【当前实际使用模式】动态选择（不要用静态 profile）：
    文档 §1.2.4：步态切换会自动切换运动模式。切到 0x3003 会把 ControlUsageMode
    变成 1（导航），此时 §1.2.5 的比例轴指令失效，必须改用 §1.2.6 的真实轴指令。
    若用静态 profile 判断，切完导航步态遥控器会把自己锁死。

用法：
    python3 robot_sim.py 31004                 # 另开一个终端跑模拟器
    python3 web_control.py --host 127.0.0.1 --port 31004
    # 浏览器打开 http://127.0.0.1:8000

    python3 web_control.py                     # 默认连真机 10.21.33.103:30004
"""

import argparse
import asyncio
import json
import os
import queue
import threading
import time
from collections import deque

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse

# ---- 复用已经过测试的协议层与客户端 ----
from asdu_udp_control import (
    CMD_AXIS, CMD_AXIS_REAL, GAIT_NAMES, MODE_NAMES, MOTION_NAMES,
    RobotClient, build_apdu, extract_error, make_gait_switch,
    make_motion_state, make_velocity, now_str, parse_apdu, wait_for_state,
    wait_until_stopped,
)

# 关节编号 -> 名称。顺序取自文档 §1.3.1.2 对 MotorStatus.Joint 的说明：
# LeftFrontHipX, LeftFrontHipY, LeftFrontKnee, LeftFrontWheel,
# RightFrontHipX, ..., RightBackWheel
JOINT_NAMES = [
    "左前髋X", "左前髋Y", "左前膝", "左前轮",
    "右前髋X", "右前髋Y", "右前膝", "右前轮",
    "左后髋X", "左后髋Y", "左后膝", "左后轮",
    "右后髋X", "右后髋Y", "右后膝", "右后轮",
]


class WebRobotClient(RobotClient):
    """在 RobotClient 上补一层：抓取设备状态上报里的 16 路电机/驱动器温度。

    设备状态上报的 Items 里是 DeviceTemperature / BatteryList / CPU / GPS，
    既没有 BasicStatus 也没有 MotionStatus，所以父类的 _update_status 不会碰它。
    这里用子类覆盖 _handle_packet 旁路抓取，**不改动已经测试过的 asdu_udp_control.py**。

    判断依据是 Items 里有没有 DeviceTemperature，**不依赖 Type** ——
    因为文档 §1.3.1.3 把这个上报的 Type 写成 0x0010 0002，实机是 0x0030 0002
    （见 README 第 9 节），按 Items 判断两种都对得上。
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self._device = {}
        self._device_lock = threading.Lock()

    def _handle_packet(self, data):
        _, body = parse_apdu(data)
        if body:
            items = body.get("PatrolDevice", {}).get("Items")
            if isinstance(items, dict) and "DeviceTemperature" in items:
                dt = items.get("DeviceTemperature") or {}
                bats = items.get("BatteryList") or []
                with self._device_lock:
                    self._device = {
                        "motor": list(dt.get("Motor") or []),
                        "driver": list(dt.get("Driver") or []),
                        "battery": dict(bats[0]) if bats and isinstance(bats[0], dict) else {},
                        "ts": time.monotonic(),
                    }
        super()._handle_packet(data)

    @property
    def device(self):
        """最近一次设备状态上报（含温度）。"""
        with self._device_lock:
            return dict(self._device)

# ============================================================
# 配置
# ============================================================

WATCHDOG_MS = 400           # 浏览器意图超时
CONTROL_HZ = 20             # 控制循环频率
# 每周期(50ms)目标变化量上限。原值 0.05 表示"每秒最多涨 1.0"，
# 从 0 爬到 0.5 要 **0.5 秒** —— 结果就是"点一下"完全没反应：
#   单击 ≈ 0.1 秒 → 输出只涨到 ≈0.1 → 落在轴指令的死区里（实测 0.15 都不动）
# 改成 0.15：0→0.5 只要 0.2 秒，单击也能动起来，同时仍是个斜坡不会瞬间满速。
# 【实机教训】轴指令在低值区不响应且非单调，所以必须让它尽快爬出死区。
SLEW_PER_TICK = 0.15        # 每周期(50ms)目标变化量上限（= 每秒最多涨 3.0）
MAX_SCALE = 1.0             # 速度滑条上限。1.0 = 满量程（导航模式 X 约 1.67 m/s）。
                            # 这是【上限】不是【起步值】：滑条起步仍由 --scale 决定。
                            # 用 --max-scale 可调小，比如 0.6 约等于 1.0 m/s。
STATUS_STALE_S = 1.2        # 状态上报超时（BasicStatus 是 2Hz，留出丢两帧的余量）
ABORT_SEVERITY = 4          # 活动故障 severity >= 此值 -> 停止下发（3WARN/4ERROR/5FATAL）
MAX_CONTINUOUS_DRIVE_S = 30.0   # 连续运动上限，超过强制归零并需重新按
SEND_FAIL_LIMIT = 5         # 连续发送失败次数上限

MOTION_RL_CONTROL = 17
MOTION_CROUCH = 4
MOTION_STAND = 1

# 按【当前实际使用模式】选择指令码与满量程。
# §1.2.5 比例轴指令仅常规(0)/辅助(2)模式生效，量程是"相对最大速度的比例"。
# §1.2.6 真实轴指令仅导航(1)模式生效，量程是物理量（文档给的限值）。
MODE_AXIS = {
    0: {"cmd": CMD_AXIS,      "label": "常规模式/比例量", "cap": {"x": 1.0, "y": 1.0, "yaw": 1.0}},
    2: {"cmd": CMD_AXIS,      "label": "辅助模式/比例量", "cap": {"x": 1.0, "y": 1.0, "yaw": 1.0}},
    1: {"cmd": CMD_AXIS_REAL, "label": "导航模式/物理量", "cap": {"x": 1.67, "y": 0.4, "yaw": 1.0}},
}

# 【实机实测 2026-09-19】转向符号与文档相反。
# 文档 §1.2.5 写"参数值的正负方向请参考右手坐标系确定"（该图在文本版里丢失），
# 但实机表现：下发 Yaw=+1，机器狗实际**左转**；界面上按"右转"却向左转。
# 这里统一取反，让界面语义正确（右转就是右转）。
# 若厂商固件更新后此行为改变，把它改成 +1.0 即可（或命令行传 --yaw-sign）。
YAW_SIGN = -1.0

GAIT_CHOICES = {
    0x1001: "基础（常规）",
    0x1003: "楼梯（常规）",
    0x3002: "平地（导航）",
    0x3003: "楼梯（导航）",
}


def now():
    return time.monotonic()


def _clamp(v, lo=-1.0, hi=1.0):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return 0.0
    return max(lo, min(hi, v))


# ============================================================
# 事件广播（线程安全，客户端按 id 增量拉取）
# ============================================================

class Events:
    def __init__(self, maxlen=300):
        self.lock = threading.Lock()
        self.buf = deque(maxlen=maxlen)
        self.seq = 0

    def add(self, level, text):
        with self.lock:
            self.seq += 1
            self.buf.append({"id": self.seq, "ts": time.time(),
                             "level": level, "text": text})

    def since(self, last_id):
        with self.lock:
            return [e for e in self.buf if e["id"] > last_id]


EVENTS = Events()


def log_event(level, text):
    EVENTS.add(level, text)
    print(f"[{level.upper()}] {text}", flush=True)


# ============================================================
# 共享状态
# ============================================================

class Teleop:
    def __init__(self, scale=0.15):
        self.lock = threading.Lock()
        self.intent = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.intent_ts = 0.0
        self.controller = None
        self.scale = scale
        self.estop = False
        self.action_running = False
        self.out = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.gate = ""
        self.must_release = False       # 需先松手才能再动（见 set_intent 注释）
        self.held_during_action = False # 动作执行期间操作员是否一直按着
        self.drive_since = 0.0          # 持续运动起始时刻（用于时长限制）
        self.cmd_label = "—"

    def set_intent(self, cid, x, y, yaw):
        """返回 "" 表示接受；否则返回被拒绝的原因（供调用方记日志）。"""
        with self.lock:
            if cid != self.controller:
                return f"客户端 {cid} 不是控制者（当前控制者 {self.controller}）"
            v = {"x": _clamp(x), "y": _clamp(y), "yaw": _clamp(yaw)}
            moving = any(abs(t) > 0.01 for t in v.values())

            if self.action_running:
                # 动作期间不接受意图（避免步态切换被缓存期间堆下一条速度指令），
                # 但记录"操作员仍按着"，动作结束后要求他先松手
                if moving:
                    self.held_during_action = True
                return "有动作正在执行，暂不接受操控"

            if self.must_release:
                # 只有"动作期间一直按着"或"连续运动超时"才置此标志，
                # 目的是防止动作结束的瞬间突然恢复运动。操作员没按着时不应误伤。
                if moving:
                    return "需要先松开所有操作再重新按住"
                self.must_release = False

            self.intent = v
            self.intent_ts = now()
            return ""

    def clear_intent(self):
        with self.lock:
            self.intent = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            self.intent_ts = now()

    def claim(self, cid):
        with self.lock:
            self.controller = cid
            self.intent = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            self.intent_ts = now()
            # 必须显式清掉：上一个客户端断开时 release() 会把它置 True，
            # 新接管的操作员什么都没按过，若不清理他第一下按会被静默吞掉
            self.must_release = False
            self.held_during_action = False
            self.drive_since = 0.0

    def release(self, cid):
        """客户端断开 —— 断开即视为松手，立刻归零。"""
        with self.lock:
            if self.controller == cid:
                self.controller = None
            self.intent = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            self.intent_ts = 0.0
            self.must_release = True

    def begin_action(self):
        with self.lock:
            self.intent = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            self.intent_ts = now()
            self.held_during_action = False
            self.must_release = False

    def end_action(self):
        """动作结束时，只有操作员全程按着才要求他先松手。"""
        with self.lock:
            self.must_release = self.held_during_action
            self.held_during_action = False

    def snapshot(self):
        with self.lock:
            return (dict(self.intent), self.intent_ts, self.controller, self.scale,
                    self.estop, self.action_running, dict(self.out), self.gate,
                    self.must_release, self.cmd_label)


# ============================================================
# 20Hz 控制循环
# ============================================================

class ControlLoop(threading.Thread):
    def __init__(self, client, teleop, abort_severity=ABORT_SEVERITY, observe=False,
                 yaw_sign=YAW_SIGN):
        super().__init__(daemon=True, name="control")
        self.client = client
        self.teleop = teleop
        self.abort_severity = abort_severity
        self.observe = observe      # True = 只读观察：照常算 out，但一帧都不发出去
        self.yaw_sign = yaw_sign    # -1 = 实机实测方向，见 YAW_SIGN 注释
        self.running = True
        self.send_fail = 0
        self.last_gate_log = 0.0
        self.stats = {"hz": 0.0, "sent": 0}
        self._hz_t0 = now()
        self._hz_n = 0

    def run(self):
        period = 1.0 / CONTROL_HZ
        next_t = now()
        while self.running:
            try:
                self.tick()
            except OSError as e:
                self.send_fail += 1
                if self.send_fail == SEND_FAIL_LIMIT:
                    log_event("error", f"连续 {self.send_fail} 次发送失败: {e} —— 已停止下发")
                time.sleep(0.05)
            except Exception as e:
                log_event("error", f"控制循环异常: {e!r}")
                time.sleep(0.1)
            next_t += period
            if next_t <= now():
                while next_t <= now():
                    next_t += period
            else:
                time.sleep(next_t - now())

    def tick(self):
        t = self.teleop
        status = self.client.status
        mode = status.get("ControlUsageMode")
        profile = MODE_AXIS.get(mode)

        gate = self._gate(status, mode)

        with t.lock:
            ctrl = t.controller
            fresh = (now() - t.intent_ts) * 1000.0 < WATCHDOG_MS
            blocked = t.estop or t.action_running or bool(gate) or profile is None
            if ctrl and fresh and not blocked:
                cap = profile["cap"]
                target = {k: t.intent[k] * t.scale * cap[k] for k in ("x", "y", "yaw")}
            else:
                target = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            t.gate = gate

            # 连续运动时长限制
            moving = any(abs(v) > 0.01 for v in target.values())
            if moving:
                if t.drive_since == 0.0:
                    t.drive_since = now()
                elif now() - t.drive_since > MAX_CONTINUOUS_DRIVE_S:
                    target = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                    t.must_release = True
                    t.gate = f"连续运动超过 {MAX_CONTINUOUS_DRIVE_S:.0f}s，已强制归零，请松开后重新按"
            else:
                t.drive_since = 0.0

            # 每周期变化量限幅
            out = {}
            for k in ("x", "y", "yaw"):
                d = max(-SLEW_PER_TICK, min(SLEW_PER_TICK, target[k] - t.out[k]))
                out[k] = round(t.out[k] + d, 4)
            t.out = out
            t.cmd_label = profile["label"] if profile else "—"

        if self.observe:
            # 只读观察模式：out 照常算好并上报（界面上能看到"本该发什么速度"），
            # 但不发送任何轴指令。用于首次连真机时验证界面和状态，机器狗零风险。
            self._hz_n += 1
            self._tick_hz()
            return

        if self.send_fail >= SEND_FAIL_LIMIT:
            return      # 发送持续失败，暂停下发（仍保留心跳与接收）

        cmd = profile["cmd"] if profile else CMD_AXIS
        apdu, _ = build_apdu(make_velocity(out["x"], out["y"], 0.0, 0.0, 0.0,
                                           out["yaw"] * self.yaw_sign, command=cmd))
        try:
            self.client._sendto(apdu)
            self.send_fail = 0
            self._hz_n += 1
        except OSError:
            raise
        self._tick_hz()

    def _tick_hz(self):
        """每秒统计一次实际控制循环频率，供界面显示。"""
        dt = now() - self._hz_t0
        if dt >= 1.0:
            self.stats["hz"] = self._hz_n / dt
            self._hz_n = 0
            self._hz_t0 = now()

    def _gate(self, status, mode):
        """返回拦截原因；空串表示允许运动。"""
        if not status or "MotionState" not in status:
            return "尚未收到机器人状态上报"
        age = now() - float(getattr(self.client, "_basic_ts", 0.0) or 0.0)
        if age > STATUS_STALE_S:
            return f"状态上报已中断 {age:.1f}s（检查 30004 加密是否已关闭）"
        if status.get("HES") == 1:
            return "机器人硬急停(HES)已触发"
        if self.client.max_active_severity >= self.abort_severity:
            names = "、".join(f["name"] for f in self.client.active_faults.values())
            return f"存在故障（severity>={self.abort_severity}）：{names}"
        if mode not in MODE_AXIS:
            return f"未知使用模式 {mode}，无法选择轴指令"
        if status.get("MotionState") != MOTION_RL_CONTROL:
            ms = status.get("MotionState")
            return f"未处于 RL 控制状态（当前 {ms} {MOTION_NAMES.get(ms, '?')}），请先起立"
        return ""


# ============================================================
# 动作队列
# ============================================================

class ActionWorker(threading.Thread):
    def __init__(self, client, teleop):
        super().__init__(daemon=True, name="action")
        self.client = client
        self.teleop = teleop
        self.q = queue.Queue()
        self.running = True

    def submit(self, name, **kw):
        self.q.put((name, kw))

    def run(self):
        while self.running:
            try:
                name, kw = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            self.teleop.begin_action()
            with self.teleop.lock:
                self.teleop.action_running = True
            log_event("info", f"开始执行动作：{name}")
            try:
                handler = getattr(self, f"do_{name}", None)
                if handler is None:
                    log_event("error", f"未知动作：{name}")
                else:
                    handler(**kw)
            except Exception as e:
                log_event("error", f"动作 {name} 异常：{e!r}")
            finally:
                with self.teleop.lock:
                    self.teleop.action_running = False
                self.teleop.end_action()
                log_event("info", f"动作结束：{name}")

    # ---- 动作定义 ----

    def do_stand(self):
        r = self.client.send_and_wait(make_motion_state(MOTION_STAND), "起立", timeout=2.0)
        code, msg = extract_error(r) if r else (None, None)
        if code not in (None, 0):
            log_event("error", f"起立被拒绝：ErrorCode={code} {msg}")
            return
        # 文档 §2.2.1：ASDU 起立完成后自动进入 RL 控制(17)
        # poll 取小值：轮询间隔就是"动作还没结束、操作员的按键会被丢弃"的窗口长度
        if wait_for_state(self.client, motion=MOTION_RL_CONTROL, timeout=20, poll=0.05):
            log_event("ok", "已起立并进入 RL 控制状态")
        else:
            cur = self.client.status.get("MotionState")
            log_event("error", f"20s 内未进入 RL 控制状态（当前 {cur}）")

    def do_crouch(self):
        r = self.client.send_and_wait(make_motion_state(MOTION_CROUCH), "趴下", timeout=2.0)
        code, msg = extract_error(r) if r else (None, None)
        if code not in (None, 0):
            log_event("error", f"趴下被拒绝：ErrorCode={code} {msg}")
            return
        if wait_for_state(self.client, motion=MOTION_CROUCH, timeout=15, poll=0.05):
            log_event("ok", "已趴下")
        else:
            cur = self.client.status.get("MotionState")
            log_event("warn", f"15s 内未趴下（当前 {cur}）")

    def do_gait(self, value):
        value = int(value)
        # 文档 §1.2.4：步态切换必须在完全停止后下发，否则会被缓存
        log_event("info", "等待机器人完全停止 ...")
        if not wait_until_stopped(self.client, timeout=5.0):
            log_event("warn", "未能确认停止，步态切换可能被缓存（文档 §1.2.4）")
        r = self.client.send_and_wait(make_gait_switch(value), "步态切换", timeout=2.0)
        code, msg = extract_error(r) if r else (None, None)
        if code not in (None, 0):
            log_event("error", f"步态切换被拒绝：ErrorCode={code} {msg}")
            return
        if wait_for_state(self.client, gait=value, timeout=10, poll=0.05):
            mode = self.client.status.get("ControlUsageMode")
            log_event("ok", f"已切换到 {GAIT_CHOICES.get(value, hex(value))}；"
                            f"当前使用模式 {mode}（{MODE_NAMES.get(mode, '?')}）")
            if value in (0x3002, 0x3003):
                log_event("warn", "导航步态：爬楼梯必须在进入楼梯前切换，严禁进入后再切（文档 §1.2.4）")
        else:
            g = self.client.status.get("Gait")
            log_event("error", f"10s 内未切换（当前 {hex(g) if isinstance(g, int) else g}）")

    def do_mode(self, value):
        value = int(value)
        body = {"PatrolDevice": {"Type": 0x00100002, "Command": 0x00500002,
                                 "Time": now_str(), "Items": {"Mode": value}}}
        r = self.client.send_and_wait(body, "使用模式切换", timeout=3.0)
        code, msg = extract_error(r) if r else (None, None)
        if code not in (None, 0):
            log_event("error", f"模式切换被拒绝：ErrorCode={code} {msg}")
        else:
            log_event("ok", f"使用模式切换 -> {MODE_NAMES.get(value, value)}")


# ============================================================
# Web 应用
# ============================================================

app = FastAPI(title="机器狗遥控")
STATE = {}
CLIENTS_LOCK = threading.Lock()
INTENT_REJECT_LAST = {}   # 拒绝原因 -> 上次记录时刻（限流用）
DEBUG_FAULT = {"severity": None}   # 仅供联调：本地把故障等级压住，便于测试运动


def build_state_msg(cid):
    client = STATE["client"]
    teleop = STATE["teleop"]
    st = client.status
    (intent, _ts, controller, scale, estop, acting, out, gate,
     must_release, cmd_label) = teleop.snapshot()

    faults = [{"code": c, "name": i.get("name"), "severity": i.get("severity"),
               "resources": i.get("resources", [])}
              for c, i in client.active_faults.items()]

    # 关节温度。同时把被 0x8001(电机过温)/0x8002(驱动器过温) 点名的关节标出来，
    # 故障的 Resources 形如 ["joint#0", "joint#4"]。
    dev = client.device
    motor_t = dev.get("motor") or []
    driver_t = dev.get("driver") or []
    flagged = set()
    for f in faults:
        if f["code"] in (0x8001, 0x8002):
            for r in f.get("resources") or []:
                if isinstance(r, str) and r.startswith("joint#"):
                    try:
                        flagged.add(int(r.split("#", 1)[1]))
                    except ValueError:
                        pass
    temps = []
    for i, nm in enumerate(JOINT_NAMES):
        temps.append({
            "i": i, "name": nm,
            "motor": motor_t[i] if i < len(motor_t) else None,
            "driver": driver_t[i] if i < len(driver_t) else None,
            "flagged": i in flagged,
        })
    tmax = max([t["driver"] for t in temps if t["driver"] is not None], default=None)

    mode = st.get("ControlUsageMode")
    return {
        "t": "state",
        "you": cid,
        "observe": bool(STATE.get("observe")),
        "controller": controller,
        "can_control": controller == cid,
        "scale": scale,
        "estop": estop,
        "action_running": acting,
        "must_release": must_release,
        "gate": gate,
        "cmd_label": cmd_label,
        "intent": intent,
        "out": out,
        "watchdog_ms": WATCHDOG_MS,
        "control_hz": STATE["control"].stats["hz"],
        "connected": "MotionState" in st,
        "motion_state": st.get("MotionState"),
        "motion_state_name": MOTION_NAMES.get(st.get("MotionState"), "?"),
        "gait": st.get("Gait"),
        "gait_name": GAIT_NAMES.get(st.get("Gait"), "?"),
        "usage_mode": mode,
        "usage_mode_name": MODE_NAMES.get(mode, "?"),
        "axis_kind": MODE_AXIS.get(mode, {}).get("label", "不支持"),
        "linear_x": st.get("LinearX"),
        "linear_y": st.get("LinearY"),
        "angular_z": st.get("AngularZ"),
        "model": st.get("Model"),
        "version": st.get("Version"),
        "charge": st.get("Charge"),
        "hes": st.get("HES"),
        "faults": faults,
        "temps": temps,
        "temp_max": tmax,
        "battery_info": dev.get("battery") or {},
        "temps_age": (time.monotonic() - dev["ts"]) if dev.get("ts") else None,
        "gaits": [{"value": k, "label": v} for k, v in GAIT_CHOICES.items()],
        "modes": [{"value": k, "label": v} for k, v in MODE_NAMES.items()],
    }


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    with CLIENTS_LOCK:
        STATE["next_cid"] = STATE.get("next_cid", 0) + 1
        cid = f"c{STATE['next_cid']}"
    log_event("info", f"客户端 {cid} 已连接")
    await ws.send_text(json.dumps({"t": "hello", "you": cid, "watchdog_ms": WATCHDOG_MS,
                                   "control_hz": CONTROL_HZ, "max_scale": MAX_SCALE},
                                  ensure_ascii=False))

    async def sender():
        last_event = EVENTS.seq
        while True:
            await ws.send_text(json.dumps(build_state_msg(cid), ensure_ascii=False))
            for e in EVENTS.since(last_event):
                last_event = max(last_event, e["id"])
                await ws.send_text(json.dumps({"t": "event", **e}, ensure_ascii=False))
            await asyncio.sleep(0.1)

    send_task = asyncio.create_task(sender())
    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            handle_client_msg(cid, msg)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        log_event("error", f"客户端 {cid} 异常：{e!r}")
    finally:
        send_task.cancel()
        STATE["teleop"].release(cid)
        log_event("info", f"客户端 {cid} 已断开（已释放控制权并归零）")


def handle_client_msg(cid, msg):
    t = msg.get("t")
    teleop = STATE["teleop"]
    if t == "claim":
        teleop.claim(cid)
        log_event("info", f"客户端 {cid} 取得控制权")
    elif t == "intent":
        reason = teleop.set_intent(cid, msg.get("x", 0), msg.get("y", 0), msg.get("yaw", 0))
        if reason:
            # 被拒的意图原本是静默丢弃的，出问题时完全看不到原因。
            # 这里记日志，但按原因限流，避免 20Hz 刷屏。
            now_t = time.monotonic()
            if now_t - INTENT_REJECT_LAST.get(reason, 0) > 3.0:
                INTENT_REJECT_LAST[reason] = now_t
                log_event("warn", f"操控被拒绝：{reason}")
    elif t == "release":
        teleop.clear_intent()
    elif t == "estop":
        with teleop.lock:
            teleop.estop = True
            teleop.intent = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            teleop.must_release = True
        log_event("warn", "急停已触发（零速锁存）。注意：这是软件零速，不是硬件急停！")
    elif t == "estop_reset":
        with teleop.lock:
            teleop.estop = False
            teleop.intent = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            # 点"解除急停"本身就是一次明确的、离散的操作，操作员此时并没按着方向键，
            # 所以这里不置 must_release，否则解除后第一下按会被静默吞掉
            teleop.must_release = False
        log_event("warn", "急停已解除")
    elif t == "scale":
        with teleop.lock:
            teleop.scale = max(0.02, min(MAX_SCALE, float(msg.get("value", 0.15))))
    elif t == "action":
        # 动作必须由持有控制权的客户端发起。
        # 否则会出现：A 正在操控机器狗，B 点一下"趴下"就把狗按倒 —— 很危险。
        # 之前只对运动意图校验控制权、动作没校验，行为不一致，已修正。
        if teleop.controller != cid:
            log_event("warn", f"客户端 {cid} 未持有控制权，拒绝其动作请求（请先点「接管控制」）")
            return
        if teleop.action_running:
            log_event("warn", "已有动作在执行，忽略本次请求")
            return
        name = msg.get("name")
        if name == "gait":
            STATE["actions"].submit("gait", value=msg.get("value"))
        elif name == "mode":
            STATE["actions"].submit("mode", value=msg.get("value"))
        elif name in ("stand", "crouch"):
            STATE["actions"].submit(name)
        else:
            log_event("error", f"未知动作 {name}")


@app.get("/")
async def index():
    with open(os.path.join(STATE["web_dir"], "index.html"), encoding="utf-8") as f:
        return HTMLResponse(f.read())


@app.get("/app.js")
async def appjs():
    with open(os.path.join(STATE["web_dir"], "app.js"), encoding="utf-8") as f:
        return HTMLResponse(f.read(), media_type="application/javascript")


@app.get("/style.css")
async def stylecss():
    with open(os.path.join(STATE["web_dir"], "style.css"), encoding="utf-8") as f:
        return HTMLResponse(f.read(), media_type="text/css")


@app.get("/api/health")
async def health():
    return JSONResponse({"ok": True, "control_hz": STATE["control"].stats["hz"]})


# ============================================================
# 启动
# ============================================================

def main():
    # 必须在任何对 MAX_SCALE 的读取（包括下面的 default=MAX_SCALE）之前声明
    global MAX_SCALE

    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("ROBOT_HOST", "10.21.33.103"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("ROBOT_PORT", "30004")))
    ap.add_argument("--web-port", type=int, default=int(os.environ.get("WEB_PORT", "8000")))
    ap.add_argument("--scale", type=float, default=0.50,
                    help="滑条起步值（不是上限，上限见 --max-scale）。"
                         "0.50 约合 0.83 m/s")
    ap.add_argument("--abort-severity", type=int, default=ABORT_SEVERITY,
                    help="活动故障 severity >= 此值时停止下发（3=WARN 4=ERROR 5=FATAL）")
    ap.add_argument("--observe", action="store_true",
                    help="只读观察模式：只发心跳、不发任何轴指令。首次连真机时用来安全验证界面")
    ap.add_argument("--yaw-sign", type=float, default=YAW_SIGN,
                    help="转向符号。默认 -1（实机实测：下发 +Yaw 实际左转）。改 +1 可恢复文档约定")
    ap.add_argument("--max-scale", type=float, default=MAX_SCALE,
                    help=f"速度滑条的上限（1.0 = 满量程，导航模式下 X 约 1.67 m/s）。"
                         f"默认 {MAX_SCALE}；调小可降低误操作风险，如 0.6 约合 1.0 m/s")
    args = ap.parse_args()
    STATE["observe"] = bool(args.observe)

    MAX_SCALE = max(0.02, min(1.0, args.max_scale))

    STATE["web_dir"] = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")

    print("=" * 66)
    print(" 机器狗 Web 遥控")
    print(f"   机器人    {args.host}:{args.port}")
    print(f"   网页      http://127.0.0.1:{args.web_port}")
    print(f"   控制循环  {CONTROL_HZ}Hz   看门狗 {WATCHDOG_MS}ms   速度上限 {MAX_SCALE}")
    print("=" * 66)
    print("【提醒】连不上真机时，先确认 robotserve 已关闭 30004 端口加密（文档 §1.1.2）")
    print("【提醒】使用网页遥控时请关闭手机 App 的手动控制，否则 2s 同源约束")
    print("        （文档 §1.5 的 0xE006）会让两边互相踢掉，表现为「指令偶尔没反应」")
    print("【提醒】本页面上的「急停」是软件零速，不是硬件急停。文档中 ASDU 没有急停指令")
    print("        （§1.2.3 注：软急停仅支持查询）。真正的安全手段是机器人本体的急停。")
    print(f"【提醒】本机联调：先跑 python3 robot_sim.py {args.port}")
    print()

    # 用一个固定 profile 供 RobotClient 内部使用；实际轴指令码由控制循环按当前模式选
    profile = {"name": "Web遥控", "gait": 0x1003, "cmd": CMD_AXIS,
               "unit": "norm", "x": 0.0, "modes": (0, 1, 2)}
    client = WebRobotClient(args.host, args.port, profile=profile)
    teleop = Teleop(scale=min(args.scale, MAX_SCALE))
    STATE["client"] = client
    STATE["teleop"] = teleop

    client.start()

    # 等待首次状态上报，给出明确诊断。
    # 注意等的是 Model 而不是 MotionState：MotionStatus(10Hz) 也带 MotionState，
    # 只等它的话可能拿到一个还没收到 BasicStatus 的半成品状态，
    # 打印出来就是 Model=None / Mode=None，容易让人以为连错了。
    import time as _t
    deadline = _t.monotonic() + 5
    while _t.monotonic() < deadline and "Model" not in client.status:
        _t.sleep(0.1)
    if "Model" not in client.status:
        log_event("error", "5s 内未收到状态上报。请检查：")
        log_event("error", "  1) robotserve 配置中 30004 端口加密是否已关闭（文档 §1.1.2）")
        log_event("error", "  2) 机器人地址/端口是否正确、网络可达、防火墙放行")
    else:
        st = client.status
        log_event("ok", f"已连上机器人：Model={st.get('Model')} Version={st.get('Version')} "
                        f"MotionState={st.get('MotionState')} Gait={hex(st.get('Gait', 0))} "
                        f"Mode={st.get('ControlUsageMode')}")

    control = ControlLoop(client, teleop, abort_severity=args.abort_severity,
                          observe=args.observe, yaw_sign=args.yaw_sign)
    control.start()
    STATE["control"] = control
    actions = ActionWorker(client, teleop)
    actions.start()
    STATE["actions"] = actions

    def watcher():
        last = None
        while True:
            st = client.status
            cur = (st.get("MotionState"), st.get("Gait"), st.get("ControlUsageMode"))
            if cur != last and cur[0] is not None:
                log_event("info", f"机身状态 MotionState={cur[0]}({MOTION_NAMES.get(cur[0],'?')}) "
                                  f"Gait={hex(cur[1]) if isinstance(cur[1], int) else cur[1]}"
                                  f"({GAIT_NAMES.get(cur[1],'?')}) "
                                  f"Mode={cur[2]}({MODE_NAMES.get(cur[2],'?')})")
                last = cur
            time.sleep(0.3)

    threading.Thread(target=watcher, daemon=True, name="watch").start()

    import uvicorn
    try:
        uvicorn.run(app, host="0.0.0.0", port=args.web_port, log_level="warning")
    finally:
        log_event("warn", "关闭中：先停控制循环，再下发零速")
        control.running = False
        control.join(timeout=2)
        actions.running = False
        try:
            client.stop_motion()
        except Exception as e:
            log_event("error", f"归零失败：{e!r}")
        client.stop()


if __name__ == "__main__":
    main()
