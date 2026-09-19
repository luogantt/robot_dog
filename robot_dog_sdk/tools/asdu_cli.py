#!/usr/bin/env python3
"""
ASDU 手动测试工具 —— 相当于这个 UDP 协议的 "curl"。

单进程单 socket：后台按 1Hz 发心跳（这样机器人会把状态回推到本进程的源端口），
前台打印收到的所有回包。所有指令都从同一个源端口发出，满足文档 §1.5
0xE006 "2s 内指令须来自同一客户端" 的要求。

用法：
    python3 asdu_cli.py status                # 只发心跳并监听，看状态上报（默认 10s）
    python3 asdu_cli.py status 30             # 监听 30s
    python3 asdu_cli.py stand                 # 起立
    python3 asdu_cli.py crouch                # 趴下
    python3 asdu_cli.py gait 0x1003           # 切步态
    python3 asdu_cli.py axis 0.12             # 轴指令 X=0.12（比例量），持续 2s
    python3 asdu_cli.py raw eb91eb90...       # 直接发一段十六进制

前置条件：robotserve 配置中已关闭 30004 端口加密（文档 §1.1.2）。
"""

import json
import os
import socket
import struct
import sys
import threading
import time

# 可用环境变量覆盖，便于对着别的 IP 或本地模拟器测试：
#   ASDU_HOST=192.168.1.50 ASDU_PORT=30004 python3 asdu_cli.py status
HOST = os.environ.get("ASDU_HOST", "10.21.33.103")
PORT = int(os.environ.get("ASDU_PORT", "30004"))

SYNC = b'\xeb\x91\xeb\x90'
HDR = 16

MOTION_NAMES = {-2: "软急停", 0: "默认(未上报)", 1: "站立", 2: "关节阻尼",
                4: "趴下", 5: "标零", 17: "RL控制", 0x1001: "阻尼趴下"}
GAIT_NAMES = {0: "无", 0x1001: "基础(常规)", 0x1003: "楼梯(常规)",
              0x3002: "平地(导航)", 0x3003: "楼梯(导航)"}
MODE_NAMES = {0: "常规模式", 1: "导航模式", 2: "辅助模式"}
SEV_NAMES = {3: "WARN", 4: "ERROR", 5: "FATAL"}

_lock = threading.Lock()
_msg_id = 0
_pkt_id = 0
_print_lock = threading.Lock()


def log(msg):
    with _print_lock:
        print(msg, flush=True)


def build_apdu(type_, command, items):
    """构造一帧 APDU：16 字节协议头 + JSON（文档 §1.1.5）。"""
    global _msg_id, _pkt_id
    body = {"PatrolDevice": {"Type": type_, "Command": command,
                             "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                             "Items": items}}
    b = json.dumps(body, separators=(',', ':')).encode()
    with _lock:
        mid, pid = _msg_id, _pkt_id
        _msg_id = (_msg_id + 1) % 0x10000
        _pkt_id = (_pkt_id + 1) % 256
    return struct.pack('<4sHHBBB5s', SYNC, len(b), mid, 0x01, pid, 0x01, b'\x00' * 5) + b


def parse_apdu(data):
    """返回 (kind, 描述文本)。kind 用于控制打印详略。

    kind: basic / motion / fault / error / other / bad
    """
    if len(data) < HDR or data[0:4] != SYNC:
        return "bad", f"非 ASDU 数据 {len(data)} 字节"
    blen, mid, fmt, pid, ver = struct.unpack('<HHBBB', data[4:11])
    if fmt != 0x01:
        return "bad", f"msgId={mid} 非 JSON 格式 (format=0x{fmt:02x})"
    try:
        body = json.loads(data[HDR:HDR + blen].decode())
    except Exception as e:
        return "bad", f"msgId={mid} JSON 解析失败: {e}"
    pd = body.get("PatrolDevice", {})
    t, c, items = pd.get("Type"), pd.get("Command"), pd.get("Items", {})

    if isinstance(items, dict) and "BasicStatus" in items:
        bs = items["BasicStatus"]
        ms, g, m = bs.get("MotionState"), bs.get("Gait"), bs.get("ControlUsageMode")
        return "basic", (f"BasicStatus  MotionState={ms}({MOTION_NAMES.get(ms, '?')})  "
                         f"Gait=0x{g:04x}({GAIT_NAMES.get(g, '?')})  "
                         f"Mode={m}({MODE_NAMES.get(m, '?')})  "
                         f"Model={bs.get('Model')}  Charge={bs.get('Charge')}")
    if isinstance(items, dict) and "MotionStatus" in items:
        msd = items["MotionStatus"]
        return "motion", (f"MotionStatus LinearX={msd.get('LinearX')} "
                          f"LinearY={msd.get('LinearY')} "
                          f"AngularZ={msd.get('AngularZ')} "
                          f"Gait=0x{(msd.get('Gait') or 0):04x}")
    if isinstance(items, dict) and "ErrorList" in items:
        el = items["ErrorList"]
        el = el if isinstance(el, list) else [el]
        out = []
        for f in el:
            if not isinstance(f, dict):
                continue
            sev = max(f.get("Severities") or [0])
            out.append(f"{SEV_NAMES.get(sev, sev)} Code=0x{f.get('Code', 0):04x} "
                       f"{f.get('Name')} 部件={f.get('Resources')}")
        # ErrorList 存在但为空 = 当前无故障，是正常情况，不要打印空行
        if not out:
            return "fault-ok", ""
        return "fault", "异常上报 " + " | ".join(out)
    if isinstance(items, dict) and "ErrorCode" in items:
        code = items["ErrorCode"]
        return "error", (f"ErrorCode=0x{code:04x}  "
                         f"ErrorMessage={items.get('ErrorMessage')}  "
                         f"(Type=0x{t:08x} Command=0x{c:08x})")
    # 未知类型：截断输出。真机会推送文档未记录的大报文（如设备状态含 CPU/温度/GPS
    # 数组，单条几 KB），原样打印会把关键信息淹掉。
    dump = json.dumps(items, ensure_ascii=False)
    if len(dump) > 200:
        dump = dump[:200] + f"...(共 {len(dump)} 字符)"
    return "other", f"Type=0x{t:08x} Command=0x{c:08x} Items={dump}"


class Client:
    def __init__(self, host, port):
        self.target = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))  # 绑定端口，保证回包能收到
        self.sock.settimeout(0.5)
        self.local = self.sock.getsockname()
        self.running = True
        self.counts = {}

    def send(self, apdu, label=""):
        self.sock.sendto(apdu, self.target)
        log(f"[发送] {label}  ({len(apdu)} 字节)")

    def recv_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            kind, desc = parse_apdu(data)
            if not desc:
                continue          # 无内容可打印（例如"当前无故障"的空 ErrorList）
            if kind in ("basic", "motion"):
                # 状态类回包频率高（2Hz / 10Hz），每 10 次打一条，其余只计数
                self.counts[kind] = self.counts.get(kind, 0) + 1
                if self.counts[kind] % 10 != 1:
                    continue
                desc = f"{desc}   [第 {self.counts[kind]} 次]"
            elif kind == "error":
                # 心跳的正常应答不打印，只打真正的错误或命令应答
                code = desc.split("0x")[1][:4]
                if code == "0000" and "Command=0x00000005" in desc:
                    continue
                self.counts["error"] = self.counts.get("error", 0) + 1
            log(f"[接收] {desc}")

    def heartbeat_loop(self, period=1.0):
        while self.running:
            try:
                self.send(build_apdu(0x00100064, 0x00000005, {}), "心跳")
            except OSError as e:
                log(f"[错误] 心跳发送失败: {e}")
                break
            time.sleep(period)

    def start_background(self):
        threading.Thread(target=self.recv_loop, daemon=True).start()
        threading.Thread(target=self.heartbeat_loop, daemon=True).start()

    def close(self):
        self.running = False
        time.sleep(0.3)
        try:
            self.sock.close()
        except OSError:
            pass


def run_duration(client, seconds):
    """监听 seconds 秒并统计状态上报频率。计数从调用时刻重新开始。"""
    client.counts.clear()
    t0 = time.monotonic()
    time.sleep(seconds)
    elapsed = time.monotonic() - t0
    nb = client.counts.get("basic", 0)
    nm = client.counts.get("motion", 0)
    log(f"\n[信息] {elapsed:.1f}s 内收到 BasicStatus {nb} 次、MotionStatus {nm} 次")
    log(f"[提示] 实测频率 {nb/elapsed:.1f}Hz / {nm/elapsed:.1f}Hz，"
        f"文档 §1.3.1.1/§1.3.1.2 规定应为 2Hz / 10Hz")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    cmd = sys.argv[1].lower()
    client = Client(HOST, PORT)
    log(f"[信息] 本机源端口 {client.local[1]}，目标 {HOST}:{PORT}")
    log("[提醒] 若收不到任何回包，先确认 robotserve 已关闭 30004 端口加密（文档 §1.1.2）")
    client.start_background()

    try:
        if cmd == "status":
            run_duration(client, float(sys.argv[2]) if len(sys.argv) > 2 else 10.0)
        elif cmd == "stand":
            client.send(build_apdu(0x00100001, 0x00200002, {"MotionParam": 1}), "起立")
            run_duration(client, 8.0)
        elif cmd == "crouch":
            client.send(build_apdu(0x00100001, 0x00200002, {"MotionParam": 4}), "趴下")
            run_duration(client, 8.0)
        elif cmd == "rl":
            client.send(build_apdu(0x00100001, 0x00200002, {"MotionParam": 17}), "RL控制")
            run_duration(client, 5.0)
        elif cmd == "gait":
            g = int(sys.argv[2], 0)
            client.send(build_apdu(0x00100001, 0x00300002, {"GaitParam": g}),
                        f"切步态 0x{g:04x} ({GAIT_NAMES.get(g, '?')})")
            run_duration(client, 5.0)
        elif cmd == "mode":
            m = int(sys.argv[2], 0)
            client.send(build_apdu(0x00100002, 0x00500002, {"Mode": m}),
                        f"切使用模式 {m} ({MODE_NAMES.get(m, '?')})")
            run_duration(client, 5.0)
        elif cmd == "axis":
            # 文档 §1.2.5：X/Y/Z/Roll/Pitch/Yaw 为最大速度的比例 [-1,1]
            # 该指令不返回任何响应帧，只能靠观察 MotionStatus 判断
            x = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
            y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
            yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
            dur = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0
            items = {"X": x, "Y": y, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": yaw}
            log(f"[信息] 轴指令 X={x} Y={y} Yaw={yaw} 持续 {dur}s @20Hz")
            log("[提醒] 该指令无响应帧（文档 §1.2.5），中断请按 Ctrl+C")
            zero = {"X": 0.0, "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
            end = time.monotonic() + dur
            n = 0
            while time.monotonic() < end:
                # 每帧都要重新构造：msgId 必须逐帧递增（文档 §1.1.5）
                client.sock.sendto(build_apdu(0x00100001, 0x00100002, items),
                                   client.target)
                n += 1
                time.sleep(0.05)
            log(f"[发送] 轴指令 {n} 帧（约 {n/dur:.0f}Hz）")
            for _ in range(10):  # 按控制周期归零，不要在同一个时刻连发
                client.sock.sendto(build_apdu(0x00100001, 0x00100002, zero),
                                   client.target)
                time.sleep(0.05)
            log("[发送] 归零 10 帧")
            run_duration(client, 3.0)
        elif cmd == "raw":
            apdu = bytes.fromhex(sys.argv[2].replace(" ", ""))
            client.send(apdu, f"原始 {len(apdu)} 字节")
            run_duration(client, 5.0)
        else:
            print(__doc__)
    except KeyboardInterrupt:
        log("\n[中断] 正在归零 ...")
        try:
            for _ in range(10):
                client.send(build_apdu(0x00100001, 0x00100002,
                                       {"X": 0.0, "Y": 0.0, "Z": 0.0,
                                        "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}), "归零")
                time.sleep(0.05)
        except Exception as e:
            log(f"[错误] 归零失败: {e}")
    finally:
        client.close()


if __name__ == "__main__":
    main()
