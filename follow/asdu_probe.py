#!/usr/bin/env python3
"""ASDU 心跳探针 —— 确认"能不能摸到运控、狗现在什么状态"。

跟随程序跑在 .102（相机所在机），但运动控制在 .103:30004 —— 是跨机器的。
本脚本先在 .102 上跑一次，确认网络和协议能通，再谈发指令。

    python3 asdu_probe.py                          # 打 .103:30004
    python3 asdu_probe.py --host 127.0.0.1         # 在狗本机上打自己
    python3 asdu_probe.py --seconds 8

输出：
  * 有没有收到状态帧（收到 = 心跳通了，且路径反向往回也通）
  * MotionState / Gait / ControlUsageMode
  * 当前活跃故障的最高严重等级

只发心跳、只读状态，不发任何运动指令 —— 纯只读，安全。
"""

import argparse
import socket
import struct
import sys
import time

HEADER_SYNC = b"\xeb\x91\xeb\x90"
TYPE_HEARTBEAT = 0x00100064
CMD_HEARTBEAT = 0x00000005

MOTION_NAMES = {
    -2: "软急停", 0: "空闲/未上报", 1: "站立", 2: "关节阻尼", 3: "开机阻尼",
    4: "趴下", 5: "标零", 17: "RL控制", 0x1001: "阻尼趴下",
}
GAIT_NAMES = {0x1001: "基础(常规)", 0x1003: "楼梯(常规)",
              0x3002: "平地(导航)", 0x3003: "楼梯(导航)"}
MODE_NAMES = {0: "常规", 1: "导航", 2: "辅助"}
SEV_NAMES = {3: "WARN", 4: "ERROR", 5: "FATAL"}

_msg_id = 0


def build_heartbeat():
    """16 字节头 + JSON。协议见《软件开发指南》§1.1.5 / §1.2.1。"""
    import json
    from datetime import datetime
    global _msg_id
    body = {"PatrolDevice": {
        "Type": TYPE_HEARTBEAT, "Command": CMD_HEARTBEAT,
        "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "Items": {}}}
    b = json.dumps(body, separators=(",", ":")).encode()
    mid = _msg_id
    _msg_id = (_msg_id + 1) % 0x10000
    return struct.pack("<4sHHBBB5s", HEADER_SYNC, len(b), mid, 0x01, 0, 0x01,
                       b"\x00" * 5) + b


def parse(data):
    """→ (type, command, items) 或 None。"""
    import json
    if len(data) < 16 or data[:4] != HEADER_SYNC:
        return None
    blen = struct.unpack("<H", data[4:6])[0]
    try:
        body = json.loads(data[16:16 + blen].decode("utf-8", "replace"))
    except Exception:
        return None
    pd = body.get("PatrolDevice") or {}
    return pd.get("Type"), pd.get("Command"), pd.get("Items") or {}


def main():
    ap = argparse.ArgumentParser(description="ASDU 心跳探针")
    ap.add_argument("--host", default="10.21.33.103")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--raw", type=int, default=0,
                    help="打印前 N 帧的原始 JSON（查 Type 到底是多少）")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.3)
    target = (args.host, args.port)

    print(f"目标 {args.host}:{args.port}")
    print(f"本机出口 {sock.getsockname()}  ← 这个 IP:端口 决定 0xE006 的归属")
    print(f"发心跳 {args.seconds:.0f}s ...\n")

    t0 = time.monotonic()
    last_beat = 0.0
    got = 0
    status = {}
    faults = []
    kinds = {}
    device = {}          # 设备状态上报（电池/温度），Items 里有 DeviceTemperature

    try:
        while time.monotonic() - t0 < args.seconds:
            now = time.monotonic()
            if now - last_beat > 0.9:          # §1.2.1 建议 >=1Hz
                sock.sendto(build_heartbeat(), target)
                last_beat = now
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as e:
                print(f"[socket 错误] {e}")
                break
            got += 1
            p = parse(data)
            if not p:
                continue
            typ, cmd, items = p
            if args.raw > 0 and got <= args.raw:
                import json
                blen = struct.unpack("<H", data[4:6])[0]
                body = json.loads(data[16:16 + blen].decode("utf-8", "replace"))
                pd = body.get("PatrolDevice", {})
                keys = list((pd.get("Items") or {}).keys())
                print(f"[raw {got}] Type={pd.get('Type')!r} "
                      f"(=0x{pd.get('Type', 0):08x})  "
                      f"Command={pd.get('Command')!r}  Items keys={keys}")
            kinds[f"0x{typ:08x}/0x{cmd:08x}"] = kinds.get(
                f"0x{typ:08x}/0x{cmd:08x}", 0) + 1
            bs = items.get("BasicStatus")
            if isinstance(bs, dict):
                status = bs
            el = items.get("ErrorList")
            if isinstance(el, list):
                faults = el
            # 设备状态上报：Items 里是 DeviceTemperature / BatteryList / CPU
            # （Type 实机是 0x0030 0002，文档写 0x0010 0002 —— 按 Items 判断两种都对）
            if "DeviceTemperature" in items or "BatteryList" in items:
                device = items
    finally:
        sock.close()

    print(f"{'='*56}")
    if got == 0:
        print("❌ 一个状态帧都没收到。")
        print("   心跳只发不收，可能原因：")
        print("   · 机器狗没上电 / 运控程序没跑")
        print("   · .102 到 .103 的网络不通（先 ping）")
        print("   · 端口/加密配置不对（文档 §1.1.2：默认启用 DTLS 加密）")
        print("   · 有别的客户端占着（0xE006：2s 内要求同一客户端）")
        return 1

    print(f"✅ 收到 {got} 个包，说明双向通路正常\n")
    print("收到的消息类型（Type/Command → 帧数）:")
    for k, v in sorted(kinds.items(), key=lambda kv: -kv[1])[:8]:
        print(f"   {k}  ×{v}")

    if status:
        ms = status.get("MotionState")
        gait = status.get("Gait")
        mode = status.get("ControlUsageMode")
        print("\nBasicStatus:")
        print(f"   MotionState      = {ms}  ({MOTION_NAMES.get(ms, '未知')})")
        print(f"   Gait             = {gait}  ({GAIT_NAMES.get(gait, '未知')})")
        print(f"   ControlUsageMode = {mode}  ({MODE_NAMES.get(mode, '未知')})")
        print(f"   HES(硬急停)       = {status.get('HES')}  (0=未触发 1=已触发)")
        print(f"   Charge           = {status.get('Charge')}")
        print(f"   Model/Version    = {status.get('Model')} / {status.get('Version')}")
        if ms == 17:
            print("\n   ✓ 已在 RL 控制(17) —— 此时才接受轴指令/步态指令")
        else:
            print(f"\n   ⚠ 不在 RL 控制(17) —— 轴指令会被静默忽略，"
                  f"需先下发站立(§1.2.3)")
    else:
        print("\n（没解析到 BasicStatus）")

    if device:
        dt = device.get("DeviceTemperature") or {}
        motor = list(dt.get("Motor") or [])
        driver = list(dt.get("Driver") or [])
        bats = device.get("BatteryList") or []
        print("\n设备状态:")
        if motor:
            print(f"   电机温度   max={max(motor):.1f}  "
                  f"({', '.join(f'{t:.0f}' for t in motor)})")
        if driver:
            print(f"   驱动器温度 max={max(driver):.1f}  "
                  f"({', '.join(f'{t:.0f}' for t in driver)})")
        for i, b in enumerate(bats):
            if isinstance(b, dict):
                print(f"   电池#{i}  电压={b.get('Voltage')}  "
                      f"电量={b.get('BatteryLevel')}  "
                      f"温度={b.get('battery_temperature')}  "
                      f"充电中={b.get('charge')}")
    else:
        print("\n（未收到设备状态上报，无法读电池/温度）")

    if faults:
        worst = max((max(f.get("Severities") or [0]) for f in faults), default=0)
        print(f"\n活跃故障 {len(faults)} 条，最高等级 "
              f"{worst} ({SEV_NAMES.get(worst, '?')})")
        for f in faults[:6]:
            sev = max(f.get("Severities") or [0])
            print(f"   [{SEV_NAMES.get(sev, sev)}] {f.get('Name')} "
                  f"code=0x{(f.get('Code') or 0):04x} "
                  f"res={f.get('Resources')}")
        if worst >= 5:
            print("\n   🛑 有 FATAL 故障 —— 跟随程序不应启动")
    else:
        print("\n无活跃故障上报")
    return 0


if __name__ == "__main__":
    sys.exit(main())
