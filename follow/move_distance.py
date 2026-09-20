#!/usr/bin/env python3
"""闭环定距移动 —— 让狗走指定距离（米）后自己停。

    move_distance.py --distance 3.0 --go          # 前进 3 米
    move_distance.py --distance -1.5 --go         # 后退 1.5 米
    move_distance.py --distance 3.0              # 干跑，只说会发什么

**不用固定时长**：轴指令停发后机器人会不会继续跑，文档没写（我们没验过），
所以按时间估算不可靠。这里积分 `MotionStatus.LinearX` 得到**实际走过的距离**，
走够就归零。

纯标准库，不依赖 ROS。
"""

import argparse
import json
import socket
import struct
import sys
import time
from datetime import datetime

SYNC = b"\xeb\x91\xeb\x90"
T_HEARTBEAT = (0x00100064, 0x00000005)
T_AXIS = (0x00100001, 0x00100002)
AXES = ("X", "Y", "Z", "Roll", "Pitch", "Yaw")


def pdu(typ, cmd, items):
    body = {"PatrolDevice": {"Type": typ, "Command": cmd,
                             "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                             "Items": items}}
    b = json.dumps(body, separators=(",", ":")).encode()
    return struct.pack("<4sHHBBB5s", SYNC, len(b), 0, 0x01, 0, 0x01,
                       b"\x00" * 5) + b


def main():
    ap = argparse.ArgumentParser(description="闭环定距移动")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--distance", type=float, default=3.0,
                    help="目标距离（米）。正=前进，负=后退")
    ap.add_argument("--speed", type=float, default=0.5,
                    help="速度比例量（默认 0.5，实测 ≈0.83 m/s）")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--timeout", type=float, default=40.0, help="总超时（秒）")
    ap.add_argument("--go", action="store_true")
    a = ap.parse_args()

    if a.distance == 0:
        sys.exit("距离不能为 0")
    sign = 1.0 if a.distance > 0 else -1.0
    target = abs(a.distance)
    v = sign * abs(a.speed)

    print(f"目标：{'前进' if sign > 0 else '后退'} {target:.2f} 米"
          f"   速度比例 {v:+.2f}   目标机 {a.host}:{a.port}")
    if not a.go:
        print("\n[干跑] 没有发任何指令。加 --go 才真发。")
        return 0

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 0))
    s.settimeout(0.05)
    T = (a.host, a.port)

    # ---- 前置检查 ----
    ms = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5:
        s.sendto(pdu(*T_HEARTBEAT, {}), T)
        try:
            d, _ = s.recvfrom(65535)
            it = json.loads(d[16:].decode())["PatrolDevice"]["Items"]
            bs = it.get("BasicStatus")
            if isinstance(bs, dict):
                ms = bs.get("MotionState")
                break
        except Exception:
            pass
        time.sleep(0.1)
    if ms is None:
        print("✗ 收不到状态帧")
        return 1
    print(f"MotionState = {ms}")
    if ms != 17:
        print(f"✗ 不是 RL 控制(17) —— 轴指令会被静默忽略，先让狗起立")
        return 2

    # ---- 走 ----
    print(f"\n开始走 ...")
    vals = {k: 0.0 for k in AXES}
    vals["X"] = v
    zeros = {k: 0.0 for k in AXES}

    dist = 0.0                 # 带符号的实际位移
    lx = 0.0                   # 最近一帧的 LinearX
    peak = 0.0
    last_beat = 0.0
    last_print = 0.0
    t_prev = time.monotonic()
    started = False
    t0 = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            elapsed = now - t0

            if elapsed > a.timeout:
                print(f"\n⚠️ 超时 {a.timeout:.0f}s，已走 {dist:+.2f}m —— 停止")
                break
            if abs(dist) >= target:
                break

            if now - last_beat > 0.9:
                s.sendto(pdu(*T_HEARTBEAT, {}), T)
                last_beat = now
            s.sendto(pdu(*T_AXIS, vals), T)

            # 收反馈并积分
            try:
                while True:
                    d, _ = s.recvfrom(65535)
                    it = json.loads(d[16:].decode())["PatrolDevice"]["Items"]
                    mst = it.get("MotionStatus")
                    if isinstance(mst, dict):
                        lx = float(mst.get("LinearX") or 0.0)
                        peak = max(peak, abs(lx))
                        if abs(lx) > 0.02:
                            started = True
                        # 只在已经动起来之后积分，避免静止噪声累积
                        if started:
                            dist += lx * dt
            except Exception:
                pass

            if now - last_print > 0.4:
                print(f"  {elapsed:5.1f}s  已走 {dist:+.2f}m / {target:.2f}m"
                      f"   LinearX={lx:+.3f}")
                last_print = now

            time.sleep(1.0 / a.hz)
    finally:
        print("\n归零 ...")
        for _ in range(12):
            try:
                s.sendto(pdu(*T_AXIS, zeros), T)
            except Exception:
                pass
            time.sleep(0.05)
        s.close()

    print(f"结束：积分位移 {dist:+.2f} m   峰值速度 {peak:.2f} m/s")
    if abs(dist) < target * 0.5:
        print("⚠️ 走得比目标少很多 —— 可能：热降额 / 卡住 / 反馈不准")
    return 0


if __name__ == "__main__":
    sys.exit(main())
