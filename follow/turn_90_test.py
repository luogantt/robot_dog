#!/usr/bin/env python3
"""闭环定角转向测试 —— 转 90° 后自己停。

为什么不直接"Yaw=0.20 持续 N 秒"
--------------------------------------------------------------------------
因为实测发现 AngularZ 正负来回摆（+0.70 → -0.39 → +0.16 …），
开环按时间算角度完全不可靠。所以改成【闭环】：实时积分 MotionStatus 的
AngularZ 得到已转角度，转到目标就停。

    默认左转 90°     python3 turn_90_test.py --go
    右转 90°         python3 turn_90_test.py --degrees -90 --go
    干跑（不发）     python3 turn_90_test.py

控制策略
--------------------------------------------------------------------------
    · 巡航段：Yaw = --yaw-cruise（默认 0.20）
    · 剩不到 --fine-band 度时降到 --yaw-fine（默认 0.12）防过冲
    · 到目标角度立即归零
    · 3 秒还没转够 --min-progress 度 → 判定"不转"，自动停
    · 意外平移速度过大 → 自动停
    · 总超时 8 秒
    · finally 连续发零速

若输出类似：
    3.0s 净转角仅 +3.2°
    纯 Yaw 未形成持续转身
则说明当前步态下 Yaw 只有瞬时角速度响应、无法形成稳定原地偏航 ——
下一步应查步态/控制模式，或改测 X+Yaw 联合运动，而不是继续加大 Yaw。
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asdu_probe import MOTION_NAMES, build_heartbeat, parse  # noqa: E402

import socket          # noqa: E402
import struct          # noqa: E402

CMD_AXIS = (0x00100001, 0x00100002)
MOTION_RL = 17
AXES = ("X", "Y", "Z", "Roll", "Pitch", "Yaw")
YAW_LIMIT = 0.30       # 该轴硬上限（原地转身不会撞东西，可放宽）
DEG = 180.0 / math.pi


def build_axis(values):
    """§1.2.5 运动控制（轴指令）。"""
    import json
    from datetime import datetime
    items = {a: 0.0 for a in AXES}
    for k, v in values.items():
        items[k] = float(v)
    body = {"PatrolDevice": {
        "Type": CMD_AXIS[0], "Command": CMD_AXIS[1],
        "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "Items": items}}
    b = json.dumps(body, separators=(",", ":")).encode()
    return struct.pack("<4sHHBBB5s", b"\xeb\x91\xeb\x90", len(b), 0,
                       0x01, 0, 0x01, b"\x00" * 5) + b


class Link:
    def __init__(self, host, port):
        self.target = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.02)
        self.motion_state = None
        self.gait = None
        self.faults = []
        self.az = 0.0            # AngularZ, rad/s
        self.lx = 0.0
        self.ly = 0.0
        self.responses = []

    def heartbeat(self):
        self.sock.sendto(build_heartbeat(), self.target)

    def send(self, values):
        self.sock.sendto(build_axis(values), self.target)

    def pump(self):
        deadline = time.monotonic() + 0.02
        while time.monotonic() < deadline:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (socket.timeout, OSError):
                break
            p = parse(data)
            if not p:
                continue
            _typ, _cmd, items = p
            if "ErrorCode" in items:
                self.responses.append((items.get("ErrorCode"),
                                       items.get("ErrorMessage")))
            bs = items.get("BasicStatus")
            if isinstance(bs, dict):
                self.motion_state = bs.get("MotionState")
                self.gait = bs.get("Gait")
            ms = items.get("MotionStatus")
            if isinstance(ms, dict):
                for key, attr in (("AngularZ", "az"), ("LinearX", "lx"),
                                  ("LinearY", "ly")):
                    v = ms.get(key)
                    if isinstance(v, (int, float)):
                        setattr(self, attr, float(v))
            el = items.get("ErrorList")
            if isinstance(el, list):
                self.faults = el

    def worst_severity(self):
        return max((max(f.get("Severities") or [0]) for f in self.faults),
                   default=0)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description="闭环定角转向测试")
    ap.add_argument("--host", default="10.21.33.103")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--degrees", type=float, default=90.0,
                    help="目标角度，正=左转(逆时针)，负=右转。默认 +90")
    ap.add_argument("--yaw-cruise", type=float, default=0.20)
    ap.add_argument("--yaw-fine", type=float, default=0.12)
    ap.add_argument("--fine-band", type=float, default=20.0,
                    help="剩这么多度时切到微调速度。默认 20（即 70° 处）")
    ap.add_argument("--min-progress", type=float, default=10.0,
                    help="stall-time 内至少要转这么多度，否则判不转")
    ap.add_argument("--stall-time", type=float, default=3.0)
    ap.add_argument("--max-lin", type=float, default=0.15,
                    help="意外平移速度上限 m/s，超了自动停")
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--go", action="store_true")
    args = ap.parse_args()

    if abs(args.degrees) < 1e-6:
        sys.exit("目标角度不能为 0")
    sign = 1.0 if args.degrees > 0 else -1.0
    target = abs(args.degrees)
    cruise = min(abs(args.yaw_cruise), YAW_LIMIT)
    fine = min(abs(args.yaw_fine), YAW_LIMIT)
    if cruise != args.yaw_cruise or fine != args.yaw_fine:
        print(f"[钳制] 转速上限 {YAW_LIMIT}")

    print(f"目标 {args.host}:{args.port}")
    print(f"目标角度 {'左转' if sign > 0 else '右转'} {target:.0f}°")
    print(f"策略：巡航 Yaw={sign*cruise:+.3f} → 剩 {args.fine_band:.0f}° 内降到 "
          f"{sign*fine:+.3f} → 到位归零")
    print(f"保护：{args.stall_time:.0f}s 内转不到 {args.min_progress:.0f}° 判不转；"
          f"平移 >{args.max_lin} m/s 停；总超时 {args.timeout:.0f}s")
    if not args.go:
        print("\n[干跑] 没有发任何东西。加 --go 才真发。")
        return 0

    link = Link(args.host, args.port)
    try:
        print("\n检查状态 ...")
        for _ in range(30):
            link.heartbeat()
            link.pump()
            if link.motion_state is not None:
                break
            time.sleep(0.1)
        ms = link.motion_state
        print(f"  MotionState = {ms} ({MOTION_NAMES.get(ms, '未知')})  "
              f"Gait = {link.gait}")
        if ms is None:
            print("[拒绝] 收不到状态帧。")
            return 2
        if ms != MOTION_RL:
            print(f"[拒绝] 不在 RL 控制({MOTION_RL})，Yaw 会被静默忽略。")
            print("       先让狗起立（§1.2.3 MotionParam=1）。")
            return 2
        sev = link.worst_severity()
        if sev >= 5:
            print(f"[拒绝] 有 FATAL 故障（等级 {sev}）。")
            return 2

        print(f"\n开始转向 ...")
        turned = 0.0                 # 已转角度（度，带符号）
        period = 1.0 / args.hz
        t0 = time.monotonic()
        t_prev = t0
        last_beat = last_print = 0.0
        next_t = t0
        peak_lin = 0.0
        verdict = "超时未达目标"

        while True:
            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            elapsed = now - t0

            if now - last_beat > 0.9:
                link.heartbeat()
                last_beat = now
            link.pump()

            # 积分角速度得到已转角度。dt 用实测间隔，不用名义周期。
            turned += link.az * dt * DEG
            peak_lin = max(peak_lin, math.hypot(link.lx, link.ly))

            # --- 到位 ---
            if abs(turned) >= target:
                verdict = "到达目标角度"
                break
            # --- 判不转 ---
            if elapsed >= args.stall_time and abs(turned) < args.min_progress:
                verdict = "STALL"
                break
            # --- 意外平移 ---
            if peak_lin > args.max_lin:
                verdict = f"平移过大 {peak_lin:.3f} m/s"
                break
            # --- 超时 ---
            if elapsed >= args.timeout:
                verdict = "超时未达目标"
                break

            remain = target - abs(turned)
            mag = fine if remain <= args.fine_band else cruise
            link.send({"Yaw": sign * mag})

            if now - last_print >= 0.25:
                print(f"    t={elapsed:4.2f}s  已转={turned:+6.1f}°  "
                      f"剩余={sign*remain:+5.1f}°  Yaw={sign*mag:+.3f}  "
                      f"AngularZ={link.az:+.4f}  "
                      f"平移={math.hypot(link.lx, link.ly):.3f}")
                last_print = now

            next_t += period
            if next_t > time.monotonic():
                time.sleep(next_t - time.monotonic())

        print(f"\n{'='*58}")
        print(f"结束原因：{verdict}")
        print(f"净转角 {turned:+.1f}°（目标 {sign*target:+.0f}°）")
        print(f"峰值平移速度 {peak_lin:.3f} m/s")
        if link.responses:
            print("通用响应：")
            for ec, em in link.responses[:5]:
                print(f"   ErrorCode={ec} \"{em}\"")
        else:
            print("（无 ErrorCode 响应，符合 §1.2.5『轴指令不返回响应帧』）")

        if verdict == "STALL":
            print(f"\n{args.stall_time:.1f}s 净转角仅 {turned:+.1f}°")
            print("纯 Yaw 未形成持续转身")
            print("\n→ 当前步态下 Yaw 只有瞬时角速度响应，不能形成稳定原地偏航。")
            print("  下一步不要继续加大 Yaw，而应：")
            print("   · 查步态/控制模式（当前 Gait=0x1001 基础步态）")
            print("   · 或测 X+Yaw 联合运动（行走中转向）")
            print("     axis_test.py --axis X --value 0.06 --axis2 Yaw "
                  "--value2 0.15 --hold 2.5 --go")
            return 3
        if verdict == "到达目标角度":
            print("\n✅ 转向闭环成功 —— Yaw 通道可用，且闭环积分是准的。")
            return 0
        return 1

    except KeyboardInterrupt:
        print("\n[中断]")
        return 130
    finally:
        print("归零 ...")
        try:
            for _ in range(10):
                link.send({"Yaw": 0.0})
                time.sleep(0.03)
        except Exception:
            pass
        link.close()


if __name__ == "__main__":
    sys.exit(main())
