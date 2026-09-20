#!/usr/bin/env python3
"""ASDU 轴指令最小风险实测 —— 回答三个从未验证过的问题。

    1) 轴指令到底能不能让狗动？          （文档没说，我们从没实测过）
    2) 方向对不对？                      （Yaw 正 = 左转？）
    3) 停发指令后狗会不会继续跑？        （文档没写看门狗）

用法
--------------------------------------------------------------------------
    # 先看会发什么，不发（默认）
    axis_test.py --axis X --value 0.02 --hold 0.5

    # 真发
    axis_test.py --axis X --value 0.02 --hold 0.5 --go

    # 看门狗测试：发完直接杀进程，不归零，看狗停不停
    axis_test.py --axis X --value 0.02 --hold 0.5 --kill-test --go

安全
--------------------------------------------------------------------------
  · 默认不发（必须显式 --go）
  · 发前检查 MotionState=17、无 FATAL，不满足直接拒绝
  · 值上限按轴分档硬钳（见 HARD_LIMIT）—— 平移轴收紧（会撞东西），
    旋转轴放宽（原地转不会撞）
  · 正常路径下 finally 一定归零；--kill-test 是唯一例外（故意的）
  · 全程读回 LinearX/LinearY/AngularZ，区分"指令下去了狗动了"和
    "指令下去了狗没动"（静默失败 —— 文档说轴指令不返回任何响应帧）
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asdu_probe import MOTION_NAMES, build_heartbeat, parse  # noqa: E402

import socket                            # noqa: E402
import struct                            # noqa: E402

CMD_AXIS = (0x00100001, 0x00100002)
MOTION_RL = 17

AXES = ("X", "Y", "Z", "Roll", "Pitch", "Yaw")

# 按轴分档的硬上限，不是一刀切 —— 理由是风险性质不同：
#   X/Y 平移：标度已实测（0.06 → 0.127 m/s，满量程 ≈2.1 m/s）。
#             0.15 ≈ 0.32 m/s，2 米场地够用，再大就要撞东西了。
#   Yaw 旋转：原地转身不会撞到任何东西，风险等级完全不同，可以放更开。
#   Z/Roll/Pitch：范围和单位文档都没给（§1.2.6 那列是"-"），保持最小。
HARD_LIMIT = {"X": 0.15, "Y": 0.15, "Z": 0.08,
              "Roll": 0.08, "Pitch": 0.08, "Yaw": 0.30}


def build_axis(values, value=None):
    """§1.2.5 运动控制（轴指令）。未指定的轴一律填 0。

    两种调用方式都支持，因为两个脚本的用法不同：
        build_axis("Yaw", 0.2)                  ← turn_90_test.py
        build_axis({"X": 0.06, "Yaw": 0.15})    ← 本文件的 CLI（双轴）
    """
    import json
    from datetime import datetime
    if isinstance(values, str):
        values = {values: 0.0 if value is None else float(value)}
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
    """心跳 + 状态解析 + 轴指令。读回三个速度反馈用于判断有没有真的动。"""

    def __init__(self, host, port):
        self.target = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)
        self.motion_state = None
        self.faults = []
        self.fx = {"LinearX": 0.0, "LinearY": 0.0, "AngularZ": 0.0}
        self.responses = []          # (Type, Command, ErrorCode, ErrorMessage)
        self.kinds = {}              # 收到的帧类型计数

    def heartbeat(self):
        self.sock.sendto(build_heartbeat(), self.target)

    def send(self, values):
        self.sock.sendto(build_axis(values), self.target)

    def pump(self):
        n = 0
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
            key = f"0x{_typ:08x}/0x{_cmd:08x}"
            self.kinds[key] = self.kinds.get(key, 0) + 1
            # 通用响应（§1.5）带 ErrorCode/ErrorMessage —— 被拒绝时全靠它
            if "ErrorCode" in items:
                self.responses.append(
                    (_typ, _cmd, items.get("ErrorCode"),
                     items.get("ErrorMessage")))
            bs = items.get("BasicStatus")
            if isinstance(bs, dict):
                self.motion_state = bs.get("MotionState")
                n += 1
            ms = items.get("MotionStatus")
            if isinstance(ms, dict):
                for k in self.fx:
                    v = ms.get(k)
                    if isinstance(v, (int, float)):
                        self.fx[k] = float(v)
                n += 1
            el = items.get("ErrorList")
            if isinstance(el, list):
                self.faults = el
        return n

    def worst_severity(self):
        return max((max(f.get("Severities") or [0]) for f in self.faults),
                   default=0)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description="ASDU 轴指令最小风险实测")
    ap.add_argument("--host", default="10.21.33.103")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--axis", choices=AXES, default="X")
    ap.add_argument("--value", type=float, default=0.02)
    ap.add_argument("--axis2", choices=AXES, default=None,
                    help="第二个轴（可同时发，用来测『边走边转』）")
    ap.add_argument("--value2", type=float, default=0.0)
    ap.add_argument("--hold", type=float, default=0.5, help="持续秒数")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--tail", type=float, default=1.5, help="归零后观察秒数")
    ap.add_argument("--kill-test", action="store_true",
                    help="发完直接杀进程、不归零（测看门狗）")
    ap.add_argument("--go", action="store_true", help="真的发指令")
    args = ap.parse_args()

    want = {args.axis: args.value}
    if args.axis2:
        want[args.axis2] = args.value2
    if all(abs(x) < 1e-9 for x in want.values()):
        sys.exit("值不能全为 0 —— 那测不出任何东西")

    send = {}
    for a, x in want.items():
        lim = HARD_LIMIT[a]
        clamped = max(-lim, min(lim, x))
        if abs(clamped - x) > 1e-12:
            print(f"[钳制] {a}: {x:+g} → {clamped:+g}（该轴硬上限 ±{lim}）")
        send[a] = clamped

    desc = "  ".join(f"{a}={x:+.3f}" for a, x in send.items())
    print(f"目标 {args.host}:{args.port}")
    print(f"将发：{desc}，持发 {args.hold}s @ {args.hz:.0f}Hz，"
          f"然后归零观察 {args.tail}s")
    if args.kill_test:
        print(f"★ kill-test：{args.hold}s 后【直接杀进程、不归零】"
              f"（这是唯一不归零的路径，故意的）")
    if not args.go:
        print("\n[干跑] 没有发任何东西。加 --go 才真发。")
        return 0

    link = Link(args.host, args.port)
    base = dict(link.fx)
    try:
        # --- 前置检查 ---
        print("\n检查状态 ...")
        for _ in range(30):
            link.heartbeat()
            link.pump()
            if link.motion_state is not None:
                break
            time.sleep(0.1)
        ms = link.motion_state
        print(f"  MotionState = {ms} ({MOTION_NAMES.get(ms, '未知')})")
        if ms is None:
            print("[拒绝] 收不到状态帧，通路有问题，不发指令。")
            return 2
        if ms != MOTION_RL:
            print(f"[拒绝] 不在 RL 控制({MOTION_RL})，轴指令会被静默忽略。")
            return 2
        sev = link.worst_severity()
        print(f"  最高故障等级 = {sev} (3=WARN 4=ERROR 5=FATAL)")
        if sev >= 5:
            print("[拒绝] 有 FATAL 故障。")
            return 2

        base = dict(link.fx)
        print(f"\n基线反馈  LinearX={base['LinearX']:+.4f}  "
              f"LinearY={base['LinearY']:+.4f}  AngularZ={base['AngularZ']:+.4f}")

        # --- 发指令 ---
        print(f"\n开始发 {desc} ...")
        peak = {k: 0.0 for k in base}
        period = 1.0 / args.hz
        t0 = time.monotonic()
        last_beat = 0.0
        next_t = t0
        samples = 0
        last_trace = 0.0
        while time.monotonic() - t0 < args.hold:
            now = time.monotonic()
            if now - last_beat > 0.9:
                link.heartbeat()
                last_beat = now
            link.send(send)
            link.pump()
            samples += 1
            for k in peak:
                peak[k] = max(peak[k], abs(link.fx[k] - base[k]))
            # 实时轨迹：能看出速度是"一路保持"还是"一冲就回落"
            if now - last_trace >= 0.25:
                print(f"    t={now-t0:4.2f}s  "
                      f"LinearX={link.fx['LinearX']:+.4f}  "
                      f"LinearY={link.fx['LinearY']:+.4f}  "
                      f"AngularZ={link.fx['AngularZ']:+.4f}")
                last_trace = now
            next_t += period
            if next_t > time.monotonic():
                time.sleep(next_t - time.monotonic())
        print(f"  发了 {samples} 条，用时 {time.monotonic()-t0:.2f}s")

        if args.kill_test:
            # 故意不归零、不发心跳、直接退出 —— 看狗会不会自己停
            print("\n★ 杀进程（不归零）。观察狗是否继续运动 ...")
            link.sock.close()
            os._exit(0)

        # --- 归零 ---
        print("\n归零 ...")
        t1 = time.monotonic()
        while time.monotonic() - t1 < args.tail:
            link.heartbeat()
            link.send({a: 0.0 for a in send})
            link.pump()
            time.sleep(0.05)

        # --- 收到的通用响应（关键诊断） ---
        print(f"\n{'='*58}")
        print("收到的帧类型：")
        for k, c in sorted(link.kinds.items(), key=lambda kv: -kv[1]):
            print(f"   {k}  ×{c}")
        if link.responses:
            print("\n★ 通用响应（§1.5 ErrorCode）:")
            seen = set()
            for typ, cmd, ec, em in link.responses:
                sig = (typ, cmd, ec, em)
                if sig in seen:
                    continue
                seen.add(sig)
                print(f"   Type=0x{typ:08x} Command=0x{cmd:08x} "
                      f"ErrorCode=0x{(ec or 0):04x} ({ec})  \"{em}\"")
            print("   → 非 0 的 ErrorCode 就是被拒绝的原因")
        else:
            print("\n★ 没有收到任何 ErrorCode 响应 —— "
              "符合文档说的『轴指令不返回任何响应帧』")

        print(f"\n发指令期间相对基线的最大变化：")
        for k in peak:
            flag = "← 有响应" if peak[k] > 0.005 else ""
            print(f"  {k:9s} Δmax = {peak[k]:.4f}   {flag}")
        print(f"\n结束时反馈  LinearX={link.fx['LinearX']:+.4f}  "
              f"LinearY={link.fx['LinearY']:+.4f}  "
              f"AngularZ={link.fx['AngularZ']:+.4f}")

        moved = peak["LinearX"] > 0.005 or peak["LinearY"] > 0.005 \
            or peak["AngularZ"] > 0.005
        if moved:
            print("\n✅ 轴指令确实产生了运动 —— 这条链路是通的。")
            print("   检查方向：X 正应为前进；Yaw 正应为逆时针（左转）。")
        else:
            print("\n⚠️ 指令发出去了，但反馈没有任何变化。可能是：")
            print("   · 轴指令被忽略（模式不对？但已确认 MotionState=17）")
            print("   · 值太小，0.02 不足以产生可测的运动")
            print("   · 有别的客户端占着控制权（0xE006：2s 内要求同一客户端）")
            print("   · 反馈字段本身没在更新")
            print("   → 先确认没有手柄/App 连着，再试 --value 0.05")
        return 0
    finally:
        try:
            if not args.kill_test:
                for _ in range(5):
                    link.send({a: 0.0 for a in send})
                    time.sleep(0.03)
            link.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
