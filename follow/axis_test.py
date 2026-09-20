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

# 按轴分档的硬上限。依据不再是猜的，而是官方 SDK 源码里的满量程
# （S10_sdk_deploy/interface/user_command/keyboard_interface.hpp:29-31）：
#     max_forward_ = 1.0   max_side_ = 0.6   max_yaw_ = 1.0
# 经 s10_policy_runner.hpp:207 缩放后变成物理量：
#     forward × 1.5 → 1.5 m/s    side × 0.5 → 0.5 m/s    yaw × 0.6 → 0.6 rad/s
# 官方键盘/手柄/DDS 三路输入都工作在满量程，输入范围是 [-1,1]。
#
# 而 robot_dog_sdk/web_control.py 里，网页端实际下发的是
#     target = intent × scale × cap        （scale 默认 0.50，滑条上限 MAX_SCALE=1.0）
# 也就是说网页端最猛能发到 ±1.0 —— 那是"已验证能用"的取值范围。
#
#   X 平移：放到 0.5。原以为 0.15 够了（按"0.06 → 0.127 m/s"的标度），
#     但实测这个通道【非单调】：0.06 → 0.127 m/s，0.12 → 0.027 m/s
#     （指令翻倍、速度掉到 1/5）。老标度作废，直接给到网页端默认量级。
#   Y 侧移：放到 0.8。网页端默认就发 0.5，之前用 0.15 测当然没反应。
#   Yaw 旋转：原地转身不会撞到任何东西，按官方满量程放开。
#   Z/Roll/Pitch：文档 §1.2.6 那列是"-"，范围未知，保持最小。
HARD_LIMIT = {"X": 0.50, "Y": 0.80, "Z": 0.08,
              "Roll": 0.08, "Pitch": 0.08, "Yaw": 1.00}


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
    ap.add_argument("--allow-clamp", action="store_true",
                    help="值超出该轴硬上限时，允许按钳制值继续（默认直接拒绝）")
    args = ap.parse_args()

    want = {args.axis: args.value}
    if args.axis2:
        want[args.axis2] = args.value2
    if all(abs(x) < 1e-9 for x in want.values()):
        sys.exit("值不能全为 0 —— 那测不出任何东西")

    send, clamped_any = {}, []
    for a, x in want.items():
        lim = HARD_LIMIT[a]
        clamped = max(-lim, min(lim, x))
        if abs(clamped - x) > 1e-12:
            clamped_any.append(f"{a}: {x:+g} → {clamped:+g}（上限 ±{lim}）")
        send[a] = clamped

    # 钳制【不再静默通过】—— 否则你以为在测某个值，实际发的是另一个，
    # 白跑一轮还拿到错误结论（这个坑已经踩过两次）。
    if clamped_any:
        for msg in clamped_any:
            print(f"[钳制] {msg}")
        if not args.allow_clamp:
            print("\n[拒绝执行] 值被硬上限改写，实际发的和你要求的不一样。")
            print("           改小参数，或确知后果时加 --allow-clamp 强制按钳制值发。")
            return 4

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
            # 只读一次时钟再判断 —— 先比后算会让差值为负，sleep 抛
            # ValueError 把整个线程打断（实机踩过，检测线程直接死掉）
            _d = next_t - time.monotonic()
            if _d > 0:
                time.sleep(_d)
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
