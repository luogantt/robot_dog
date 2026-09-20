#!/usr/bin/env python3
"""姿态 / 步态指令 —— 起立、趴下、标零、切步态。

    motion_cmd.py --stand            # 起立（=1），等它自动进 RL 控制(17)
    motion_cmd.py --crouch           # 趴下（=4）
    motion_cmd.py --state 17         # 直接指定 MotionParam
    motion_cmd.py --gait 0x1001      # 切步态
    motion_cmd.py --status           # 只读当前状态，不发任何指令

默认干跑，加 --go 才真发。

为什么要轮询而不是等响应：§1.5 的通用响应只说明"请求被收下了"，不代表
动作完成。姿势转换要几秒，必须靠 §1.3.1.1 的 BasicStatus 轮询 MotionState。

注意 §1.2.3 的注：「空闲」与「软急停」仅支持查询，不能下发转换。
所以能发的只有 1(站立) / 2(关节阻尼) / 4(趴下) / 5(标零) / 17(RL控制) /
0x1001(阻尼趴下)。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asdu_probe import (GAIT_NAMES, MODE_NAMES, MOTION_NAMES,  # noqa: E402
                        SEV_NAMES, build_heartbeat, parse)

import socket    # noqa: E402
import struct    # noqa: E402

CMD_MOTION = (0x00100001, 0x00200002)   # §1.2.3 运动状态转换
CMD_GAIT = (0x00100001, 0x00300002)     # §1.2.4 运动步态切换
MOTION_RL = 17
MOTION_STAND = 1
MOTION_CROUCH = 4

# §1.2.3：空闲(0) 与 软急停(-2) 仅支持查询，不能下发 —— 明确禁掉，
# 免得写错参数把狗搞进未定义状态。
FORBIDDEN = {0: "空闲（§1.2.3 注：仅支持查询，不能下发）",
             -2: "软急停（§1.2.3 注：仅支持查询，不能下发）"}


def build_body(typ, cmd, items):
    import json
    from datetime import datetime
    body = {"PatrolDevice": {
        "Type": typ, "Command": cmd,
        "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "Items": items}}
    b = json.dumps(body, separators=(",", ":")).encode()
    return struct.pack("<4sHHBBB5s", b"\xeb\x91\xeb\x90", len(b), 0,
                       0x01, 0, 0x01, b"\x00" * 5) + b


class Link:
    def __init__(self, host, port):
        self.target = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)
        self.motion_state = None
        self.gait = None
        self.mode = None
        self.hes = None
        self.faults = []
        self.responses = []

    def heartbeat(self):
        self.sock.sendto(build_heartbeat(), self.target)

    def send(self, pdu):
        self.sock.sendto(pdu, self.target)

    def pump(self, max_wait=0.05):
        self.sock.settimeout(max_wait)
        deadline = time.monotonic() + max_wait
        while time.monotonic() < deadline:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (socket.timeout, OSError):
                break
            p = parse(data)
            if not p:
                continue
            _t, _c, items = p
            if "ErrorCode" in items:
                self.responses.append((items.get("ErrorCode"),
                                       items.get("ErrorMessage")))
            bs = items.get("BasicStatus")
            if isinstance(bs, dict):
                self.motion_state = bs.get("MotionState")
                self.gait = bs.get("Gait")
                self.mode = bs.get("ControlUsageMode")
                self.hes = bs.get("HES")
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


def show(link, tag=""):
    ms = link.motion_state
    print(f"{tag}MotionState={ms} ({MOTION_NAMES.get(ms, '未知')})  "
          f"Gait={link.gait} ({GAIT_NAMES.get(link.gait, '?')})  "
          f"Mode={link.mode} ({MODE_NAMES.get(link.mode, '?')})  "
          f"HES={link.hes}")


def main():
    ap = argparse.ArgumentParser(description="姿态/步态指令")
    ap.add_argument("--host", default="10.21.33.103")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--stand", action="store_true", help="起立（MotionParam=1）")
    ap.add_argument("--crouch", action="store_true", help="趴下（MotionParam=4）")
    ap.add_argument("--state", type=int, default=None, help="直接指定 MotionParam")
    ap.add_argument("--gait", type=lambda s: int(s, 0), default=None,
                    help="步态值，如 0x1001")
    ap.add_argument("--status", action="store_true", help="只读状态，不发指令")
    ap.add_argument("--timeout", type=float, default=25.0, help="等待到位秒数")
    ap.add_argument("--go", action="store_true", help="真的发指令")
    args = ap.parse_args()

    which = sum(bool(x) for x in (args.stand, args.crouch,
                                  args.state is not None, args.gait is not None,
                                  args.status))
    if which != 1:
        sys.exit("必须且只能指定一个：--stand / --crouch / --state / "
                 "--gait / --status")

    want = 1 if args.stand else 4 if args.crouch else args.state
    if want is not None and want in FORBIDDEN:
        sys.exit(f"[拒绝] MotionParam={want}：{FORBIDDEN[want]}")

    link = Link(args.host, args.port)
    try:
        print(f"目标 {args.host}:{args.port}")
        for _ in range(30):
            link.heartbeat()
            link.pump(0.05)
            if link.motion_state is not None:
                break
            time.sleep(0.1)
        if link.motion_state is None:
            print("[错误] 收不到状态帧，检查通路。")
            return 2
        show(link, "当前  ")
        sev = link.worst_severity()
        print(f"      最高故障等级={sev} ({SEV_NAMES.get(sev, '-')})")

        if args.status:
            return 0
        if want == MOTION_STAND and link.motion_state == MOTION_RL:
            print("\n已经在 RL 控制(17)，不需要起立。")
            return 0

        if args.gait is not None:
            desc = f"步态 -> {hex(args.gait)} ({GAIT_NAMES.get(args.gait, '?')})"
        else:
            desc = f"姿态 -> {want} ({MOTION_NAMES.get(want, '?')})"

        if not args.go:
            print(f"\n[干跑] 将发：{desc}。加 --go 才真发。")
            return 0

        pdu = (build_body(CMD_GAIT[0], CMD_GAIT[1], {"GaitParam": args.gait})
               if args.gait is not None
               else build_body(CMD_MOTION[0], CMD_MOTION[1],
                               {"MotionParam": want}))
        print(f"\n下发：{desc}")
        link.send(pdu)

        t0 = time.monotonic()
        last = 0.0
        while time.monotonic() - t0 < args.timeout:
            link.heartbeat()
            link.pump(0.05)
            now = time.monotonic()
            if args.gait is None:
                # 等中间态会永远等不到，必须认终态（这两个都实测踩过）：
                #   起立(1) → 终态是 RL 控制(17)（§2.2.1 注：起身后自动进入）
                #   趴下(4) → 终态可能是 空闲(0)（实测：趴完落到 0 而不是 4）
                reached = (link.motion_state == want
                           or (want == MOTION_STAND
                               and link.motion_state == MOTION_RL)
                           or (want == MOTION_CROUCH
                               and link.motion_state == 0))
                if reached:
                    print(f"✅ 到位：MotionState={want} "
                          f"({MOTION_NAMES.get(want, '?')})，用时 "
                          f"{now-t0:.1f}s")
                    # 站立会【自动】进 RL 控制（§2.2.1 注）
                    if want == 1 and link.motion_state != 17:
                        print("   等待自动进入 RL 控制(17) ...")
                        t1 = time.monotonic()
                        while time.monotonic() - t1 < 20:
                            link.heartbeat()
                            link.pump(0.05)
                            if link.motion_state == 17:
                                print(f"✅ 已进入 RL 控制(17)，"
                                      f"总用时 {time.monotonic()-t0:.1f}s")
                                break
                        else:
                            print(f"⚠️ 20s 内未进入 17（当前 "
                                  f"{link.motion_state}）")
                    return 0
            else:
                if link.gait == args.gait:
                    print(f"✅ 步态已切换，用时 {now-t0:.1f}s")
                    show(link, "      ")
                    return 0
            if now - last > 2.0:
                show(link, f"  ...{now-t0:4.1f}s ")
                last = now
        print(f"⚠️ {args.timeout:.0f}s 内未确认到位")
        show(link, "最终  ")
        if link.responses:
            print("通用响应：")
            for ec, em in link.responses[:5]:
                print(f"   ErrorCode={ec} \"{em}\"")
        return 1
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
