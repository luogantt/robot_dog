#!/usr/bin/env python3
"""跟随控制律的闭环仿真 —— 不碰真机，先看行为收不收敛。

    python sim_follow.py                    # 默认：tag 正对，看收敛
    python sim_follow.py --tag-yaw 12       # 复现实测：tag 贴歪 12 度
    python sim_follow.py --tag-yaw 12 --show

运动学模型（够用即可，不追求逼真）：
    前进 speed = vx × MAX_SPEED
    转向 yaw_rate = wz × MAX_YAW_RATE
    狗的位置/朝向按上式积分；tag 固定在世界里不动。
"""

import argparse
import math
import sys

import numpy as np

from follow_law import FollowConfig, FollowController

DT = 1.0 / 15.0


def rot(v, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def main():
    ap = argparse.ArgumentParser(description="跟随控制律闭环仿真")
    ap.add_argument("--tag-pos", type=float, nargs=2, default=[2.5, 0.0],
                    help="tag 在世界系的位置（狗初始在原点，朝 +X）")
    ap.add_argument("--tag-yaw", type=float, default=0.0,
                    help="tag 相对正对狗的偏航角（度）。实测那张贴歪约 12°")
    ap.add_argument("--steps", type=int, default=300)
    ap.add_argument("--max-speed", type=float, default=1.0, help="vx=1 时的 m/s")
    ap.add_argument("--max-yaw-rate", type=float, default=1.0,
                    help="wz=1 时的 rad/s")
    ap.add_argument("--target", type=float, default=1.0)
    ap.add_argument("--show", action="store_true", help="逐步打印")
    args = ap.parse_args()

    tag = np.array(args.tag_pos, dtype=float)
    # tag 法向量指向狗（正对时是 -X），再叠加 --tag-yaw 的偏转
    n_world = rot(np.array([-1.0, 0.0]), args.tag_yaw)

    dog = np.array([0.0, 0.0])
    psi = 0.0                                   # 朝向，0 = +X
    ctrl = FollowController(FollowConfig(target_dist_m=args.target))

    print(f"tag 世界位置 {tag}  法向偏角 {args.tag_yaw:+.0f}°")
    print(f"目标距离 {args.target:.2f}m   "
          f"理想终点 = tag + {args.target:.1f}×法向 = "
          f"{tag + args.target * n_world}")
    print()

    # 收敛判据：不能比浮点相等（tag 歪时 ey 不是精确 0，wz 会残留 ~1e-4），
    # 改成"最近 SETTLE_WIN 步内位移和转角都几乎没变"。
    SETTLE_WIN = 30          # 2 秒
    SETTLE_EPS_M = 0.005
    SETTLE_EPS_DEG = 0.5

    hist = []
    past = []
    for k in range(args.steps):
        # 世界 → 狗体系
        rel = rot(tag - dog, -math.degrees(psi))
        n_body = rot(n_world, -math.degrees(psi))
        n_g = n_body / max(float(np.linalg.norm(n_body)), 1e-9)

        vx, wz, why = ctrl.update((rel[0], rel[1], 0.0), (n_g[0], n_g[1]),
                                  k * DT)
        dist = float(np.linalg.norm(rel))
        hist.append((k * DT, dist, vx, wz, psi))

        if args.show or k % 30 == 0:
            print(f"  t={k*DT:5.2f}s 距={dist:5.2f}m 朝向={math.degrees(psi):+6.1f}° "
                  f"vx={vx:+.3f} wz={wz:+.3f} | {why}")

        past.append((dog.copy(), psi))
        if len(past) > SETTLE_WIN:
            past.pop(0)
        if len(past) == SETTLE_WIN and k > SETTLE_WIN:
            moved = max(float(np.linalg.norm(p[0] - past[0][0])) for p in past)
            turned = max(abs(math.degrees(p[1] - past[0][1])) for p in past)
            if moved < SETTLE_EPS_M and turned < SETTLE_EPS_DEG:
                err = float(np.linalg.norm(dog - (tag + args.target * n_world)))
                if abs(dist - args.target) < 0.20:
                    print(f"\n✅ 收敛：t={k*DT:.2f}s 停在距 tag {dist:.3f}m 处")
                    print(f"   位置 {dog}  朝向 {math.degrees(psi):+.1f}°")
                    print(f"   与理想终点偏差 {err:.3f}m"
                          f"（含 {FollowConfig().pos_deadband_m:.2f}m 位置死区）")
                    return 0 if err < 0.20 else 1
                print(f"\n⚠️ 停稳了但停在 {dist:.2f}m，目标 {args.target:.2f}m "
                      f"（差 {abs(dist-args.target):.2f}m）—— 卡住了")
                return 1

        speed = vx * args.max_speed
        yaw_rate = wz * args.max_yaw_rate
        dog = dog + rot(np.array([speed * DT, 0.0]), math.degrees(psi))
        psi += yaw_rate * DT

    print(f"\n❌ {args.steps} 步内没收敛。末态：距 {hist[-1][1]:.2f}m "
          f"vx={hist[-1][2]:+.3f} wz={hist[-1][3]:+.3f}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
