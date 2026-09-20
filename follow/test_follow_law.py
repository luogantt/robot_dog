#!/usr/bin/env python3
"""跟随控制律的离线单元测试 —— 不需要相机、ROS、机器狗。

    python test_follow_law.py

覆盖需求里的每一条：
  1) 看不见 → 不动
  2) 距离 <= 1m → 不动（不倒退）
  3) 距离 >  1m → 追击，方向沿 tag 法向量的地面投影
  4) 到达"距 tag 1m"的位置 → 停
外加：转向方向、跳变保护、变化率限幅、法向不可靠时的回退。
"""

import math
import sys

from follow_law import FollowConfig, FollowController

DT = 1.0 / 15.0          # 15Hz
FAILED = []


def check(name, cond, detail=""):
    print(f"  {'✅' if cond else '❌'} {name}" + (f"   {detail}" if detail else ""))
    if not cond:
        FAILED.append(name)


def run(ctrl, pos, normal, t):
    return ctrl.update(pos, normal, t)


def toward_dog(pos):
    """tag 法向量指向狗（tag 正面朝狗）时的地面投影单位向量。"""
    x, y = pos[0], pos[1]
    n = math.hypot(x, y)
    return (-x / n, -y / n)


def main():
    # ---------- 1) 看不见 → 不动 ----------
    print("\n[1] 看不见 tag")
    c = FollowController(FollowConfig())
    for i in range(5):
        vx, wz, why = run(c, None, None, i * DT)
    check("无检测 → 零输出", vx == 0.0 and wz == 0.0, f"vx={vx} wz={wz}")

    # ---------- 2) 距离 <= 1m → 不动 ----------
    print("\n[2] 已在 1m 内 → 不动")
    for d in (0.99, 0.80, 0.30):
        c = FollowController(FollowConfig())
        pos = (d, 0.0, -0.2)
        for i in range(6):
            vx, wz, why = run(c, pos, toward_dog(pos), i * DT)
        check(f"距 {d:.2f}m → 零输出", vx == 0.0 and wz == 0.0,
              f"vx={vx:+.3f} wz={wz:+.3f} ({why})")

    # ---------- 3) 正前方 3m → 前进，不转 ----------
    print("\n[3] 正前方 3m，tag 正面朝狗 → 前进且不转")
    c = FollowController(FollowConfig())
    pos = (3.0, 0.0, -0.2)
    for i in range(20):
        vx, wz, why = run(c, pos, toward_dog(pos), i * DT)
    check("vx > 0（前进）", vx > 0.05, f"vx={vx:+.3f}")
    check("wz ≈ 0（不转）", abs(wz) < 1e-6, f"wz={wz:+.4f}")
    check("vx 未超上限", vx <= FollowConfig().max_vx + 1e-9,
          f"vx={vx:.3f} ≤ {FollowConfig().max_vx}")

    # ---------- 4) 转向方向 ----------
    print("\n[4] 转向方向（Yaw 正 = 逆时针 = 左转）")
    c = FollowController(FollowConfig())
    pos = (1.5, 1.3, -0.2)                     # tag 在左前方
    for i in range(20):
        vx, wz, why = run(c, pos, toward_dog(pos), i * DT)
    check("tag 在左 → wz > 0（左转）", wz > 0.05, f"wz={wz:+.3f}")

    c = FollowController(FollowConfig())
    pos = (1.5, -1.3, -0.2)                    # tag 在右前方
    for i in range(20):
        vx, wz, why = run(c, pos, toward_dog(pos), i * DT)
    check("tag 在右 → wz < 0（右转）", wz < -0.05, f"wz={wz:+.3f}")

    # ---------- 5) 目标点是"距 tag 1m"而不是"贴着 tag" ----------
    print("\n[5] 停的位置是距 tag 1m 处，不是贴着 tag")
    c = FollowController(FollowConfig())
    t = 0.0
    pos = (3.0, 0.0, -0.2)
    for _ in range(300):
        vx, wz, why = run(c, pos, toward_dog(pos), t)
        t += DT
        pos = (max(pos[0] - vx * 1.0, 0.05), pos[1], pos[2])   # 假设 vx=1 → 1m/s
    check("收敛到 ~1m 附近停下", 0.95 <= pos[0] <= 1.35, f"停在 {pos[0]:.2f}m")
    check("停下时输出为零", abs(vx) < 1e-9 and abs(wz) < 1e-9, f"vx={vx} wz={wz}")

    # ---------- 6) 法向量不可靠 → 回退 ----------
    print("\n[6] 法向量不可靠（normal=None）→ 退回直接朝 tag 走")
    c = FollowController(FollowConfig())
    pos = (3.0, 0.0, -0.2)
    for i in range(20):
        vx, wz, why = run(c, pos, None, i * DT)
    check("仍然前进", vx > 0.05, f"vx={vx:+.3f}")
    check("走的是回退路径", "直朝tag" in why or "回退" in why, why)

    # ---------- 7) 距离跳变保护 ----------
    print("\n[7] 距离跳变保护")
    c = FollowController(FollowConfig())
    run(c, (3.0, 0.0, -0.2), toward_dog((3, 0, 0)), 0.0)
    vx, wz, why = run(c, (0.5, 0.0, -0.2), toward_dog((0.5, 0, 0)), DT)
    check("突变帧被丢弃", vx == 0.0 and wz == 0.0 and "跳变" in why, why)
    # 连续超限 3 次后应认账（否则会永久拒绝）
    c2 = FollowController(FollowConfig())
    run(c2, (3.0, 0.0, -0.2), None, 0.0)
    outs = [run(c2, (0.5, 0.0, -0.2), None, (i + 1) * DT) for i in range(6)]
    accepted = any(o[2] != "" and "跳变" not in o[2] for o in outs)
    check("连续 3 帧后接受新位置", accepted,
          f"第3帧起 reason={outs[2][2][:28]}")

    # ---------- 8) 变化率限幅 ----------
    print("\n[8] 变化率限幅（防止指令阶跃）")
    # 从"静止"起步：看不见 → vx=0，然后突然出现一个远距离 tag。
    # 期望不是一步跳到满值，而是按 max_accel_hz 一帧一帧爬上去。
    # ★ 限幅速率必须【小于】满值，否则一帧就到位，这个测试会变成假通过。
    RATE = 0.5
    cfg = FollowConfig(max_accel_hz=RATE)
    c = FollowController(cfg)
    run(c, None, None, 0.0)
    ramp = [run(c, (5.0, 0.0, -0.2), None, (i + 1) * DT)[0] for i in range(4)]
    check("限幅速率 < 满值（否则测不出东西）", RATE * DT < cfg.max_vx,
          f"每帧步长 {RATE*DT:.4f} < max_vx {cfg.max_vx}")
    check("起步不是阶跃（首帧 < 满值）", ramp[0] < cfg.max_vx - 1e-9,
          f"首帧 vx={ramp[0]:.3f} < {cfg.max_vx:.3f}")
    check("逐帧递增", all(ramp[i] < ramp[i + 1] + 1e-9 for i in range(3)),
          " → ".join(f"{v:.3f}" for v in ramp))
    check("每帧增量 ≤ max_accel_hz×dt",
          all(ramp[i + 1] - ramp[i] <= RATE * DT + 1e-9 for i in range(3)),
          f"最大增量 {max(ramp[i+1]-ramp[i] for i in range(3)):.4f} "
          f"≤ {RATE*DT:.4f}")

    # ---------- 9) 开关式转向（实测 Yaw 中间值无效，只有满量程可用）----------
    print("\n[9] 开关式转向（bang-bang）")
    cfg = FollowConfig()
    check("默认启用开关式", cfg.yaw_bangbang is True)
    check("默认满量程", cfg.max_wz == 1.00, f"max_wz={cfg.max_wz}")

    # 方位角小于死区 → 完全不转（防止在死区里来回摆）
    c = FollowController(FollowConfig())
    pos = (5.0, 0.30, -0.2)         # 3.4° 偏角，小于 8° 死区
    for i in range(5):
        vx, wz, why = run(c, pos, None, i * DT)
    check("死区内不转向", wz == 0.0, f"偏角 {math.degrees(math.atan2(0.30,5.0)):.1f}° → wz={wz}")

    # 超出死区 → 立即满量程（不受变化率限幅）
    for deg, want in ((15.0, 1.0), (-15.0, -1.0)):
        c = FollowController(FollowConfig())
        r = math.radians(deg)
        pos = (5.0, 5.0 * math.tan(r), -0.2)
        run(c, pos, None, 0.0)                       # 先喂一帧
        vx, wz, why = run(c, pos, None, DT)          # 第二帧
        check(f"偏角 {deg:+.0f}° → 单帧到满量程 {want:+.1f}",
              abs(wz - want) < 1e-9, f"wz={wz:+.3f}")

    # 比例式仍可切回（调试用）
    c = FollowController(FollowConfig(yaw_bangbang=False))
    r = math.radians(3.0)                            # 小偏角
    pos = (5.0, 5.0 * math.tan(r), -0.2)
    for i in range(3):
        vx, wz, why = run(c, pos, None, i * DT)
    check("比例式仍可用（小值转向）", 0.0 < abs(wz) < 1.0,
          f"wz={wz:+.3f}（开关式下这里会是 0）")

    # ---------- 汇总 ----------
    print(f"\n{'='*56}")
    if FAILED:
        print(f"❌ {len(FAILED)} 项失败：{FAILED}")
        return 1
    print("✅ 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
