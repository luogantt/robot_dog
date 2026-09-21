#!/usr/bin/env python3
"""跟随控制律的离线单元测试 —— 不需要相机、ROS、机器狗。

    python test_follow_law.py

覆盖需求里的每一条：
  1) 看不见 → 不动
  2) 距离 <= 1m → 不动（不倒退）
  3) 距离 >  1m → 追击，朝 tag 走
  4) 到达"距 tag 1m"的位置 → 停
外加：转向方向、跳变保护、变化率限幅。
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


def run(ctrl, pos, t):
    return ctrl.update(pos, t)


def main():
    # ---------- 1) 看不见 → 不动 ----------
    print("\n[1] 看不见 tag")
    c = FollowController(FollowConfig())
    for i in range(5):
        vx, wz, why = run(c, None, i * DT)
    check("无检测 → 零输出", vx == 0.0 and wz == 0.0, f"vx={vx} wz={wz}")

    # ---------- 2) 距离 <= 1m → 不动 ----------
    print("\n[2] 已在 1m 内 → 不动")
    for d in (0.99, 0.80, 0.30):
        c = FollowController(FollowConfig())
        pos = (d, 0.0, -0.2)
        for i in range(6):
            vx, wz, why = run(c, pos, i * DT)
        check(f"距 {d:.2f}m → 零输出", vx == 0.0 and wz == 0.0,
              f"vx={vx:+.3f} wz={wz:+.3f} ({why})")

    # ---------- 3) 正前方 3m → 前进，不转 ----------
    print("\n[3] 正前方 3m → 前进且不转")
    c = FollowController(FollowConfig())
    pos = (3.0, 0.0, -0.2)
    for i in range(20):
        vx, wz, why = run(c, pos, i * DT)
    check("vx > 0（前进）", vx > 0.05, f"vx={vx:+.3f}")
    check("wz ≈ 0（不转）", abs(wz) < 1e-6, f"wz={wz:+.4f}")
    check("vx 未超上限", vx <= FollowConfig().max_vx + 1e-9,
          f"vx={vx:.3f} ≤ {FollowConfig().max_vx}")

    # ---------- 4) 转向方向 ----------
    print("\n[4] 转向方向（Yaw 正 = 逆时针 = 左转）")
    c = FollowController(FollowConfig())
    pos = (1.5, 1.3, -0.2)                     # tag 在左前方
    for i in range(20):
        vx, wz, why = run(c, pos, i * DT)
    check("tag 在左 → wz > 0（左转）", wz > 0.05, f"wz={wz:+.3f}")

    c = FollowController(FollowConfig())
    pos = (1.5, -1.3, -0.2)                    # tag 在右前方
    for i in range(20):
        vx, wz, why = run(c, pos, i * DT)
    check("tag 在右 → wz < 0（右转）", wz < -0.05, f"wz={wz:+.3f}")

    # ---------- 5) 目标点是"距 tag 1m"而不是"贴着 tag" ----------
    print("\n[5] 停的位置是距 tag 1m 处，不是贴着 tag")
    c = FollowController(FollowConfig())
    t = 0.0
    pos = (3.0, 0.0, -0.2)
    for _ in range(300):
        vx, wz, why = run(c, pos, t)
        t += DT
        # 运动学模型：vx=1 → 满量程 1.714 m/s（实测标度），必须乘 DT。
        # ⚠️ 原来漏了 DT（每步走 vx 米）—— max_vx 小的时候侥幸能过，
        #    提到 0.9 就每步走 0.9m、直接冲过头，暴露了模型是错的。
        pos = (max(pos[0] - vx * 1.714 * DT, 0.05), pos[1], pos[2])
    # 容差要覆盖【测试模型的步长】：这里是"先算 vx、再按 vx 走一整步"，
    # 而 vx 最大 0.9 → 一步最多 0.1m，所以落点必然在 target 附近 ±0.1m 内。
    # 真实系统里"到位→立即发零"下个周期（50ms）就生效，过冲只有约 5cm。
    check("收敛到 target 附近停下（±模型步长）", 0.85 <= pos[0] <= 1.35,
          f"停在 {pos[0]:.2f}m（target 1.00m）")
    check("停下时输出为零", abs(vx) < 1e-9 and abs(wz) < 1e-9, f"vx={vx} wz={wz}")

    # ---------- 6) 距离跳变保护 ----------
    print("\n[6] 距离跳变保护")
    c = FollowController(FollowConfig())
    run(c, (3.0, 0.0, -0.2), 0.0)
    vx, wz, why = run(c, (0.5, 0.0, -0.2), DT)
    check("突变帧被丢弃", vx == 0.0 and wz == 0.0 and "跳变" in why, why)
    # 连续超限 3 次后应认账（否则会永久拒绝）
    c2 = FollowController(FollowConfig())
    run(c2, (3.0, 0.0, -0.2), 0.0)
    outs = [run(c2, (0.5, 0.0, -0.2), (i + 1) * DT) for i in range(6)]
    accepted = any(o[2] != "" and "跳变" not in o[2] for o in outs)
    check("连续 3 帧后接受新位置", accepted,
          f"第3帧起 reason={outs[2][2][:28]}")

    # ---------- 7) 变化率限幅 ----------
    print("\n[7] 变化率限幅（防止指令阶跃）")
    # 从"静止"起步：看不见 → vx=0，然后突然出现一个远距离 tag。
    # 期望不是一步跳到满值，而是按 max_accel_hz 一帧一帧爬上去。
    # ★ 限幅速率必须【小于】满值，否则一帧就到位，这个测试会变成假通过。
    RATE = 0.5
    cfg = FollowConfig(max_accel_hz=RATE)
    c = FollowController(cfg)
    run(c, None, 0.0)
    ramp = [run(c, (5.0, 0.0, -0.2), (i + 1) * DT)[0] for i in range(4)]
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

    # ---------- 8) 转向模式：默认比例式，开关式仍可切 ----------
    print("\n[8] 转向模式")
    cfg = FollowConfig()
    check("默认【比例式】（跟随用）", cfg.yaw_bangbang is False)
    check("转向上限 1.0", cfg.max_wz == 1.00, f"max_wz={cfg.max_wz}")

    # 死区内完全不转（两种模式都该如此）
    c = FollowController(FollowConfig())
    pos = (5.0, 0.30, -0.2)         # 3.4° 偏角，小于 5° 死区
    for i in range(5):
        vx, wz, why = run(c, pos, i * DT)
    check("死区内不转向", wz == 0.0,
          f"偏角 {math.degrees(math.atan2(0.30,5.0)):.1f}° → wz={wz}")

    # 比例式：偏角越大转得越快，且单调
    outs = []
    for deg in (15.0, 25.0, 35.0):
        c = FollowController(FollowConfig())
        r = math.radians(deg)
        pos = (5.0, 5.0 * math.tan(r), -0.2)
        for i in range(6):
            vx, wz, why = run(c, pos, i * DT)
        outs.append(wz)
    check("比例式：偏角越大 wz 越大（单调）",
          all(outs[i] < outs[i + 1] for i in range(2)),
          " → ".join(f"{v:+.3f}" for v in outs))

    # ★ 死区跃迁：算出的中间值落在 Yaw 死区（0.33~0.45）里时必须跳过，
    #   不能原样发出去 —— 原样发出去机器人会「时转时不转」，跟随一顿一顿。
    cfg = FollowConfig()
    bad = []
    for deg10 in range(50, 400):          # 5.0° ~ 40.0°，步长 0.1°
        deg = deg10 / 10.0
        c = FollowController(FollowConfig())
        r = math.radians(deg)
        pos = (5.0, 5.0 * math.tan(r), -0.2)
        for i in range(6):
            vx, wz, why = run(c, pos, i * DT)
        if 0.0 < abs(wz) < cfg.yaw_dead_hi:
            bad.append((deg, wz))
    check(f"没有任何偏角会输出 (0, {cfg.yaw_dead_hi}) 之间的值（跳过死区）",
          not bad,
          f"违规 {len(bad)} 处，例如 " + ", ".join(f"{d}°→{w:+.3f}" for d, w in bad[:3]) if bad
          else f"扫了 5~40° 共 350 个角度，全部落在 0 或 ≥{cfg.yaw_dead_hi}")

    # 开关式仍可切（实验用）
    c = FollowController(FollowConfig(yaw_bangbang=True))
    r = math.radians(15.0)
    pos = (5.0, 5.0 * math.tan(r), -0.2)
    run(c, pos, 0.0)                             # 先喂一帧
    vx, wz, why = run(c, pos, DT)                # 第二帧
    check("开关式：死区外单帧到满量程", abs(wz - 1.0) < 1e-9, f"wz={wz:+.3f}")

    # ---------- 9) 距离误差积分（追【移动】目标的关键）----------
    print("\n[9] 距离误差积分（PI 里的 I）")
    # 目标以 1.5 m/s 匀速远离，狗按指令速度走。
    # 纯 P 的稳态距离 = target + v/满速 ≈ 1.9m —— 超出相机 1.5m 上限，
    # 于是越落越远 → 丢 tag → 停 → 你跑回来 → 再追（死循环）。
    # 有 I 项时积分器会爬到"维持 1.5 m/s 所需的速度"上，误差归零。
    FULL = 1.714          # 满量程 m/s（实测标度）
    TAG_V = 1.5           # 目标远离速度

    def chase(ki, steps=500):
        c = FollowController(FollowConfig(ki_dist=ki))
        dog, tag, t = 0.0, 1.0, 0.0
        for _ in range(steps):
            vx, wz, why = run(c, (tag - dog, 0.0, -0.2), t)
            dog += vx * FULL * DT          # 狗按指令速度走（满量程 × 比例量）
            tag += TAG_V * DT              # 目标匀速远离
            t += DT
        return dog, tag

    d_i, t_i = chase(FollowConfig().ki_dist)
    d_p, t_p = chase(0.0)
    gap_i, gap_p = t_i - d_i, t_p - d_p
    check("有 I：稳态距离回到 target 附近（<1.5m 相机上限）",
          gap_i < 1.5, f"{gap_i:.2f}m")
    check("无 I：确实落后更多（对照）",
          gap_p > gap_i + 0.2, f"{gap_p:.2f}m（有 I 时 {gap_i:.2f}m）")
    check("I 增益与限幅都够用（限幅 ≥ 满量程，否则顶死追不上）",
          0.0 < FollowConfig().ki_dist and FollowConfig().int_limit >= 1.0,
          f"ki={FollowConfig().ki_dist} limit={FollowConfig().int_limit}")

    # ---------- 汇总 ----------
    print(f"\n{'='*56}")
    if FAILED:
        print(f"❌ {len(FAILED)} 项失败：{FAILED}")
        return 1
    print("✅ 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
