#!/usr/bin/env python3
"""跟随控制律 —— 纯逻辑，只依赖 numpy/math。

单独成文件是有意的：这里没有 ROS、没有 cv2、没有 socket，所以可以在任何
机器上单元测试（见 test_follow_law.py）。真机那套 I/O 在 follow_controller.py。

规格
--------------------------------------------------------------------------
  1) 看不见 tag                    → 输出零，不动
  2) 狗距 tag <= target_dist_m      → 输出零，不动（不倒退）
  3) 狗距 tag >  target_dist_m      → 追击
       目标点 T = tag位置 + target × n，n = 沿 tag→狗 方向的单位向量
       即"直接朝 tag 走到 target 距离"
       前进量 ∝ T 的前向分量，转向 ∝ T 的方位角

坐标系：机体系，X 前 / Y 左 / Z 上，原点 = 机体几何中心。
输出：vx, wz 为比例量 [-1,1]（对应 ASDU §1.2.5 轴指令的 X 和 Yaw）。
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class FollowConfig:
    tag_id: int = 1
    target_dist_m: float = 1.0      # 目标地面距离
    # 速度上限。标度用 2026-09-20 的直连实测（见 TECHNICAL_REPORT §2.7）：
    #   指令 X=0.5 持发 1.5s → LinearX 稳态 0.857 m/s  →  满量程 ≈ 1.71 m/s
    #   实测序列 0.170(0.25s) → 0.395(0.50s) → 0.723(0.80s) → 0.857(1.10s)
    #   —— 单调爬升、稳态明确，且 1.71 与 §1.2.6 标的 ±1.67 m/s 吻合。
    #   ⇒ 这条通道是【线性】的，可以用比例控制。
    # 所以 max_vx = 0.50 ≈ 0.86 m/s，约为人正常步速的 2/3。
    #
    # 【旧值 0.12 是错的，别再改回去】它按"指令 0.06 → 0.127 m/s"推的标度，
    # 而那个读数是在【源端点被污染】的环境下拿到的（本机两条等价路由 →
    # 不同 socket 拿到不同源 IP → 多个客户端互相抢控制权），不是通道特性。
    # 按旧值只有 0.2 m/s —— 之前记的"前进响应弱、追不上人"就是这个造成的，
    # 不是机器人不给力。详见 TECHNICAL_REPORT §2.7。
    max_vx: float = 0.90            # 前进比例量上限 ≈ 1.54 m/s（实测标度 1.71）
    # 转向：比例式。旧注释说"Yaw 非单调、只有满量程能起转"—— 那批数据出自
    # 【源端点被污染】的测量期，2026-09-21 复测（三档扫描读回 AngularZ）：
    #     0.20 → 完全不动            0.25 → 连测 5 次有 2 次不转（40% 失败）
    #     0.35 / 0.40 → 死区 ≈0      0.45 以上 → 稳定
    #     0.50 → 43°/s 稳   1.00 → 102°/s 稳   比值 2.37 ⇒ 基本线性
    # ⚠️ 所以 Yaw 【确实有死区】(0.35~0.40)，和 X 不一样 —— X 是全程线性。
    # 但只要不落在死区里，比例式可行 → 跟随改用比例式。开关式会
    # 「转过头 → 转回来 → 再过头」，追移动目标时走 Z 字。
    max_wz: float = 1.00            # 转向上限；0.5≈43°/s，1.0≈102°/s
    yaw_bangbang: bool = False      # False=比例式（跟随用）；True=开关式（实验用）
    yaw_deadband_deg: float = 5.0   # 方位误差小于这个角度就不转向，防抖
    # 【已移除】沿 tag 法向量偏移目标点的方案（2026-09-20 删除）。
    # 需求原话是 T = tag位置 + target × n（n = tag 法向量），实测两个问题：
    #  1) 法向量的 EMA 滤波会形成正反馈：狗一转身 → 滤波滞后 → 目标点
    #     永远偏在死区外 → 无限原地打转。15Hz 下滞后 12.7°，调参修不掉。
    #  2) 转向通道本身不可靠（只有满量程能起转，起转时刻不可预测）。
    # 于是改为恒定的"直接朝 tag 走到 target 距离" —— 同样满足"距 tag 1 米"
    # 这个目标，且几乎不需要转向。代价：不会自动站到 tag 正前方。
    # 若将来要重新引入，先看 TECHNICAL_REPORT §2.4 的符号坑。
    kp_dist: float = 1.0            # 前向误差(m) → 速度比例
    # ---- 距离误差【积分】项（PI 里的 I）----
    # 纯 P 追【匀速远离】的目标会有稳态误差：目标以 v 跑，P 的平衡点是
    #     dist = target + v / (kp_dist × 满速)
    # 人跑 1.5 m/s → 1.0 + 0.875 = 1.875m，而相机最远 1.5m ——
    # 于是越落越远 → 丢 tag → 停 → 你跑回来 → 再追……死循环。
    #
    # ⚠️ 这里【不能用 D 项】：D 响应的是距离的【变化率】，而稳态时距离不变、
    #    变化率为 0 —— D 项在稳态下恒为 0，消除不了斜坡输入的稳态误差。
    #    （第一版就是这么写错的，单元测试里"有前馈"和"无前馈"结果一模一样。）
    #    必须用 I：积分器会自己爬到"维持跟随所需的速度"上，误差才能归零。
    ki_dist: float = 1.0            # 距离误差积分增益
    # ⚠️ 限幅必须 ≥ 满量程（1.0）：跟随 1.5 m/s 时积分要输出 0.875 才够，
    #    限到 0.8 就顶死了 —— 狗只能跑到 1.37 m/s，永远差一口气。
    int_limit: float = 1.0          # 积分限幅（防积分饱和）
    kp_yaw: float = 2.0             # 方位角(rad) → 角速度比例
    pos_deadband_m: float = 0.10    # 距目标点这么近就不再前进（只压 P 项，见 update）
    # 到位的【滞后带】：进到 target 以内就停，但要退到 target+这个距离才重新起步。
    # 单阈值在追移动目标时会高频「停-走-停」：贴到 target 就停、目标一走又起步。
    # 加滞后（施密特触发）切换频率降一个量级；而且停住时【不清积分】——
    # 重新起步能立刻跟上，不用重新把积分爬上去。
    resume_hyst_m: float = 0.15
    jump_limit_m: float = 0.60      # 单帧距离突变上限
    max_yaw_rate_hz: float = 3.0    # 转向变化率限幅（比例量/秒）
    max_accel_hz: float = 2.0       # 前进变化率限幅（比例量/秒）


class FollowController:
    """无状态外设、纯计算。update() 每帧调一次。"""

    def __init__(self, cfg: FollowConfig):
        self.cfg = cfg
        self.last_pos = None
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.last_t = None
        self.jump_strikes = 0
        self.int_e = 0.0            # 距离误差的积分（PI 里的 I）
        self.holding = False        # 是否处于"到位停住"（带滞后，见 update）
        self.reason = "init"

    def _slew(self, want, prev, rate_hz, dt):
        if dt <= 0:
            return float(want)
        step = rate_hz * dt
        return float(np.clip(want, prev - step, prev + step))

    def update(self, pos_body, now):
        """pos_body: (前,左,上) 米 或 None。

        返回 (vx, wz, reason)。
        """
        c = self.cfg
        dt = (now - self.last_t) if self.last_t is not None else 0.0
        self.last_t = now

        # --- 1) 看不见 → 不动 ---
        # 漏检宽限【不在这里做】。它在 follow_controller 的 I/O 层：那里才拿得到
        # "最后一次真的看到 tag"的位置，宽限期内沿用该位置继续闭环（比冻结上一条
        # 指令好：狗移动后几何依然大致成立）。本函数保持纯函数 —— pos_body is None
        # 就是"不要动"，所以可以离线单元测试。
        if pos_body is None:
            self.reason = "看不见 → 停"
            self.last_vx = self.last_wz = 0.0
            # 同时清掉积分 —— 看不见时不该继续积累误差，否则重新看见会猛冲一下
            self.int_e = 0.0
            return 0.0, 0.0, self.reason

        P = np.array([float(pos_body[0]), float(pos_body[1])])
        dist = float(np.linalg.norm(P))

        # --- 距离跳变保护 ---
        # 连续 3 帧都超限就认账：说明位置是真的变了（不是误检），否则一旦
        # 发生一次大跳变，last_pos 不更新会永远拒绝下去。
        if self.last_pos is not None:
            d_prev = float(np.linalg.norm(self.last_pos))
            if abs(dist - d_prev) > c.jump_limit_m:
                self.jump_strikes += 1
                if self.jump_strikes < 3:
                    self.reason = (f"距离跳变 {abs(dist-d_prev):.2f}m "
                                   f"(第{self.jump_strikes}次) → 丢弃该帧")
                    return 0.0, 0.0, self.reason
        self.jump_strikes = 0
        self.last_pos = P

        # --- 2) 到位判断（带滞后）---
        # 单阈值会在追移动目标时高频「停-走-停」：贴到 target 就停、目标一走
        # 又起步。加 resume_hyst_m 的滞后（施密特触发），切换频率降一个量级。
        # 停住期间【保留积分】—— 目标一动就能立刻跟上，不用重新爬积分。
        if self.holding:
            if dist > c.target_dist_m + c.resume_hyst_m:
                self.holding = False
        elif dist <= c.target_dist_m:
            self.holding = True
        if self.holding:
            self.reason = f"到位 {dist:.2f}m ≤ {c.target_dist_m:.2f}m → 停"
            self.last_vx = self.last_wz = 0.0
            return 0.0, 0.0, self.reason

        # --- 3) 追击：目标点 = tag 位置 + target × (tag→狗 方向) ---
        # 即"直接朝 tag 走到 target 距离"。法向量偏移方案已移除，见 FollowConfig。
        n = -P / max(dist, 1e-6)
        T = P + c.target_dist_m * n

        ex, ey = float(T[0]), float(T[1])
        # ---- P + I ----
        # I 项负责消除"追移动目标"的稳态误差（说明见 FollowConfig）。
        # 积分带限幅：万一 tag 长时间卡在一个够不到的位置，积分也不会无限涨。
        if c.ki_dist > 0.0 and dt > 1e-3:
            self.int_e = float(np.clip(self.int_e + ex * dt,
                                       -c.int_limit, c.int_limit))
        # 死区只压【P 项】，不压 I 项 —— 否则追移动目标时会在死区边界
        # 卡成"一顿一顿"（P 被清零、I 也被清零，速度反复掉下来）
        p_term = 0.0 if abs(ex) < c.pos_deadband_m else c.kp_dist * ex
        vx = float(np.clip(p_term + c.ki_dist * self.int_e, 0.0, c.max_vx))
        heading = math.atan2(ey, ex)        # 左正 → 左转（Yaw 正 = 逆时针）
        heading_deg = math.degrees(heading)
        if abs(heading_deg) < c.yaw_deadband_deg:
            # 死区在两种模式下都要 —— 正对 tag 时的微小偏差不做修正，
            # 否则输出会在 0 附近来回抖（追移动目标时会走成小锯齿）
            wz = 0.0
        elif c.yaw_bangbang:
            # 开关式（实验用）：死区外直接满量程。追移动目标会走 Z 字，
            # 但那正是它当初被引入的原因 —— 以为只有满量程能起转。
            wz = math.copysign(c.max_wz, heading_deg)
        else:
            # 比例式（默认）：方位误差越大转得越快。0.5≈43°/s、1.0≈102°/s，
            # 只要期望值不落在 0.35~0.40 那个死区里就可靠（见 FollowConfig）。
            wz = float(np.clip(c.kp_yaw * heading, -c.max_wz, c.max_wz))

        vx = self._slew(vx, self.last_vx, c.max_accel_hz, dt)
        if not c.yaw_bangbang:
            # 开关式【不做】变化率限幅：实测 yaw 起转本身就慢且不可预测，
            # 再加 0→1.0 要 0.33s 的爬升会把它拖得完全没反应。
            wz = self._slew(wz, self.last_wz, c.max_yaw_rate_hz, dt)
        self.last_vx, self.last_wz = vx, wz

        self.reason = (f"追击 距={dist:.2f}m "
                       f"目标点=({ex:+.2f},{ey:+.2f})")
        return vx, wz, self.reason
