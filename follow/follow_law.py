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
    # 速度上限按【实测标度】定：指令 0.06 持发 3.0s → 尺子量得 0.33m
    # （与 LinearX 逐帧积分出的 33cm 完全一致 → 反馈可当实测值用）。
    # 扣掉约 0.4s 起步加速 → 稳态 ≈ 0.127 m/s，满量程 ≈ 2.1 m/s。
    # 所以 max_vx=0.12 大约是 0.25 m/s。
    # （§1.2.6 标的 ±1.67 m/s 是真实轴指令的量程，和这里不是一回事，别混用）
    max_vx: float = 0.12            # 前进比例量上限 ≈ 0.25 m/s
    # 转向：实测 Yaw 通道【非单调】—— 0.5 → 21°，0.8 → 1.4°，1.0 → 180°。
    # 中间值不可用，只有满量程能可靠起转。所以默认用开关式（bang-bang）
    # 而不是比例式，绕开整条失效区间。
    max_wz: float = 1.00            # 满量程 —— 唯一实测有效的值
    yaw_bangbang: bool = True       # True=开关式；False=比例式(仅调试用)
    yaw_deadband_deg: float = 8.0   # 方位误差小于这个角度就不转向，防抖
    # 目标点是否沿 tag 法向量偏移 1 米（需求原话）。
    # 默认【关】。原因有二，都是仿真实测出来的：
    #  1) 法向量的 EMA 滤波会形成正反馈：狗一转身 → 滤波滞后 → 目标点
    #     永远偏在死区外 → 无限原地打转。15Hz 下滞后 12.7°，调参修不掉。
    #  2) 转向通道本身不可靠（只有满量程能起转，起转时刻不可预测）。
    # 关掉后狗"直接朝 tag 走到 1 米"，同样满足"距 tag 1 米"这个目标，
    # 且几乎不需要转向。代价：不会自动站到 tag 正前方（离理想点 0.24~0.72m）。
    use_tag_normal: bool = False
    kp_dist: float = 1.0            # 前向误差(m) → 速度比例
    kp_yaw: float = 1.2             # 方位角(rad) → 角速度比例
    pos_deadband_m: float = 0.10    # 距目标点这么近就不再前进
    lost_timeout_s: float = 0.0     # 0=丢一帧即停(严格)；>0=宽限期内保持上条指令
    jump_limit_m: float = 0.60      # 单帧距离突变上限
    max_yaw_rate_hz: float = 3.0    # 转向变化率限幅（比例量/秒）
    max_accel_hz: float = 2.0       # 前进变化率限幅（比例量/秒）
    normal_ema_alpha: float = 0.30  # 法向量低通系数(0~1)，越小越平滑
                                    # 实测 tag 只有 34px 时，静止 tag 的法向
                                    # 横向分量会抖 ±0.2 → 不滤的话转向会抖


class FollowController:
    """无状态外设、纯计算。update() 每帧调一次。"""

    def __init__(self, cfg: FollowConfig):
        self.cfg = cfg
        self.last_pos = None
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.last_t = None
        self.jump_strikes = 0
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
        if pos_body is None:
            if (c.lost_timeout_s > 0 and self.last_seen_t > 0
                    and now - self.last_seen_t <= c.lost_timeout_s):
                self.reason = f"丢失宽限 {now - self.last_seen_t:.2f}s"
                return self.last_vx, self.last_wz, self.reason
            self.reason = "看不见 → 停"
            self.last_vx = self.last_wz = 0.0
            return 0.0, 0.0, self.reason

        self.last_seen_t = now
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

        # --- 2) 已在目标距离内 → 不动 ---
        if dist <= c.target_dist_m:
            self.reason = f"到位 {dist:.2f}m ≤ {c.target_dist_m:.2f}m → 停"
            self.last_vx = self.last_wz = 0.0
            return 0.0, 0.0, self.reason

        # --- 3) 追击：目标点 = tag 位置 + target × (tag→狗 方向) ---
        # 即"直接朝 tag 走到 target 距离"。法向量偏移方案已移除，见 FollowConfig。
        n = -P / max(dist, 1e-6)
        T = P + c.target_dist_m * n

        ex, ey = float(T[0]), float(T[1])
        vx = float(np.clip(c.kp_dist * ex, 0.0, c.max_vx))
        if abs(ex) < c.pos_deadband_m:
            vx = 0.0
        heading = math.atan2(ey, ex)        # 左正 → 左转（Yaw 正 = 逆时针）
        heading_deg = math.degrees(heading)
        if c.yaw_bangbang:
            # 开关式：死区内不转，否则满量程。
            # 实测 Yaw 的中间值（0.5/0.8）不产生稳定转向，比例式无法工作。
            wz = 0.0 if abs(heading_deg) < c.yaw_deadband_deg \
                else math.copysign(c.max_wz, heading_deg)
        else:
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
