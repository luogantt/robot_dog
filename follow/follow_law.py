#!/usr/bin/env python3
"""跟随控制律 —— 纯逻辑，只依赖 numpy/math。

单独成文件是有意的：这里没有 ROS、没有 cv2、没有 socket，所以可以在任何
机器上单元测试（见 test_follow_law.py）。真机那套 I/O 在 follow_controller.py。

规格
--------------------------------------------------------------------------
  1) 看不见 tag                    → 输出零，不动
  2) 狗距 tag <= target_dist_m      → 输出零，不动（不倒退）
  3) 狗距 tag >  target_dist_m      → 追击
       目标点 T = tag位置 + target × n
       其中 n 是 tag 法向量在地面的投影（单位向量）
       前进量 ∝ T 的前向分量，转向 ∝ T 的方位角
     n 不可靠时退回 n = 沿 tag→狗 方向，等价于"直接朝 tag 走到 target 距离"

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
    max_wz: float = 0.12            # 转向比例量上限（转向标度尚未实测，先压小）
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
        self.last_seen_t = 0.0
        self.last_pos = None
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.last_t = None
        self.jump_strikes = 0
        self.n_filt = None          # 法向量的低通状态
        self.reason = "init"

    def _filter_normal(self, n):
        """单位向量的指数滑动平均（球面上的近似）。alpha 越小越平滑。"""
        a = float(np.clip(self.cfg.normal_ema_alpha, 0.0, 1.0))
        if self.n_filt is None or a >= 1.0:
            self.n_filt = np.asarray(n, dtype=float)
            return self.n_filt
        merged = (1.0 - a) * self.n_filt + a * np.asarray(n, dtype=float)
        norm = float(np.linalg.norm(merged))
        if norm < 1e-6:
            # 平滑结果互相抵消（多半是符号在跳）——丢掉旧状态重新起步
            self.n_filt = np.asarray(n, dtype=float)
            return self.n_filt
        self.n_filt = merged / norm
        return self.n_filt

    def _slew(self, want, prev, rate_hz, dt):
        if dt <= 0:
            return float(want)
        step = rate_hz * dt
        return float(np.clip(want, prev - step, prev + step))

    def update(self, pos_body, normal_g, now):
        """pos_body: (前,左,上) 米 或 None；normal_g: (nx,ny) 单位向量 或 None。

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

        # --- 3) 追击：目标点 ---
        if normal_g is not None:
            n = self._filter_normal(normal_g)
            src = "法向"
        else:
            n = -P / max(dist, 1e-6)
            self.n_filt = None      # 法向断了就清滤波状态，免得下次把陈旧值混进来
            src = "回退(法向不可靠)"
        T = P + c.target_dist_m * n

        ex, ey = float(T[0]), float(T[1])
        vx = float(np.clip(c.kp_dist * ex, 0.0, c.max_vx))
        if abs(ex) < c.pos_deadband_m:
            vx = 0.0
        heading = math.atan2(ey, ex)        # 左正 → 左转（Yaw 正 = 逆时针）
        wz = float(np.clip(c.kp_yaw * heading, -c.max_wz, c.max_wz))

        vx = self._slew(vx, self.last_vx, c.max_accel_hz, dt)
        wz = self._slew(wz, self.last_wz, c.max_yaw_rate_hz, dt)
        self.last_vx, self.last_wz = vx, wz

        self.reason = (f"追击[{src}] 距={dist:.2f}m "
                       f"目标点=({ex:+.2f},{ey:+.2f})")
        return vx, wz, self.reason
