#!/usr/bin/env python3
"""Tag 指令状态机 —— 相机实时检测 tag，tag id 决定机器人动作。

在相机所在机 (.102) 上运行：

    source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=2
    /home/ysc/follow_env/bin/python tag_command_fsm.py            # 干跑
    /home/ysc/follow_env/bin/python tag_command_fsm.py --go       # 真发

动作表（tag id → 轴指令）
--------------------------------------------------------------------------
      1  前进      X = +v
     18  后退      X = -v
      3  左转      Yaw = +w
     61  右转      Yaw = -w
     68  右侧移动  Y = -s
      6  停止      全零
      0  楼梯步态  切 0x1003（楼梯 · 常规模式）
     69  基础步态  切 0x1001（基础 · 常规模式）—— 从楼梯切回来
     63  高台步态  切 0x1002（高台 · 常规模式）   ← 原来这里是「左侧移动」

    没检测到任何指令 tag  → 视为"停止"（看不见就不动）

架构（沿用今天验证过的两线程解耦）
--------------------------------------------------------------------------
    线程A 检测：相机 15Hz → 检测所有 tag → 更新"当前 tag id"
    线程B 控制：固定 20Hz → 查动作表 → 发轴指令 + 心跳

    实机教训：单线程"检测完才发一次指令"只能跑到 5Hz，狗走起来明显卡顿。
    轴指令必须【持续流式下发】。

安全
--------------------------------------------------------------------------
  · tag 漏检在 --lost-timeout 内 → 沿用最后一次命中的 tag（宽限期，防止
    指令被切成碎片）；超出宽限或从未检测到指令 tag → 停止
  · 状态上报中断 > STATUS_STALE_S，或不在 RL 控制(17) → 停止
    ⚠ 判据必须是【上报新鲜度】，不能只看 motion_state 的值 —— 通信断了
      这个变量会永远停在最后一个值，程序会以为机器人还在线。
  · Ctrl+C / 异常 → finally 连续归零
  · 速度上限硬钳制，且【被钳制就拒绝启动】而不是静默改写
  ⚠ 【未实现，别当成已有保障】"有 FATAL 故障 → 停止"：RobotLink 只解析
    BasicStatus / MotionStatus，没有解析 ErrorList，拿不到故障等级。
    要故障门得先给 RobotLink 加故障解析（或改用 robot_dog_sdk 的
    RobotClient）。起跑前查故障请用 asdu_probe.py。
"""

import argparse
import os
import signal
import sys
import threading
import time



import cv2

import rclpy

from dt_apriltags import Detector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d435i_tag_probe import (DEFAULT_TAG_SIZE_M, FAMILY,  # noqa: E402
                             Probe, imgmsg_to_bgr, tag_side_px)
from tag_follower import RobotLink  # noqa: E402
import web_view  # noqa: E402

# 观测界面的事件环 —— 终端打印的同时也进这里，浏览器里能看到同一份记录。
EVENTS = web_view.Events()
def install_sigterm_as_interrupt():
    """让 SIGTERM 也走 KeyboardInterrupt 分支，触发 finally 里的归零。

    Python 默认收到 SIGTERM 会【直接退出、不跑 finally】—— 这个程序的
    finally 里有"连续发零速"，不跑就等于停车指令发不出去。

    而停服务的常规手段（kill、systemctl stop、部署脚本）用的都是 SIGTERM，
    所以必须在这里接管。SIGINT（Ctrl+C）本来就会抛 KeyboardInterrupt。
    """
    def _handler(signum, frame):
        raise KeyboardInterrupt
    try:
        signal.signal(signal.SIGTERM, _handler)
    except (ValueError, OSError):
        pass    # 非主线程调用时 signal.signal 会抛，忽略即可

MOTION_RL = 17
STATUS_STALE_S = 1.2        # 状态上报中断多久算断（BasicStatus 是 2Hz，容丢两帧）

# 动作类型
AXIS, GAIT, STOP = "axis", "gait", "stop"

# tag id → (动作名, 类型, 轴/步态值, 带符号的幅度)。
# 最终输出 = 这里的幅度 × 该轴 --mag × 该轴 --sign，所以改这个就是改
# "这个动作能有多快"。
TAG_ACTIONS = {
    1:  ("前进",      AXIS, "x",   +1.0),
    # 后退减半：相机装在【前部、朝前下方】，后退时狗看不到身后 —— 留出反应时间。
    # 注意前进/后退共用 x 轴，但各有自己的幅度，所以 --mag-x 调的是两者共同的
    # 倍率；要单独改后退就只改这一行。
    18: ("后退",      AXIS, "x",   -0.5),
    # 转向减半（±1.0 → ±0.5）：原来满量程约 102°/s，原地 2 秒就转半圈，
    # 太快、不好控制。0.5 实测约 43°/s，稳态很平（见下）。
    # 依据 2026-09-21 实机实测（axis_test 三档扫描，读回 MotionStatus.AngularZ）：
    #     指令 0.25 → 峰值 0.47 后衰减到 0.2 rad/s   ⚠️ 不稳，不可用
    #     指令 0.50 → 0.68~0.81 rad/s 稳  ≈ 43°/s   ✅
    #     指令 1.00 → 1.71~2.03 rad/s 稳  ≈102°/s
    #   ⇒ 0.5 与 1.0 之比 2.37（指令翻倍），【基本线性、单调】。
    # 旧记录说"Yaw 非单调（0.5→21°、0.8→1.4°）"与本次实测矛盾 —— 那条同样
    # 出自【源端点被污染】的测量期，和 X 的"非单调"是一回事，已作废。
    # 可用区间大致 0.4~1.0；0.25 太低不稳，别往下调。
    3:  ("左转",      AXIS, "yaw", +0.5),
    61: ("右转",      AXIS, "yaw", -0.5),
    # ⚠️ tag 63 原来是「左侧移动」，2026-09-21 改成「高台步态」。
    #    代价：「左侧移动」这个动作没有 tag 了（右侧移动仍是 tag 68，
    #    于是 Y 轴只剩负方向）。要用左侧移得另找一个 tag。
    63: ("高台步态",  GAIT, 0x1002, 0.0),     # 高台 · 常规模式
    68: ("右侧移动",  AXIS, "y",   -1.0),
    6:  ("停止",      STOP, None,   0.0),
    # GAIT 动作的第 3 个槽位放【目标步态值】（AXIS 动作那里放的是轴名）。
    # 三个步态都是"常规模式"：切完 ControlUsageMode 仍是 0，比例轴指令继续有效。
    # （别换成 0x3002/0x3003 那两个"导航模式"的 —— 会把模式切成 1，
    #   §1.2.5 的比例轴指令随即失效，本状态机其他 tag 会全部失灵）
    #
    # 高台 0x1002：厂商文档里【没有列出】这个步态（全文只有
    #   0x1001 / 0x1003 / 0x3002 / 0x3003），是使用者从厂商处得知的 ——
    #   来源是「官方 App 在【基础（常规）模式】下可以选高台」。
    #   ✅ 已于 2026-09-21 实机验证（.102 + .103，CD1/PRO）：机器人 BasicStatus
    #      回报 Gait=4098(0x1002)，ControlUsageMode 保持 0（常规），
    #      与 0x1001 可来回切换、可重复。切换耗时约 0.5s（停稳 0.3 + 切换 0.2）。
    0:  ("楼梯步态",  GAIT, 0x1003, 0.0),     # 楼梯 · 常规模式
    69: ("基础步态",  GAIT, 0x1001, 0.0),     # 基础 · 常规模式（从楼梯切回）
}

# 步态值 → 可读名（打印用）
GAIT_NAMES = {0x1001: "基础(常规)", 0x1002: "高台(常规)", 0x1003: "楼梯(常规)",
              0x3002: "平地(导航)", 0x3003: "楼梯(导航)"}

# ⚠️ 楼梯步态有两个，后果完全不同：
#     0x1003 楼梯（常规模式）—— 保持 ControlUsageMode=0，比例轴指令继续有效
#     0x3003 楼梯（导航模式）—— 会把模式切成 1，【§1.2.5 的比例轴指令随即失效】，
#                               本状态机的其他 tag 会全部失灵
# 所以默认选 0x1003。要改必须清楚上面这条。
STAIR_GAIT_DEFAULT = 0x1003

CMD_GAIT = (0x00100001, 0x00300002)   # §1.2.4 运动步态切换

# 各轴的默认幅度（比例量），乘在 TAG_ACTIONS 的幅度上。
#
# 【标度已实测，别再照旧结论调】2026-09-20 直连实测（详见 TECHNICAL_REPORT §2.7）：
#   X：指令 0.5 持发 1.5s → LinearX 稳态 0.857 m/s ⇒ 满量程 ≈ 1.71 m/s。
#      → 通道【线性】，且与 §1.2.6 标的 ±1.67 m/s 吻合 → 中间值可以用。
#   之前记的"低值区不响应、非单调"是在【源端点被污染】的环境下测出来的
#   伪影（本机两条等价路由 → 多个客户端抢控制权），不是通道特性。
#   Y / Yaw：还没用同样方法复测。旧记录（Y=0.15 无反应；Yaw 0.5→21°、
#      0.8→1.4°）同样出自污染期，可疑但【未验证】，所以暂仍按满量程用。
#      想改用中间值，先照 §2.7 那个方法复测一次。
DEFAULT_MAG = {"x": 1.00, "y": 1.00, "yaw": 1.00}

# 各轴硬上限。超了就拒绝启动 —— 静默钳制会让你以为在测 A、实际发的是 B
# （这个坑今天踩过两次）。
HARD_LIMIT = {"x": 1.00, "y": 1.00, "yaw": 1.00}


class SharedTag:
    """线程A 写、线程B 读的"当前 tag"。

    ★ 两个时间戳必须分开存（这里踩过，代价很大）★
        frame_ts —— 检测线程最后一次【处理完一帧】的时间。只反映线程活没活。
        good_ts  —— 最后一次【真的命中动作表里的 tag】的时间。过期判断用这个。

    原先只有一个 ts，而漏检帧也会刷新它，于是 --lost-timeout 实际只检测
    "检测线程卡死"，想要的"漏检宽限"从来没生效过 —— 漏一帧就立刻归零。

    实测代价（2026-09-20 的 run.log）：命中率 8% 时指令被切成 0.1~0.3 秒的
    碎片，中间夹着"停止(无tag)"；而机器人需要约 1 秒连续指令才能起速
    （实测 0.25s→0.17、0.50s→0.39、1.10s→0.86 m/s）。所以每个脉冲最多把
    速度推到 0.17 m/s 就归零 —— 狗只会抽动、根本迈不起步子。
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.tag_id = None       # 本帧命中的 id；漏检帧为 None
        self.px = 0.0
        self.good_id = None      # 最后一次命中，供漏检宽限期沿用
        self.good_px = 0.0
        self.frame_ts = 0.0
        self.good_ts = 0.0       # 0 = 从未命中
        self.frames = 0
        self.hits = 0
        self.det_ms = 0.0        # 最近一帧的解算耗时（观测界面用）
        self.seen_counts = {}    # 本次运行各 id 命中次数

    def put(self, tag_id, px, now, det_ms=0.0):
        with self.lock:
            self.tag_id, self.px, self.frame_ts = tag_id, px, now
            self.frames += 1
            self.det_ms = det_ms
            if tag_id is not None:
                self.good_id, self.good_px, self.good_ts = tag_id, px, now
                self.hits += 1
                self.seen_counts[tag_id] = self.seen_counts.get(tag_id, 0) + 1

    def get(self):
        with self.lock:
            return (self.tag_id, self.px, self.frame_ts, self.good_ts,
                    self.good_id, self.good_px, self.det_ms)

    def stats(self):
        with self.lock:
            return self.frames, self.hits, dict(self.seen_counts)


def log_state(msg):
    """状态机自己的日志（和每秒的状态行区分开，带时间戳）。

    同时喂给观测界面的事件环，浏览器里能看到和终端一样的状态切换记录。
    """
    print(f"  [{time.strftime('%H:%M:%S')}] {msg}", flush=True)
    EVENTS.add("ok" if msg.startswith("✅")
               else ("warn" if msg.startswith("⚠") else "info"), msg)


def detect_cmd_tag(det, gray, fx, fy, cx0, cy0, tag_size, min_px):
    """检测画面里所有【在动作表里】的 tag，返回最大的那个 (id, 像宽)。

    多个指令 tag 同时出现时取画面最大的 —— 最近的通常最可靠。
    min_px 用来挡掉远处的小 tag（过小容易误检）。
    """
    res = det.detect(gray, estimate_tag_pose=True,
                     camera_params=[fx, fy, cx0, cy0], tag_size=tag_size)
    if not res:
        return None, 0.0
    best, best_px = None, 0.0
    for r in res:
        tid = int(r.tag_id)
        if tid not in TAG_ACTIONS:
            continue
        w = tag_side_px(r.corners)
        if w < min_px:
            continue
        if w > best_px:
            best, best_px = tid, w
    return best, best_px


class DetectorThread(threading.Thread):
    """线程A：相机 → 检测 → 更新共享 tag。只碰相机，不碰 socket。"""

    def __init__(self, node, det, shared, tag_size, min_px, hz, cam=None):
        super().__init__(daemon=True, name="detector")
        self.node, self.det, self.shared = node, det, shared
        self.tag_size, self.min_px = tag_size, min_px
        self.period = 1.0 / hz
        self.running = True
        self.cam = cam                 # 观测界面的相机健康计数，可为 None
        k = node.info.k
        self.fx, self.fy, self.cx0, self.cy0 = k[0], k[4], k[2], k[5]

    def run(self):
        next_t = time.monotonic()
        while self.running:
            try:
                if not self.node.frames:
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                    continue
                msg = self.node.frames[-1]    # 只取最新，丢弃积压
                self.node.frames.clear()
                rclpy.spin_once(self.node, timeout_sec=0.0)
            except Exception:
                # 关停时主线程可能已经 rclpy.shutdown() 了，检测线程里的
                # spin_once 就会抛 RCLError / ExternalShutdownException。
                # 那是正常收尾、不是崩溃 —— 静默退出即可。否则日志末尾
                # 会挂一大段 traceback，看起来像程序炸了（实际归零早发完了）。
                if not rclpy.ok():
                    break
                raise
            if self.cam is not None:
                self.cam.tick(msg)

            bgr = imgmsg_to_bgr(msg)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            t0 = time.monotonic()
            tid, px = detect_cmd_tag(self.det, gray, self.fx, self.fy,
                                     self.cx0, self.cy0, self.tag_size,
                                     self.min_px)
            det_ms = (time.monotonic() - t0) * 1000.0
            self.shared.put(tid, px, time.monotonic(), det_ms)

            next_t += self.period
            # 只读一次时钟再判断 —— 先比后算会让差值为负，sleep 抛
            # ValueError 把整个线程打断（实机踩过）
            _d = next_t - time.monotonic()
            if _d > 0:
                time.sleep(_d)
            else:
                next_t = time.monotonic()


def main():
    install_sigterm_as_interrupt()
    ap = argparse.ArgumentParser(description="Tag 指令状态机")
    ap.add_argument("--robot", default="10.21.33.103")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--go", action="store_true", help="真的下发指令")
    ap.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE_M)
    ap.add_argument("--hz", type=float, default=20.0, help="控制循环频率")
    ap.add_argument("--det-hz", type=float, default=15.0)
    ap.add_argument("--lost-timeout", type=float, default=1.0,
                    help="漏检宽限（秒）：这么久没再看到 tag 才停止；宽限内沿用上次"
                         "命中的 tag 继续发指令。默认 1.0 是照【实测约 1s 的起步"
                         "时间】定的 —— 宽限小于它的话，任何一次漏检都要重新爬坡，"
                         "指令会碎成 0.1~0.3 秒的脉冲，狗迈不起步子")
    ap.add_argument("--min-px", type=float, default=20.0,
                    help="tag 最小像宽，挡掉远处小 tag 的误检")
    ap.add_argument("--stair-gait", type=lambda s: int(s, 0),
                    default=STAIR_GAIT_DEFAULT,
                    help=f"tag id=0 要切到的步态。默认 {hex(STAIR_GAIT_DEFAULT)}"
                         f"（楼梯/常规模式）。⚠ 只影响 tag 0 —— 其余步态 tag"
                         f"（如 tag 69 → 0x1001）的目标步态写在 TAG_ACTIONS 里。"
                         f"改成 0x3003 会切到导航模式，届时比例轴指令失效、"
                         f"其他 tag 全部失灵")
    ap.add_argument("--settle-s", type=float, default=0.3,
                    help="切步态前等机器人停稳的时间（§1.2.4 要求完全停止）")
    ap.add_argument("--gait-timeout", type=float, default=15.0,
                    help="等步态切换完成的超时")
    ap.add_argument("--duration", type=float, default=0.0, help="0=一直跑")
    # 各轴幅度与符号（实测发现方向可能和文档不符，所以做成可翻转）
    ap.add_argument("--mag-x", type=float, default=DEFAULT_MAG["x"])
    ap.add_argument("--mag-y", type=float, default=DEFAULT_MAG["y"])
    ap.add_argument("--mag-yaw", type=float, default=DEFAULT_MAG["yaw"])
    ap.add_argument("--sign-x", type=float, default=1.0, choices=(1.0, -1.0))
    ap.add_argument("--sign-y", type=float, default=1.0, choices=(1.0, -1.0))
    ap.add_argument("--sign-yaw", type=float, default=1.0, choices=(1.0, -1.0))
    ap.add_argument("--web-port", type=int, default=8010,
                    help="只读观测界面的端口 → http://<本机>:端口/ 。0 = 不开界面")
    args = ap.parse_args()

    mag = {"x": args.mag_x, "y": args.mag_y, "yaw": args.mag_yaw}
    sign = {"x": args.sign_x, "y": args.sign_y, "yaw": args.sign_yaw}

    # 幅度校验：超上限直接拒绝，不静默改写
    bad = [f"{a}={mag[a]} > 上限 {HARD_LIMIT[a]}"
           for a in mag if abs(mag[a]) > HARD_LIMIT[a]]
    if bad:
        for b in bad:
            print(f"[超限] {b}")
        sys.exit("[拒绝启动] 幅度超过该轴硬上限。改小参数，别让它静默钳制。")

    shared = SharedTag()
    cam = web_view.CamStats()
    # 观测界面要的快照：HTTP 线程读、主循环写。锁只保护一次整体赋值。
    view_lock = threading.Lock()
    view_state = {"name": "启动中", "out": {}, "tgt": "", "gate": "",
                  "ticks": 0, "sent": 0}
    view = None

    rclpy.init()
    node = Probe()
    det = Detector(families=FAMILY, nthreads=2, quad_decimate=1.0,
                   quad_sigma=0.8, refine_edges=1, decode_sharpening=0.5)
    link = RobotLink(args.robot, args.port)

    print(f"运控 {args.robot}:{args.port}   模式 "
          f"{'★ 真实控制 ★' if args.go else '干跑（只算不发）'}")
    print(f"tag_size={args.tag_size*1000:.1f}mm  最小像宽={args.min_px:.0f}px  "
          f"控制 {args.hz:.0f}Hz / 检测 ≤{args.det_hz:.0f}Hz  "
          f"漏检宽限 {args.lost_timeout*1000:.0f}ms  "
          f"状态中断阈值 {STATUS_STALE_S*1000:.0f}ms")
    print("动作表：")
    for tid in sorted(TAG_ACTIONS):
        name, kind, axis, s = TAG_ACTIONS[tid]
        if kind == STOP:
            print(f"   id={tid:<3d} {name:<8s} 全零")
        elif kind == GAIT:
            # tag 0 的目标步态可用 --stair-gait 覆盖；其余 GAIT tag 用表里的值
            g = args.stair_gait if tid == 0 else axis
            mode = "常规" if g in (0x1001, 0x1003) else "导航"
            print(f"   id={tid:<3d} {name:<8s} 切步态 {hex(g)}"
                  f"（{GAIT_NAMES.get(g, '?')}）"
                  f"  ← 先停稳再切（§1.2.4）")
            if mode == "导航":
                print("        ⚠️ 导航步态会把使用模式切成 1，"
                      "比例轴指令随即失效 —— 其他 tag 将全部失灵！")
        else:
            eff = s * sign[axis] * mag[axis]
            print(f"   id={tid:<3d} {name:<8s} {axis} = {eff:+.3f}"
                  f"   (动作 {s:+.2f} × 幅度 {mag[axis]:.2f} "
                  f"× 符号 {sign[axis]:+.0f})")

    det_thread = None
    t_start = time.monotonic()
    ticks = sent = 0
    try:
        if not node.wait_for(15.0):
            print("[错误] 没拿到相机图像/内参。检查 ROS_DOMAIN_ID=2 和相机服务。")
            return 1
        k = node.info.k
        print(f"\n相机就绪 fx={k[0]:.1f} {node.info.width}x{node.info.height}")

        if args.go:
            print("检查机器人状态 ...")
            for _ in range(20):
                link.heartbeat()
                link.pump()
                if link.motion_state is not None:
                    break
                time.sleep(0.1)
            # 注意：RobotLink 只解析 BasicStatus/MotionStatus，没有故障列表
            # （max_active_severity 是 robot_dog_sdk 里 RobotClient 的方法，
            #   别混用）。要故障得上 asdu_probe。
            print(f"  MotionState={link.motion_state} Gait={link.gait} "
                  f"Mode={link.mode}")
            if link.motion_state != MOTION_RL:
                print(f"[拒绝运行] 未处于 RL 控制({MOTION_RL})。请先让狗起立。")
                return 2
        print()

        det_thread = DetectorThread(node, det, shared, args.tag_size,
                                    args.min_px, args.det_hz, cam=cam)
        det_thread.start()

        period = 1.0 / args.hz
        t_start = time.monotonic()
        next_t = t_start
        last_beat = last_print = 0.0
        cur_name = None
        # 步态切换子状态机（见循环里的 GAIT 分支）
        gait_phase = "idle"          # idle | settling | switching | done
        gait_target = None           # 本次切换的目标步态值；换目标就重跑流程
        gait_t0 = 0.0
        gait_sent_t = 0.0

        # ---- 只读观测界面（不影响控制：只读快照，从不发送任何 ASDU 报文）----
        if args.web_port:
            def snapshot():
                with view_lock:
                    vs = dict(view_state)          # O(1) 拷贝，锁只到这里
                frames, hits, counts = shared.stats()
                frames_cam, fps, age, w, h, enc = cam.snap()
                snap = {
                    "now": time.time(),
                    "app": {
                        "name": "tag_command_fsm",
                        "go": args.go,
                        "uptime_s": time.monotonic() - t_start,
                        "control_hz": vs["ticks"] / max(1e-9, time.monotonic() - t_start),
                        "det_hz": frames / max(1e-9, time.monotonic() - t_start),
                        "det_ms": vs.get("det_ms"),
                        "events": EVENTS.tail(25),
                    },
                    "det": {"rows": [
                        {"k": "当前动作", "v": vs["name"]},
                        {"k": "tag id", "v": "—" if vs.get("tid") is None
                            else f'{vs["tid"]}  ({vs.get("px", 0):.0f}px)'},
                        # 配色：本帧=绿；宽限内=黄；丢失=红；从未检测到=不标色
                        # （还没举 tag 时"从未检测到"是正常状态，标红是误报）
                        {"k": "检测时效", "v": vs.get("tgt") or "—",
                         "lvl": ("ok" if vs.get("tgt") == "本帧"
                                 else ("warn" if "宽限" in (vs.get("tgt") or "")
                                       else ("err" if "丢失" in (vs.get("tgt") or "")
                                             else "")))},
                        {"k": "正在发出 X/Y/Yaw",
                         "v": "—" if not vs.get("out") else
                              "{:+.3f} / {:+.3f} / {:+.3f}".format(
                                  vs["out"].get("x", 0.0), vs["out"].get("y", 0.0),
                                  vs["out"].get("yaw", 0.0))},
                        {"k": "状态门", "v": vs.get("gate") or "未拦截",
                         "lvl": "err" if vs.get("gate") else "ok"},
                        {"k": "解算耗时", "v": ("—" if vs.get("det_ms") is None
                                              else f'{vs["det_ms"]:.0f} ms')},
                        {"k": "漏检宽限 / 状态中断",
                         "v": f'{args.lost_timeout * 1000:.0f} / '
                              f'{STATUS_STALE_S * 1000:.0f} ms'},
                        {"k": "累计 tick / 轴指令",
                         "v": f'{vs.get("ticks", 0)} / {vs.get("sent", 0)}'},
                    ]},
                }
                snap.update(web_view.robot_snapshot(link))
                snap["camera"] = web_view.camera_snapshot(
                    frames_cam, hits, fps, age, w, h, enc)
                return snap

            view = web_view.WebView(snapshot, port=args.web_port)
            view.start()

        while True:
            now = time.monotonic()
            if args.duration > 0 and now - t_start > args.duration:
                break

            # ---- 心跳 / 收状态（放在本轮判断之前，本轮用最新状态）----
            if now - last_beat > 1.0:
                link.heartbeat()
                last_beat = now
            link.pump(0.004)   # 只给它 4ms，别占满 50ms 周期

            # ---- 读最新 tag ----
            tid, px, frame_ts, good_ts, good_id, good_px, det_ms = shared.get()
            # 过期判断用【最后一次真的命中】的时间，不是最后一帧的时间。
            # 漏检但在宽限期内 → 沿用上次的 tag，指令保持连续（关键：机器人
            # 要约 1 秒连续指令才能起速，指令一碎就走不起来）
            if good_ts <= 0:
                tid, px, tgt = None, 0.0, "从未检测到"
            else:
                age = now - good_ts
                if age > args.lost_timeout:
                    tid, px, tgt = None, 0.0, f"丢失 {age:.2f}s"
                elif tid is None:
                    tid, px = good_id, good_px        # 宽限期内沿用
                    tgt = f"漏检 {age:.2f}s(宽限内)"
                else:
                    tgt = "本帧"

            # ---- 状态机：tag id → 动作 ----
            if tid is None:
                name, kind, axis, s = "停止(无tag)", STOP, None, 0.0
            else:
                name, kind, axis, s = TAG_ACTIONS[tid]

            out = {"x": 0.0, "y": 0.0, "yaw": 0.0}

            if kind == AXIS:
                out[axis] = s * sign[axis] * mag[axis]
                gait_phase = "idle"
            elif kind == STOP:
                gait_phase = "idle"
            else:   # GAIT —— 步态切换子状态机
                # §1.2.4：步态切换必须在机器人【完全停止】后下发，否则指令
                # 被缓存、停稳后才执行。所以分三段走：停稳 → 下发 → 等生效。
                # GAIT 动作的第 3 槽位放的是目标步态值（AXIS 动作放轴名）。
                tg = axis
                label = name          # 保留动作名；下面的 name 会被子状态覆盖
                # ⚠️ 目标变了必须重跑整个流程 —— 否则从 tag 0（已 done）直接换到
                #    tag 69 时 gait_phase 还是 "done"，一个分支都不进，静默失效。
                if tg != gait_target:
                    gait_phase, gait_target = "idle", tg
                # 已经在目标步态就不用再走一遍"停稳→切换"
                if gait_phase == "idle" and link.gait == tg:
                    gait_phase = "done"
                    log_state(f"{label}：已经在 {hex(tg)}"
                              f"（{GAIT_NAMES.get(tg, '?')}），无需切换")
                if gait_phase == "idle":
                    gait_phase, gait_t0 = "settling", now
                    log_state(f"{label}：先停稳（{args.settle_s:.1f}s）"
                              f"再切 {hex(tg)}（{GAIT_NAMES.get(tg, '?')}）")
                if gait_phase == "settling":
                    name = f"{label}·停稳中"
                    if now - gait_t0 >= args.settle_s:
                        gait_phase, gait_t0 = "switching", now
                if gait_phase == "switching":
                    name = f"{label}·切换中({hex(link.gait or 0)}→{hex(tg)})"
                    # 每 0.5s 重发一次 —— 轴指令无响应帧、丢包静默，只能靠重发
                    if now - gait_sent_t > 0.5:
                        if args.go:
                            link.send_gait(tg)
                        gait_sent_t = now
                    if link.gait == tg:
                        gait_phase = "done"
                        mode = link.mode
                        log_state(f"✅ 步态已切到 {hex(tg)}"
                                  f"（{GAIT_NAMES.get(tg, '?')}）"
                                  f"使用模式 {mode}")
                        if mode == 1:
                            log_state("⚠️ 使用模式变成 1（导航）—— "
                                      "§1.2.5 比例轴指令已失效，其他 tag 全废！"
                                      "请用 tag 69 切回 0x1001")
                    elif now - gait_t0 > args.gait_timeout:
                        gait_phase = "idle"
                        log_state(f"⚠️ {args.gait_timeout:.0f}s 内未切到 "
                                  f"{hex(tg)}（当前 {hex(link.gait or 0)}）"
                                  f"—— 下轮重试")
                if gait_phase == "done":
                    name = f"{label}·已生效({hex(tg)})"

            # ---- 状态门：上报中断 / FATAL 故障 / 不在 RL 控制 → 停止 ----
            # ⚠ 必须放在 pump() 之后，用最新状态判断。只看 motion_state 的值
            #    不够：通信断了它会永远停在最后一个值（比如 17），程序会以为
            #    机器人还在线，继续 20Hz 下发运动指令。
            st_age = now - link.last_status_t if link.last_status_t > 0 else None
            gate = ""                      # 门因，空=没拦（观测界面显示用）
            if st_age is None:
                out = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                name = "停止(尚未收到状态上报)"
                gate = "尚未收到状态上报"
            elif st_age > STATUS_STALE_S:
                out = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                name = f"停止(状态上报中断 {st_age:.1f}s)"
                gate = f"状态上报中断 {st_age:.1f}s"
            elif link.worst_severity() >= 5:
                # 【本次新增】本文件开头一直声称"有 FATAL 故障 → 停止"，但
                # RobotLink 原先根本没解析 ErrorList，这条门从来没生效过。
                # 现在解析补上了，门也补上。severity 5 = FATAL。
                out = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                name = "停止(FATAL 故障)"
                gate = "FATAL 故障（severity ≥ 5）"
            elif link.motion_state != MOTION_RL:
                out = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                name = f"停止(非RL控制 {link.motion_state})"
                gate = f"非 RL 控制（当前 {link.motion_state}）"

            # ---- 每个 tick 都发（持续流式下发）----
            if args.go:
                link.send_axis(out["x"], out["yaw"], out["y"])
                sent += 1
            ticks += 1

            # ---- 更新观测界面快照（HTTP 线程读；锁只保护这一次整体赋值）----
            with view_lock:
                view_state.update(name=name, out=dict(out), tgt=tgt, gate=gate,
                                  ticks=ticks, sent=sent, tid=tid, px=px,
                                  det_ms=det_ms)

            # ---- 状态切换时打一条 ----
            if name != cur_name:
                print(f"  [{now-t_start:6.1f}s] ★ 状态 → {name}"
                      + (f"   (id={tid} {px:.0f}px)" if tid is not None else ""))
                cur_name = name

            if now - last_print > 1.0 and now - t_start > 1.0:
                print(f"    {now-t_start:6.1f}s 控制{ticks/(now-t_start):4.1f}Hz "
                      f"检测{shared.stats()[0]/(now-t_start):4.1f}Hz | "
                      f"tag={tid if tid is not None else '无'} [{tgt}] | "
                      f"X={out['x']:+.3f} Y={out['y']:+.3f} "
                      f"Yaw={out['yaw']:+.3f} | {name}")
                last_print = now

            next_t += period
            _d = next_t - time.monotonic()
            if _d > 0:
                time.sleep(_d)
            else:
                next_t = time.monotonic()

    except KeyboardInterrupt:
        print("\n[中断]")
    finally:
        if view is not None:
            view.stop()
        if det_thread:
            det_thread.running = False
        print("归零 ...")
        if args.go:
            link.stop()
        link.close()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            # 收到信号时 rclpy 可能已经自己关过了，重复关会抛 RCLError。
            # 归零在它之前就发完了，这里崩掉只是收尾难看。
            pass
        el = max(1e-9, time.monotonic() - t_start)
        frames, hits, counts = shared.stats()
        print(f"结束：检测帧 {frames}，命中 {hits} "
              f"({100.0*hits/max(1,frames):.0f}%)  → 检测 {frames/el:.1f}Hz | "
              f"控制 tick {ticks} ({ticks/el:.1f}Hz)，轴指令 {sent}")
        if counts:
            print("各 tag 命中次数：" + "  ".join(
                f"id{k}={v}" for k, v in sorted(counts.items())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
