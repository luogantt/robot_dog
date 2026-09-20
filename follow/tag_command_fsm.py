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
     63  左侧移动  Y = +s
     68  右侧移动  Y = -s
      6  停止      全零
      0  楼梯步态  切 0x1003（楼梯 · 常规模式）
     69  基础步态  切 0x1001（基础 · 常规模式）—— 从楼梯切回来

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
    3:  ("左转",      AXIS, "yaw", +1.0),
    61: ("右转",      AXIS, "yaw", -1.0),
    63: ("左侧移动",  AXIS, "y",   +1.0),
    68: ("右侧移动",  AXIS, "y",   -1.0),
    6:  ("停止",      STOP, None,   0.0),
    # GAIT 动作的第 3 个槽位放【目标步态值】（AXIS 动作那里放的是轴名）。
    # 两个步态都是"常规模式"：切完 ControlUsageMode 仍是 0，比例轴指令继续有效。
    # （别换成 0x3002/0x3003 那两个"导航模式"的 —— 会把模式切成 1，
    #   §1.2.5 的比例轴指令随即失效，本状态机其他 tag 会全部失灵）
    0:  ("楼梯步态",  GAIT, 0x1003, 0.0),     # 楼梯 · 常规模式
    69: ("基础步态",  GAIT, 0x1001, 0.0),     # 基础 · 常规模式（从楼梯切回）
}

# 步态值 → 可读名（打印用）
GAIT_NAMES = {0x1001: "基础(常规)", 0x1003: "楼梯(常规)",
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
    """线程A 写、线程B 读的"当前 tag"。带时间戳供过期判断。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.tag_id = None
        self.px = 0.0
        self.ts = 0.0
        self.frames = 0
        self.hits = 0
        self.seen_counts = {}      # 本次运行各 id 命中次数

    def put(self, tag_id, px, now):
        with self.lock:
            self.tag_id, self.px, self.ts = tag_id, px, now
            self.frames += 1
            if tag_id is not None:
                self.hits += 1
                self.seen_counts[tag_id] = self.seen_counts.get(tag_id, 0) + 1

    def get(self):
        with self.lock:
            return self.tag_id, self.px, self.ts

    def stats(self):
        with self.lock:
            return self.frames, self.hits, dict(self.seen_counts)


def log_state(msg):
    """状态机自己的日志（和每秒的状态行区分开，带时间戳）。"""
    print(f"  [{time.strftime('%H:%M:%S')}] {msg}", flush=True)


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

    def __init__(self, node, det, shared, tag_size, min_px, hz):
        super().__init__(daemon=True, name="detector")
        self.node, self.det, self.shared = node, det, shared
        self.tag_size, self.min_px = tag_size, min_px
        self.period = 1.0 / hz
        self.running = True
        k = node.info.k
        self.fx, self.fy, self.cx0, self.cy0 = k[0], k[4], k[2], k[5]

    def run(self):
        next_t = time.monotonic()
        while self.running:
            if not self.node.frames:
                rclpy.spin_once(self.node, timeout_sec=0.05)
                continue
            msg = self.node.frames[-1]        # 只取最新，丢弃积压
            self.node.frames.clear()
            rclpy.spin_once(self.node, timeout_sec=0.0)

            bgr = imgmsg_to_bgr(msg)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            tid, px = detect_cmd_tag(self.det, gray, self.fx, self.fy,
                                     self.cx0, self.cy0, self.tag_size,
                                     self.min_px)
            self.shared.put(tid, px, time.monotonic())

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
                                    args.min_px, args.det_hz)
        det_thread.start()

        period = 1.0 / args.hz
        t_start = time.monotonic()
        next_t = t_start
        last_beat = last_print = 0.0
        cur_name = None
        # 步态切换子状态机（见循环里的 GAIT 分支）
        gait_phase = "idle"          # idle | settling | switching | done
        gait_t0 = 0.0
        gait_sent_t = 0.0

        while True:
            now = time.monotonic()
            if args.duration > 0 and now - t_start > args.duration:
                break

            tid, px, ts = shared.get()
            stale = (now - ts) > args.lost_timeout if ts > 0 else True
            if stale:
                tid = None

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
                                  f"（使用模式 {mode}）")
                        if mode == 1:
                            log_state("⚠️ 使用模式变成 1（导航）—— "
                                      "§1.2.5 比例轴指令已失效，其他 tag 全废！"
                                      "请切回 0x1001")
                    elif now - gait_t0 > args.gait_timeout:
                        gait_phase = "idle"
                        log_state(f"⚠️ {args.gait_timeout:.0f}s 内未切到 "
                                  f"{hex(tg)}（当前 {hex(link.gait or 0)}）"
                                  f"—— 下轮重试")
                if gait_phase == "done":
                    name = f"楼梯步态·已生效({hex(tg)})"

            # ---- 状态门 ----
            if link.motion_state is not None and link.motion_state != MOTION_RL:
                out = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                name = f"停止(非RL控制 {link.motion_state})"

            # ---- 心跳 / 收状态 ----
            if now - last_beat > 1.0:
                link.heartbeat()
                last_beat = now
            n = link.pump(0.004)
            if n:
                pass

            # ---- 每个 tick 都发（持续流式下发）----
            if args.go:
                link.send_axis(out["x"], out["yaw"], out["y"])
                sent += 1
            ticks += 1

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
