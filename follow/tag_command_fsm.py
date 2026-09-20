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

    没检测到任何指令 tag  → 视为"停止"（看不见就不动）

架构（沿用今天验证过的两线程解耦）
--------------------------------------------------------------------------
    线程A 检测：相机 15Hz → 检测所有 tag → 更新"当前 tag id"
    线程B 控制：固定 20Hz → 查动作表 → 发轴指令 + 心跳

    实机教训：单线程"检测完才发一次指令"只能跑到 5Hz，狗走起来明显卡顿。
    轴指令必须【持续流式下发】。

安全
--------------------------------------------------------------------------
  · tag 过期（--lost-timeout 内没有新检测）→ 停止
  · 未检测到指令 tag → 停止
  · 不在 RL 控制(17) / 有 FATAL 故障 → 停止
  · Ctrl+C / 异常 → finally 连续归零
  · 速度上限硬钳制，且【被钳制就拒绝启动】而不是静默改写
"""

import argparse
import os
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

MOTION_RL = 17

# tag id → (动作名, 轴, 符号)。轴取值 'x' / 'y' / 'yaw'，None 表示停止。
# 幅度在下面按轴分别给（单位是比例量 [-1,1]）。
TAG_ACTIONS = {
    1:  ("前进",      "x",   +1.0),
    18: ("后退",      "x",   -1.0),
    3:  ("左转",      "yaw", +1.0),
    61: ("右转",      "yaw", -1.0),
    63: ("左侧移动",  "y",   +1.0),
    68: ("右侧移动",  "y",   -1.0),
    6:  ("停止",      None,   0.0),
}

# 各轴的默认幅度（比例量）。依据今天实测：
#   X  —— 前进/后退。实测响应偏弱，给到 0.3
#   Y  —— 侧移。实测 0.15 无反应、0.5 有反应，默认 0.5
#   Yaw—— 转向。实测 0.5/0.8 几乎不转、1.0 能转 180°/2s，【只有满量程可用】
DEFAULT_MAG = {"x": 0.30, "y": 0.50, "yaw": 1.00}

# 各轴硬上限。超了就拒绝启动 —— 静默钳制会让你以为在测 A、实际发的是 B
# （这个坑今天踩过两次）。
HARD_LIMIT = {"x": 0.60, "y": 0.80, "yaw": 1.00}


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
    ap = argparse.ArgumentParser(description="Tag 指令状态机")
    ap.add_argument("--robot", default="10.21.33.103")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--go", action="store_true", help="真的下发指令")
    ap.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE_M)
    ap.add_argument("--hz", type=float, default=20.0, help="控制循环频率")
    ap.add_argument("--det-hz", type=float, default=15.0)
    ap.add_argument("--lost-timeout", type=float, default=0.25,
                    help="多久没有新检测就视为丢失并停止（秒）")
    ap.add_argument("--min-px", type=float, default=20.0,
                    help="tag 最小像宽，挡掉远处小 tag 的误检")
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
          f"过期阈值 {args.lost_timeout*1000:.0f}ms")
    print("动作表：")
    for tid in sorted(TAG_ACTIONS):
        name, axis, s = TAG_ACTIONS[tid]
        if axis is None:
            print(f"   id={tid:<3d} {name:<8s} 全零")
        else:
            eff = s * sign[axis] * mag[axis]
            print(f"   id={tid:<3d} {name:<8s} {axis} = {eff:+.3f}"
                  f"   (幅度 {mag[axis]} × 符号 {sign[axis]:+.0f})")

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
                name, axis, s = "停止(无tag)", None, 0.0
            else:
                name, axis, s = TAG_ACTIONS[tid]

            out = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            if axis is not None:
                out[axis] = s * sign[axis] * mag[axis]

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
                      f"tag={tid if tid is not None else '无'} | "
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
        node.destroy_node()
        rclpy.shutdown()
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
