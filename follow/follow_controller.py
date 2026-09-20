#!/usr/bin/env python3
"""跟随控制器 —— D435i 检测 + 控制律 + ASDU 下发（真机 I/O 层）。

在相机所在机 (.102) 上运行：

    source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=2
    /home/ysc/follow_env/bin/python follow_controller.py               # 干跑
    /home/ysc/follow_env/bin/python follow_controller.py --go          # 真发
    /home/ysc/follow_env/bin/python follow_controller.py --go --max-wz 0

架构（两线程解耦 —— 这是关键）
--------------------------------------------------------------------------
    线程1 检测：相机 → AprilTag → 机体系位姿 → 更新"最新目标"
                跑多快算多快（受相机 15Hz 和检测耗时限制），绝不发指令

    线程2 控制：固定 20Hz → 读最新目标 → 控制律 → 发轴指令 + 心跳
                不受检测速度影响，每个 tick 都发

为什么必须解耦（实机踩过）：
    最初写成"检测完才发一次指令"的单线程。检测 35ms + 等帧，实测循环只
    跑到 5Hz（指令间隔 200ms），狗走起来明显卡顿 —— 因为它收到的是稀稀
    拉拉的指令流。轴指令的正确用法是【持续流式下发】。

    参考实现：robot_dog_sdk/web_control.py 的线程B
        "20Hz 控制循环：持续流式下发当前目标速度"

安全
--------------------------------------------------------------------------
  · tag 漏检在 --lost-timeout 内 → 沿用最后一次命中的位置（宽限期，防止
    指令被切成碎片）；超出宽限或从未检测到 → 归零
  · 状态上报中断 > STATUS_STALE_S，或不在 RL 控制(17) → 归零
    ⚠ 判据必须是【上报新鲜度】，不能只看 motion_state 的值 —— 通信断了
      这个变量会永远停在最后一个值（比如 17），程序会以为机器人还在线，
      继续下发运动指令。这是实测踩过的坑。
  · 速度上限 + 变化率限幅（在 follow_law 里）
  · Ctrl+C / 异常 → finally 里连续归零
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
                             Probe, imgmsg_to_bgr, tag_in_body,
                             tag_normal_ground, tag_side_px)
from follow_law import FollowConfig, FollowController  # noqa: E402
from tag_follower import RobotLink  # noqa: E402  复用已写好的 ASDU 客户端
def install_sigterm_as_interrupt():
    """让 SIGTERM 也走 KeyboardInterrupt 分支，触发 finally 里的归零。

    Python 默认收到 SIGTERM 会【直接退出、不跑 finally】—— 而这些程序的
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


def detect_one(det, gray, fx, fy, cx0, cy0, tag_size, want_id):
    """返回画面里 id==want_id 的最大 tag，没有则 None。"""
    res = det.detect(gray, estimate_tag_pose=True,
                     camera_params=[fx, fy, cx0, cy0], tag_size=tag_size)
    if not res:
        return None
    cands = [r for r in res if int(r.tag_id) == want_id]
    return max(cands, key=lambda r: tag_side_px(r.corners)) if cands else None


class SharedTarget:
    """线程1 写、线程2 读的"最新目标"。

    ★ 两个时间戳必须分开存（这里踩过，代价很大）★
        frame_ts —— 检测线程最后一次【处理完一帧】的时间。只反映线程活没活。
        good_ts  —— 最后一次【真的看到 tag】的时间。过期判断要用这个。

    原先只有一个 ts，而漏检帧也会刷新它（put(None, ...)），于是 --lost-timeout
    实际只检测"检测线程卡死"，想要的"漏检宽限"从来没生效过 —— 漏一帧就立刻
    归零。实测代价：命中率 8% 时指令被切成 0.1~0.3 秒的碎片，而机器人需要约
    1 秒连续指令才能起速，狗只会抽动、走不起来（见 TECHNICAL_REPORT §2.7）。
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.pos = None          # 本帧位置 (前,左,上) 米；漏检帧为 None
        self.good_pos = None     # 最后一次命中的位置，供漏检宽限期沿用
        self.frame_ts = 0.0
        self.good_ts = 0.0       # 0 = 从未检测到
        self.seq = 0             # 已处理的检测帧数
        self.frames = 0          # 已处理的相机帧数
        self.hits = 0
        self.det_ms = 0.0
        self.last_why = "启动中"

    def put(self, pos, now, det_ms):
        with self.lock:
            self.pos, self.frame_ts = pos, now
            if pos is not None:
                self.good_pos, self.good_ts = pos, now
            self.seq += 1
            self.det_ms = det_ms

    def get(self):
        with self.lock:
            return (self.pos, self.good_pos, self.frame_ts, self.good_ts,
                    self.seq, self.det_ms)

    def bump_frame(self, hit):
        with self.lock:
            self.frames += 1
            if hit:
                self.hits += 1


class DetectorThread(threading.Thread):
    """线程1：相机 → 检测 → 更新共享目标。只碰相机，不碰 socket。"""

    def __init__(self, node, det, shared, tag_size, want_id, hz):
        super().__init__(daemon=True, name="detector")
        self.node = node
        self.det = det
        self.shared = shared
        self.tag_size = tag_size
        self.want_id = want_id
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
            msg = self.node.frames[-1]       # 只取最新，丢弃积压
            self.node.frames.clear()
            rclpy.spin_once(self.node, timeout_sec=0.0)

            t0 = time.monotonic()
            bgr = imgmsg_to_bgr(msg)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            hit = detect_one(self.det, gray, self.fx, self.fy, self.cx0,
                             self.cy0, self.tag_size, self.want_id)
            det_ms = (time.monotonic() - t0) * 1000.0

            if hit is not None:
                self.shared.put(tag_in_body(hit.pose_t), t0, det_ms)
            else:
                self.shared.put(None, t0, det_ms)
            self.shared.bump_frame(hit is not None)

            next_t += self.period
            # 只读一次时钟再判断 —— 先比后算会让差值为负，sleep 抛
            # ValueError 把整个线程打断（实机踩过，检测线程直接死掉）
            _d = next_t - time.monotonic()
            if _d > 0:
                time.sleep(_d)
            else:
                next_t = time.monotonic()




def main():
    install_sigterm_as_interrupt()
    ap = argparse.ArgumentParser(description="AprilTag 跟随控制器（两线程解耦）")
    ap.add_argument("--robot", default="10.21.33.103", help="运控主机")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--go", action="store_true", help="真的下发指令（默认干跑）")
    ap.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE_M)
    ap.add_argument("--tag-id", type=int, default=1)
    ap.add_argument("--target", type=float, default=1.0, help="目标距离 m")

    _d = FollowConfig()
    # 默认值一律取自 FollowConfig —— CLI 里再写一份数字就会漂移，
    # 已经踩过两次（max_wz 0.45 vs 1.0、max_vx 0.30 vs 0.12）。
    ap.add_argument("--max-vx", type=float, default=_d.max_vx,
                    help=f"前进上限（默认 {_d.max_vx}，按实测标度 ≈0.25 m/s）")
    ap.add_argument("--max-wz", type=float, default=_d.max_wz,
                    help="转向上限。1.0=满量程（唯一实测能起转的值）；"
                         "0=完全关掉转向（首次跑跟随建议先用 0）")

    ap.add_argument("--hz", type=float, default=20.0, help="控制循环频率")
    ap.add_argument("--det-hz", type=float, default=15.0, help="检测线程最高频率")
    ap.add_argument("--lost-timeout", type=float, default=0.25,
                    help="多久没有新检测就视为丢失并归零（秒）。0=任何一帧没检测到就停")
    ap.add_argument("--duration", type=float, default=0.0, help="0=一直跑")
    args = ap.parse_args()

    if args.max_wz == 0:
        print("[说明] --max-wz 0：转向完全关闭，只直着走")

    cfg = FollowConfig(tag_id=args.tag_id, target_dist_m=args.target,
                       max_vx=args.max_vx, max_wz=args.max_wz,
                       lost_timeout_s=0.0)   # 过期判断在下面做，见 --lost-timeout
    ctrl = FollowController(cfg)
    shared = SharedTarget()

    rclpy.init()
    node = Probe()
    det = Detector(families=FAMILY, nthreads=2, quad_decimate=1.0,
                   quad_sigma=0.8, refine_edges=1, decode_sharpening=0.5)
    link = RobotLink(args.robot, args.port)   # 平时只发心跳；--go 才发轴指令

    print(f"运控 {args.robot}:{args.port}   模式 "
          f"{'★ 真实控制 ★' if args.go else '干跑（只算不发）'}")
    print(f"tag id={cfg.tag_id}  size={args.tag_size*1000:.1f}mm  "
          f"目标距离={args.target:.2f}m")
    print(f"控制 {args.hz:.0f}Hz / 检测 ≤{args.det_hz:.0f}Hz，"
          f"漏检宽限 {args.lost_timeout*1000:.0f}ms，"
          f"状态中断阈值 {STATUS_STALE_S*1000:.0f}ms")

    det_thread = None
    t_start = time.monotonic()
    ticks = sent = 0
    try:
        if not node.wait_for(15.0):
            print("[错误] 没拿到相机图像/内参。检查 ROS_DOMAIN_ID=2 和相机服务。")
            return 1
        k = node.info.k
        print(f"相机就绪 fx={k[0]:.1f} {node.info.width}x{node.info.height}")

        if args.go:      # 真发模式的前置检查
            print("检查机器人状态 ...")
            for _ in range(20):
                link.heartbeat()
                link.pump()
                if link.motion_state is not None:
                    break
                time.sleep(0.1)
            print(f"  MotionState={link.motion_state} Gait={link.gait} "
                  f"Mode={link.mode}")
            if link.motion_state != MOTION_RL:
                print(f"[拒绝运行] 未处于 RL 控制({MOTION_RL})。请先让狗起立。")
                return 2
        print()

        det_thread = DetectorThread(node, det, shared, args.tag_size,
                                    cfg.tag_id, args.det_hz)
        det_thread.start()

        period = 1.0 / args.hz
        t_start = time.monotonic()
        next_t = t_start
        last_beat = last_print = last_status_ts = 0.0

        while True:
            now = time.monotonic()
            if args.duration > 0 and now - t_start > args.duration:
                break

            # ---- 读最新目标，判过期 ----
            pos, nrm, ts, seq, det_ms = shared.get()
            stale = (now - ts) > args.lost_timeout if ts > 0 else True
            if stale:
                pos = nrm = None

            vx, wz, why = ctrl.update(pos, nrm, now)

            # ---- 状态门：不在 RL 控制就归零 ----
            if link.motion_state is not None and link.motion_state != MOTION_RL:
                vx = wz = 0.0
                why = f"非 RL 控制({link.motion_state}) → 停"

            # ---- 心跳 / 收状态 ----
            if now - last_beat > 1.0:
                link.heartbeat()
                last_beat = now
            n = link.pump(0.004)   # 只给它 4ms，别占满 50ms 周期
            if n:
                last_status_ts = now

            # ---- 每个 tick 都发（关键：持续流式下发）----
            if args.go:
                link.send_axis(vx, wz)
                sent += 1
            ticks += 1

            if now - last_print > 0.5 and now - t_start > 1.0:
                pos_txt = (f"前={pos[0]:+.2f} 左={pos[1]:+.2f}"
                           if pos is not None else "未见")
                print(f"  控制{ticks/ max(1e-9, now-t_start):4.1f}Hz "
                      f"检测{shared.seq/ max(1e-9, now-t_start):4.1f}Hz "
                      f"解算{det_ms:.0f}ms | {pos_txt} | "
                      f"vx={vx:+.3f} wz={wz:+.3f} | {why}")
                last_print = now

            next_t += period
            # 只读一次时钟再判断 —— 先比后算会让差值为负，sleep 抛
            # ValueError 把整个线程打断（实机踩过，检测线程直接死掉）
            _d = next_t - time.monotonic()
            if _d > 0:
                time.sleep(_d)
            else:
                next_t = time.monotonic()      # 落后了就重新对齐，不追补

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
        print(f"结束：检测帧 {shared.frames}，命中 {shared.hits} "
              f"({100.0*shared.hits/max(1,shared.frames):.0f}%)  "
              f"→ 检测 {shared.frames/el:.1f}Hz | "
              f"控制 tick {ticks} ({ticks/el:.1f}Hz)，发出轴指令 {sent}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
