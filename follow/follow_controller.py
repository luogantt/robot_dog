#!/usr/bin/env python3
"""跟随控制器 —— D435i 检测 + 控制律 + ASDU 下发（真机 I/O 层）。

在相机所在机 (.102) 上运行：

    source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=2
    /home/ysc/follow_env/bin/python follow_controller.py                # 干跑，只算不发
    /home/ysc/follow_env/bin/python follow_controller.py --go           # 真发指令
    /home/ysc/follow_env/bin/python follow_controller.py --lost-timeout 0.3

控制律本身在 follow_law.py（纯逻辑，可离线单元测试）。本文件只负责：
相机 → 检测 → 机体系变换 → 交给控制律 → ASDU 下发 → 安全兜底。

安全
--------------------------------------------------------------------------
  · 看不见 tag / 已在目标距离内 → 零输出（见 follow_law）
  · 真发模式要求 MotionState=17
  · 速度上限 + 变化率限幅（在 follow_law 里）
  · Ctrl+C / 异常 → finally 里归零
"""

import argparse
import os
import sys
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

MOTION_RL = 17


def detect_one(det, gray, fx, fy, cx0, cy0, tag_size, want_id):
    """返回画面里 id==want_id 的最大 tag，没有则 None。"""
    res = det.detect(gray, estimate_tag_pose=True,
                     camera_params=[fx, fy, cx0, cy0], tag_size=tag_size)
    if not res:
        return None
    cands = [r for r in res if int(r.tag_id) == want_id]
    return max(cands, key=lambda r: tag_side_px(r.corners)) if cands else None


def main():
    ap = argparse.ArgumentParser(description="AprilTag 跟随控制器")
    ap.add_argument("--robot", default="10.21.33.103", help="运控主机")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--go", action="store_true", help="真的下发指令（默认干跑）")
    ap.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE_M)
    ap.add_argument("--tag-id", type=int, default=1)
    ap.add_argument("--target", type=float, default=1.0, help="目标距离 m")
    ap.add_argument("--max-vx", type=float, default=0.30)
    ap.add_argument("--max-wz", type=float, default=0.45)
    ap.add_argument("--lost-timeout", type=float, default=0.0,
                    help="0=丢一帧即停（严格）；>0=宽限期内保持上条指令")
    ap.add_argument("--duration", type=float, default=0.0, help="0=一直跑")
    ap.add_argument("--hz", type=float, default=15.0)
    args = ap.parse_args()

    cfg = FollowConfig(tag_id=args.tag_id, target_dist_m=args.target,
                       max_vx=args.max_vx, max_wz=args.max_wz,
                       lost_timeout_s=args.lost_timeout)
    ctrl = FollowController(cfg)

    rclpy.init()
    node = Probe()
    det = Detector(families=FAMILY, nthreads=2, quad_decimate=1.0,
                   quad_sigma=0.8, refine_edges=1, decode_sharpening=0.5)
    link = RobotLink(args.robot, args.port)   # 平时只发心跳；--go 才发轴指令

    print(f"运控 {args.robot}:{args.port}   模式 "
          f"{'★ 真实控制 ★' if args.go else '干跑（只算不发）'}")
    print(f"tag id={cfg.tag_id}  size={args.tag_size*1000:.1f}mm  "
          f"目标距离={args.target:.2f}m  "
          f"丢失策略={'严格' if args.lost_timeout <= 0 else f'宽限{args.lost_timeout}s'}")

    try:
        if not node.wait_for(15.0):
            print("[错误] 没拿到相机图像/内参。检查 ROS_DOMAIN_ID=2 和相机服务。")
            return 1
        k = node.info.k
        fx, fy, cx0, cy0 = k[0], k[4], k[2], k[5]
        print(f"相机就绪 fx={fx:.1f} {node.info.width}x{node.info.height}")

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

        period = 1.0 / args.hz
        t_start = time.monotonic()
        next_t = t_start
        last_beat = last_print = 0.0
        frames = hits = 0

        while True:
            now = time.monotonic()
            if args.duration > 0 and now - t_start > args.duration:
                break

            if not node.frames:          # 等新帧
                rclpy.spin_once(node, timeout_sec=0.05)
                continue
            msg = node.frames[-1]        # 只取最新，丢弃积压
            node.frames.clear()
            rclpy.spin_once(node, timeout_sec=0.0)

            bgr = imgmsg_to_bgr(msg)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            hit = detect_one(det, gray, fx, fy, cx0, cy0, args.tag_size,
                             cfg.tag_id)
            frames += 1

            if hit is not None:
                pos = tag_in_body(hit.pose_t)
                nrm = tag_normal_ground(hit.pose_R)
                hits += 1
            else:
                pos = nrm = None

            vx, wz, why = ctrl.update(pos, nrm, now)

            if now - last_beat > 1.0:    # 心跳 >=1Hz，§1.2.1
                link.heartbeat()
                last_beat = now
            link.pump()

            if args.go:
                link.send_axis(vx, wz)

            if now - last_print > 0.33:
                if pos is not None:
                    print(f"  前={pos[0]:+.2f} 左={pos[1]:+.2f} "
                          f"法向={'有' if nrm is not None else '无'} | "
                          f"vx={vx:+.3f} wz={wz:+.3f} | {why}")
                else:
                    print(f"  未检测到 id={cfg.tag_id} | vx=0 wz=0 | {why}")
                last_print = now

            next_t += period
            if next_t <= time.monotonic():
                next_t = time.monotonic()
            else:
                time.sleep(next_t - time.monotonic())

    except KeyboardInterrupt:
        print("\n[中断]")
    finally:
        print("归零中 ...")
        if args.go:
            link.stop()
        link.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"结束：{frames} 帧，命中 {hits} "
              f"({100.0*hits/max(1,frames):.0f}%)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
