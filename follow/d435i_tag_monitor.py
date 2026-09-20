#!/usr/bin/env python3
"""D435i AprilTag 实时监视器 —— 调试跟随算法用。

    源 /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=2
    /home/ysc/follow_env/bin/python d435i_tag_monitor.py --tag-size 0.1 --duration 120

每秒打一行摘要，可选把每一帧写 CSV 供事后分析：

    ... d435i_tag_monitor.py --csv /tmp/tag_probe/trace.csv --duration 60

关于 tag-size
--------------------------------------------------------------------------
它必须是 tag【黑色外框的边长】（米），不是纸张尺寸，也不是白色留白后的
尺寸。这个值错 2 倍，距离就错 2 倍。本项目的实测值是 0.0596（见
DEFAULT_TAG_SIZE_M）。

拿不准时用 --calibrate：把 tag 放到卷尺量准的已知距离上，它会反算真实
tag 尺寸 —— 比查打印设置可靠。
"""

import argparse
import csv
import math
import os
import sys
import time

import cv2
import numpy as np

import rclpy

from dt_apriltags import Detector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d435i_tag_probe import (DEFAULT_TAG_SIZE_M, FAMILY,  # noqa: E402
                             Probe, imgmsg_to_bgr, tag_in_body,
                             tag_side_px)


def make_detector():
    return Detector(families=FAMILY, nthreads=2, quad_decimate=1.0,
                    quad_sigma=0.8, refine_edges=1, decode_sharpening=0.5)


def main():
    ap = argparse.ArgumentParser(description="D435i AprilTag 实时监视器")
    ap.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE_M,
                    help=f"tag 黑框边长（米）。默认 {DEFAULT_TAG_SIZE_M}（已标定）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="运行秒数，0 = 一直跑（Ctrl+C 停）")
    ap.add_argument("--csv", default="", help="把每帧结果写到这里")
    ap.add_argument("--calibrate", type=float, default=0.0,
                    help="已知真实距离（米）。给定时反算 tag 真实边长")
    ap.add_argument("--save", default="", help="标注图保存目录")
    args = ap.parse_args()

    if args.save:
        os.makedirs(args.save, exist_ok=True)
    if args.calibrate > 0:
        print(f"★ 标定模式：请把 tag 放在距镜头 {args.calibrate:.2f} m 处别动\n")

    rclpy.init()
    node = Probe()
    det = make_detector()
    csv_f = csv_w = None
    if args.csv:
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        csv_f = open(args.csv, "w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow(["t", "id", "px_width", "dist_m", "x_m", "y_m",
                        "cx", "cy", "det_ms"])

    try:
        print("等待相机 ...", flush=True)
        if not node.wait_for(15.0):
            print("[错误] 没拿到图像/内参。检查 ROS_DOMAIN_ID=2 和相机服务。")
            return 1

        k = node.info.k
        fx, fy, cx0, cy0 = k[0], k[4], k[2], k[5]
        print(f"内参 fx={fx:.2f} fy={fy:.2f} cx={cx0:.2f} cy={cy0:.2f}   "
              f"{node.info.width}x{node.info.height}   "
              f"tag_size={args.tag_size*1000:.0f}mm\n")

        tag_size = args.tag_size
        calib_z_sum, calib_n = 0.0, 0
        res, r, pxw, t = [], None, 0.0, np.zeros(3)
        t_start = time.monotonic()
        t_print = 0.0
        n, hits = 0, 0
        seen_frames = 0
        last_bgr = None

        while True:
            now = time.monotonic()
            if args.duration > 0 and now - t_start > args.duration:
                break

            # 取最新一帧（丢弃积压，保证看的是当前时刻）
            if not node.frames:
                rclpy.spin_once(node, timeout_sec=0.1)
                continue
            msg = node.frames[-1]
            node.frames.clear()
            rclpy.spin_once(node, timeout_sec=0.0)

            bgr = imgmsg_to_bgr(msg)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            t0 = time.monotonic()
            res = det.detect(gray, estimate_tag_pose=True,
                             camera_params=[fx, fy, cx0, cy0],
                             tag_size=tag_size)
            dt_ms = (time.monotonic() - t0) * 1000.0
            n += 1

            if res:
                r = max(res, key=lambda rr: tag_side_px(rr.corners))
                pxw = tag_side_px(r.corners)
                t = r.pose_t.flatten()
                tb = tag_in_body(t)          # 机体系：X前 / Y左 / Z上
                hits += 1
                seen_frames = 0
                last_bgr = bgr
                if csv_w:
                    csv_w.writerow([f"{now - t_start:.3f}", int(r.tag_id),
                                    f"{pxw:.2f}", f"{t[2]:.4f}", f"{t[0]:.4f}",
                                    f"{t[1]:.4f}", f"{r.center[0]:.1f}",
                                    f"{r.center[1]:.1f}", f"{dt_ms:.1f}"])
                if args.calibrate > 0:
                    # 距离 ∝ 假定尺寸，故 真实尺寸 = 假定尺寸 × 真实距离 / 测得距离
                    calib_z_sum += float(t[2])
                    calib_n += 1
                    if calib_n % 15 == 0:
                        real = tag_size * args.calibrate / (calib_z_sum / calib_n)
                        print(f"   反算 tag 真实边长 ≈ {real*1000:.1f} mm   "
                              f"(测得 {calib_z_sum/calib_n:.3f} m，"
                              f"假定 {tag_size*1000:.0f}mm)")
            else:
                seen_frames += 1

            if now - t_print >= 1.0:
                if res:
                    t = r.pose_t.flatten()
                    print(f"[{now - t_start:6.1f}s] id={r.tag_id} "
                          f"像宽={pxw:5.1f}px  光轴距={t[2]:.3f}m | "
                          f"机体: 前={tb[0]:+.3f} 左={tb[1]:+.3f} "
                          f"上={tb[2]:+.3f} m   {dt_ms:.0f}ms")
                elif seen_frames > 0:
                    print(f"[{now - t_start:6.1f}s] 未检测到 "
                          f"(已连续 {seen_frames} 帧丢失)")
                t_print = now

        print(f"\n{'='*60}")
        print(f"共 {n} 帧，检测 {hits} 次 = {100.0*hits/max(1,n):.0f}%")

        if args.calibrate > 0 and calib_n:
            z_bar = calib_z_sum / calib_n
            print(f"标定结论：tag 真实边长 ≈ "
                  f"{tag_size * args.calibrate / z_bar * 1000:.1f} mm")
            print(f"（依据：假定 {tag_size*1000:.0f}mm 时测得 {z_bar:.3f}m，"
                  f"实际放在 {args.calibrate:.2f}m；共 {calib_n} 帧）")
            print(f"→ 之后请用 --tag-size "
                  f"{tag_size * args.calibrate / z_bar:.4f} 重跑")

        if args.save and last_bgr is not None and res:
            ann = last_bgr.copy()
            cv2.polylines(ann, [r.corners.astype(int)], True, (0, 255, 0), 2)
            cv2.putText(ann, f"id={r.tag_id} {r.pose_t[2][0]:.2f}m",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            p = os.path.join(args.save, "monitor_last.jpg")
            cv2.imwrite(p, ann)
            print(f"最后标注图 → {p}")
        if args.csv:
            print(f"逐帧数据 → {args.csv}")
        return 0
    except KeyboardInterrupt:
        print("\n[中断]")
        return 0
    finally:
        if csv_f:
            csv_f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
