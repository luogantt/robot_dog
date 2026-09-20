#!/usr/bin/env python3
"""D435i (ROS2) 抓帧 + AprilTag 检测 + 真实位姿解算 —— 调试跟随算法用。

在相机所在机器 (.102) 上运行：

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=2
    /home/ysc/follow_env/bin/python d435i_tag_probe.py --save /tmp/tag_probe

与 tag_follower.py 的区别
--------------------------------------------------------------------------
tag_follower.py 走的是 /dev/videoN 直读鱼眼相机，靠"像宽反比 d=K/px"
估距离 —— 因为那颗相机没有标定文件，针孔模型不成立。

D435i 出厂自带标定 (camera_info 里有 fx/fy/cx/cy，畸变 D 全零)，所以这里
直接用 dt_apriltags 的位姿解算拿真实 3D 距离（单位：米），不需要任何标定。

不依赖 cv_bridge：按 encoding 手工解 Image，少一层依赖就少一个坑。
"""

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from dt_apriltags import Detector

FAMILY = "tagStandard41h12"

# tag 黑框边长（米），即【黑色外框外沿】的边长 —— 不是纸张尺寸，也不是
# 白色静区的尺寸。这个值错 2 倍，距离就错 2 倍。
#
# 标定历史（这个值踩过坑，别凭标称尺寸填）：
#   第一版打印：标称 10cm，用 --calibrate 放 0.60m 处反算 —— 实测 59.6mm，
#               只有标称的 0.596 倍（静区 + 打印缩放导致的）。按 10cm 算时
#               距离读数整体偏大一倍。
#   当前版本：用户按 20cm 打印。
#
# 【重要】状态机 tag_command_fsm.py 只用 tag 的 id，不用距离，所以这个值
# 不影响它。只有 follow_controller.py 的跟随距离依赖它。
# 换 tag 后想精确，跑一次标定：
#   d435i_tag_monitor.py --calibrate <卷尺量准的距离> --duration 10
DEFAULT_TAG_SIZE_M = 0.20

# ---------------------------------------------------------------- 相机外参
#
# 机体坐标系：原点 = 机体几何中心，X 朝前 / Y 朝左 / Z 朝上（右手系）
# 相机安装：位置 (315, 0, 100) mm，roll = yaw = 0，pitch 低头 35°（固定角）
#
# 相机光学系是 ROS 光学约定：x 右 / y 下 / z 前。
#
# dt_apriltags 的 pose_t 就是 tag 在【光学系】里的位置（米）。

CAM_MOUNT_XYZ_M = (0.315, 0.0, 0.100)
CAM_PITCH_DEG = 35.0


def cam_to_body_R(pitch_deg=CAM_PITCH_DEG):
    """相机光学系 → 机体坐标系的旋转矩阵（列 = 相机三轴在机体系下的表示）。

    推导：
      未俯仰时  Z_c=+X_body(前)  X_c=-Y_body(右)  Y_c=-Z_body(下)
      机身低头 θ 即绕相机 X_c 轴转 θ，于是
        Z_c = (cosθ, 0, -sinθ)     光轴指向前下方
        X_c = (0, -1, 0)           右
        Y_c = Z_c × X_c = (-sinθ, 0, -cosθ)
    """
    th = math.radians(pitch_deg)
    c, s = math.cos(th), math.sin(th)
    return np.array([[0.0, -s, c],
                     [-1.0, 0.0, 0.0],
                     [0.0, -c, -s]])


def tag_in_body(pose_t, mount=CAM_MOUNT_XYZ_M, pitch_deg=CAM_PITCH_DEG):
    """tag 在相机光学系下的位置 → 机体坐标系下的位置（米）。

    返回 (X前, Y左, Z上)，原点为机体几何中心。
    """
    p_cam = np.asarray(pose_t, dtype=float).reshape(3)
    return np.asarray(mount, dtype=float) + cam_to_body_R(pitch_deg) @ p_cam


# ---------------------------------------------------------------- 图像解码

def imgmsg_to_bgr(msg):
    """sensor_msgs/Image → BGR ndarray。不依赖 cv_bridge。"""
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()

    if enc in ("rgb8", "bgr8"):
        arr = buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        return arr[:, :, ::-1].copy() if enc == "rgb8" else arr.copy()
    if enc in ("mono8", "8uc1"):
        g = buf.reshape(h, step)[:, :w]
        return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"未处理的 encoding: {msg.encoding}")


# ---------------------------------------------------------------- 采集节点

class Probe(Node):
    """抓若干帧，同时拿到 camera_info。"""

    def __init__(self):
        super().__init__("d435i_tag_probe")
        self.qos = qos_profile_sensor_data
        self.frames = []
        self.info = None
        self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.on_img, self.qos)
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.on_info, self.qos)

    def on_img(self, msg):
        self.frames.append(msg)

    def on_info(self, msg):
        self.info = msg

    def wait_for(self, timeout=10.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.frames and self.info is not None:
                return True
        return False


# ---------------------------------------------------------------- 主逻辑

def tag_side_px(corners):
    """tag 四角的最大边长（像素）。"""
    c = corners
    return max(float(np.linalg.norm(c[i] - c[(i + 1) % 4])) for i in range(4))


def main():
    ap = argparse.ArgumentParser(description="D435i AprilTag 探针")
    ap.add_argument("--frames", type=int, default=40, help="采集帧数")
    ap.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE_M,
                    help=f"tag 黑框实际边长（米）。默认 {DEFAULT_TAG_SIZE_M}（已标定）")
    ap.add_argument("--save", default="", help="保存目录（存标注图 + 原始帧）")
    ap.add_argument("--quiet", action="store_true", help="只打印汇总")
    args = ap.parse_args()

    if args.save:
        os.makedirs(args.save, exist_ok=True)

    rclpy.init()
    node = Probe()
    try:
        print("等待相机 ...", flush=True)
        if not node.wait_for(15.0):
            print("[错误] 15s 内没拿到图像/内参。检查：")
            print("       systemctl is-active realsense-camera.service")
            print("       echo $ROS_DOMAIN_ID   # 应为 2")
            return 1

        k = node.info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        print(f"内参 fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
        print(f"畸变 D={list(node.info.d)[:5]}  "
              f"分辨率 {node.info.width}x{node.info.height}")

        det = Detector(families=FAMILY, nthreads=2, quad_decimate=1.0,
                       quad_sigma=0.8, refine_edges=1, decode_sharpening=0.5)
        print(f"检测器 {FAMILY} 已就绪，采 {args.frames} 帧 ...\n")

        hits, results, last_annot = 0, [], None
        t_start = time.monotonic()

        while len(node.frames) < args.frames and \
                time.monotonic() - t_start < 30.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        n = len(node.frames)
        for i, msg in enumerate(node.frames):
            bgr = imgmsg_to_bgr(msg)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            t0 = time.monotonic()
            res = det.detect(gray, estimate_tag_pose=True,
                             camera_params=[fx, fy, cx, cy],
                             tag_size=args.tag_size)
            dt_ms = (time.monotonic() - t0) * 1000.0

            if res:
                hits += 1
                r = max(res, key=lambda r: tag_side_px(r.corners))
                pxw = tag_side_px(r.corners)
                t = r.pose_t.flatten()          # [x, y, z] 米
                results.append({
                    "frame": i, "id": int(r.tag_id), "px_width": pxw,
                    "cx": float(r.center[0]), "cy": float(r.center[1]),
                    "x": float(t[0]), "y": float(t[1]), "z": float(t[2]),
                    "det_ms": dt_ms,
                })
                if not args.quiet:
                    print(f"  [{i:3d}] id={r.tag_id}  像宽={pxw:6.1f}px  "
                          f"距离={t[2]:.3f}m  横向={t[0]:+.3f}m  "
                          f"垂直={t[1]:+.3f}m  中心=({r.center[0]:.0f},"
                          f"{r.center[1]:.0f})  {dt_ms:.1f}ms")

                # 标注（保留最后一帧）
                ann = bgr.copy()
                cv2.polylines(ann, [r.corners.astype(int)], True, (0, 255, 0), 2)
                cv2.circle(ann, tuple(r.center.astype(int)), 4, (0, 0, 255), -1)
                # 画面中心十字 —— 看 tag 偏了多少
                cv2.drawMarker(ann, (int(node.info.width / 2),
                                     int(node.info.height / 2)),
                               (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.line(ann, tuple(r.center.astype(int)),
                         (int(node.info.width / 2), int(node.info.height / 2)),
                         (0, 255, 255), 1)
                cv2.putText(ann, f"id={r.tag_id} {t[2]:.2f}m px={pxw:.0f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0), 2)
                cv2.putText(ann, f"x={t[0]:+.2f} y={t[1]:+.2f}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 1)
                last_annot = ann
            elif not args.quiet:
                print(f"  [{i:3d}] 未检测到")

        # ---- 汇总 ----
        print(f"\n{'='*62}")
        print(f"检测率 {hits}/{n} = {100.0*hits/max(1,n):.0f}%    "
              f"tag_size 假设 = {args.tag_size*1000:.0f}mm")
        if results:
            pxw = np.array([r["px_width"] for r in results])
            z = np.array([r["z"] for r in results])
            ct = np.array([r["cx"] for r in results])
            det_ms = np.array([r["det_ms"] for r in results])
            print(f"像宽   {pxw.mean():.1f} ± {pxw.std():.1f} px")
            print(f"距离   {z.mean():.3f} ± {z.std():.3f} m  "
                  f"[{z.min():.3f}, {z.max():.3f}]")
            print(f"横向   {np.array([r['x'] for r in results]).mean():+.3f} m")
            print(f"tag 中心 x 像素 {ct.mean():.1f}  (画面中心 "
                  f"{node.info.width/2:.0f} → 偏差 "
                  f"{ct.mean() - node.info.width/2:+.1f}px)")
            print(f"检测耗时 {det_ms.mean():.1f} ± {det_ms.std():.1f} ms")
            print(f"\n距离随 tag 实际尺寸线性缩放（当前按 "
                  f"{args.tag_size*1000:.0f}mm 算）：")
            for s in (50, 100, 150, 200):
                print(f"    若 tag 实际 {s:3d}mm → 距离 {z.mean()*s/(args.tag_size*1000):.3f} m")
            if len(results) > 1:
                print("\n逐帧距离: "
                      + ", ".join(f"{r['z']:.2f}" for r in results[:12]))
        else:
            print("始终未检测到 tag。已保存原始帧 —— 看画面里到底有什么。")

        if args.save:
            if last_annot is not None:
                p = os.path.join(args.save, "annotated.jpg")
                cv2.imwrite(p, last_annot)
                print(f"\n标注图 → {p}")
            if node.frames:
                raw = imgmsg_to_bgr(node.frames[-1])
                p = os.path.join(args.save, "raw.jpg")
                cv2.imwrite(p, raw)
                print(f"原始帧 → {p}")
                print(f"画面尺寸 {raw.shape[1]}x{raw.shape[0]}  "
                      f"均值亮度 {raw.mean():.1f}  "
                      f"最亮 {raw.max()} 最暗 {raw.min()}")
            p = os.path.join(args.save, "result.json")
            with open(p, "w") as f:
                json.dump({"intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
                           "tag_size_m": args.tag_size,
                           "frames": n, "hits": hits,
                           "detections": results}, f, indent=2)
            print(f"结果 → {p}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
