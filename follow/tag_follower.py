#!/usr/bin/env python3
"""
机器狗 AprilTag 追踪 —— 让狗跟着 tag 走，保持在 1 米距离。

在【机器狗本体】上运行（10.21.33.103）。

    python3 tag_follower.py --camera 6              # 干跑：只检测不发指令
    python3 tag_follower.py --camera 6 --go         # 真的控制（谨慎）
    python3 tag_follower.py --camera 6 --calib      # 测距标定模式

控制通道：ASDU UDP → 127.0.0.1:30004（复用 robot_dog_sdk 的协议层，
已实测可用；且机器狗自己的 golai 遥控也是走这条）。

--------------------------------------------------------------------------
设计要点
--------------------------------------------------------------------------
1) 距离怎么算 —— 不用针孔模型，用"像宽反比"
   相机是鱼眼镜头，针孔模型误差大，而且机器狗上没有标定文件。
   所以用最稳的经验关系：

        distance ≈ K / tag_pixel_width

   K 用 --calib 在已知距离上测一次即可。这个关系对鱼眼同样成立
   （鱼眼只改变像素怎么分布，不改变"越近越大"这个单调关系）。

2) 控制律 —— 带死区的位置控制
        d > target + deadband  →  前进
        |d - target| <= deadband → 停
        d < target             →  停（不倒退，按需求）
   转向：让 tag 保持在画面水平中心。

3) 安全（缺一不可）
   - tag 丢失超过 LOST_TIMEOUT  → 立即停
   - 距离读数突变超过 JUMP_LIMIT → 丢掉该帧
   - 速度/角速度硬限幅
   - 最小发指令间隔，避免失控
   - Ctrl+C / 异常 → finally 里归零
   - 启动时必须已在 RL 控制(MotionState=17)，否则拒绝运行
"""

import argparse
import os
import socket
import struct
import sys
import time

import cv2

try:
    from dt_apriltags import Detector
except ImportError:
    print("缺少 dt_apriltags。在机器狗上装：")
    print("  python3 -m venv --system-site-packages /home/user/follow_env")
    print("  /home/user/follow_env/bin/pip install -i "
          "https://pypi.mirrors.ustc.edu.cn/simple dt-apriltags")
    sys.exit(1)

# ---------------------------------------------------------------- 配置

ROBOT_HOST = "127.0.0.1"     # 在狗上跑，发给自己
ROBOT_PORT = 30004
TAG_FAMILY = "tagStandard41h12"

TARGET_DIST_M = 1.0          # 目标距离
DEADBAND_M = 0.15            # 死区：这个范围内不动
LOST_TIMEOUT_S = 0.5         # tag 丢失多久后停车
JUMP_LIMIT_M = 0.6           # 单帧距离突变超过这个值就丢弃

MAX_VX = 0.35                # 前进速度上限（比例量，满量程 1.0）
MAX_WZ = 0.45                # 转向速度上限
KP_DIST = 1.2                # 距离比例增益（米 → 速度比例）
KP_YAW = 1.6                 # 横向偏差比例增益

CTRL_HZ = 15                 # 控制频率

# 默认标定值：像宽 100px 对应 1.0m。**必须用 --calib 实测替换**
DEFAULT_K = 100.0


# ---------------------------------------------------------------- ASDU 协议层

HEADER_SYNC = b'\xeb\x91\xeb\x90'
_pkt_id = 0
_msg_id = 0


def build_apdu(type_, command, items):
    """16 字节协议头 + JSON。与 robot_dog_sdk 的协议层一致。"""
    global _msg_id, _pkt_id
    import json
    from datetime import datetime
    body = {"PatrolDevice": {
        "Type": type_, "Command": command,
        "Time": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        "Items": items}}
    b = json.dumps(body, separators=(',', ':')).encode()
    mid, pid = _msg_id, _pkt_id
    _msg_id = (_msg_id + 1) % 0x10000
    _pkt_id = (_pkt_id + 1) % 256
    return struct.pack('<4sHHBBB5s', HEADER_SYNC, len(b), mid, 0x01, pid, 0x01,
                       b'\x00' * 5) + b


CMD_HEARTBEAT = (0x00100064, 0x00000005)
CMD_MOTION = (0x00100001, 0x00200002)
CMD_AXIS = (0x00100001, 0x00100002)

MOTION_RL = 17


class RobotLink:
    """极简客户端：心跳 + 状态解析 + 轴指令。

    只做这个程序需要的事，不引入 robot_dog_sdk 的完整实现，
    这样在机器狗上是一个自包含的单文件。
    """

    def __init__(self, host, port):
        self.target = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.motion_state = None
        self.gait = None
        self.mode = None
        self.linear_x = 0.0
        self.last_status_t = 0.0
        self.axis_sent = 0

    def heartbeat(self):
        self.sock.sendto(build_apdu(*CMD_HEARTBEAT, {}), self.target)

    def pump(self, max_wait=0.05):
        """收包并更新状态。返回本周期收到的状态帧数。

        max_wait: 本周期最多在这里花多少秒。默认 0.05 是单线程跟随循环的
        老取值；20Hz 控制循环必须给更小的值（如 0.004），否则光收状态就
        把整个周期占满 —— 实测 50ms → 控制循环只能跑到 10Hz。
        状态帧是排队在 socket 缓冲里的，recvfrom 有数据就立即返回，
        等得短不会丢帧。
        """
        self.sock.settimeout(max_wait)
        n = 0
        deadline = time.monotonic() + max_wait
        while time.monotonic() < deadline:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                break
            except OSError:
                break
            if len(data) < 16 or data[:4] != HEADER_SYNC:
                continue
            blen = struct.unpack('<H', data[4:6])[0]
            try:
                import json
                body = json.loads(data[16:16 + blen].decode())
            except Exception:
                continue
            items = body.get("PatrolDevice", {}).get("Items", {}) or {}
            bs = items.get("BasicStatus")
            if isinstance(bs, dict):
                self.motion_state = bs.get("MotionState")
                self.gait = bs.get("Gait")
                self.mode = bs.get("ControlUsageMode")
                self.last_status_t = time.monotonic()
                n += 1
            ms = items.get("MotionStatus")
            if isinstance(ms, dict):
                self.linear_x = ms.get("LinearX") or 0.0
                n += 1
        return n

    def send_axis(self, x, yaw, y=0.0):
        """轴指令（§1.2.5）。y 是侧移，默认 0（老调用方不用改）。"""
        self.sock.sendto(build_apdu(*CMD_AXIS,
                                    {"X": x, "Y": y, "Z": 0.0,
                                     "Roll": 0.0, "Pitch": 0.0, "Yaw": yaw}),
                         self.target)
        self.axis_sent += 1

    def stop(self, cycles=8, period=0.05):
        for _ in range(cycles):
            try:
                self.send_axis(0.0, 0.0)
            except Exception:
                break
            time.sleep(period)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------- 追踪器

class TagFollower:
    def __init__(self, cam_index, k, tag_size_m=0.1, go=False,
                 target=TARGET_DIST_M, deadband=DEADBAND_M, show=False):
        self.cam_index = cam_index
        self.k = k
        self.tag_size_m = tag_size_m
        self.go = go
        self.target = target
        self.deadband = deadband
        self.show = show

        self.det = Detector(families=TAG_FAMILY, nthreads=2,
                            quad_decimate=1.0, quad_sigma=0.8,
                            refine_edges=1, decode_sharpening=0.5)
        self.cap = None
        self.link = RobotLink(ROBOT_HOST, ROBOT_PORT)

        # 初始化为当前时刻，否则"还没见过 tag"时会显示成系统开机时长
        self.last_seen_t = time.monotonic()
        self.ever_seen = False
        self.last_dist = None
        self.frames = 0
        self.hits = 0

    # ---- 相机 ----
    def open_camera(self):
        cap = cv2.VideoCapture(self.cam_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        for _ in range(5):      # 丢掉前几帧，等曝光稳定
            cap.read()
        self.cap = cap
        return True

    # ---- 检测 ----
    def detect(self, frame):
        """返回 (dist_m, dx_norm, tag_id) 或 None。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        res = self.det.detect(gray)
        if not res:
            return None
        # 多个 tag 时取画面最大的那个（离得最近，最可靠）
        best = None
        best_w = -1
        for r in res:
            c = r.corners
            w = max(abs(c[0][0] - c[1][0]), abs(c[1][0] - c[2][0]),
                    abs(c[2][0] - c[3][0]), abs(c[3][0] - c[0][0]))
            if w > best_w:
                best_w, best = w, r
        if best is None or best_w <= 1:
            return None

        # 像宽反比测距（鱼眼同样适用）
        dist = self.k / best_w

        # 横向偏差：tag 中心相对画面中心的偏移，归一化到 [-1,1]
        cx = frame.shape[1] / 2.0
        dx = (best.center[0] - cx) / cx

        return dist, dx, best.tag_id, best_w

    # ---- 控制律 ----
    def control(self, dist, dx):
        """返回 (vx, wz)，均为比例量 [-1,1]。"""
        err = dist - self.target
        if err > self.deadband:
            vx = min(MAX_VX, KP_DIST * err)
        elif err < -self.deadband:
            vx = 0.0            # 太近：按需求不倒退
        else:
            vx = 0.0            # 死区内：停

        wz = max(-MAX_WZ, min(MAX_WZ, -KP_YAW * dx))
        return vx, wz

    # ---- 主循环 ----
    def run(self):
        if not self.open_camera():
            print(f"[错误] 打不开相机 {self.cam_index}")
            print("       可能需要 sudo，或把用户加入 video 组：")
            print("       sudo usermod -aG video $USER  然后重新登录")
            return 1

        print(f"相机 {self.cam_index} 已打开 640x480")
        print(f"标定 K = {self.k:.1f}（像宽 px × 距离 m）")
        print(f"目标距离 {self.target:.2f}m，死区 ±{self.deadband:.2f}m")
        print(f"模式：{'★ 真实控制 ★' if self.go else '干跑（只检测，不发指令）'}")
        print()

        # 等状态，确认在 RL 控制
        print("等待机器人状态 ...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5:
            self.link.heartbeat()
            self.link.pump()
            if self.link.motion_state is not None:
                break
        ms = self.link.motion_state
        print(f"  MotionState={ms}  Gait={self.link.gait}  Mode={self.link.mode}")
        if ms != MOTION_RL:
            if self.go:
                # 真的控制时才拦。轴指令在非 RL 控制下无效，发了也是白发。
                print(f"[拒绝运行] 未处于 RL 控制({MOTION_RL})，请先让狗起立。")
                print("            轴指令在非 RL 控制下无效。")
                return 2
            # 干跑模式不拦：检测算法可以在狗趴着的时候单独调
            print(f"  (干跑模式，不检查 RL 控制；正式控制时会拦)")
        else:
            print("  ✓ 已在 RL 控制状态")
        print()

        period = 1.0 / CTRL_HZ
        next_t = time.monotonic()
        last_beat = 0.0
        last_print = 0.0

        try:
            while True:
                ok, frame = self.cap.read()
                if not ok:
                    print("[错误] 读帧失败")
                    break
                self.frames += 1

                r = self.detect(frame)

                # 距离突变保护
                if r is not None and self.last_dist is not None:
                    if abs(r[0] - self.last_dist) > JUMP_LIMIT_M:
                        r = None

                now = time.monotonic()

                if r is not None:
                    dist, dx, tid, pxw = r
                    self.last_seen_t = now
                    self.last_dist = dist
                    self.hits += 1
                    self.ever_seen = True
                    vx, wz = self.control(dist, dx)
                else:
                    dist = dx = None
                    tid = None
                    # tag 丢失 → 停车
                    if now - self.last_seen_t > LOST_TIMEOUT_S:
                        vx = wz = 0.0
                    else:
                        vx = wz = 0.0

                # 心跳（1Hz）
                if now - last_beat > 1.0:
                    self.link.heartbeat()
                    last_beat = now
                self.link.pump()

                # 发指令
                if self.go:
                    self.link.send_axis(vx, wz)

                # 打印
                if now - last_print > 0.5:
                    if dist is not None:
                        seen = "看得见" if now - self.last_seen_t < LOST_TIMEOUT_S else "丢失"
                        print(f"  tag={tid} 距离≈{dist:.2f}m 横向={dx:+.2f} "
                              f"→ vx={vx:+.2f} wz={wz:+.2f}  [{seen}]")
                    elif self.ever_seen:
                        print(f"  未检测到 tag（已丢失 {now - self.last_seen_t:.1f}s）"
                              f" → 输出归零")
                    else:
                        print("  未检测到 tag（从未看到过） → 输出归零")
                    last_print = now

                if self.show:
                    if dist is not None:
                        cv2.putText(frame, f"{dist:.2f}m px={pxw:.0f}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    cv2.imshow("follow", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                next_t += period
                # 只读一次时钟再判断：先比后算会让差值为负，sleep 抛
                # ValueError（实机踩过，检测线程因此整个死掉）
                _d = next_t - time.monotonic()
                if _d > 0:
                    time.sleep(_d)
                else:
                    next_t = time.monotonic()

        except KeyboardInterrupt:
            print("\n[中断]")
        finally:
            print("归零中 ...")
            if self.go:
                self.link.stop()
            self.link.close()
            if self.cap:
                self.cap.release()
            if self.show:
                cv2.destroyAllWindows()
            print(f"结束。共 {self.frames} 帧，检测到 {self.hits} 次 "
                  f"({100.0*self.hits/max(1,self.frames):.0f}%)，"
                  f"发出 {self.link.axis_sent} 条轴指令")
        return 0


def do_calib(cam_index, tag_size_m):
    """标定模式：显示像宽，让你在不同距离上读 K = 像宽 × 距离。"""
    from dt_apriltags import Detector
    det = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=1.0,
                   quad_sigma=0.8, refine_edges=1, decode_sharpening=0.5)
    cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"打不开相机 {cam_index}（试试 sudo）")
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(5):
        cap.read()

    print("标定模式：把 tag 放在已知距离上，读出像宽，算 K = 像宽 × 距离")
    print(f"实际 tag 边长 = {tag_size_m*1000:.0f}mm")
    print("按 q 退出\n")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            res = det.detect(gray)
            for r in res:
                c = r.corners
                w = max(abs(c[0][0]-c[1][0]), abs(c[1][0]-c[2][0]),
                        abs(c[2][0]-c[3][0]), abs(c[3][0]-c[0][0]))
                cv2.polylines(frame, [c.astype(int)], True, (0, 255, 0), 2)
                cv2.putText(frame, f"id={r.tag_id} w={w:.0f}px",
                            (int(r.center[0]), int(r.center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                print(f"  id={r.tag_id}  像宽={w:.1f}px   "
                      f"若此时距离 1.0m 则 K={w*1.0:.0f}，"
                      f"1.5m 则 K={w*1.5:.0f}")
            cv2.imshow("calib", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


def main():
    ap = argparse.ArgumentParser(description="机器狗 AprilTag 追踪")
    ap.add_argument("--camera", type=int, default=6,
                    help="相机序号 /dev/videoN（默认 6）")
    ap.add_argument("--k", type=float, default=DEFAULT_K,
                    help=f"测距标定值 K = 像宽(px) × 距离(m)。默认 {DEFAULT_K}")
    ap.add_argument("--tag-size", type=float, default=0.1,
                    help="tag 实际边长（米），仅标定模式用")
    ap.add_argument("--go", action="store_true",
                    help="真的发送控制指令（不加则是干跑）")
    ap.add_argument("--target", type=float, default=TARGET_DIST_M,
                    help=f"目标距离（米），默认 {TARGET_DIST_M}")
    ap.add_argument("--show", action="store_true", help="显示画面窗口")
    ap.add_argument("--calib", action="store_true", help="测距标定模式")
    args = ap.parse_args()

    if args.calib:
        return do_calib(args.camera, args.tag_size)

    f = TagFollower(args.camera, args.k, args.tag_size,
                    go=args.go, target=args.target, show=args.show)
    return f.run()


if __name__ == "__main__":
    sys.exit(main())
