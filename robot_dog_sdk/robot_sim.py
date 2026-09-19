#!/usr/bin/env python3
"""
机器狗模拟器 —— 不接真机就能开发/调试 Web 界面。

按《软件开发指南》V1.0.1 的行为实现：
  - §1.1.5  16 字节协议头 + JSON ASDU
  - §1.2.1  心跳：记住客户端地址，之后向该地址推送状态
  - §1.2.3  运动状态转换（起立后自动进入 RL 控制 17）
  - §1.2.4  步态切换：未完全停止时指令被缓存，停下后才生效
  - §1.2.5  轴指令：比例量 [-1,1]，无响应帧
  - §1.3.1.1/§1.3.1.2  BasicStatus 2Hz / MotionStatus 10Hz
  - §1.3.1.4  异常状态上报 2Hz
  - §1.5     通用响应回填请求的 msgId

用法：
    python3 robot_sim.py                      # 默认监听 0.0.0.0:30004
    python3 robot_sim.py 31004                # 指定端口
    python3 robot_sim.py 31004 --fault 8      # 8 秒后注入一个 FATAL 故障
    python3 robot_sim.py 31004 --verbose      # 打印每一条收到的指令

然后让 web_control.py / asdu_cli.py 指向 127.0.0.1:<端口> 即可。
"""

import json
import math
import socket
import struct
import sys
import threading
import time

SYNC = b'\xeb\x91\xeb\x90'
HDR = 16

# ---- 枚举（照抄文档）----
GAIT_BASIC_CTRL = 0x1001   # 基础（常规运动模式）
GAIT_STAIR_CTRL = 0x1003   # 楼梯（常规运动模式）
GAIT_FLAT_NAV = 0x3002     # 平地（导航运动模式）
GAIT_STAIR_NAV = 0x3003    # 楼梯（导航运动模式）

MOTION_CROUCH = 4
MOTION_STAND = 1
MOTION_RL = 17

MAX_VX_RATIO = 1.67   # 比例轴指令 X=±1 对应的最大线速度（m/s），取自 §1.2.6 的限值
MAX_VY_RATIO = 0.4
MAX_WZ_RATIO = 1.0

_pkt = 0
_plock = threading.Lock()


def build(body, msg_id):
    global _pkt
    b = json.dumps(body, separators=(',', ':')).encode()
    with _plock:
        p = _pkt
        _pkt = (_pkt + 1) % 256
    return struct.pack('<4sHHBBB5s', SYNC, len(b), msg_id, 0x01, p, 0x01,
                       b'\x00' * 5) + b


def parse(data):
    if len(data) < HDR or data[0:4] != SYNC:
        return None, None
    blen, mid, fmt, pid, ver = struct.unpack('<HHBBB', data[4:11])
    if fmt != 0x01:
        return mid, None
    try:
        return mid, json.loads(data[HDR:HDR + blen].decode())
    except Exception:
        return mid, None


class RobotSim(threading.Thread):
    def __init__(self, port=30004, verbose=False, fault_at=None):
        super().__init__(daemon=True)
        self.port = port
        self.verbose = verbose
        self.fault_at = fault_at
        self.running = True

        # --- 状态 ---
        self.motion_state = MOTION_CROUCH
        self.gait = GAIT_BASIC_CTRL
        self.usage_mode = 0            # ControlUsageMode: 0 常规 / 1 导航 / 2 辅助
        self.vx = self.vy = self.wz = 0.0   # 实际速度（m/s, m/s, rad/s）
        self.height = 0.0
        self.x = self.y = self.yaw = 0.0    # 里程计积分（便于在界面上看到真的在动）
        self.charge = 0
        self.battery = 62
        self.hes = 0
        self.model = "CD1-SIM"
        self.version = "STD"
        self.pm = 0

        self.client = None             # 最近一次发心跳的来源地址
        self.stand_done_at = None      # 起立完成的时刻
        self.pending_gait = None       # 因未停止而被缓存的步态
        self.faults = {}               # code -> info
        self.fault_pushed = False
        self.hot_fault_sent = False    # --hot 模式下只推一次过温故障

        self.rx_axis = 0
        self.rx_total = 0
        self.rx_axis_cmds = set()      # 收到过哪些轴指令码（0x00100002 / 0x00110002）

        # 温度模型：16 路电机/驱动器。空载约 35/45°C，按速度发热、向环境散热。
        # 实机实测的典型值参考：静止站立久了前髋 X 能到 77/82°C。
        self.motor_t = [35.0] * 16
        self.driver_t = [45.0] * 16
        self.hot_joints = set()        # 强制加热的关节（--hot 用）

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.01)
        self.t0 = time.monotonic()
        self.last_axis_at = 0.0

    # ---------------- 主循环 ----------------
    def run(self):
        print(f"[SIM] 模拟机器人已启动，监听 UDP 0.0.0.0:{self.port}")
        print(f"[SIM] 初始：MotionState={self.motion_state} Gait=0x{self.gait:04x} "
              f"Mode={self.usage_mode}")
        t_basic = t_motion = t_device = time.monotonic()
        prev = time.monotonic()

        while self.running:
            now = time.monotonic()
            dt = now - prev
            prev = now

            try:
                data, addr = self.sock.recvfrom(65535)
                self.client = addr
                self.rx_total += 1
                self.handle(data, addr)
            except socket.timeout:
                pass

            self.tick(dt, now)

            if self.client:
                if now >= t_basic:
                    t_basic = now + 0.5
                    self.push_basic()
                if now >= t_motion:
                    t_motion = now + 0.1
                    self.push_motion()
                if now >= t_device:
                    t_device = now + 0.5      # 文档 §1.3.1.3：设备状态 2Hz
                    self.push_device()
            if self.fault_at and not self.fault_pushed and now - self.t0 >= self.fault_at:
                self.fault_pushed = True
                self.push_fault(0x8014, "joint_position_over_limit", 5, ["11"])
            if self.hot_joints and self.client and not self.hot_fault_sent:
                self.hot_fault_sent = True
                # 复现实机：0x8001 电机过温 / 0x8002 驱动器过温，点名 joint#0 和 joint#4
                self.push_fault(0x8001, "motor_over_temp", 3, ["joint#0", "joint#4"])
                self.push_fault(0x8002, "joint_driver_over_temp", 3, ["joint#0", "joint#4"])

    def tick(self, dt, now):
        self.update_thermal(dt)

        # 起立 -> 自动进入 RL 控制（§2.2.1）
        if self.stand_done_at and now >= self.stand_done_at:
            self.motion_state = MOTION_RL
            self.stand_done_at = None
            self.log("[SIM] 起立完成，自动进入 RL 控制状态(17)")

        # 轴指令流停止超过 0.5s 则视为失控，速度归零（模拟真实机器人的看门狗）
        if self.last_axis_at and now - self.last_axis_at > 0.5:
            if self.vx or self.vy or self.wz:
                self.log("[SIM] 0.5s 无轴指令，速度归零（失控保护）")
            self.vx = self.vy = self.wz = 0.0

        # 位置积分
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw += self.wz * dt

        # 已停止且在等待切步态 -> 现在生效
        if self.pending_gait is not None and abs(self.vx) < 0.02 and abs(self.wz) < 0.03:
            self.apply_gait(self.pending_gait)
            self.pending_gait = None
            self.log(f"[SIM] 缓存的步态切换生效 -> 0x{self.gait:04x}")

    def apply_gait(self, g):
        """文档 §1.2.4：步态切换会自动切换到对应的运动模式。

        UNVERIFIED：文档只写了"会自动切换运动模式"，没有明确说这个"运动模式"
        就是 BasicStatus.ControlUsageMode。这里按"是同一个"来实现，属于假设，
        需实机确认（切 0x3003 后立刻读 ControlUsageMode 是否为 1）。
        """
        self.gait = g
        if g in (GAIT_FLAT_NAV, GAIT_STAIR_NAV):
            self.usage_mode = 1
        elif g in (GAIT_BASIC_CTRL, GAIT_STAIR_CTRL):
            self.usage_mode = 0

    def log(self, msg):
        print(msg, flush=True)

    # ---------------- 收包处理 ----------------
    def handle(self, data, addr):
        msg_id, body = parse(data)
        if not body:
            return
        pd = body.get("PatrolDevice", {})
        t, c = pd.get("Type"), pd.get("Command")
        items = pd.get("Items", {}) or {}

        if self.verbose:
            self.log(f"[SIM] <- Type=0x{t:08x} Command=0x{c:08x} Items={items}")

        # --- 心跳 §1.2.1 ---
        if t == 0x00100064 and c == 0x00000005:
            self.reply(msg_id, t, c, {"ErrorCode": 0, "ErrorMessage": "Success"})
            return

        # --- 使用模式切换 §1.2.2 ---
        if t == 0x00100002 and c == 0x00500002:
            self.usage_mode = items.get("Mode", self.usage_mode)
            self.reply(msg_id, t, c, {"ErrorCode": 0, "ErrorMessage": "Success"})
            self.log(f"[SIM] 使用模式 -> {self.usage_mode}")
            return

        # --- 运动状态转换 §1.2.3 ---
        if t == 0x00100001 and c == 0x00200002:
            p = items.get("MotionParam")
            if p == -2:
                self.reply(msg_id, t, c, {"ErrorCode": 0xE008,
                                          "ErrorMessage": "Not allowed operation"})
                return
            self.reply(msg_id, t, c, {"ErrorCode": 0, "ErrorMessage": "Success"})
            if p == MOTION_STAND:
                self.motion_state = MOTION_STAND
                self.stand_done_at = time.monotonic() + 1.1
            elif p == MOTION_CROUCH:
                self.motion_state = MOTION_CROUCH
                self.vx = self.vy = self.wz = 0.0
            else:
                self.motion_state = p
            self.log(f"[SIM] 运动状态 -> {p}")
            return

        # --- 步态切换 §1.2.4 ---
        if t == 0x00100001 and c == 0x00300002:
            g = items.get("GaitParam")
            self.reply(msg_id, t, c, {"ErrorCode": 0, "ErrorMessage": "Success"})
            if abs(self.vx) < 0.02 and abs(self.wz) < 0.03:
                self.apply_gait(g)
                self.log(f"[SIM] 步态 -> 0x{g:04x}，使用模式随之变为 {self.usage_mode}")
            else:
                self.pending_gait = g
                self.log(f"[SIM] 机器人仍在运动，步态 0x{g:04x} 已缓存（§1.2.4）")
            return

        # --- 轴指令 §1.2.5 / 真实轴 §1.2.6 ---
        if t == 0x00100001 and c in (0x00100002, 0x00110002):
            if self.motion_state != MOTION_RL:
                # 未进入 RL 控制，忽略（真机行为未在文档中明确）
                return
            real = (c == 0x00110002)
            if real and self.usage_mode != 1:
                return          # §1.2.6 仅导航模式生效
            if not real and self.usage_mode not in (0, 2):
                return          # §1.2.5 仅常规/辅助模式生效

            x = float(items.get("X", 0.0))
            y = float(items.get("Y", 0.0))
            w = float(items.get("Yaw", 0.0))
            if real:
                self.vx, self.vy, self.wz = x, y, w          # 物理量，原样传递
            else:
                self.vx = x * MAX_VX_RATIO * (0.5 if self.gait == GAIT_STAIR_CTRL else 1.0)
                self.vy = y * MAX_VY_RATIO
                self.wz = w * MAX_WZ_RATIO
            self.last_axis_at = time.monotonic()
            self.rx_axis += 1
            self.rx_axis_cmds.add(c)     # 记录收到过哪种轴指令，供测试断言
            return          # §1.2.5/§1.2.6 不返回响应帧

    def reply(self, msg_id, t, c, items):
        body = {"PatrolDevice": {"Type": t, "Command": c,
                                 "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                                 "Items": items}}
        self.send(build(body, msg_id))

    def push(self, type_, items):
        body = {"PatrolDevice": {"Type": type_, "Command": 0x00F00000,
                                 "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                                 "Items": items}}
        self.send(build(body, 0))

    def send(self, pkt):
        if not self.client:
            return
        try:
            self.sock.sendto(pkt, self.client)
        except OSError:
            pass

    def push_basic(self):
        self.push(0x00100064, {"BasicStatus": {
            "MotionState": self.motion_state, "Gait": self.gait,
            "HES": self.hes, "ControlUsageMode": self.usage_mode,
            "PowerManagement": self.pm, "Sleep": False,
            "Model": self.model, "Version": self.version, "Charge": self.charge}})

    def push_motion(self):
        self.push(0x00100001, {"MotionStatus": {
            "AccX": 0, "AccY": 0, "AccZ": 0,
            "AngularZ": round(self.wz, 4), "Gait": self.gait, "Height": self.height,
            "LinearX": round(self.vx, 4), "LinearY": round(self.vy, 4),
            "MotionState": self.motion_state,
            "OmegaX": 0, "OmegaY": 0, "OmegaZ": 0, "Payload": 0,
            "Pitch": 0, "RemainMile": 0, "Roll": 0,
            "Yaw": round(self.yaw, 4)},
            "MotorStatus": {"Joint": [0.0] * 16}})

    def push_device(self):
        """设备状态上报：电池 / 16 路电机与驱动器温度 / CPU / GPS。

        注意 Type 用 0x00300002 而不是文档 §1.3.1.3 写的 0x00100002 ——
        实机实测就是这个值（见 README 第 9 节），模拟器跟着实机走。
        """
        self.push(0x00300002, {
            "BatteryList": [{
                "BatteryLevel": self.battery, "Voltage": 73.7,
                "battery_temperature": 36.0, "charge": False, "serial": ""}],
            "BatteryStatus": {},
            "CPU": {},
            "DevEnable": {"FP": 1, "FanSpeed": 0, "GPS": 1, "LED": 1,
                          "Lidar": 1, "LoadPower": 1, "Video": 1},
            "DeviceTemperature": {
                "Motor": [round(v, 2) for v in self.motor_t],
                "Driver": [round(v, 2) for v in self.driver_t]},
            "GPS": {"FixQuality": 0, "Latitude": 0.0, "Longitude": 0.0},
        })

    def update_thermal(self, dt):
        """简化热模型：按运动强度发热，向环境散热。用于让温度面板有真实变化。"""
        load = min(1.0, abs(self.vx) / 1.5 + abs(self.wz) / 1.5)
        amb_m, amb_d = 35.0, 45.0
        for i in range(16):
            # 前髋 X(0/4) 承重，静止站立也持续发热（对应实机实测现象）
            extra = 0.0
            if i in (0, 4):
                extra = 28.0 if self.motion_state in (1, 17) else 6.0
            target_m = amb_m + extra + load * 30.0
            target_d = amb_d + extra + load * 30.0
            if i in self.hot_joints:
                target_m = target_d = 80.0
            # 一阶趋近，时间常数约 8 秒
            k = min(1.0, dt / 8.0)
            self.motor_t[i] += (target_m - self.motor_t[i]) * k
            self.driver_t[i] += (target_d - self.driver_t[i]) * k

    def push_fault(self, code, name, severity, resources):
        self.faults[code] = True
        self.push(0x0010007f, {"ErrorList": [{
            "Code": code, "Details": "", "Grouped": False, "Name": name,
            "Resources": resources, "Severities": [severity],
            "Source": ["sim"], "SourceIds": [1],
            "Timestamp": {"Nanosec": 0, "Sec": int(time.time())}, "Type": 1}]})
        self.log(f"[SIM] 注入故障 0x{code:04x} {name} severity={severity}")

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    port = 30004
    args = [a for a in sys.argv[1:]]
    verbose = "--verbose" in args
    hot = "--hot" in args
    fault_at = None
    if "--fault" in args:
        i = args.index("--fault")
        fault_at = float(args[i + 1])
        del args[i:i + 2]
    args = [a for a in args if not a.startswith("--")]
    if args:
        port = int(args[0])

    sim = RobotSim(port=port, verbose=verbose, fault_at=fault_at)
    if hot:
        # 复现实机那个场景：前髋 X(0/4) 过热 + 报 0x8001/0x8002
        sim.hot_joints = {0, 4}
        sim.motor_t[0] = sim.motor_t[4] = 76.9
        sim.driver_t[0] = 82.5
        sim.driver_t[4] = 77.9
    sim.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[SIM] 退出")
        sim.stop()


if __name__ == "__main__":
    main()
