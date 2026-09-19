"""通过 ASDU/UDP 直接给 S10 发轴指令（替代 /STEER → rl_deploy 通道）。

协议依据：《软件开发指南》V1.0.1
    §1.1.5  16 字节协议头
           偏移 0-3   同步字 EB 91 EB 90
           偏移 4-5   ASDU 长度（小端）
           偏移 6-7   报文 ID（小端，逐帧递增）
           偏移 8     ASDU 格式 0x01 = JSON
           偏移 9     包编号
           偏移 10    协议版本 0x01
           偏移 11-15 预留 5 字节
    §1.2.1  心跳 Type=0x00100064 Command=0x00000005
    §1.2.5  轴指令 Type=0x00100001 Command=0x00100002，无响应帧

------------------------------------------------------------------------------
为什么可以直接替代 /STEER
------------------------------------------------------------------------------
官方 SDK 把两个通道写进**同一个结构体的同一个字段**：

    dds_command_interface.hpp :  usr_cmd_->forward_vel_scale = msg->data.x
    gamepad_interface.hpp     :  usr_cmd_->forward_vel_scale = items["X"]

也就是说 `/STEER` 的 x 和 ASDU 的 X 是**同一个量、同一套标度**。
因此 command_mux 已经算好的归一化值可以**原样照搬**，不需要重新标定。

------------------------------------------------------------------------------
已知限制（务必知悉）
------------------------------------------------------------------------------
1. **没有响应帧**（§1.2.5 明文规定）。发了不等于生效，丢包是静默的。
   → 只能靠机器人状态上报（/MOTION_INFO 等）侧面观察，或在实机上验证。
2. **有 2 秒同源约束**（§1.5 的 0xE006）。同一时刻只能有一个客户端发轴指令，
   否则会被拒绝。用手机 App 或别的程序同时控制会互相踢掉。
3. **需要机器人处于 RL 控制状态**（§2.2.1）。轴指令在非 RL 控制下无效。
   进入方式：ASDU 下发站立(1) → 机器人自动进入 RL 控制(17)。
"""

import json
import socket
import struct
import time

HEADER_SYNC = b'\xeb\x91\xeb\x90'
HEADER_LEN = 16
HEADER_FORMAT_JSON = 0x01
HEADER_VERSION = 0x01

TYPE_HEARTBEAT = 0x00100064
CMD_HEARTBEAT = 0x00000005
TYPE_MOTION = 0x00100001
CMD_AXIS = 0x00100002
CMD_MOTION_STATE = 0x00200002

# 文档 §1.2.3 / §2.2.1：站立完成后机器人自动进入 RL 控制状态，轴指令才生效
MOTION_RL_CONTROL = 17

MOTION_NAMES = {-2: '软急停', 0: '默认(未上报)', 1: '站立', 2: '关节阻尼',
                4: '趴下', 5: '标零', 17: 'RL控制', 0x1001: '阻尼趴下'}


class AsduLink:
    """极简 ASDU UDP 客户端。只做这个项目需要的事：发轴指令 + 心跳。

    不依赖 ROS，也不依赖 robot_dog_sdk，便于在 AGX 上独立运行和单元测试。
    """

    def __init__(self, host: str, port: int, heartbeat_hz: float = 1.0,
                 label: str = 'asdu'):
        self.host = host
        self.port = int(port)
        self.target = (host, self.port)
        self.heartbeat_period = 1.0 / max(0.1, heartbeat_hz)
        self.label = label

        self._msg_id = 0
        self._pkt_id = 0
        self._last_heartbeat = 0.0
        self.send_failures = 0
        self.axis_sent = 0
        self.heartbeats_sent = 0

        # 机器人状态（靠心跳触发的主动上报，见 poll()）
        self.motion_state = None
        self.gait = None
        self.usage_mode = None
        self.last_status_time = 0.0
        self.status_received = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 必须显式 bind：否则端口要等第一次 sendto 才分配，而 poll() 会先于它执行
        # （Windows 直接报 WSAEINVAL，Linux 静默收不到东西）。
        # 绑一个临时端口就够；机器人是往"发心跳的那个源端口"回推状态的（§1.2.1），
        # 收发同端口才能收到。
        self.sock.bind(('0.0.0.0', 0))
        self.sock.setblocking(False)   # poll() 用非阻塞收包
        self.local_port = self.sock.getsockname()[1]

    # ---------------- 组包 ----------------

    def _build(self, msg_type: int, command: int, items: dict) -> bytes:
        body = json.dumps({
            'PatrolDevice': {
                'Type': msg_type,
                'Command': command,
                'Time': time.strftime('%Y-%m-%d %H:%M:%S'),
                'Items': items,
            }
        }, separators=(',', ':')).encode('utf-8')

        msg_id = self._msg_id
        self._msg_id = (self._msg_id + 1) % 0x10000
        pkt_id = self._pkt_id
        self._pkt_id = (self._pkt_id + 1) % 256

        header = struct.pack('<4sHHBBB5s', HEADER_SYNC, len(body), msg_id,
                             HEADER_FORMAT_JSON, pkt_id, HEADER_VERSION,
                             b'\x00' * 5)
        return header + body

    # ---------------- 发送 ----------------

    def send_axis(self, x: float, y: float, yaw: float) -> bool:
        """发一帧轴指令。x/y/yaw 为归一化比例 [-1,1]。失败返回 False。"""
        packet = self._build(TYPE_MOTION, CMD_AXIS, {
            'X': float(x), 'Y': float(y), 'Z': 0.0,
            'Roll': 0.0, 'Pitch': 0.0, 'Yaw': float(yaw),
        })
        try:
            self.sock.sendto(packet, self.target)
            self.axis_sent += 1
            return True
        except OSError:
            self.send_failures += 1
            return False

    def send_heartbeat(self) -> bool:
        """发一帧心跳。文档建议不小于 1Hz；机器人据此回推状态。"""
        packet = self._build(TYPE_HEARTBEAT, CMD_HEARTBEAT, {})
        try:
            self.sock.sendto(packet, self.target)
            self.heartbeats_sent += 1
            self._last_heartbeat = time.monotonic()
            return True
        except OSError:
            self.send_failures += 1
            return False

    def maybe_heartbeat(self) -> None:
        """按周期补发心跳，供高频循环里顺带调用（不额外开线程）。"""
        now = time.monotonic()
        if now - self._last_heartbeat >= self.heartbeat_period:
            self.send_heartbeat()

    def send_motion_state(self, motion_param: int) -> bool:
        """下发运动状态转换（1=站立 4=趴下）。有响应帧，但这里不等。"""
        packet = self._build(TYPE_MOTION, CMD_MOTION_STATE,
                             {'MotionParam': int(motion_param)})
        try:
            self.sock.sendto(packet, self.target)
            return True
        except OSError:
            self.send_failures += 1
            return False

    # ---------------- 接收机器人状态 ----------------

    def poll(self) -> int:
        """非阻塞收取机器人主动上报的状态，更新 motion_state 等字段。

        机器人只向"持续发心跳的 IP:端口"推状态（§1.2.1），所以我们发心跳之后
        才会收到东西。返回本次处理的帧数。

        BasicStatus 里有 MotionState / Gait / ControlUsageMode（§1.3.1.1，2Hz）。
        """
        count = 0
        while True:
            try:
                data, _addr = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            if len(data) < HEADER_LEN or data[:4] != HEADER_SYNC:
                continue
            body_len = struct.unpack('<H', data[4:6])[0]
            try:
                body = json.loads(data[HEADER_LEN:HEADER_LEN + body_len].decode('utf-8'))
            except Exception:                                  # noqa: BLE001
                continue
            items = body.get('PatrolDevice', {}).get('Items', {}) or {}
            basic = items.get('BasicStatus')
            if isinstance(basic, dict):
                self.motion_state = basic.get('MotionState')
                self.gait = basic.get('Gait')
                self.usage_mode = basic.get('ControlUsageMode')
                self.last_status_time = time.monotonic()
                self.status_received += 1
                count += 1
        return count

    def status_age(self) -> float:
        """距上次收到状态的时间（秒）；从没收到过则返回 inf。"""
        if self.last_status_time <= 0.0:
            return float('inf')
        return time.monotonic() - self.last_status_time

    def is_rl_control(self) -> bool:
        return self.motion_state == MOTION_RL_CONTROL

    def describe(self) -> str:
        state = self.motion_state
        name = MOTION_NAMES.get(state, '?') if isinstance(state, int) else '未收到'
        age = self.status_age()
        age_text = '从未' if age == float('inf') else f'{age:.1f}s前'
        return (f'MotionState={state}({name}) Gait={self.gait} '
                f'Mode={self.usage_mode} 状态={age_text}')

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass
