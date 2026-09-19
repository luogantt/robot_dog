"""ASDU 报文格式单元测试 —— 不需要 ROS，纯字节校验。

协议依据：《软件开发指南》V1.0.1 §1.1.5 / §1.2.1 / §1.2.5
"""

import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from s10_route_nav.asdu_link import (  # noqa: E402
    CMD_AXIS, CMD_HEARTBEAT, HEADER_SYNC, AsduLink, TYPE_HEARTBEAT,
)


def test_header_layout():
    """16 字节协议头逐字段核对（§1.1.5）。"""
    link = AsduLink('127.0.0.1', 9, label='test')
    packet = link._build(0x00100001, 0x00100002, {'X': 0.5})

    assert packet[0:4] == b'\xeb\x91\xeb\x90', '同步字必须是 EB 91 EB 90'

    body_len, msg_id, fmt, pkt_id, ver = struct.unpack('<HHBBB', packet[4:11])
    assert body_len == len(packet) - 16, '长度字段必须是正文字节数'
    assert fmt == 0x01, 'ASDU 格式必须是 0x01(JSON)'
    assert ver == 0x01, '协议版本必须是 0x01'
    assert packet[11:16] == b'\x00' * 5, '预留 5 字节必须为 0'
    assert len(packet) == 16 + body_len, '总长必须 = 16 + 正文长度'
    println(f'  协议头 OK: bodyLen={body_len} msgId={msg_id} pktId={pkt_id}')


def test_msg_id_increments():
    """报文 ID 必须逐帧递增（§1.1.5 要求每帧唯一）。"""
    link = AsduLink('127.0.0.1', 9)
    ids = []
    for _ in range(5):
        packet = link._build(0x00100001, 0x00100002, {'X': 0.0})
        ids.append(struct.unpack('<H', packet[6:8])[0])
    assert ids == [0, 1, 2, 3, 4], f'msgId 应递增，实际 {ids}'
    println(f'  msgId 递增 OK: {ids}')


def test_axis_body():
    """轴指令正文格式（§1.2.5）。"""
    link = AsduLink('127.0.0.1', 9)
    packet = link._build(0x00100001, 0x00100002,
                         {'X': 0.533, 'Y': -0.4, 'Z': 0.0,
                          'Roll': 0.0, 'Pitch': 0.0, 'Yaw': 0.25})
    body = json.loads(packet[16:].decode('utf-8'))
    pd = body['PatrolDevice']
    assert pd['Type'] == 0x00100001, '轴指令 Type 必须是 0x00100001'
    assert pd['Command'] == 0x00100002, '轴指令 Command 必须是 0x00100002'
    for key in ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'):
        assert key in pd['Items'], f'Items 缺少 {key}'
    assert pd['Items']['X'] == 0.533
    assert 'Time' in pd, '缺少 Time 字段'
    println(f'  轴指令正文 OK: {pd["Items"]}')


def test_heartbeat_body():
    """心跳正文格式（§1.2.1）。"""
    link = AsduLink('127.0.0.1', 9)
    packet = link._build(TYPE_HEARTBEAT, CMD_HEARTBEAT, {})
    body = json.loads(packet[16:].decode('utf-8'))
    pd = body['PatrolDevice']
    assert pd['Type'] == 0x00100064
    assert pd['Command'] == 0x00000005
    assert pd['Items'] == {}, '心跳的 Items 必须为空'
    println('  心跳正文 OK')


def test_send_and_receive():
    """真的发一帧 UDP，确认对端收到同样的字节。"""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    server.settimeout(3.0)
    port = server.getsockname()[1]

    received = {}

    def recv():
        try:
            data, _ = server.recvfrom(65535)
            received['data'] = data
        except socket.timeout:
            pass

    thread = threading.Thread(target=recv, daemon=True)
    thread.start()

    link = AsduLink('127.0.0.1', port)
    assert link.send_axis(0.5, 0.0, -0.25), '发送应成功'
    thread.join(timeout=4)
    server.close()

    data = received.get('data')
    assert data is not None, '对端没收到任何数据'
    assert data[:4] == HEADER_SYNC
    body = json.loads(data[16:].decode('utf-8'))
    assert body['PatrolDevice']['Items']['X'] == 0.5
    assert body['PatrolDevice']['Items']['Yaw'] == -0.25
    assert link.axis_sent == 1
    println(f'  UDP 收发 OK: {len(data)} 字节，对端解析正确')


def test_heartbeat_rate_limiting():
    """心跳按周期补发，不应每帧都发。"""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    server.settimeout(0.2)
    port = server.getsockname()[1]

    link = AsduLink('127.0.0.1', port, heartbeat_hz=10.0)   # 周期 0.1s
    link.maybe_heartbeat()          # 第一次应发
    assert link.heartbeats_sent == 1
    link.maybe_heartbeat()          # 紧接着调用不应发
    link.maybe_heartbeat()
    assert link.heartbeats_sent == 1, '高频调用不应重复发心跳'
    time.sleep(0.12)
    link.maybe_heartbeat()
    assert link.heartbeats_sent == 2, '超过周期后应补发'
    server.close()
    println('  心跳限频 OK（1 次 → 0 次 → 补发 1 次）')


def test_poll_parses_status():
    """poll() 能解析机器人主动上报的 BasicStatus（§1.3.1.1）。"""
    import socket as _socket

    server = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    port = server.getsockname()[1]
    link = AsduLink('127.0.0.1', port)

    # 先告诉 link 目标是自己，然后伪造一帧 BasicStatus 发给它
    basic = {'PatrolDevice': {
        'Type': 0x00100064, 'Command': 0x00F00000,
        'Time': '2026-09-19 12:00:00',
        'Items': {'BasicStatus': {'MotionState': 17, 'Gait': 0x1001,
                                  'ControlUsageMode': 0}}}}
    raw = json.dumps(basic, separators=(',', ':')).encode('utf-8')
    packet = struct.pack('<4sHHBBB5s', HEADER_SYNC, len(raw), 0, 0x01, 0,
                         0x01, b'\x00' * 5) + raw
    server.sendto(packet, ('127.0.0.1', link.local_port))

    assert link.poll() == 1, '应解析出 1 帧'
    assert link.motion_state == 17, f'MotionState 应为 17，实际 {link.motion_state}'
    assert link.gait == 0x1001
    assert link.is_rl_control() is True
    assert link.status_age() < 1.0
    assert link is not None

    # 非 RL 状态必须被判否
    basic['PatrolDevice']['Items']['BasicStatus']['MotionState'] = 4
    raw = json.dumps(basic, separators=(',', ':')).encode('utf-8')
    packet = struct.pack('<4sHHBBB5s', HEADER_SYNC, len(raw), 1, 0x01, 0,
                         0x01, b'\x00' * 5) + raw
    server.sendto(packet, ('127.0.0.1', link.local_port))
    link.poll()
    assert link.is_rl_control() is False, '趴下(4) 不应被判为 RL 控制'

    # 垃圾包不应崩
    server.sendto(b'garbage', ('127.0.0.1', link.local_port))
    link.poll()
    assert link.motion_state == 4, '垃圾包不应覆盖已有状态'

    server.close()
    println(f'  状态解析 OK: {link.describe()}')


def test_status_age_when_never_received():
    """从未收到状态时 status_age 应为 inf，门控才会拒绝运动。"""
    link = AsduLink('127.0.0.1', 9)
    assert link.status_age() == float('inf')
    assert link.is_rl_control() is False
    println('  从未收到状态 → age=inf，门控拒绝 OK')


def println(message=''):
    print(message, flush=True)


def main():
    println('=== ASDU 报文格式测试 ===')
    failures = []
    for name, fn in [
        ('协议头格式', test_header_layout),
        ('msgId 递增', test_msg_id_increments),
        ('轴指令正文', test_axis_body),
        ('心跳正文', test_heartbeat_body),
        ('UDP 收发', test_send_and_receive),
        ('心跳限频', test_heartbeat_rate_limiting),
        ('状态解析', test_poll_parses_status),
        ('无状态时拒绝', test_status_age_when_never_received),
    ]:
        try:
            fn()
            println(f'PASS  {name}')
        except AssertionError as error:
            println(f'FAIL  {name}: {error}')
            failures.append(name)
        except Exception as error:                       # noqa: BLE001
            println(f'ERROR {name}: {error!r}')
            failures.append(name)
    println()
    if failures:
        println(f'{len(failures)} 项失败: {failures}')
        return 1
    println('全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
