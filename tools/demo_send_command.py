#!/usr/bin/env python3
"""
============================================================
 最小 Demo：如何给机器狗发一条 ASDU 命令
============================================================

这个文件只干一件事——把"发一条命令"拆开给你看明白。
读完之后：
    想要实用工具     -> asdu_cli.py
    想要完整控制流程 -> asdu_udp_control.py

【安全默认值】
本脚本默认是 **dry-run（只打印要发的字节，不真的发）**。
要真的发出去，必须显式加 --send：

    python3 demo_send_command.py                    # 只打印，不发送
    python3 demo_send_command.py --send             # 真的发送

    python3 demo_send_command.py --send --host 192.168.1.50

【会真的动起来】
加 --send 后，本 demo 会让机器狗：
    1) 发心跳（只读状态，不动）
    2) 起立        <- 机器狗会站起来
    3) 轴指令 X=0.12 1 秒 后归零   <- 机器狗会往前走
请确保机器狗周围有足够空间、有人看护。

参考：《软件开发指南》V1.0.1
"""

import json
import socket
import struct
import sys
import time

# ============================================================
# 配置
# ============================================================

ROBOT_IP = "10.21.33.103"
ROBOT_PORT = 30004  # 文档 §1.1.2：UDP 端口，默认启用 DTLS 加密，需技术支持关掉

SYNC = b'\xeb\x91\xeb\x90'  # 文档 §1.1.5：固定同步字 0xeb 0x91 0xeb 0x90
HEADER_LEN = 16  # 文档 §1.1.5：协议头固定 16 字节

# 命令码（文档 §1.2 控制类 ASDU 消息集）
CMD_HEARTBEAT = (0x00100064, 0x00000005)  # §1.2.1 心跳
CMD_MOTION_STATE = (0x00100001, 0x00200002)  # §1.2.3 运动状态转换（起立/趴下）
CMD_GAIT = (0x00100001, 0x00300002)  # §1.2.4 运动步态切换
CMD_AXIS = (0x00100001, 0x00100002)  # §1.2.5 轴指令（比例量，无响应帧）


# ============================================================
# 第 1 步：把一条命令打包成字节
# ============================================================

def build_packet(msg_type, command, items, msg_id=1):
    """把命令打包成 [16 字节协议头] + [JSON 正文]。

    协议头的字段顺序和字节序见文档 §1.1.5，一个字都不能错：
        偏移 0-3   同步字      eb 91 eb 90
        偏移 4-5   ASDU 长度   小端（低字节在前）
        偏移 6-7   报文 ID     小端，0->65535 循环
        偏移 8     ASDU 格式   0x01 = JSON
        偏移 9     包编号      0->255 循环
        偏移 10    协议版本    固定 0x01
        偏移 11-15 预留        填 0
    """
    # --- JSON 正文 ---
    # 注意：JSON 里的 Type/Command 写十进制。文档示例里的 "0x0010 0064" 只是给人看的，
    # 不是合法的 JSON，实际发出去是 1048676。
    body = {
        "PatrolDevice": {
            "Type": msg_type,
            "Command": command,
            "Time": time.strftime("%Y-%m-%d %H:%M:%S"),  # 本地时区
            "Items": items,
        }
    }
    body_bytes = json.dumps(body, separators=(',', ':')).encode('utf-8')

    # --- 16 字节协议头 ---
    # '<4sHHBBB5s' 里的 '<' 表示小端且不加对齐填充
    header = struct.pack(
        '<4sHHBBB5s',
        SYNC,                    # 4s  同步字
        len(body_bytes),         # H   长度（必须是正文的字节数）
        msg_id % 0x10000,        # H   报文 ID
        0x01,                    # B   格式：JSON
        msg_id % 256,            # B   包编号
        0x01,                    # B   版本
        b'\x00' * 5,             # 5s  预留
    )
    return header + body_bytes


# ============================================================
# 第 2 步：发出去 / 收回来
# ============================================================

def parse_response(data):
    """把收到的字节解成 dict；不是合法 ASDU 就返回 None。"""
    if len(data) < HEADER_LEN or data[0:4] != SYNC:
        return None
    body_len, msg_id, fmt, pkt_id, ver = struct.unpack('<HHBBB', data[4:11])
    try:
        body = json.loads(data[HEADER_LEN:HEADER_LEN + body_len].decode('utf-8'))
    except Exception:
        return None
    pd = body.get("PatrolDevice", {})
    return {
        "msg_id": msg_id,
        "type": pd.get("Type"),
        "command": pd.get("Command"),
        "items": pd.get("Items", {}),
    }


def send_and_receive(sock, packet, target, want=None, timeout=3.0):
    """发送一帧，然后等【命令响应】。

    这里有个必须处理的问题：
    机器狗会不停地把状态（BasicStatus 2Hz / MotionStatus 10Hz）推给你，
    这些**推送帧会插在命令响应前面**。所以不能"收一个就当响应"，
    必须按 Type+Command 过滤——推送帧的 Command 恒为 0x00f00000。

    want=(Type, Command)：期望的响应，为 None 表示这个是纯推送流不用等响应。
    """
    sock.sendto(packet, target)
    print(f"  -> 已发送 {len(packet)} 字节")

    if want is None:
        # 文档 §1.2.5：轴指令不返回任何响应帧，只能观察状态变化
        print("  <- 该指令无响应帧（文档 §1.2.5），无法从这里确认结果")
        return None

    skipped = 0
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        r = parse_response(data)
        if r is None:
            continue
        if r["command"] == 0x00F00000:  # 主动推送的状态帧，不是我们要的响应
            skipped += 1
            continue
        if (r["type"], r["command"]) != want:
            skipped += 1
            continue
        if skipped:
            print(f"  （期间跳过了 {skipped} 个推送/无关帧）")
        print(f"  <- msgId={r['msg_id']} Type=0x{r['type']:08x} "
              f"Command=0x{r['command']:08x}")
        print(f"     Items={r['items']}")
        return r

    print(f"  <- 等待 {timeout}s 未收到匹配的响应（检查加密是否已关闭）")
    return None


# ============================================================
# 第 3 步：跑一遍
# ============================================================

def main():
    do_send = "--send" in sys.argv
    host = ROBOT_IP
    if "--host" in sys.argv:
        host = sys.argv[sys.argv.index("--host") + 1]
    target = (host, ROBOT_PORT)

    print("=" * 62)
    print(" ASDU 发送命令 Demo")
    print(f" 目标: {host}:{ROBOT_PORT}")
    print(f" 模式: {'真的发送' if do_send else 'DRY-RUN（只打印，不发送）'}")
    print("=" * 62)
    if not do_send:
        print(" 提示：加 --send 才会真的发出去，本模式只演示字节怎么拼。\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0)

    # ---------- 示例 1：心跳（只读，不会让机器狗动）----------
    print("\n【示例 1】心跳 —— 拿到状态上报的入场券")
    pkt = build_packet(*CMD_HEARTBEAT, {})
    print(f"  报文 {len(pkt)} 字节: {pkt.hex()[:64]}...")
    if do_send:
        send_and_receive(sock, pkt, target, want=CMD_HEARTBEAT)
        print("  提示：心跳建议不小于 1Hz 持续发，机器狗才会持续推状态（文档 §1.2.1）")

    # ---------- 示例 2：有响应的命令 ----------
    print("\n【示例 2】起立 —— 有响应帧，能立刻知道成没成")
    pkt = build_packet(*CMD_MOTION_STATE, {"MotionParam": 1})
    print(f"  正文: {pkt[HEADER_LEN:].decode('utf-8')}")
    if do_send:
        r = send_and_receive(sock, pkt, target, want=CMD_MOTION_STATE)
        # 响应里 ErrorCode=0 表示成功（文档 §1.5）
        if (r or {}).get("items", {}).get("ErrorCode") == 0:
            print("  [OK] 机器狗接受了起立指令，正在起身 ...")
        # 起立完成后机器狗会自动进入 RL 控制状态（文档 §2.2.1），
        # 之后才接受步态/速度指令。这里持续发心跳并打印状态变化。
        print("  接下来持续发心跳，观察状态变化（12 秒）...")
        last = None
        end = time.monotonic() + 12
        while time.monotonic() < end:
            sock.sendto(build_packet(*CMD_HEARTBEAT, {}), target)
            # 关键：状态推送是 2Hz/10Hz，会很快堆满接收缓冲区。
            # 必须把缓冲里的包全部取出来、只用**最新**的那个，
            # 否则你会一直在读几秒前的旧状态。
            try:
                sock.settimeout(0.05)
                while True:
                    data, _ = sock.recvfrom(65535)
                    r = parse_response(data)
                    if r is None:
                        continue
                    bs = r["items"].get("BasicStatus")
                    if bs:
                        cur = (bs.get("MotionState"), bs.get("Gait"),
                               bs.get("ControlUsageMode"))
                        if cur != last:
                            print(f"    MotionState={cur[0]}  Gait=0x{cur[1]:04x}  "
                                  f"ControlUsageMode={cur[2]}")
                            last = cur
            except socket.timeout:
                pass
            sock.settimeout(3.0)
            time.sleep(1.0)

    # ---------- 示例 3：无响应的命令 ----------
    print("\n【示例 3】轴指令 —— 没有响应帧，只能看状态")
    pkt = build_packet(*CMD_AXIS, {"X": 0.12, "Y": 0.0, "Z": 0.0,
                                   "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0})
    print(f"  正文: {pkt[HEADER_LEN:].decode('utf-8')}")
    print("  注意 X 是【最大速度的比例】，范围 [-1,1]（文档 §1.2.5），不是 m/s")
    if do_send:
        print("  以 20Hz 发 1 秒 ...")
        end = time.monotonic() + 1.0
        n = 0
        while time.monotonic() < end:
            # 每帧都要重新打包：msgId 必须逐帧递增（文档 §1.1.5）
            sock.sendto(build_packet(*CMD_AXIS,
                                     {"X": 0.12, "Y": 0.0, "Z": 0.0,
                                      "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
                                     msg_id=n), target)
            n += 1
            time.sleep(0.05)
        print(f"  已发 {n} 帧，现在归零 ...")
        for i in range(10):
            sock.sendto(build_packet(*CMD_AXIS,
                                     {"X": 0.0, "Y": 0.0, "Z": 0.0,
                                      "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
                                     msg_id=n + i), target)
            time.sleep(0.05)
        print("  已归零")

    sock.close()
    print("\n" + "=" * 62)
    print(" Demo 结束")
    if not do_send:
        print(" 本次是 DRY-RUN，什么都没发出去。")
        print(f" 要真的发送：python3 {sys.argv[0]} --send")
    print("=" * 62)


if __name__ == "__main__":
    main()
