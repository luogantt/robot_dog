#!/usr/bin/env python3
"""
机器狗 SDK 启动器（Windows / Linux 通用）

用法：
    python start.py              弹出菜单
    python start.py check        只检查环境，不启动任何服务
    python start.py status       查询真机状态（只读，只发心跳）
    python start.py sim          本机仿真（模拟机器狗 + Web 界面）
    python start.py observe      只读观察真机（不发任何运动指令）
    python start.py control      控制真机（可操控，谨慎）

Windows 上双击 start.bat 即可；Linux（机器狗主机）上用 ./start.sh 或直接跑本文件。

把菜单逻辑放在 Python 而不是批处理里，是因为 cmd.exe 按当前代码页逐字节读 .bat，
中文会让字节错位、把后面的行解析错。批处理只保留 ASCII 外壳。
"""

import argparse
import os
import socket
import subprocess
import sys
import threading
import time
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))

ROBOT_HOST = os.environ.get("ROBOT_HOST", "10.21.33.103")
ROBOT_PORT = int(os.environ.get("ROBOT_PORT", "30004"))
WEB_PORT = int(os.environ.get("WEB_PORT", "8000"))
SIM_PORT = int(os.environ.get("SIM_PORT", "31004"))

BAR = "=" * 60


def say(msg=""):
    print(msg, flush=True)


def title(text):
    say()
    say(f"  {BAR}")
    say(f"    {text}")
    say(f"  {BAR}")


# ---------------------------------------------------------------- 检查

def check_deps():
    missing = []
    for mod in ("fastapi", "uvicorn", "websockets"):
        try:
            __import__(mod)
        except ImportError:
            missing.append(mod)
    return missing


def port_in_use(port, host="127.0.0.1"):
    """端口是否已被占用（用来提前发现 '8000 已被别的程序占了'）。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex((host, port)) == 0


def do_check():
    title("环境自检")
    say(f"  Python      {sys.version.split()[0]}  ({sys.executable})")
    missing = check_deps()
    if missing:
        say(f"  依赖        缺少: {', '.join(missing)}")
        say()
        say("  安装命令:")
        say(f"      {sys.executable} -m pip install -r requirements.txt")
        say()
        return 1
    say("  依赖        fastapi / uvicorn / websockets 齐备")
    say(f"  项目目录    {HERE}")
    say()
    say("  环境就绪。可以运行:")
    say("      python start.py sim       本机仿真，先玩这个")
    say("      python start.py observe   只读观察真机")
    say("      python start.py control   控制真机")
    say()
    return 0


# ---------------------------------------------------------------- 各模式

def do_status():
    title("查询真机状态（只读，只发心跳）")
    say(f"  机器人  {ROBOT_HOST}:{ROBOT_PORT}")
    say()
    return subprocess.call([sys.executable, os.path.join(HERE, "tools", "asdu_cli.py"),
                            "status", "10"], cwd=HERE)


def _run_web(extra_args, open_browser=True):
    """前台运行 Web 服务；顺带在几秒后自动打开浏览器。"""
    if port_in_use(WEB_PORT):
        say(f"  [错误] 端口 {WEB_PORT} 已被占用。")
        say("         可能是上一次的服务还在跑，或别的程序占了这个端口。")
        say(f"         换个端口: 设置环境变量 WEB_PORT 后重试")
        return 1

    if open_browser:
        threading.Timer(3.0, lambda: webbrowser.open(f"http://127.0.0.1:{WEB_PORT}")).start()

    cmd = [sys.executable, os.path.join(HERE, "web_control.py"), "--web-port", str(WEB_PORT)]
    cmd += extra_args
    try:
        return subprocess.call(cmd, cwd=HERE)
    except KeyboardInterrupt:
        return 0


def do_observe():
    title("只读观察模式")
    say("  只发心跳，不向机器狗发送任何运动指令。")
    say("  摇杆和按钮只影响本地显示，机器狗不会动。")
    say()
    say(f"  机器人    {ROBOT_HOST}:{ROBOT_PORT}")
    say(f"  Web 界面  http://127.0.0.1:{WEB_PORT}")
    say()
    say("  按 Ctrl+C 停止。")
    return _run_web(["--observe"])


def do_sim():
    title("本机仿真模式")
    say("  模拟机器狗 + Web 界面，完全不碰真机。")
    say()
    say(f"  模拟机器狗  127.0.0.1:{SIM_PORT}  (子进程)")
    say(f"  Web 界面    http://127.0.0.1:{WEB_PORT}")
    say()
    say("  按 Ctrl+C 停止（会同时停掉模拟器）。")
    say()

    sim = subprocess.Popen([sys.executable, os.path.join(HERE, "robot_sim.py"), str(SIM_PORT)],
                           cwd=HERE)
    try:
        time.sleep(1.5)
        return _run_web(["--host", "127.0.0.1", "--port", str(SIM_PORT), "--scale", "0.3"])
    finally:
        sim.terminate()
        try:
            sim.wait(timeout=3)
        except subprocess.TimeoutExpired:
            sim.kill()


def do_control():
    title("控制真机 —— 请先确认以下四件事")
    say()
    say("    1. 机器狗在平地上（不在桌上、台阶上、斜坡上）")
    say("    2. 周围 1 米内无人无物")
    say("    3. 有人在旁边看护")
    say("    4. 手机 App 的手动控制已关闭")
    say()
    say("  操作顺序   接管控制 -> 起立 -> 确认方向 -> 再走")
    say("  急停       界面上的红色按钮，或机器狗本体的物理急停")
    say()
    say("  速度       0.50 起步（约 0.83 m/s），滑条可拉到 1.00（约 1.67 m/s）。")
    say("             1.00 接近小跑，室内慎用 —— 拖到 0.80 以上数字会变红。")
    say()
    say(f"  机器人    {ROBOT_HOST}:{ROBOT_PORT}")
    say(f"  Web 界面  http://127.0.0.1:{WEB_PORT}")
    say()
    if not SKIP_CONFIRM:
        try:
            input("  确认无误请按回车继续（Ctrl+C 取消）: ")
        except (KeyboardInterrupt, EOFError):
            say("\n  已取消。")
            return 0
    say()
    return _run_web(["--scale", "0.50"])


# ---------------------------------------------------------------- 菜单

def menu():
    while True:
        title("机器狗 SDK")
        say()
        say("    [1]  本机仿真      模拟机器狗 + Web 界面（不碰真机，先玩这个）")
        say("    [2]  只读观察真机   只发心跳，不发任何运动指令")
        say("    [3]  控制真机       可操控有风险的，请先确认环境安全")
        say("    [4]  查询真机状态   命令行只读检查（只发心跳）")
        say("    [5]  环境自检       只检查 Python 和依赖")
        say("    [0]  退出")
        say()
        try:
            n = input("  请输入序号: ").strip()
        except (KeyboardInterrupt, EOFError):
            say()
            return 0
        return {"1": "sim", "2": "observe", "3": "control",
                "4": "status", "5": "check", "0": None}.get(n, "?")


# ---------------------------------------------------------------- 入口

SKIP_CONFIRM = False   # --yes 时跳过"控制真机"的确认提示（供脚本/远程调用）

RUNNERS = {
    "check": do_check,
    "status": do_status,
    "sim": do_sim,
    "observe": do_observe,
    "control": do_control,
}


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("mode", nargs="?",
                    choices=["check", "status", "sim", "observe", "control"],
                    help="不指定则弹出菜单")
    ap.add_argument("-y", "--yes", action="store_true",
                    help="跳过「控制真机」的确认提示")
    args = ap.parse_args()

    global SKIP_CONFIRM
    SKIP_CONFIRM = args.yes

    mode = args.mode
    if mode is None:
        while True:
            mode = menu()
            if mode is None:
                return 0
            if mode != "?":
                break
            say("  无效的序号，请重新选择。")
            time.sleep(1)

    if mode != "check":
        missing = check_deps()
        if missing:
            title("缺少依赖")
            say(f"  缺少: {', '.join(missing)}")
            say()
            say(f"  安装: {sys.executable} -m pip install -r requirements.txt")
            say()
            return 1

    return RUNNERS[mode]()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        say("\n  已中断。")
        sys.exit(0)
