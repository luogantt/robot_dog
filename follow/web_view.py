#!/usr/bin/env python3
"""只读观测 Web 界面 —— 纯标准库，随主程序一起跑。

    from web_view import WebView
    view = WebView(snapshot_fn, port=8010)
    view.start()          # 起 daemon 线程，不阻塞调用方
    ...
    view.stop()

★ 本模块【绝不发送任何 ASDU 报文】★
  它只做两件事：调 snapshot_fn() 拿一份状态拷贝、序列化成 JSON 发给浏览器。
  本项目已经吃够"多发送源互相踢"（文档 §1.5 的 0xE006）的亏 ——
  观测界面必须零发送，否则它自己就成了第二个发送源。
  本模块不认识 RobotLink、不碰 socket，数据从哪来完全由调用方决定。

为什么用标准库而不是 FastAPI
--------------------------------------------------------------------------
  follow_env 里没有 fastapi/uvicorn。装它等于给一个**控制程序**引入部署依赖，
  多一个装不上就跑不起来的东西。界面是 5Hz 只读轮询，http.server 完全够用。

线程模型
--------------------------------------------------------------------------
  主程序（20Hz 控制循环）—— 不碰本模块，只负责定时更新共享状态快照。
  HTTPServer 线程        —— accept 连接，每个请求派一个短命线程。
  handler 线程           —— 调 snapshot_fn()、序列化、写回；全程 try/except。

  所以本模块对控制循环的全部影响 = 「snapshot_fn 里加锁读一次」的开销。
  因此 snapshot_fn **必须是 O(1) 拷贝** —— 不能在里面做编解码、I/O 或遍历大数组。
"""

import collections
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# 名字表从 asdu_probe 借 —— 那边是这三个表的单一来源，本模块不另写一份。
# （asdu_probe 的 main() 有 __main__ 保护，import 它没有副作用）
from asdu_probe import GAIT_NAMES, MODE_NAMES, MOTION_NAMES  # noqa: E402


def _maxnum(xs):
    vals = [float(v) for v in (xs or []) if isinstance(v, (int, float))]
    return max(vals) if vals else None


# ---------------------------------------------------------------- 快照组装
#
# 机器人和相机这两块对两个主程序是一样的，放这里组装，保证 JSON 契约只有一处定义；
# 程序各自专有的字段（当前 tag / 正在发的指令 / 状态门原因）由调用方自己塞进
# app 和 det 两块。

def robot_snapshot(link):
    """把 RobotLink 里的机器人状态拼成前端要的那几块。**纯读取，无副作用。**

    这里不加锁：RobotLink 的字段都是标量、或由接收线程整块替换的 list
    （pump() 里已经 list(...) 拷过一份）。最坏情况读到上一帧的值 ——
    对观测无所谓，但不做任何拷贝以外的工作，避免拖慢 HTTP 线程。
    """
    now = time.monotonic()
    dev = link.device or {}
    motor = dev.get("motor") or []
    driver = dev.get("driver") or []
    return {
        "robot": {
            "status_age_s": ((now - link.last_status_t)
                             if link.last_status_t > 0 else None),
            "connected": link.motion_state is not None,
            "motion_state": link.motion_state,
            "motion_name": MOTION_NAMES.get(link.motion_state, ""),
            "gait": link.gait,
            "gait_name": GAIT_NAMES.get(link.gait, ""),
            "mode": link.mode,
            "mode_name": MODE_NAMES.get(link.mode, ""),
            "hes": link.hes,
            "charge": link.charge,
            "model": link.model,
            "version": link.version,
        },
        "vel": dict(link.vel or {}),
        "temps": {
            "age_s": (now - dev["ts"]) if dev.get("ts") else None,
            "motor": motor,
            "driver": driver,
            "motor_max": _maxnum(motor),
            "driver_max": _maxnum(driver),
        },
        "battery": dev.get("battery") or {},
        "faults": [{"code": f.get("Code"),
                    "name": f.get("Name"),
                    "severity": max(f.get("Severities") or [0]),
                    "resources": f.get("Resources") or []}
                   for f in (link.faults or [])],
        "faults_worst": link.worst_severity(),
    }


def camera_snapshot(frames, hits, fps, last_frame_age_s,
                    width=None, height=None, encoding=None):
    """相机健康块。**只报"通畅不通畅"，不传图像。**"""
    return {
        "ok": last_frame_age_s is not None and last_frame_age_s < 1.0,
        "fps": fps,
        "last_frame_age_s": last_frame_age_s,
        "width": width,
        "height": height,
        "encoding": encoding,
        "frames": frames,
        "hits": hits,
        "hit_rate": (hits / frames) if frames else None,
    }


class Events:
    """线程安全的有界事件环，供界面右下角的日志面板用。

    学 robot_dog_sdk/web_control.py 的 Events：单调递增的 id + 前端按 id 增量拉取，
    这样重连的浏览器不会重复显示旧事件。任何线程都能 add()（所以有锁）。
    """

    def __init__(self, cap=300):
        self._lock = threading.Lock()
        self._buf = collections.deque(maxlen=cap)
        self._seq = 0

    def add(self, level, text):
        with self._lock:
            self._seq += 1
            self._buf.append({"id": self._seq, "ts": time.time(),
                              "level": level, "text": text})

    def tail(self, n=25):
        with self._lock:
            return list(self._buf)[-n:]


class CamStats:
    """相机健康计数。检测线程每处理一帧调一次 tick()，HTTP 线程调 snap()。

    只认 msg 的 .width/.height/.encoding 三个属性（鸭子类型），所以不依赖 ROS,
    可以在没有 rclpy 的机器上单测。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.frames = 0
        self.last_frame_ts = 0.0
        self.width = None
        self.height = None
        self.encoding = None
        self.fps = 0.0
        self._fps_t0 = time.monotonic()
        self._fps_n = 0

    def tick(self, msg):
        now = time.monotonic()
        with self._lock:
            self.frames += 1
            self.last_frame_ts = now
            self.width = getattr(msg, "width", None)
            self.height = getattr(msg, "height", None)
            self.encoding = getattr(msg, "encoding", None)
            self._fps_n += 1
            el = now - self._fps_t0
            if el >= 1.0:                     # 每秒结算一次，别每帧都算
                self.fps = self._fps_n / el
                self._fps_t0, self._fps_n = now, 0

    def snap(self):
        """→ camera_snapshot() 要的那几个参数。"""
        with self._lock:
            age = ((time.monotonic() - self.last_frame_ts)
                   if self.last_frame_ts else None)
            return self.frames, self.fps, age, self.width, self.height, self.encoding


def _json_default(o):
    """兜底序列化。

    温度/速度可能是 numpy 标量（相机那几个脚本里到处都是 np）。界面不该因为
    某个值不是原生 float 就整个 500 —— 转成 float，实在不行转字符串。
    """
    try:
        return float(o)
    except Exception:
        return str(o)


def _guess_lan_ip(probe_dst=("10.21.33.103", 30004)):
    """猜本机对外的 IP，用来打印可访问的地址。

    用 UDP connect 问内核选哪张网卡 —— **不实际发包**（connect 对 UDP 只是
    绑定对端地址，不产生流量）。失败就退回 127.0.0.1。
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(probe_dst)
            return s.getsockname()[0]
        finally:
            s.close()
    except Exception:
        return "127.0.0.1"


class WebView:
    """只读观测界面。start() 起线程，stop() 关。

    参数
        snapshot_fn: 无副作用的回调，返回一个可 JSON 序列化的 dict。
                     必须是 O(1) 拷贝（加锁读一次），不要在里面做重活。
        host/port:   默认 0.0.0.0:8010（避开 .103 上 Web 遥控的 8000）。
        web_dir:     静态文件目录，默认本文件同级的 web/。
    """

    def __init__(self, snapshot_fn, host="0.0.0.0", port=8010, web_dir=None):
        self.snapshot_fn = snapshot_fn
        self.host = host
        self.port = port
        self.web_dir = web_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "web")
        self._httpd = None
        self._thread = None
        self.ok = False
        self.err = ""

    # ---------------------------------------------------------------- 启停

    def start(self):
        """起服务。**失败只告警不抛** —— 界面起不来不该拦住控制程序。"""
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                # 5Hz 轮询会把 stderr 刷满，默认的每请求一行必须关掉
                pass

            def do_GET(self):
                try:
                    view._route(self)
                except Exception as e:
                    # 任何异常只让这一个请求 500，绝不冒泡到控制循环
                    try:
                        view._send(self, 500, "text/plain; charset=utf-8",
                                   f"快照生成失败: {e!r}".encode("utf-8"))
                    except Exception:
                        pass

        try:
            httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as e:
            self.err = str(e)
            print(f"[web] 观测界面启动失败（{e}）—— 控制程序继续运行，不影响控制")
            return False

        httpd.daemon_threads = True
        self._httpd = httpd
        self._thread = threading.Thread(target=httpd.serve_forever,
                                        daemon=True, name="web")
        self._thread.start()
        self.ok = True
        print(f"[web] 观测界面已启动  http://{_guess_lan_ip()}:{self.port}/")
        return True

    def stop(self):
        if not self._httpd:
            return
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except Exception:
            pass
        self._httpd = None

    # ---------------------------------------------------------------- 路由

    def _route(self, h):
        path = h.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            return self._file(h, "index.html", "text/html; charset=utf-8")
        if path == "/style.css":
            return self._file(h, "style.css", "text/css; charset=utf-8")
        if path == "/app.js":
            return self._file(h, "app.js", "application/javascript; charset=utf-8")
        if path == "/api/state":
            # 序列化在锁外做 —— snapshot_fn 已经返回拷贝了，这里再怎么慢都不碰共享状态
            snap = self.snapshot_fn()
            body = json.dumps(snap, ensure_ascii=False,
                              default=_json_default).encode("utf-8")
            return self._send(h, 200, "application/json; charset=utf-8", body)
        if path == "/api/health":
            return self._send(h, 200, "application/json; charset=utf-8",
                              b'{"ok":true}')
        return self._send(h, 404, "text/plain; charset=utf-8", b"not found")

    def _file(self, h, name, ctype):
        """每请求读盘 —— 改完 HTML/CSS 刷新即可，不用重启（学 SDK 的做法）。"""
        p = os.path.join(self.web_dir, name)
        try:
            with open(p, "rb") as f:
                body = f.read()
        except OSError as e:
            return self._send(h, 404, "text/plain; charset=utf-8",
                              f"缺少 {name}: {e}".encode("utf-8"))
        return self._send(h, 200, ctype, body)

    def _send(self, h, code, ctype, body):
        h.send_response(code)
        h.send_header("Content-Type", ctype)
        h.send_header("Content-Length", str(len(body)))
        h.send_header("Cache-Control", "no-store")
        h.end_headers()
        h.wfile.write(body)
