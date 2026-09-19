#!/usr/bin/env python3
"""End-to-end test of web_control.py against robot_sim.py. No browser needed."""
import asyncio
import json
import os
import subprocess
import sys
import time
import urllib.request

# 项目根目录 = 本文件所在目录的上一级
SDK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = sys.executable
SIM_PORT = 31050
WEB_PORT = 8055

RESULTS = []


def check(label, ok, detail=""):
    RESULTS.append((label, ok, detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {label}" + (f"  [{detail}]" if detail else ""), flush=True)


def start(cmd, name):
    print(f"[启动] {name}: {' '.join(cmd)}", flush=True)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, encoding="utf-8", errors="replace")
    return p


async def main():
    import websockets

    sim = start([PY, os.path.join(SDK, "robot_sim.py"), str(SIM_PORT)], "模拟机器人")
    time.sleep(1.5)
    srv = start([PY, os.path.join(SDK, "web_control.py"), "--host", "127.0.0.1",
                 "--port", str(SIM_PORT), "--web-port", str(WEB_PORT)], "Web 服务")
    time.sleep(4.0)

    try:
        # ---------- 1) HTTP 静态页 ----------
        print("\n### 1) HTTP 接口")
        html = urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/", timeout=5).read().decode("utf-8")
        check("GET / 返回页面", "机器狗遥控" in html, f"{len(html)} 字节")
        js = urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/app.js", timeout=5).read().decode("utf-8")
        check("GET /app.js 返回脚本", "WebSocket" in js, f"{len(js)} 字节")
        css = urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/style.css", timeout=5).read().decode("utf-8")
        check("GET /style.css 返回样式", ".joy" in css, f"{len(css)} 字节")

        # ---------- 2) WebSocket ----------
        print("\n### 2) WebSocket 连接与状态推送")
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws:
            hello = json.loads(await asyncio.wait_for(ws.recv(), 5))
            check("收到 hello", hello.get("t") == "hello", str(hello))
            my_id = hello.get("you")

            st = None
            for _ in range(30):
                m = json.loads(await asyncio.wait_for(ws.recv(), 5))
                if m.get("t") == "state":
                    st = m
                    break
            check("收到 state", st is not None)
            check("读到机身状态", st.get("motion_state") is not None,
                  f"MotionState={st.get('motion_state')} Gait={st.get('gait')} "
                  f"Model={st.get('model')}")
            check("初始无控制权", st.get("can_control") is False)
            check("步态选项已下发", len(st.get("gaits") or []) == 4,
                  str([g["label"] for g in (st.get("gaits") or [])]))
            # 温度面板数据来自"设备状态上报"（实机 Type=0x00300002）。模拟器也要发，
            # 否则这条测不到 —— 这是唯一覆盖该上报路径的断言。
            temps = st.get("temps") or []
            check("温度数据已下发（16 路关节）", len(temps) == 16,
                  f"{len(temps)} 路, 最高 {st.get('temp_max')}°C")
            if temps:
                check("温度含电机与驱动器两路",
                      all(t.get("motor") is not None and t.get("driver") is not None
                          for t in temps),
                      str(temps[0]))

            # ---------- 3) 控制权 ----------
            print("\n### 3) 控制权")
            await ws.send(json.dumps({"t": "claim"}))
            for _ in range(20):
                m = json.loads(await asyncio.wait_for(ws.recv(), 5))
                if m.get("t") == "state" and m.get("can_control"):
                    st = m
                    break
            check("claim 后取得控制权", st.get("can_control") is True,
                  f"controller={st.get('controller')} you={my_id}")

            # ---------- 4) 状态门：趴下时不应运动 ----------
            print("\n### 4) 状态门（趴下状态应拒绝运动）")
            for _ in range(10):
                await ws.send(json.dumps({"t": "intent", "x": 1.0, "y": 0, "yaw": 0}))
                await asyncio.sleep(0.1)
            st = await latest_state(ws)
            check("趴下时速度仍为 0（状态门生效）", abs(st.get("linear_x") or 0) < 0.01,
                  f"LinearX={st.get('linear_x')} gate={st.get('gate')}")

            # ---------- 5) 起立 ----------
            print("\n### 5) 起立动作")
            await ws.send(json.dumps({"t": "action", "name": "stand"}))
            st = await wait_for(ws, lambda s: s.get("motion_state") == 17, 25)
            check("起立后进入 RL 控制(17)", st is not None and st.get("motion_state") == 17,
                  f"MotionState={st.get('motion_state') if st else '超时'}")
            # 必须等服务端把动作标记为结束。机器人状态先变、服务端轮询后知，
            # 这个窗口内的按键会被服务端按"动作执行中"丢弃并置上 must_release，
            # 导致后面一直按不动 —— 那是测试的竞态，不是服务端的问题。
            await action_done(ws)

            # ---------- 6) 运动 ----------
            print("\n### 6) 按住运动")
            # 必须用 hold 持续喂意图：只发一次的话 400ms 后看门狗会（正确地）归零
            st = await hold(ws, {"x": 1.0, "y": 0, "yaw": 0}, 1.0)
            check("按住后开始运动", abs(st.get("linear_x") or 0) > 0.01,
                  f"LinearX={st.get('linear_x')} gate={st.get('gate')!r} "
                  f"must_release={st.get('must_release')} out={st.get('out')}")
            check("速度爬升到限幅值附近", abs(st.get("linear_x") or 0) > 0.15,
                  f"LinearX={st.get('linear_x')}")
            st = await hold(ws, {"x": 1.0, "y": 0, "yaw": 0}, 1.0)
            check("持续意图下维持运动", abs(st.get("linear_x") or 0) > 0.15,
                  f"LinearX={st.get('linear_x')}")

            # ---------- 6b) 平移符号 ----------
            # 文档 §1.2.5：Y=+1 是左移、Y=-1 是右移，**实机与文档一致**。
            # 所以后端【不能】像 Yaw 那样取反 —— 界面「左移」发 y=+1，机器人必须收到正值。
            # 之前界面把「左移」映射成 y=-1，实机就往右走，是界面写错了。
            print("\n### 6b) 平移符号（Y 轴不取反）")
            st = await hold(ws, {"x": 0.0, "y": 1.0, "yaw": 0.0}, 0.8)
            ly = st.get("linear_y")
            check("「左移」(y=+1) → 实机 LinearY 为正",
                  isinstance(ly, (int, float)) and ly > 0.01,
                  f"意图 y=+1，实机 LinearY={ly}")

            # ---------- 6b) 转向符号 ----------
            # 实机实测：文档 §1.2.5 的右手坐标系约定与实机相反，下发 +Yaw 实际左转。
            # 所以后端必须取反（YAW_SIGN=-1），让界面上的"右转"真的右转。
            # 这条钉住这个约定，避免以后改动把它翻回去。
            print("\n### 6c) 转向符号（YAW_SIGN）")
            st = await hold(ws, {"x": 0.0, "y": 0.0, "yaw": 1.0}, 0.8)
            wz = st.get("angular_z")
            check("界面「右转」→ 实机收到的 Yaw 为负（已按 YAW_SIGN 取反）",
                  isinstance(wz, (int, float)) and wz < -0.01,
                  f"意图 yaw=+1，实机 AngularZ={wz}")

            # ---------- 7) 看门狗（最关键） ----------
            print("\n### 7) 看门狗：停止发送意图应自动归零")
            t0 = time.monotonic()
            st = await wait_for(ws, lambda s: abs(s.get("linear_x") or 0) < 0.01, 3)
            dt = time.monotonic() - t0
            check("停发意图后自动归零", st is not None, f"耗时 {dt:.2f}s")
            check("归零足够快（<1.5s）", dt < 1.5, f"{dt:.2f}s")

            # ---------- 8) 断言 WebSocket 断开也归零 ----------
            print("\n### 8) 断开 WebSocket 后归零")
            st = await hold(ws, {"x": 1.0, "y": 0, "yaw": 0}, 0.6)
            moving = abs(st.get("linear_x") or 0) > 0.01
            check("断开前确实在动", moving, f"LinearX={st.get('linear_x')}")

        # ws 已关闭 —— 重新连一个客户端观察机器人是否停了
        async with websockets.connect(f"ws://127.0.0.1:{WEB_PORT}/ws") as ws2:
            st = await wait_for(ws2, lambda s: abs(s.get("linear_x") or 0) < 0.01, 4, idle=True)
            check("WebSocket 断开后速度归零", st is not None,
                  f"LinearX={st.get('linear_x') if st else '未归零'}")
            check("断开后控制权已释放", st is not None and st.get("controller") is None,
                  f"controller={st.get('controller') if st else '?'}")

            # ---------- 9) 急停 ----------
            print("\n### 9) 急停")
            await ws2.send(json.dumps({"t": "claim"}))
            await wait_for(ws2, lambda s: s.get("can_control"), 5, idle=True)
            st = await wait_for(ws2, lambda s: s.get("motion_state") == 17, 10, idle=True,
                                poke={"t": "intent", "x": 1.0, "y": 0, "yaw": 0})
            # 触发急停
            await ws2.send(json.dumps({"t": "estop"}))
            st = await wait_for(ws2, lambda s: s.get("estop") is True, 3, idle=True)
            check("急停置位", st is not None and st.get("estop") is True)

            # 急停期间持续发意图，速度必须保持 0
            for _ in range(10):
                await ws2.send(json.dumps({"t": "intent", "x": 1.0, "y": 0, "yaw": 0}))
                await asyncio.sleep(0.1)
            st = await latest_state(ws2)
            check("急停期间拒绝运动", abs(st.get("linear_x") or 0) < 0.01,
                  f"LinearX={st.get('linear_x')} estop={st.get('estop')}")

            await ws2.send(json.dumps({"t": "estop_reset"}))
            st = await wait_for(ws2, lambda s: s.get("estop") is False, 3, idle=True)
            check("急停可解除", st is not None and st.get("estop") is False)

            # ---------- 10) 步态切换 ----------
            print("\n### 10) 步态切换")
            await ws2.send(json.dumps({"t": "action", "name": "gait", "value": 0x1003}))
            st = await wait_for(ws2, lambda s: s.get("gait") == 0x1003, 15, idle=True)
            check("切换到楼梯步态 0x1003", st is not None and st.get("gait") == 0x1003,
                  f"Gait={hex(st.get('gait')) if st and st.get('gait') else '超时'}")
            await action_done(ws2)

            # ---------- 11) 切到导航步态，验证不自锁 ----------
            # 文档 §1.2.4：步态切换会自动切换运动模式。切到 0x3003 会把使用模式
            # 变成 1（导航），此时 §1.2.5 的比例轴指令失效，必须改用 §1.2.6 真实轴指令。
            # 如果实现用的是静态 profile（只在 mode∈{0,2} 时允许运动），这里会自锁。
            print("\n### 11) 切到导航步态 0x3003（验证模式耦合后不自锁）")
            await ws2.send(json.dumps({"t": "action", "name": "gait", "value": 0x3003}))
            st = await wait_for(ws2, lambda s: s.get("gait") == 0x3003, 15, idle=True)
            await action_done(ws2)
            check("切到 0x3003", st is not None and st.get("gait") == 0x3003,
                  f"Gait={hex(st.get('gait')) if st and st.get('gait') else '超时'}")
            check("使用模式随之变为导航(1)", st is not None and st.get("usage_mode") == 1,
                  f"Mode={st.get('usage_mode') if st else '?'}")
            # cmd_label 由控制循环下一个周期（最多 50ms 后）才更新，这里要等而不是立刻断言
            st2 = await wait_for(ws2, lambda s: "物理量" in (s.get("cmd_label") or ""), 3, idle=True)
            check("轴指令切换为真实轴(物理量)", st2 is not None,
                  f"cmd_label={st2.get('cmd_label') if st2 else '未切换'}")
            st = await hold(ws2, {"x": 1.0, "y": 0, "yaw": 0}, 1.0)
            check("导航模式下仍能运动（说明确实改用了 0x00110002）",
                  abs(st.get("linear_x") or 0) > 0.01,
                  f"LinearX={st.get('linear_x')} gate={st.get('gate')!r}")

            # 切回常规步态，验证还能回去
            await ws2.send(json.dumps({"t": "action", "name": "gait", "value": 0x1001}))
            st = await wait_for(ws2, lambda s: s.get("gait") == 0x1001, 15, idle=True)
            await action_done(ws2)
            check("能切回基础步态 0x1001", st is not None and st.get("gait") == 0x1001,
                  f"Gait={hex(st.get('gait')) if st and st.get('gait') else '超时'}")
            check("使用模式回到常规(0)", st is not None and st.get("usage_mode") == 0,
                  f"Mode={st.get('usage_mode') if st else '?'}")
            st = await hold(ws2, {"x": 1.0, "y": 0, "yaw": 0}, 1.0)
            check("切回后仍能运动", abs(st.get("linear_x") or 0) > 0.01,
                  f"LinearX={st.get('linear_x')} gate={st.get('gate')!r}")

    finally:
        for p, name in ((srv, "Web 服务"), (sim, "模拟机器人")):
            p.terminate()
            try:
                out, _ = p.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                out, _ = p.communicate()
            if out and "--verbose" in sys.argv:
                print(f"\n--- {name} 输出 ---\n{out[-3000:]}")

    print("\n" + "=" * 62)
    fails = [r for r in RESULTS if not r[1]]
    print(f"总计 {len(RESULTS)} 项断言，通过 {len(RESULTS)-len(fails)}，失败 {len(fails)}")
    for label, _, detail in fails:
        print(f"  FAIL: {label} {detail}")
    return 1 if fails else 0


async def hold(ws, vec, seconds):
    """按住 vec 持续喂意图 seconds 秒，返回期间最后一次 state。

    必须持续喂 —— 否则看门狗（400ms）会正确地让机器人停下，
    那样测出来的就是"意图停发"而不是"持续运动"。
    """
    import websockets
    last = None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            await ws.send(json.dumps({"t": "intent", **vec}))
        except Exception:
            break
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), 0.05))
            if m.get("t") == "state":
                last = m
        except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosed):
            pass
        await asyncio.sleep(0.05)
    return last or {}


async def action_done(ws, timeout=25):
    """等服务端的当前动作彻底结束（action_running=False）。

    动作会一直占用到服务端侧 wait_for_state 轮询结束，比机器人状态变化晚一点；
    不等的话下一个动作会被服务端按"已有动作在执行"丢弃（这是正确的行为）。
    """
    import websockets
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), 0.5))
        except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosed):
            continue
        if m.get("t") == "state" and not m.get("action_running"):
            return m
    return None


async def latest_state(ws, timeout=3):
    """读到最新的 state 消息（不喂意图，用于观察"停发后会怎样"）。"""
    import websockets
    last = None
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), 0.3))
        except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosed):
            break
        if m.get("t") == "state":
            last = m
    return last or {}


async def wait_for(ws, pred, timeout, idle=False, poke=None):
    """等到 pred(state) 为真。poke 是每次轮询时顺带发送的消息（用于持续喂意图）。"""
    import websockets
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if poke:
            try:
                await ws.send(json.dumps(poke))
            except Exception:
                pass
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), 0.5))
        except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosed):
            continue
        if m.get("t") == "state" and pred(m):
            return m
    return None


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        sys.exit(130)
