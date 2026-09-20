#!/bin/bash
# ============================================================================
# 机器狗 Web 遥控 —— 启动
#
#   ./start_web.sh
#
# 停止用 ./stop_web.sh
#
# 为什么部署在 .103（机器人本体）而不是外部电脑：
#   Web 遥控不需要相机，而在本机上 ASDU 走 127.0.0.1 本地通信，
#   跨网络的路由 / 多网卡 / 源端点冲突（0xE006 要求 2 秒内轴指令同源）
#   在结构上就不存在了。
# ============================================================================

# ---- 配置（要改就改这里）----
VENV=/home/user/web_env
APP_DIR=/home/user/robot_dog_web
WEB_PORT=8000
ROBOT_HOST=127.0.0.1          # 本机就是机器人，所以是回环地址
ROBOT_PORT=30004
SCALE=0.50                    # 起步速度（界面上滑条可以再往右拉到 1.00）

PY="$VENV/bin/python"
LOG="$APP_DIR/web.log"
PIDFILE="$APP_DIR/web.pid"

if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[36m'; N=$'\e[0m'
else R=; G=; Y=; B=; N=; fi
ok()   { echo "  ${G}✓${N} $*"; }
bad()  { echo "  ${R}✗${N} $*"; }
warn() { echo "  ${Y}!${N} $*"; }
hr()   { echo "──────────────────────────────────────────────────────"; }

is_running() {
    [ -f "$PIDFILE" ] || return 1
    local pid; pid=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

# ---------------------------------------------------------------- 启动前检查
fail=0
hr; echo " 启动前检查"; hr

[ -x "$PY" ] && ok "Python 环境   $PY" \
    || { bad "找不到 $PY"; fail=1; }
[ -f "$APP_DIR/web_control.py" ] && ok "程序         $APP_DIR/web_control.py" \
    || { bad "找不到 web_control.py"; fail=1; }
[ -d "$APP_DIR/web" ] && ok "前端资源     $APP_DIR/web/" \
    || { bad "找不到 web/ 目录"; fail=1; }
"$PY" -c 'import fastapi,uvicorn,websockets' 2>/dev/null \
    && ok "依赖         fastapi / uvicorn / websockets" \
    || { bad "依赖缺失，装一下："; \
         echo "        $VENV/bin/pip install -i https://pypi.mirrors.ustc.edu.cn/simple fastapi uvicorn websockets"; fail=1; }

if is_running; then
    warn "已经在跑了（PID $(cat "$PIDFILE")）—— 不用重复启动"
    echo "       要重启：./stop_web.sh && ./start_web.sh"
    hr; exit 0
fi
if ss -ltn 2>/dev/null | grep -q ":$WEB_PORT "; then
    bad "端口 $WEB_PORT 被别的进程占着 —— 先停掉它"
    echo "       看看是谁：ss -ltnp | grep :$WEB_PORT"
    hr; exit 1
fi
ok "端口 $WEB_PORT    空闲"

if [ "$fail" != 0 ]; then hr; echo " ${R}检查未通过，先解决上面标 ✗ 的项${N}"; hr; exit 1; fi
hr

# ---------------------------------------------------------------- 启动
echo
echo "启动中 ..."
cd "$APP_DIR" || exit 1
setsid nohup "$PY" web_control.py \
    --host "$ROBOT_HOST" --port "$ROBOT_PORT" \
    --web-port "$WEB_PORT" --scale "$SCALE" \
    > "$LOG" 2>&1 < /dev/null &
echo $! > "$PIDFILE"

# 等它连上机器人（最多 12 秒）
for i in $(seq 1 24); do
    sleep 0.5
    grep -q "已连上机器人" "$LOG" 2>/dev/null && break
    is_running || break
done
echo

hr
if is_running && grep -q "已连上机器人" "$LOG" 2>/dev/null; then
    ok "已启动（PID $(cat "$PIDFILE")），并连上机器人"
    grep -m1 "已连上机器人" "$LOG" | sed 's/^/    /'
    IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    echo
    echo "  ${B}浏览器打开：http://${IP}:$WEB_PORT${N}"
    echo "  （本机也可用 http://127.0.0.1:$WEB_PORT）"
    echo
    echo "  ${Y}用的时候三步：${N}"
    echo "    ① 点【接管控制】   ← 不点的话按键全部被静默丢弃"
    echo "    ② 点【起立】       ← 必须进 RL 控制(17) 才能接受轴指令"
    echo "    ③ 按住方向键       ← 是【按住】，点一下几乎没反应"
    echo
    echo "  停止：./stop_web.sh      看日志：tail -f $LOG"
elif is_running; then
    warn "进程起来了（PID $(cat "$PIDFILE")），但没看到「已连上机器人」"
    echo "       可能机器人服务没跑，或 30004 的加密没关。日志末尾："
    tail -12 "$LOG" 2>/dev/null | sed 's/^/    /'
    exit 1
else
    bad "启动失败。日志末尾："
    tail -12 "$LOG" 2>/dev/null | sed 's/^/    /'
    rm -f "$PIDFILE"
    exit 1
fi
hr
