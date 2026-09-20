#!/bin/bash
# ============================================================================
# 机器狗本机 Web 遥控 —— 启停脚本
#
#   ./start_web.sh start      启动（已在跑就不重复启动）
#   ./start_web.sh stop       停止
#   ./start_web.sh restart    重启
#   ./start_web.sh status     看状态（进程 / 端口 / 机器人连接）
#   ./start_web.sh log        实时跟踪日志（Ctrl+C 退出，不影响服务）
#   ./start_web.sh check      环境自检（不启动）
#
# 为什么部署在 .103 本机而不是外部电脑：
#   ASDU 走 127.0.0.1 本地通信，绕开跨网络带来的路由 / 多网卡 / 源端点
#   冲突问题（0xE006 要求 2 秒内轴指令同源，跨网络时很容易踩）。
# ============================================================================

# ---- 配置（要改就改这里）----
VENV=/home/user/web_env
APP_DIR=/home/user/robot_dog_web
WEB_PORT=8000
ROBOT_HOST=127.0.0.1          # 本机就是机器人，所以是回环地址
ROBOT_PORT=30004
SCALE=0.50                    # 起步速度（Web 界面上的滑条可以再往右拉）
PY="$VENV/bin/python"
LOG="$APP_DIR/web.log"
PIDFILE="$APP_DIR/web.pid"

# ---- 颜色 ----
if [ -t 1 ]; then
  R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[36m'; N=$'\e[0m'
else
  R=; G=; Y=; B=; N=
fi
ok()   { echo "  ${G}✓${N} $*"; }
bad()  { echo "  ${R}✗${N} $*"; }
warn() { echo "  ${Y}!${N} $*"; }
hr()   { echo "──────────────────────────────────────────────────────"; }

is_running() {
    [ -f "$PIDFILE" ] || return 1
    local pid; pid=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

# ---------------------------------------------------------------- 自检
do_check() {
    local fail=0
    hr; echo " 环境自检"; hr

    [ -x "$PY" ] && ok "Python 环境   $PY" || { bad "找不到 $PY"; fail=1; }
    [ -f "$APP_DIR/web_control.py" ] && ok "程序         $APP_DIR/web_control.py" \
        || { bad "找不到 web_control.py"; fail=1; }
    [ -d "$APP_DIR/web" ] && ok "前端资源     $APP_DIR/web/" \
        || { bad "找不到 web/ 目录"; fail=1; }

    "$PY" -c 'import fastapi,uvicorn,websockets' 2>/dev/null \
        && ok "依赖         fastapi / uvicorn / websockets 都在" \
        || { bad "依赖缺失，装一下："; \
             echo "        $VENV/bin/pip install -i https://pypi.mirrors.ustc.edu.cn/simple fastapi uvicorn websockets"; fail=1; }

    # 端口占用（自己没在跑的情况下被别人占着才是问题）
    if ss -ltn 2>/dev/null | grep -q ":$WEB_PORT "; then
        if is_running; then ok "端口 $WEB_PORT    本服务在用"
        else bad "端口 $WEB_PORT    被别的进程占了（换 --web-port 或先停掉它）"; fail=1; fi
    else
        ok "端口 $WEB_PORT    空闲"
    fi

    hr
    if [ "$fail" = 0 ]; then
        echo " ${G}自检通过，可以 ./start_web.sh start${N}"
    else
        echo " ${R}自检未通过，先解决上面标 ✗ 的项${N}"
    fi
    hr
    return $fail
}

# ---------------------------------------------------------------- 启动
do_start() {
    if is_running; then
        warn "已经在跑了（PID $(cat "$PIDFILE")）—— 要重启用 restart"
        return 0
    fi
    if ss -ltn 2>/dev/null | grep -q ":$WEB_PORT "; then
        bad "端口 $WEB_PORT 被别的进程占着，先停掉它或换个端口"
        return 1
    fi

    echo "启动 Web 遥控 ..."
    cd "$APP_DIR" || return 1
    setsid nohup "$PY" web_control.py \
        --host "$ROBOT_HOST" --port "$ROBOT_PORT" \
        --web-port "$WEB_PORT" --scale "$SCALE" \
        > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"

    # 等它起来并连上机器人（最多 12 秒）
    local i
    for i in $(seq 1 24); do
        sleep 0.5
        grep -q "已连上机器人" "$LOG" 2>/dev/null && break
        is_running || break
    done
    echo

    hr
    if is_running && grep -q "已连上机器人" "$LOG" 2>/dev/null; then
        ok "已启动（PID $(cat "$PIDFILE")），并且连上机器人"
        grep -m1 "已连上机器人" "$LOG" | sed 's/^/    /'
        echo
        echo "  ${B}打开浏览器：http://$(hostname -I 2>/dev/null | awk '{print $1}'):$WEB_PORT${N}"
        echo "  （本机也可用 http://127.0.0.1:$WEB_PORT）"
        echo
        echo "  用的时候三步："
        echo "    ① 点【接管控制】   ← 不点的话按键全部被静默丢弃"
        echo "    ② 点【起立】       ← 必须进 RL 控制(17) 才能接受轴指令"
        echo "    ③ ${Y}按住${N}方向键     ← 是【按住】，点一下几乎没反应"
    elif is_running; then
        warn "进程起来了（PID $(cat "$PIDFILE")），但没看到'已连上机器人'"
        echo "       可能机器人服务没跑，或 30004 的加密没关。看日志："
        echo "       ./start_web.sh log"
    else
        bad "启动失败，日志末尾："
        tail -12 "$LOG" 2>/dev/null | sed 's/^/    /'
        return 1
    fi
    hr
}

# ---------------------------------------------------------------- 停止
do_stop() {
    if ! is_running; then
        warn "没在跑"
        rm -f "$PIDFILE"
        return 0
    fi
    local pid; pid=$(cat "$PIDFILE")
    echo "停止 PID $pid ..."
    kill "$pid" 2>/dev/null
    local i
    for i in $(seq 1 20); do
        sleep 0.25
        kill -0 "$pid" 2>/dev/null || break
    done
    if kill -0 "$pid" 2>/dev/null; then
        warn "没退干净，强制杀"
        kill -9 "$pid" 2>/dev/null
        sleep 0.5
    fi
    rm -f "$PIDFILE"
    ok "已停止"
    echo
    echo "  ${Y}注意${N}：停止后机器人会收到 0.4 秒看门狗超时的零速指令，"
    echo "        但如果你正在让它走，建议先用界面上的【急停】再停服务。"
}

# ---------------------------------------------------------------- 状态
do_status() {
    hr; echo " Web 遥控状态"; hr
    if is_running; then
        local pid; pid=$(cat "$PIDFILE")
        ok "进程     运行中（PID $pid，已跑 $(ps -o etime= -p "$pid" 2>/dev/null | tr -d ' ')）"
    else
        bad "进程     没在跑"
    fi

    if ss -ltn 2>/dev/null | grep -q ":$WEB_PORT "; then
        ok "端口     $WEB_PORT 已监听"
    else
        bad "端口     $WEB_PORT 没监听"
    fi

    if [ -f "$LOG" ]; then
        if grep -q "已连上机器人" "$LOG" 2>/dev/null; then
            ok "机器人   已连接"
        else
            warn "机器人    日志里没有'已连上机器人'"
        fi
        echo
        echo "  日志末尾（最近 6 行）："
        tail -6 "$LOG" | sed 's/^/    /'
        echo
        echo "  完整日志：./start_web.sh log"
    fi
    hr
}

case "${1:-}" in
    start)   do_start ;;
    stop)    do_stop ;;
    restart) do_stop; echo; do_start ;;
    status)  do_status ;;
    log)     tail -f "$LOG" ;;
    check)   do_check ;;
    *)
        sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
        echo
        echo "用法：$0 {start|stop|restart|status|log|check}"
        ;;
esac
