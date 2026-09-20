#!/bin/bash
# ============================================================================
# 机器狗 Web 遥控 —— 停止
#
#   ./stop_web.sh
#
# 启动用 ./start_web.sh
#
# ⚠️ 这不是急停。停服务只是让机器人收不到新指令（0.4 秒看门狗超时后归零）。
#    如果你正在让它走，先点界面上的【急停】，或者直接按机器人本体的物理急停。
# ============================================================================

APP_DIR=/home/user/robot_dog_web
WEB_PORT=8000
PIDFILE="$APP_DIR/web.pid"

if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; N=$'\e[0m'
else R=; G=; Y=; N=; fi
ok()   { echo "  ${G}✓${N} $*"; }
bad()  { echo "  ${R}✗${N} $*"; }
warn() { echo "  ${Y}!${N} $*"; }
hr()   { echo "──────────────────────────────────────────────────────"; }

# 先按 PID 文件找；找不到就回退到按进程名找
# （用 [w] 转义是防止 pgrep 匹配到本脚本自己的命令行）
PID=""
if [ -f "$PIDFILE" ]; then
    P=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$P" ] && kill -0 "$P" 2>/dev/null && PID="$P"
fi
if [ -z "$PID" ]; then
    PID=$(pgrep -f '[w]eb_control\.py' | head -1)
fi

hr
if [ -z "$PID" ]; then
    warn "没在跑（进程和端口都没有）"
    rm -f "$PIDFILE"
    hr; exit 0
fi

echo " 停止 Web 遥控（PID $PID）..."
kill "$PID" 2>/dev/null

# 等它优雅退出，最多 5 秒
for i in $(seq 1 20); do
    sleep 0.25
    kill -0 "$PID" 2>/dev/null || break
done

if kill -0 "$PID" 2>/dev/null; then
    warn "没退干净，强制杀"
    kill -9 "$PID" 2>/dev/null
    sleep 0.5
fi

rm -f "$PIDFILE"
echo
if kill -0 "$PID" 2>/dev/null; then
    bad "还在？可能需要手动处理：kill -9 $PID"
    hr; exit 1
fi

ok "已停止"
if ss -ltn 2>/dev/null | grep -q ":$WEB_PORT "; then
    warn "端口 $WEB_PORT 还被占着（可能是别的进程）"
    echo "       看看是谁：ss -ltnp | grep :$WEB_PORT"
else
    ok "端口 $WEB_PORT 已释放"
fi
hr
