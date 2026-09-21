#!/bin/bash
# ============================================================================
# 机器狗 Web 遥控【high_platform 版】—— 停止
#
#   ./stop_web_high.sh
#
# 启动用 ./start_web_high.sh
#
# ⚠️ 这不是急停。停服务只是让机器人收不到新指令（0.4 秒看门狗超时后归零）。
#    如果你正在让它走，先点界面上的【急停】，或者直接按机器人本体的物理急停。
#
# ★ 与原版 stop_web.sh 的关键差异：回退查找【按端口】而不是按进程名 ★
#   原版用 `pgrep -f '[w]eb_control\.py'` 兜底 —— 那会匹配到**任何一份**安装的
#   web_control（robot_dog_web / robot_sdk_high / 甚至你自己在别处手动起的），
#   PID 文件一丢就会杀错进程。端口是唯一标识一份安装的，按它找不会误伤。
# ============================================================================

APP_DIR=/home/user/robot_sdk_high
WEB_PORT=8000
PIDFILE="$APP_DIR/web.pid"

if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; N=$'\e[0m'
else R=; G=; Y=; N=; fi
ok()   { echo "  ${G}✓${N} $*"; }
bad()  { echo "  ${R}✗${N} $*"; }
warn() { echo "  ${Y}!${N} $*"; }
hr()   { echo "──────────────────────────────────────────────────────"; }

# 听在 $WEB_PORT 上的进程 —— 端口唯一标识一份安装
port_owner() {
    ss -ltnp 2>/dev/null | grep ":$WEB_PORT " \
        | grep -oP 'pid=\K[0-9]+' | head -1
}

# 先按本安装的 PID 文件找
PID=""
if [ -f "$PIDFILE" ]; then
    P=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$P" ] && kill -0 "$P" 2>/dev/null && PID="$P"
fi
# PID 文件不管用就按端口找（注意：不按进程名 —— 见文件头）
if [ -z "$PID" ]; then
    PID=$(port_owner)
fi

hr
if [ -z "$PID" ]; then
    warn "没在跑（PID 文件和 $WEB_PORT 端口都没有）"
    rm -f "$PIDFILE"
    hr; exit 0
fi

# 保险：确认要杀的确实是本安装的 web_control，而不是别的什么占了这个端口
CMDLINE=$(tr '\0' ' ' < "/proc/$PID/cmdline" 2>/dev/null)
if ! echo "$CMDLINE" | grep -q 'web_control\.py'; then
    bad "端口 $WEB_PORT 上跑的【不是】web_control —— 拒绝杀它"
    echo "       PID $PID: $CMDLINE"
    echo "       这是别的东西占了这个端口，手工处理吧。"
    hr; exit 1
fi

echo " 停止 high_platform Web 遥控（PID $PID）..."
echo "       $CMDLINE"
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
    warn "端口 $WEB_PORT 还被占着（应该是别的进程了）"
    echo "       看看是谁：ss -ltnp | grep :$WEB_PORT"
else
    ok "端口 $WEB_PORT 已释放"
fi

# 顺带提醒另一份安装的状态 —— 免得以为"都停了"其实旧那份还在发指令
OTHER=/home/user/robot_dog_web
if [ -f "$OTHER/web.pid" ] && kill -0 "$(cat "$OTHER/web.pid" 2>/dev/null)" 2>/dev/null; then
    echo
    warn "注意：另一份安装（$OTHER）还在跑（PID $(cat "$OTHER/web.pid")）"
    echo "       它也是轴指令发送源。要停它：cd $OTHER && ./stop_web.sh"
fi
hr
