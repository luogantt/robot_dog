#!/bin/bash
# ============================================================================
# .102 上的跟随 / Tag 状态机【focus 版】—— 停止
#
#   ./deploy_stop_focus.sh
#   ./deploy_stop_focus.sh --no-crouch     跳过"先让狗趴下"
#
# 启动用 ./deploy_start_focus.sh
#
# ⚠️ 这不是急停。停程序只是让机器人收不到新指令（0.4 秒看门狗超时后归零）。
#    如果你正在让它走，先按机器人本体的物理急停，或把 tag 拿开（看不见就不动）。
#
# ★ 与原版 deploy_stop.sh 的关键差异：回退查找【按端口】而不是按进程名 ★
#   原版用 `pgrep -f '[t]ag_command_fsm\.py|[f]ollow_controller\.py'` 兜底 ——
#   那会匹配到**任何一份**安装的程序（tag_probe / go_focus），PID 文件一丢
#   就会连另一份一起杀掉。8010 端口唯一标识一份安装，按它找不会误伤。
# ============================================================================

DIR=/home/ysc/go_focus
VENV=/home/ysc/follow_env
PY="$VENV/bin/python"
ROBOT=10.21.33.103
VIEW_PORT=8010                    # 观测界面端口 —— 用来唯一识别本安装的进程
PIDFILE="$DIR/run.pid"
LOG="$DIR/run.log"

CROUCH=1                          # 停之前先让狗趴下；--no-crouch 跳过
[ "$1" = "--no-crouch" ] && CROUCH=0

if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; N=$'\e[0m'
else R=; G=; Y=; N=; fi
ok()   { echo "  ${G}✓${N} $*"; }
bad()  { echo "  ${R}✗${N} $*"; }
warn() { echo "  ${Y}!${N} $*"; }
hr()   { echo "──────────────────────────────────────────────────────"; }

# 听在 $VIEW_PORT 上的进程 —— 端口唯一标识一份安装
port_owner() {
    ss -ltnp 2>/dev/null | grep ":$VIEW_PORT " \
        | grep -oP 'pid=\K[0-9]+' | head -1
}

# 先按本安装的 PID 文件找；找不到就按端口找（不按进程名 —— 见文件头）
PID=""
if [ -f "$PIDFILE" ]; then
    P=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$P" ] && kill -0 "$P" 2>/dev/null && PID="$P"
fi
if [ -z "$PID" ]; then
    PID=$(port_owner)
fi

hr
if [ -z "$PID" ]; then
    warn "本安装没在跑（PID 文件和 $VIEW_PORT 端口都没有）"
    rm -f "$PIDFILE"
    hr; exit 0
fi

# 保险：确认要杀的确实是本安装的主程序
CMDLINE=$(tr '\0' ' ' < "/proc/$PID/cmdline" 2>/dev/null)
if ! echo "$CMDLINE" | grep -qE 'tag_command_fsm\.py|follow_controller\.py'; then
    bad "$VIEW_PORT 端口上跑的【不是】本程序 —— 拒绝杀它"
    echo "       PID $PID: $CMDLINE"
    echo "       这是别的东西占了这个端口，手工处理吧。"
    hr; exit 1
fi

# ---------------------------------------------------------------- 先让狗趴下
# 顺序是刻意的：先趴下 → 确认到位 → 再停程序。
# 反过来的话，程序一停就没人在管狗了；万一趴下失败，会留下"站着但没人控制"
# 的状态。现在这样，趴下失败就直接返回、程序不动，状态还是可控的。
if [ "$CROUCH" = 1 ]; then
    echo
    echo " ① 先让狗趴下（避免留下'站着没人管'的状态）..."
    # 用【退出码】判断，不要去 grep 输出文字 —— motion_cmd 成功返回 0，
    # 超时返回 1。grep 输出容易因为措辞变化而失效（已经踩过一次）。
    if "$PY" "$DIR/motion_cmd.py" --host "$ROBOT" --crouch --go > /tmp/_crouch_high.log 2>&1; then
        grep -m1 "到位" /tmp/_crouch_high.log | sed 's/^ */     /'
        ok "狗已趴下"
    else
        echo "  ${R}✗${N} 趴下失败或超时 —— ${Y}程序不停止${N}，先确认狗的状态"
        echo "     实际输出："
        tail -8 /tmp/_crouch_high.log 2>/dev/null | sed 's/^/       /'
        echo "     手动重试：$PY $DIR/motion_cmd.py --host $ROBOT --crouch --go"
        echo "     确认狗是安全的之后，跳过趴下直接停：./deploy_stop_focus.sh --no-crouch"
        hr; exit 1
    fi
fi

echo
echo " ② 停止程序（PID $PID）..."
echo "    $CMDLINE"

PGID=$(ps -o pgid= -p "$PID" 2>/dev/null | tr -d ' ')
kill_group() {   # kill_group <信号>
    if [ -n "$PGID" ] && [ "$PGID" != "1" ]; then
        kill "-$1" -"$PGID" 2>/dev/null || kill "-$1" "$PID" 2>/dev/null
    else
        kill "-$1" "$PID" 2>/dev/null
    fi
}
wait_gone() {    # wait_gone <最多等几秒>
    local n=$(( $1 * 4 )) i
    for i in $(seq 1 "$n"); do
        sleep 0.25
        kill -0 "$PID" 2>/dev/null || return 0
    done
    return 1
}

# 程序里装了 SIGTERM → KeyboardInterrupt 的处理器（install_sigterm_as_interrupt），
# 所以 TERM 也能走到 finally 发归零。这里用 TERM 而不是 INT：
#   · 和 systemctl stop 的行为一致
#   · setsid 起的进程对 INT 的响应实测不稳定
if kill_group TERM && wait_gone 6; then
    ok "已优雅停止（程序已连发归零）"
else
    warn "TERM 6 秒内没退出，强制杀（不会有归零）"
    kill_group KILL
    wait_gone 2
fi

rm -f "$PIDFILE"
echo
if kill -0 "$PID" 2>/dev/null; then
    echo "  ${R}✗${N} 还在？手动处理：kill -9 $PID"
    hr; exit 1
fi
ok "已停止"

if ss -ltn 2>/dev/null | grep -q ":$VIEW_PORT "; then
    warn "端口 $VIEW_PORT 还被占着（应该是别的进程了）"
    echo "       看看是谁：ss -ltnp | grep :$VIEW_PORT"
else
    ok "观测界面端口 $VIEW_PORT 已释放"
fi

# 顺带提醒【另外两份】安装 —— 免得以为"都停了"其实还有一份在发指令
for OTHER in /home/ysc/tag_probe /home/ysc/follow_high; do
    if [ -f "$OTHER/run.pid" ] && kill -0 "$(cat "$OTHER/run.pid" 2>/dev/null)" 2>/dev/null; then
        echo
        warn "注意：另一份安装（$OTHER）还在跑（PID $(cat "$OTHER/run.pid")）"
        echo "       它也是轴指令发送源。要停它：cd $OTHER && ./deploy_stop*.sh"
    fi
done

echo
echo "  ${Y}注意${N}：程序退出前会连发归零指令，但**停止程序 ≠ 急停**。"
echo "        真正的急停是机器人本体的物理按钮。"
echo
echo "  想改用 .103 上的 Web 遥控？那边先 ./start_web_high.sh 即可（两边不能同时）。"
hr
