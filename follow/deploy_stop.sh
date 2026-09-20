#!/bin/bash
# ============================================================================
# .102 上的跟随 / Tag 状态机 —— 停止
#
#   ./deploy_stop.sh
#
# 启动用 ./deploy_start.sh
#
# ⚠️ 这不是急停。停程序只是让机器人收不到新指令（0.4 秒看门狗超时后归零）。
#    如果你正在让它走，先按机器人本体的物理急停，或把 tag 拿开
#    （看不见就不动）。
# ============================================================================

DIR=/home/ysc/tag_probe
VENV=/home/ysc/follow_env
PY="$VENV/bin/python"
ROBOT=10.21.33.103
PIDFILE="$DIR/run.pid"
LOG="$DIR/run.log"

CROUCH=1                          # 停之前先让狗趴下；./deploy_stop.sh --no-crouch 跳过
[ "$1" = "--no-crouch" ] && CROUCH=0

if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; N=$'\e[0m'
else R=; G=; Y=; N=; fi
ok()   { echo "  ${G}✓${N} $*"; }
warn() { echo "  ${Y}!${N} $*"; }
hr()   { echo "──────────────────────────────────────────────────────"; }

# 先按 PID 文件找；找不到回退按进程名找
# （[t] 转义是防止 pgrep 匹配到本脚本自己的命令行 —— 这个坑今天踩过）
PID=""
if [ -f "$PIDFILE" ]; then
    P=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$P" ] && kill -0 "$P" 2>/dev/null && PID="$P"
fi
if [ -z "$PID" ]; then
    PID=$(pgrep -f '[t]ag_command_fsm\.py|[f]ollow_controller\.py' | head -1)
fi

hr
if [ -z "$PID" ]; then
    warn "没在跑"
    rm -f "$PIDFILE"
    hr; exit 0
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
    if "$PY" "$DIR/motion_cmd.py" --host "$ROBOT" --crouch --go > /tmp/_crouch.log 2>&1; then
        grep -m1 "到位" /tmp/_crouch.log | sed 's/^ */     /'
        ok "狗已趴下"
    else
        echo "  ${R}✗${N} 趴下失败或超时 —— ${Y}程序不停止${N}，先确认狗的状态"
        echo "     实际输出："
        tail -8 /tmp/_crouch.log 2>/dev/null | sed 's/^/       /'
        echo "     手动重试：$PY $DIR/motion_cmd.py --host $ROBOT --crouch --go"
        echo "     确认狗是安全的之后，跳过趴下直接停：./deploy_stop.sh --no-crouch"
        hr; exit 1
    fi
fi

echo
echo " ② 停止程序（PID $PID）..."

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

# 确认没有残留
LEFT=$(pgrep -f '[t]ag_command_fsm\.py|[f]ollow_controller\.py' | wc -l)
if [ "$LEFT" -gt 0 ]; then
    warn "还有 $LEFT 个相关进程在跑"
    pgrep -af '[t]ag_command_fsm\.py|[f]ollow_controller\.py' | sed 's/^/    /'
else
    ok "没有残留进程"
fi

echo
echo "  ${Y}注意${N}：程序退出前会连发归零指令，但**停止程序 ≠ 急停**。"
echo "        真正的急停是机器人本体的物理按钮。"
echo
echo "  想改用 .103 上的 Web 遥控？那边先 ./start_web.sh 即可（两边不能同时）。"
hr
