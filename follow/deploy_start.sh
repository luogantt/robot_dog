#!/bin/bash
# ============================================================================
# .102 上的跟随 / Tag 状态机 —— 启动
#
#   ./deploy_start.sh                 启动（默认：Tag 指令状态机）
#   ./deploy_start.sh --follow        启动跟随模式
#   ./deploy_start.sh --dry           干跑（只检测，不发指令）
#
# 停止用 ./deploy_stop.sh
#
# ★ 本脚本【只检查、不动作】★
#   起立/趴下这类有物理后果的动作要你自己先做 —— 脚本不替你做决定。
#   理由：脚本不看着场地，也不知道你是不是刚因为它过热而主动停下的。
#   狗不在 RL 控制(17) 时轴指令会被静默忽略，所以检查不过就拒绝启动，
#   并把该跑的命令打出来。顺序永远是：【先让狗起立】→ 再跑本脚本。
#
# ⚠️ 同一时刻只能有一个「轴指令发送源」（文档 §1.5 的 0xE006）。
#    本脚本启动前会检查 .103 上的 Web 遥控是否开着 —— 开着的话会拒绝启动，
#    因为两边会互相踢。要一起用就先 ./stop_web.sh（在 .103 上）。
# ============================================================================

# ---- 配置（要改就改这里）----
DIR=/home/ysc/tag_probe
VENV=/home/ysc/follow_env
PY="$VENV/bin/python"
ROS_SETUP=/opt/ros/jazzy/setup.bash
ROS_DOMAIN=2                      # 相机在域 2，不设就看不到图像
ROBOT=10.21.33.103
WEB_HOST=10.21.33.103             # .103 上的 Web 遥控（用来检查冲突）
WEB_PORT=8000

LOG="$DIR/run.log"
PIDFILE="$DIR/run.pid"

# ---- 默认跑哪个程序 ----
MODE=fsm                          # fsm | follow | dry
TARGET_DUR=0                      # 0 = 一直跑
while [ $# -gt 0 ]; do
    case "$1" in
        --follow)   MODE=follow ;;
        --dry)      MODE=dry ;;
        --dur)      shift; TARGET_DUR="$1" ;;
        # 打印文件开头的注释块（跳过第 1、2 行，到下一个 # ==== 为止）
        -h|--help)  awk 'NR>2 && /^# ====/{exit} NR>2{sub(/^# ?/,""); print}' "$0"
                    exit 0 ;;
        *) echo "未知参数：$1（用 --help）"; exit 1 ;;
    esac
    shift
done

if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[36m'; N=$'\e[0m'
else R=; G=; Y=; B=; N=; fi
ok()   { echo "  ${G}✓${N} $*"; }
bad()  { echo "  ${R}✗${N} $*"; }
warn() { echo "  ${Y}!${N} $*"; }
hr()   { echo "──────────────────────────────────────────────────────"; }

is_running() {
    [ -f "$PIDFILE" ] || return 1
    local p; p=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$p" ] && kill -0 "$p" 2>/dev/null
}

# ---------------------------------------------------------------- 启动前检查
hr; echo " 启动前检查"; hr
fail=0

[ -x "$PY" ] && ok "Python 环境   $PY" || { bad "找不到 $PY"; fail=1; }
[ -f "$DIR/tag_command_fsm.py" ] && ok "程序目录     $DIR" || { bad "找不到 $DIR"; fail=1; }

# 相机（域 2）
if [ -f "$ROS_SETUP" ]; then
    IMG=$(bash -c "source $ROS_SETUP; export ROS_DOMAIN_ID=$ROS_DOMAIN; \
        timeout 8 ros2 topic list 2>/dev/null | grep -c '/camera/camera/color/image_raw'")
    if [ "${IMG:-0}" -ge 1 ]; then ok "相机话题     域 $ROS_DOMAIN 可见"
    else bad "域 $ROS_DOMAIN 看不到相机话题 —— 相机服务没跑？"; \
         echo "        查：systemctl status realsense-camera.service"; fail=1; fi
else
    bad "找不到 $ROS_SETUP"; fail=1
fi

# ⚠️ 冲突检查：.103 上的 Web 遥控也是轴指令发送源
if timeout 2 bash -c "echo > /dev/tcp/$WEB_HOST/$WEB_PORT" 2>/dev/null; then
    bad "检测到 $WEB_HOST:$WEB_PORT 上的 Web 遥控正在运行！"
    echo "        ${Y}它是另一个轴指令发送源，两边会互相踢（0xE006）${N}"
    echo "        先到 .103 上停掉它：ssh user@$WEB_HOST 'cd /home/user/robot_dog_web && ./stop_web.sh'"
    fail=1
else
    ok "冲突检查     $WEB_HOST:$WEB_PORT 没有 Web 遥控在跑"
fi

if is_running; then
    warn "本机程序已经在跑了（PID $(cat "$PIDFILE")）—— 要重启用 ./deploy_stop.sh 先停"
    hr; exit 0
fi

if [ "$MODE" != "dry" ]; then
    # 用现成的 asdu_probe.py 读状态（内联 heredoc 在多层引号嵌套下会失效，
    # 这里踩过一次 —— 报"读不到状态"其实是脚本自己的问题）
    MS=$("$PY" "$DIR/asdu_probe.py" --host "$ROBOT" --seconds 3 2>/dev/null \
         | grep -oP 'MotionState\s*=\s*\K-?\d+' | head -1)
    if [ "$MS" = "17" ]; then
        ok "机器人       MotionState=17 (RL控制)"
    elif [ -n "$MS" ]; then
        # 只拒绝、不动手。起立要人看着场地决定 —— 见文件头「只检查、不动作」。
        bad "机器人       MotionState=$MS —— 不是 RL 控制，轴指令会被静默忽略"
        echo "        ${Y}先让狗起立，再重跑本脚本${N}："
        echo "        $PY $DIR/motion_cmd.py --host $ROBOT --stand --go"
        echo "        （狗在楼梯/不平地面时先确认场地安全再起立）"
        fail=1
    else
        bad "机器人       读不到状态 —— 运控没跑？地址不对？"
        echo "        手动查：$PY $DIR/asdu_probe.py --host $ROBOT --seconds 5"
        fail=1
    fi
fi

if [ "$fail" != 0 ]; then hr; echo " ${R}检查未通过${N}"; hr; exit 1; fi
hr

# ---------------------------------------------------------------- 组装命令
case "$MODE" in
    fsm)    PROG="tag_command_fsm.py";  ARGS="--go"; DESC="Tag 指令状态机（真实控制）" ;;
    follow) PROG="follow_controller.py"; ARGS="--go --max-wz 0"; DESC="跟随模式（真实控制，不转向）" ;;
    dry)    PROG="tag_command_fsm.py";  ARGS="";      DESC="干跑（只检测，不发指令）" ;;
esac
[ "$TARGET_DUR" != "0" ] && ARGS="$ARGS --duration $TARGET_DUR"

# ---------------------------------------------------------------- 启动
echo
echo "启动：$DESC"
cd "$DIR" || exit 1
# 【必须 -u】Python 输出到文件时默认是块缓冲（攒满 8KB 才落盘），
# 不加的话日志是空的、tail -f 也看不到任何东西。踩过。
setsid nohup bash -c "source $ROS_SETUP; export ROS_DOMAIN_ID=$ROS_DOMAIN; \
    exec $PY -u $PROG $ARGS" > "$LOG" 2>&1 < /dev/null &
echo $! > "$PIDFILE"

# 等日志有内容（程序要等相机就绪，3 秒常常还不够；最多等 10 秒）
for i in $(seq 1 40); do
    sleep 0.25
    [ -s "$LOG" ] && break
    is_running || break
done
sleep 0.5
echo

hr
if is_running; then
    ok "已启动（PID $(cat "$PIDFILE")）"
    echo "    程序  $PROG $ARGS"
    echo "    日志  tail -f $LOG"
    echo
    echo "  ${B}操作：${N}"
    echo "    ① 把 tag 举到狗正前方 0.6~1.5 米、离地 20~30cm（相机低头 35°）"
    echo "    ② ${Y}举稳别晃${N} —— 晃一下检测率从 100% 掉到个位数，狗就迈不起步"
    echo "    ③ 停止：./deploy_stop.sh"
    echo
    echo "  启动日志："
    tail -6 "$LOG" 2>/dev/null | sed 's/^/    /'
else
    bad "启动失败，日志末尾："
    tail -14 "$LOG" 2>/dev/null | sed 's/^/    /'
    rm -f "$PIDFILE"
    exit 1
fi
hr
