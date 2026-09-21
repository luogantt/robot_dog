#!/bin/bash
# ============================================================================
# .102 上的跟随 / Tag 状态机【focus 版】—— 启动
#
#   ./deploy_start_focus.sh                 启动（默认：Tag 指令状态机）
#   ./deploy_start_focus.sh --follow        启动跟随模式
#   ./deploy_start_focus.sh --dry           干跑（只检测，不发指令）
#
# 停止用 ./deploy_stop_focus.sh
#
# ★ 本脚本【只检查、不动作】★
#   起立/趴下这类有物理后果的动作要你自己先做。顺序永远是：
#     先让狗起立（motion_cmd.py --stand --go）→ 再跑本脚本。
#
# ⚠️ 这台机器上有【三套】follow 安装，各自都是轴指令发送源：
#      /home/ysc/tag_probe    —— v1 时代那份
#      /home/ysc/follow_high  —— 高台步态那批
#      /home/ysc/go_focus     —— 本目录（PI 跟随 + 比例式转向）
#    同时开任意两个就互相踢（文档 §1.5 的 0xE006），表现是「按方向键没反应」。
#    **一次只能开一个。**
#    本脚本检查两件事来挡住这种情况：
#      · .103:8000 有没有 Web 遥控在跑（它也是发送源）
#      · 本机 8010 端口有没有被占（三套观测界面都用 8010 → 说明另一份在跑）
#
# 本版(focus)与 tag_probe 那份的代码差异：跟随改成 PI + 比例式转向
# （max_vx 0.90 ≈1.54 m/s，追得上小跑的人）；tag 63 = 高台步态(0x1002)；
# 转向 tag 定角 30° 自动停。
# ============================================================================

# ---- 配置（要改就改这里）----
DIR=/home/ysc/go_focus          # ← 本安装的目录
VENV=/home/ysc/follow_env
PY="$VENV/bin/python"
ROS_SETUP=/opt/ros/jazzy/setup.bash
ROS_DOMAIN=2                      # 相机在域 2，不设就看不到图像
ROBOT=10.21.33.103
WEB_HOST=10.21.33.103             # .103 上的 Web 遥控（用来检查冲突）
WEB_PORT=8000
VIEW_PORT=8010                    # 观测界面端口（两套都用这个 → 可用来发现另一份）

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
hr; echo " 启动前检查（focus 版）"; hr
fail=0

[ -x "$PY" ] && ok "Python 环境   $PY" || { bad "找不到 $PY"; fail=1; }
[ -f "$DIR/tag_command_fsm.py" ] && ok "程序目录     $DIR" \
    || { bad "找不到 $DIR/tag_command_fsm.py"; fail=1; }

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

# ⚠️ 冲突检查 1：.103 上的 Web 遥控也是轴指令发送源
if timeout 2 bash -c "echo > /dev/tcp/$WEB_HOST/$WEB_PORT" 2>/dev/null; then
    bad "检测到 $WEB_HOST:$WEB_PORT 上的 Web 遥控正在运行！"
    echo "        ${Y}它是另一个轴指令发送源，两边会互相踢（0xE006）${N}"
    echo "        先到 .103 上停掉它："
    echo "          robot_dog_web   → ssh user@$WEB_HOST 'cd /home/user/robot_dog_web && ./stop_web.sh'"
    echo "          robot_sdk_high  → ssh user@$WEB_HOST 'cd /home/user/robot_sdk_high && ./stop_web_high.sh'"
    fail=1
else
    ok "冲突检查     $WEB_HOST:$WEB_PORT 没有 Web 遥控在跑"
fi

# ⚠️ 冲突检查 2：本机另一份跟随安装在跑吗（两套观测界面都用 8010）
if ss -ltn 2>/dev/null | grep -q ":$VIEW_PORT "; then
    OWNER=$(ss -ltnp 2>/dev/null | grep ":$VIEW_PORT " | grep -oP 'pid=\K[0-9]+' | head -1)
    bad "本机 $VIEW_PORT 端口被占（PID ${OWNER:-?}）—— 另一份跟随安装在跑？"
    echo "        ${Y}三套 follow 安装都是轴指令发送源，同时开必然互相踢${N}"
    ps -o pid,etimes,cmd -p "${OWNER:-0}" 2>/dev/null | tail -1 | sed 's/^/         /'
    echo "        用 /proc/$OWNER/cwd 看是哪一份，再用那份目录里的 deploy_stop*.sh 停它："
    echo "          ls -l /proc/${OWNER:-?}/cwd"
    fail=1
else
    ok "冲突检查     本机 $VIEW_PORT 空闲（没有另一份跟随在跑）"
fi

if is_running; then
    warn "本安装已经在跑了（PID $(cat "$PIDFILE")）—— 要重启用 ./deploy_stop_focus.sh 先停"
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
        # 只拒绝、不动手。起立要人看着场地决定。
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
    echo "    ${B}观测界面  http://10.21.33.102:$VIEW_PORT/${N}   ← 浏览器打开可看实时状态"
    echo
    echo "  ${B}操作：${N}"
    echo "    ① 把 tag 举到狗正前方 0.6~1.5 米、离地 20~30cm（相机低头 35°）"
    echo "    ② ${Y}举稳别晃${N} —— 晃一下检测率从 100% 掉到个位数，狗就迈不起步"
    echo "    ③ 停止：./deploy_stop_focus.sh"
    echo
    echo "  ${B}本版(focus)：跟随用 PI + 比例式转向，max_vx=0.90(~1.54 m/s)${N}"
    echo "               tag 63 = 高台步态(0x1002)；转向 tag 定角 30° 自动停"
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
