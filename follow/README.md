# follow —— 相机识别 tag，指挥机器狗动作

用 `.102` 上的 **Intel RealSense D435i** 实时检测 AprilTag，根据 **tag 的 id**
让机器狗执行对应动作（前进/后退/转向/侧移/停止/切步态），或者**跟着 tag 走**。

> **赶时间的话直接看 → [`QUICKSTART.md`](QUICKSTART.md)**（五步启动，一页速查）。
>
> 本文是**完整运行手册**。设计原理、实测标定、踩过的坑见
> [`../TECHNICAL_REPORT.md`](../TECHNICAL_REPORT.md) 第 2 节。

---

## 0. 硬件前提（先看懂这张表，否则后面全是坑）

机器狗上挂着**两台独立的计算机**：

| 地址 | 账户 | 有什么 |
|---|---|---|
| **10.21.33.102** | `ysc` | **相机在这里**（D435i）、ROS 2 Jazzy、`follow_env` 环境 |
| **10.21.33.103** | `user` | **运控在这里**（ASDU 服务端，UDP 30004） |

```
D435i（.102）→ 检测 tag → 控制律 → ASDU UDP → 运控（.103）→ 狗动
```

**所有程序都跑在 `.102` 上**，通过 UDP 把指令发给 `.103`。

---

## 1. 前置条件（四条，缺一条就跑不起来）

### ① 两台机器都通
```bash
ping 10.21.33.102     # 相机机
ping 10.21.33.103     # 运控机
```

### ② 相机服务在跑（ROS 域必须是 2）
```bash
ssh ysc@10.21.33.102
systemctl is-active realsense-camera.service     # 应输出 active
systemctl show realsense-camera.service -p Environment
#   应包含 ROS_DOMAIN_ID=2
```

> ⚠️ **相机在域 2，不是默认的域 0。** 起来的程序必须 `export ROS_DOMAIN_ID=2`，
> 否则看不到相机话题、程序会报"没拿到图像"。
> （服务已通过 systemd drop-in 设为域 2，见 TECHNICAL_REPORT §2.3）

### ③ 机器人处于 **RL 控制(17)**
```bash
cd /home/ysc/tag_probe
/home/ysc/follow_env/bin/python motion_cmd.py --status
#   要看 MotionState=17。不是的话：
/home/ysc/follow_env/bin/python motion_cmd.py --stand --go
```

> **轴指令只在 `MotionState=17` 下有效**，其他状态下会被**静默忽略**
> （文档没说、也不报错）。程序里有状态门兜底，会自动输出零。

### ④ tag 图打印好
`tags/` 目录下已有 8 张，直接打印即可：

| 文件 | id | 动作 |
|---|---|---|
| `tag41_12_00001.png` | 1 | 前进 |
| `tag41_12_00018.png` | 18 | 后退 |
| `tag41_12_00003.png` | 3 | 左转 |
| `tag41_12_00061.png` | 61 | 右转 |
| `tag41_12_00063.png` | 63 | 左侧移动 |
| `tag41_12_00068.png` | 68 | 右侧移动 |
| `tag41_12_00006.png` | 6 | 停止 |
| `tag41_12_00000.png` | 0 | 切楼梯步态 |

**打印注意**：
- **白色静区（四周留白）必须一起打印** —— 裁掉就认不出
- 别用光泽相纸（反光会让 tag 变白）
- 贴平、别卷曲

---

## 2. 快速开始

### 用部署脚本（推荐）

```bash
ssh ysc@10.21.33.102
cd /home/ysc/tag_probe

./deploy_start.sh              # 启动（默认 Tag 指令状态机）
./deploy_start.sh --follow     # 启动跟随模式
./deploy_start.sh --dry        # 干跑：只检测，不发任何指令
./deploy_stop.sh               # 停止（先让狗趴下，再停程序）
```

`deploy_start.sh` 会**先做五项检查**，任一不过就拒绝启动：

```
  ✓ Python 环境   /home/ysc/follow_env/bin/python
  ✓ 程序目录     /home/ysc/tag_probe
  ✓ 相机话题     域 2 可见
  ✓ 冲突检查     10.21.33.103:8000 没有 Web 遥控在跑
  ✓ 机器人       MotionState=17 (RL控制)
```

| 检查 | 不过时的提示 |
|---|---|
| Python 环境 / 程序目录 | 直接报缺哪个 |
| **相机话题** | 会告诉你查 `systemctl status realsense-camera.service` |
| **冲突检查** | ⚠️ **`.103` 上的 Web 遥控是另一个轴指令发送源**，两边会互相踢（`0xE006`）。检测到就拒绝启动，并给出停它的命令 |
| **机器人状态** | 不是 `MotionState=17` 就拒绝，并给出起立命令 |

**`deploy_stop.sh` 的行为是刻意的**：

```
 ① 先让狗趴下（避免留下"站着没人管"的状态）...
  ✓ 狗已趴下
 ② 停止程序...  ✓ 已优雅停止（程序已连发归零）
```

**先趴下 → 确认到位 → 才停程序。** 反过来的话程序一停就没人管狗了；
万一趴下失败，脚本会**直接返回、程序不停**，状态仍然可控。
确认狗安全之后可以直接停：`./deploy_stop.sh --no-crouch`

### 手动跑（不用脚本）

```bash
cd /home/ysc/tag_probe
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2          # ← 相机在域 2，不设就看不到图像

/home/ysc/follow_env/bin/python tag_command_fsm.py            # 干跑
/home/ysc/follow_env/bin/python tag_command_fsm.py --go --duration 120
```

**然后拿着 tag 在狗前面出示即可**（位置要求见 §3）。

---

## 3. tag 出示位置（**这一步错了就什么都测不出来**）

相机装在狗前部、**朝前下方低头 35°** —— 它能看到的空间是一个固定的锥体：

| 项 | 要求 |
|---|---|
| **距离** | 狗正前方 **0.6 ~ 1.5 米** |
| **高度** | **离地 20~30 cm（膝盖以下）** |
| **横向** | 狗正前方 ±20° 内 |
| **朝向** | tag 正面朝狗，别大角度倾斜 |
| **关键** | **出示后保持不动** |

> **举在胸前、或者离得太远，相机根本看不见** —— 程序会按"看不见就不动"停住，
> 很容易被误判成程序坏了。

> ⚠️ **必须举稳。** 实测：稳稳举着时单帧抓取 **30/30 全中**；
> 手一晃，检测率掉到 **4~10%**，指令被打碎成"发 0.1 秒 / 停 0.3 秒"，
> **四足机器人迈不起步来**。

---

## 4. 程序清单

| 文件 | 干什么 | 常用命令 |
|---|---|---|
| **`deploy_start.sh`** | **启动**（五项检查 + 启动 + 确认） | `./deploy_start.sh [--follow\|--dry]` |
| **`deploy_stop.sh`** | **停止**（先让狗趴下再停程序） | `./deploy_stop.sh [--no-crouch]` |
| **`tag_command_fsm.py`** | **主程序**：tag id → 动作 | `... --go --duration 120` |
| `follow_controller.py` | 跟随模式：追着 tag 走，停在 1 米 | `... --go --max-wz 0` |
| `motion_cmd.py` | 起立/趴下/切步态 | `... --stand --go` |
| `axis_test.py` | 单轴/双轴指令测试 | `... --axis X --value 1.0 --hold 2 --go` |
| `turn_90_test.py` | 闭环定角转向（积分 AngularZ） | `... --degrees -90 --go` |
| `d435i_tag_monitor.py` | 实时监视 + tag 尺寸标定 | `... --calibrate 0.60 --duration 10` |
| `d435i_tag_probe.py` | 抓帧 + 检测 + 存图（诊断用） | `... --frames 30 --save /tmp/x` |
| `asdu_probe.py` | 只读状态/故障/电池/温度 | `... --seconds 6` |
| `gen_tags.py` | 生成 tag 图 | `... 0 1 3 6 18 61 63 68` |

**除 `asdu_probe.py` 和 `gen_tags.py` 外，其余都需要先 `source ROS` + `export ROS_DOMAIN_ID=2`。**

---

## 5. tag 指令状态机（最常用）

### 动作表

| tag id | 动作 | 发出的指令 |
|---|---|---|
| 1 | 前进 | `X = +1.0` |
| 18 | 后退 | `X = −1.0` |
| 3 | 左转 | `Yaw = +1.0` |
| 61 | 右转 | `Yaw = −1.0` |
| 63 | 左侧移动 | `Y = +1.0` |
| 68 | 右侧移动 | `Y = −1.0` |
| 6 | 停止 | 全零 |
| 0 | 切楼梯步态 | `0x1003`（**先停稳再切**） |
| 无 tag | 停止 | 全零 |

**为什么全是 1.0（满量程）？** 实测这个通道**低值区不响应、而且非单调**：
`X=0.06` 能走 0.127 m/s，但 `0.12` 只有 0.027、`0.5` 只有 0.017 —— **加大反而更慢**。
详见 [TECHNICAL_REPORT §2.7](../TECHNICAL_REPORT.md)。

### 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--go` | 关 | **不加就是干跑**，只算不发 |
| `--duration` | 0 | 跑多少秒，0 = 一直跑 |
| `--lost-timeout` | 0.25 | 多久没检测到就视为丢失并归零（秒） |
| `--min-px` | 20 | tag 最小像宽，挡掉远处小 tag 的误检 |
| `--mag-x/--mag-y/--mag-yaw` | 1.0 | 各轴幅度 |
| `--sign-x/--sign-y/--sign-yaw` | +1 | **方向反了就翻这个**（实测方向和文档不一定一致） |
| `--stair-gait` | `0x1003` | tag 0 要切到的步态 |
| `--hz` / `--det-hz` | 20 / 15 | 控制频率 / 检测频率 |

**示例**：
```bash
# 干跑，看动作表和检测情况
/home/ysc/follow_env/bin/python tag_command_fsm.py

# 真跑 2 分钟
/home/ysc/follow_env/bin/python tag_command_fsm.py --go --duration 120

# 检测断续时放宽丢失宽限（代价：看不见后还会动最多 N 秒）
/home/ysc/follow_env/bin/python tag_command_fsm.py --go --lost-timeout 1.0

# 只测前进和停止两个 tag
/home/ysc/follow_env/bin/python tag_command_fsm.py --go --duration 60
```

---

## 6. 跟随模式

```bash
# 干跑（不发指令）
/home/ysc/follow_env/bin/python follow_controller.py

# 真跑：追到距 tag 1 米停，不转向
/home/ysc/follow_env/bin/python follow_controller.py --go --max-wz 0

# 带转向（转向是开关式，会猛转头）
/home/ysc/follow_env/bin/python follow_controller.py --go --max-vx 0.6 --max-wz 1.0
```

**控制律**：`看不见 → 停` / `≤1m → 停` / `>1m → 追`。

**架构**：两线程解耦 —— 检测线程 15Hz 更新"最新目标"，控制线程固定 20Hz 持续下发。
（单线程"检测完才发一次"只能跑到 5Hz，狗会卡顿。）

⚠️ **`--tag-size` 必须和实际打印尺寸一致**，否则跟随距离整体等比偏掉。
用标定确认真实尺寸：
```bash
/home/ysc/follow_env/bin/python d435i_tag_monitor.py --calibrate 0.60 --duration 10
#   把 tag 放在卷尺量准的 0.60m 处，它会反算真实黑框边长
```

---

## 7. 诊断：出问题先跑这三个

```bash
# ① 相机出图了吗？（应 ≈15Hz）
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=2
ros2 topic hz /camera/camera/color/image_raw

# ② 通路的机器人状态、电池、温度
/home/ysc/follow_env/bin/python asdu_probe.py --seconds 6

# ③ 相机到底拍到了什么？（存图回来看）
/home/ysc/follow_env/bin/python d435i_tag_probe.py --frames 30 --save /tmp/x
```

| 现象 | 先查 |
|---|---|
| 程序说"没拿到图像" | `ROS_DOMAIN_ID=2` 设了吗？相机服务 active 吗？ |
| 检测不到 tag | ③ 抓帧看画面 —— tag 进视野了吗？多大？清楚吗？ |
| 检测断续 | **tag 举稳了吗**（最常见）；或手抖/反光/卷曲 |
| 狗不动 | ② 看 `MotionState` 是不是 17；看电量；看有无故障 |
| 动作方向反了 | 用 `--sign-x/--sign-y/--sign-yaw` 翻转 |
| 步态切不动 | 轴指令和步态指令都要求 `MotionState=17` |

---

## 8. ⚠️ 安全须知

1. **本程序的"停止"是软件零速，不是硬件急停。** 文档 §1.2.3 明确：软急停(-2)
   仅支持查询、**不支持下发** —— **这份协议里没有任何急停指令**。
   **真正的安全手段是机器人本体的物理急停按钮。**
   （`./deploy_stop.sh` 会让狗先趴下再停程序，但这仍不是急停。）
2. **⚠️ `Yaw = ±1.0` 是满量程转向（≈86°/s），狗会猛地转头。** 场地要留够。
3. **不要和其他客户端同时控制。** 文档 §1.5 的 `0xE006` 要求轴指令 2 秒内同源
   —— 手机 App / 手柄同时控制会互相踢掉，表现为"指令偶尔没反应"。
4. **轴指令不返回任何响应帧**，失败是静默的。唯一的判据是机器人的实际状态反馈。
5. **总有人在场看护。** 场地清空，手放断电位置。
6. ⚠️ **同一时刻只能有一个「轴指令发送源」**（文档 §1.5 的 `0xE006`：
   轴指令要求 2 秒内来自同一个客户端）。下面这些都是**各自独立的发送源**：

   | 发送源 | |
   |---|---|
   | 本项目的跟随 / 状态机 | |
   | **`.103` 上的 Web 遥控** | **`web_control` 无论有没有人按键都以 20Hz 持续发轴指令** |
   | 官方手柄 / 手机 App | |
   | `axis_test.py` 等测试脚本 | 每跑一次就是一个新客户端 |

   **两个以上同时在发 → 互相踢 → 谁的指令都不生效。**

   **症状**：只有**站立/趴下**能用，**前进/转向完全没反应** —— 因为前者走
   `send_and_wait` **单次请求**不参与 2 秒同源竞争，后者会被**静默踢掉**。

   > **实测**（2026-09-20）：手柄和 App 都连着时发 `X=0.5` 两秒，
   > 机器人 `LinearX` 只有 **0.022 m/s**；关掉后同一条指令 **0.86 m/s**，**差 40 倍**。

   **`deploy_start.sh` 会自动检查这一条**：探测到 `.103:8000` 有 Web 遥控在跑就拒绝启动。
   **排查口诀**：出现「动作能、轴指令不能」，第一件事是**数有几个发送源**。

---

## 9. 已知问题（截至 2026-09-20）

| # | 问题 | 状态 |
|---|---|---|
| 1 | **前进响应弱**（0.027 m/s，追不上人） | 排查方向已收敛到**电量**：会话中 68%→35%→20%，响应正好在这期间垮掉。**充电到 60% 以上再测可定性** |
| 2 | **转向起转时刻随机** | 同为满量程，起转延迟在 0.25s ~ 永不之间。所以转向用的是**开关式**（死区 8° 内不转，超过就满量程） |
| 3 | **检测断续**（举不稳时 4~10%） | 举稳时 30/30 全中，**是操作问题不是算法问题**。必要时放宽 `--lost-timeout` |
| 4 | 机器人是**轮足式**（腿末端有轮子） | 平地本应轮式滚动，但实测像"小碎步"。轮式是否需要单独启用，**待向厂商确认** |
| 5 | `tag_size` 与实物可能不一致 | 20cm 的 tag 实测像宽反推约 10~12cm，**跟随时建议先标定** |

---

## 10. 本地 ↔ 远端同步（开发时用）

源码在 `D:\lg\FOLLOW\robot_dog\follow\`，实际跑在 `.102:/home/ysc/tag_probe/`。

```bash
# 在 Git Bash 里，必须带这两个环境变量（否则路径被 MSYS 改写）
export S10_PW="<密码>" MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL="*"
S="D:/lg/FOLLOW/official_sdk/tools/s10.py"

# 上传
python "$S" 102 put "D:/lg/FOLLOW/robot_dog/follow/xxx.py" /home/ysc/tag_probe/xxx.py

# 执行
python "$S" 102 run "cd /home/ysc/tag_probe && source /opt/ros/jazzy/setup.bash && \
  export ROS_DOMAIN_ID=2 && /home/ysc/follow_env/bin/python xxx.py --go"

# 拉回文件（如图片）
python "$S" 102 get /tmp/x/raw.jpg "D:/lg/FOLLOW/official_sdk/tag_probe/raw.jpg"
```

> ⚠️ **改完本地一定要上传再跑** —— 已经发生过"远端文件落后于本地"，
> `follow_law.py` 差了个 2.5 倍的速度上限。

---

## 11. 相关

| | |
|---|---|
| 协议来源 | 《软件开发指南》V1.0.1（`../robot_dog_sdk/docs/`） |
| 协议层参考实现 | `../robot_dog_sdk/asdu_udp_control.py` |
| 设计原理 / 实测标定 / 踩过的坑 | [`../TECHNICAL_REPORT.md`](../TECHNICAL_REPORT.md) |
| 现场运行手册（导航项目） | `../s10_slam_nav/RUNBOOK.md` |
