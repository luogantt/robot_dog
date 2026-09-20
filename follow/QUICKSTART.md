# 启动跟随程序 —— 速查

**一句话**：在 `.102` 上跑 `follow_controller.py`，它用相机找 tag，让狗追到距 tag 1 米处停下。

> 完整说明见 [`README.md`](README.md)；设计原理见 [`../TECHNICAL_REPORT.md`](../TECHNICAL_REPORT.md)。

---

## 启动（五步）

```bash
# ① 登录相机那台机器
ssh ysc@10.21.33.102

# ② 进目录、配环境（这两行每次都要）
cd /home/ysc/tag_probe
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2          # ← 相机在域 2，不设就看不到图像

# ③ 先干跑，确认能检测到 tag（不发任何指令，安全）
/home/ysc/follow_env/bin/python follow_controller.py

# ④ 确认没问题后，真跑
/home/ysc/follow_env/bin/python follow_controller.py --go --max-wz 0 --duration 60

# ⑤ 停止：Ctrl+C（程序会在 finally 里连续发零速）
```

**按 `Ctrl+C` 停止后，狗不会继续动** —— 程序退出前会连发归零指令。

---

## 启动前必须确认的四件事

| # | 检查 | 命令 | 期望 |
|---|---|---|---|
| 1 | 相机服务在跑 | `systemctl is-active realsense-camera.service` | `active` |
| 2 | **机器人在 RL 控制** | `/home/ysc/follow_env/bin/python motion_cmd.py --status` | `MotionState=17` |
| 3 | 电量充足 | 同上（或 `asdu_probe.py`） | 建议 >60% |
| 4 | 程序是最新版 | 本地改完**记得上传**（见文末） | — |

**第 2 条不过**（狗趴着/空闲）：
```bash
/home/ysc/follow_env/bin/python motion_cmd.py --stand --go
```

---

## tag 要放哪儿（**这一步错了就什么都测不出来**）

相机装在狗前部、**朝前下方低头 35°**：

| 项 | 要求 |
|---|---|
| 距离 | 狗正前方 **0.6 ~ 1.5 米** |
| 高度 | **离地 20~30 cm（膝盖以下）** |
| 朝向 | tag 正面朝狗 |

> **举在胸前、或离得太远，相机根本看不见** —— 程序会按"看不见就不动"停住，
> 看起来像坏了，其实只是没进视野。

**举起来后保持不动**（晃动会让检测率从 100% 掉到个位数）。

---

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--go` | 关 | **不加就是干跑**（只算不发），第一次务必先用干跑 |
| `--duration` | 0 | 跑多少秒，**0 = 一直跑**（默认！记得 Ctrl+C） |
| `--target` | 1.0 | 目标距离（米） |
| `--max-vx` | 0.50 | 前进上限，按实测标度 ≈ **0.86 m/s**（满量程 1.71 m/s）|
| `--max-wz` | 1.0 | 转向上限。`0` = **完全不转向**，只直着走（首次建议先用 0） |
| `--lost-timeout` | 1.0 | **漏检宽限**（秒）：这么久没再看到才归零，宽限内沿用上次位置 |
| `--tag-size` | 0.20 | **tag 黑框边长（米）** —— 填错距离就整体等比偏掉 |

**常用组合**：

```bash
# 干跑，看能不能检测到
/home/ysc/follow_env/bin/python follow_controller.py

# 首次真机：不转向，低速
/home/ysc/follow_env/bin/python follow_controller.py --go --max-wz 0 --max-vx 0.3 --duration 30

# 带转向（会猛转头，见"注意"）
/home/ysc/follow_env/bin/python follow_controller.py --go --max-vx 0.6 --max-wz 1.0
```

---

## 正常运行时应该看到

```
运控 10.21.33.103:30004   模式 ★ 真实控制 ★
tag id=1  size=200.0mm  目标距离=1.00m
控制 20Hz / 检测 ≤15Hz，目标过期阈值 250ms
相机就绪 fx=603.4 640x480
检查机器人状态 ...
  MotionState=17 Gait=4097 Mode=0

  控制20.1Hz 检测15.0Hz 解算58ms | 前=+1.32 左=+0.05 | vx=+0.300 wz=+0.000 | 追击 距=1.34m
```

| 字段 | 含义 |
|---|---|
| `控制 20.1Hz` | 控制循环频率（应稳定在 20Hz 左右） |
| `检测 15.0Hz` | 检测频率（≈ 相机帧率） |
| `前= / 左=` | tag 在机体系的位置（米），**原点 = 机体中心** |
| `未见` | 当前帧没检测到 tag |
| `追击 / 到位 / 看不见 → 停` | 控制律当前在做什么 |

**行为**：`看不见 → 停`；`距 tag ≤ 1m → 停`；`> 1m → 追`。

---

## ⚠️ 三条硬提醒

1. **本程序没有急停指令** —— 文档 §1.2.3 明确软急停只能查询不能下发。
   **真正的安全手段是机器人本体的物理急停按钮。** 程序里的"停"只是发零速。
2. **`--max-wz 1.0` 是满量程转向（≈86°/s）**，狗会**猛地转头**。首次建议 `--max-wz 0`。
3. **不要同时用手机 App / 手柄控制** —— 协议 §1.5 的 `0xE006` 要求轴指令 2 秒内同源，
   两个客户端会互相踢掉，表现为"指令偶尔没反应"，极难排查。

---

## 出问题怎么办

| 现象 | 先查 |
|---|---|
| 报"没拿到相机图像/内参" | `ROS_DOMAIN_ID=2` 设了吗？相机服务 active 吗？ |
| 一直显示 `未见` | **tag 进视野了吗** —— 抓一帧看：`d435i_tag_probe.py --frames 30 --save /tmp/x` |
| 拒绝运行 / 报非 RL 控制 | `motion_cmd.py --stand --go` |
| 狗不动但你看到 `追击` | 轴指令**没有响应帧**，失败是静默的。跑 `axis_test.py --axis X --value 1.0 --hold 1.5 --go` 单独验通道 |
| 走的方向反了 | 这条路目前只能靠实测确认，方向不对告诉我，改符号 |
| 距离明显不对 | `--tag-size` 填错了。标定：`d435i_tag_monitor.py --calibrate 0.60 --duration 10` |

---

## 改完代码怎么传上去

程序跑在 `.102`，源码在本地 `D:\lg\FOLLOW\robot_dog\follow\`。**改完必须上传**：

```bash
# 在 Git Bash（Windows）里
export S10_PW="<密码>" MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL="*"
S="D:/lg/FOLLOW/official_sdk/tools/s10.py"

python "$S" 102 put "D:/lg/FOLLOW/robot_dog/follow/follow_controller.py" \
                  /home/ysc/tag_probe/follow_controller.py
```

> ⚠️ **改完本地不上传就重启程序，跑的还是旧代码。**
> 已经发生过一次：`follow_law.py` 差了个 2.5 倍的速度上限。
