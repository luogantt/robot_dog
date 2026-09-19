# follow —— 机器狗 AprilTag 追踪

让山猫 S10 机器狗追踪 AprilTag，保持在 **1 米**距离。

```
距离 > 1m + 死区  →  前进（速度按距离比例）
距离在 1m ± 死区  →  停
距离 < 1m         →  停（不倒退）
同时转向对准 tag
```

---

## 1. 硬件拓扑

机器狗上有**两台计算机**：

| 地址 | 用户 | 角色 | 关键内容 |
|---|---|---|---|
| **10.21.33.103** | `user` | 主控计算机 | `robotserve`（ASDU UDP 30004）、`golai` 网页遥控（8088/8089）、8 个 USB 鱼眼相机 |
| **10.21.33.102** | `ysc` / `lg` | Jetson 伴侣计算机 | **Intel RealSense D435i**、激光雷达、ROS2 Jazzy、Docker |

**追踪用 `.102` 上的 RealSense**，控制指令走 `.103` 的 ASDU UDP。

SSH 密码：**见内部记录，不写进仓库**。`.102` 上 `ysc` 有**免密 sudo**。

---

## 2. 相机配置（重要）

### 2.1 当前使用的配置

| 项目 | 值 |
|---|---|
| 型号 | **Intel RealSense D435i**（序列号 347622073366，固件 5.15.1.55） |
| 彩色流 | **640×480 RGB8 @15fps** ✓ 稳定 |
| 深度流 | **已关闭**（带宽不够，见下） |
| 红外 / IMU | **已关闭** |
| 话题 | `/camera/camera/color/image_raw` |
| 内参话题 | `/camera/camera/color/camera_info` |

### 2.2 相机内参（已验证）

```
fx = 603.3994140625     fy = 602.9302368164062
cx = 330.577880859375   cy = 251.92457580566406

畸变系数 D = [0.0, 0.0, 0.0, 0.0, 0.0]      ← 全零 = 已做畸变校正
畸变模型 distortion_model = "plumb_bob"
坐标系 frame_id = "camera_color_optical_frame"
```

**有了内参就能算真实的 3D 距离** —— 不需要再靠"像宽反比"估距离，
也不需要手工标定。直接用 dt-apriltags 的 `estimate_tag_pose(camera_params, tag_size)`。

### 2.3 ⚠️ 为什么必须用低带宽配置

原配置是 **1280×720 RGB@30 + 848×480 深度@30 + 双红外@30**，
这个带宽超出 Jetson USB 控制器的能力，表现为：

```
usb 2-3.1: USB disconnect, device number 40
tegra-xusb ERROR Transfer event TRB DMA ptr not part of current TD ep_index 6
usb 2-3.1: new SuperSpeed USB device number 41     ← 又枚举一遍
```

设备号 **39 → 40 → 41 反复递增**，相机在断开/重连死循环里，**节点虽然活着但一帧都发不出来**。
`dmesg` 里累计 **33 次 USB disconnect**。

**降到只开彩色 640×480@15 后稳定。**

### 2.4 启动相机

```bash
ssh ysc@10.21.33.102      # 密码见内部记录

# 先清掉卡死的旧进程
pkill -9 -f realsense2_camera_node; sleep 4

source /opt/ros/jazzy/setup.bash
cd ~
setsid nohup ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true -p enable_depth:=false \
  -p enable_infra1:=false -p enable_infra2:=false \
  -p enable_gyro:=false -p enable_accel:=false \
  -p rgb_camera.color_profile:=640x480x15 \
  > /tmp/rs2.log 2>&1 < /dev/null &

# 等约 20 秒，验证
source /opt/ros/jazzy/setup.bash
ros2 topic hz /camera/camera/color/image_raw
```

**验证成功的标志**（`/tmp/rs2.log` 里）：

```
Open profile: stream_type: Color(0), Format: RGB8, Width: 640, Height: 480, FPS: 15
RealSense Node Is Up!
```

> **注意**：`setsid nohup ... &` 这种后台启动，通过 SSH 执行时不要读取该命令的输出
> （后台进程会占住通道的 stdout，导致读取阻塞超时）。发射后不管即可。

---

## 3. tag 信息

| 项目 | 值 |
|---|---|
| 家族 | **`tagStandard41h12`** |
| ID | **1** |
| 官方原图 | https://github.com/AprilRobotics/apriltag-imgs/blob/master/tagStandard41h12/tag41_12_00001.png |
| 布局 | 9×9 模块（约 4.5×4.5cm，视打印尺寸而定） |

**家族确认过程**：文件名 `tag41_12_00001.png` 与官方 `apriltag-imgs` 仓库的命名规则一致
（`tag41_12_` = `tagStandard41h12`）。

⚠️ **`cv2.aruco` 不支持这个家族**（只有 16h5 / 25h9 / 36h10 / 36h11），必须用 `dt-apriltags` 或 `pupil-apriltags`。

⚠️ **打印注意**：尺寸尽量大（A4 打满）、四周留白边（quiet zone ≥ 1 格）、
**别用光泽相纸**（反光会让 tag 变白）、贴平不要卷曲。

---

## 4. 运行环境

### 4.1 检测库（装在 .103 上）

```bash
# Ubuntu 24.04 有 PEP 668 保护，不能直接 pip 装系统环境 → 必须用 venv
python3 -m venv --system-site-packages /home/user/follow_env
/home/user/follow_env/bin/pip install -i https://pypi.mirrors.ustc.edu.cn/simple dt-apriltags
```

依赖说明：
- `--system-site-packages`：继承系统里已有的 `cv2` 4.6.0
- 用**中科大镜像**（清华/阿里/github 在这台机器上不通，百度通 = 有外网）
- `dt-apriltags` 有**预编译的 aarch64 轮子**，不用编译

> **踩过的坑**：
> 1. Ubuntu 24.04 的 **PEP 668** 禁止直接 `pip install` 到系统环境
> 2. 循环里反复创建 `Detector` 对象会让底层 C 库 **malloc 崩溃**
>    （`malloc(): mismatching next->prev_size`）→ **每个家族单独开进程**

### 4.2 读相机权限（.103）

`.103` 上的 `user` **不在 `video` 组**，读 `/dev/video*` 需要 sudo。

```bash
sudo usermod -aG video user     # 然后重新登录
```

`.102` 上的 `ysc` 已经在 `video` 和 `plugdev` 组里，设备是 `crw-rw-rw-`，**不用 sudo**。

---

## 5. 运行追踪程序

```bash
# 在 .103 上（控制指令发给 127.0.0.1:30004）
python3 tag_follower.py --camera 6                 # 干跑：只检测不发指令
python3 tag_follower.py --camera 6 --go            # 真实控制
python3 tag_follower.py --camera 6 --calib         # 测距标定模式
```

**干跑模式不检查 RL 控制状态**，方便狗趴着时单独调检测；
**`--go` 模式会检查**，未进入 `MotionState=17` 直接拒绝运行。

### 五道安全

| 机制 | 说明 |
|---|---|
| tag 丢失 | 超过 `LOST_TIMEOUT_S`(0.5s) 立即输出归零 |
| 距离突变 | 单帧变化 > `JUMP_LIMIT_M`(0.6m) 丢弃该帧 |
| 速度限幅 | `MAX_VX=0.35`、`MAX_WZ=0.45` |
| RL 门禁 | `--go` 时未处于 RL 控制(17) 拒绝运行 |
| 异常归零 | `finally` 里必发零速 |

---

## 6. 控制通道

用 **ASDU UDP → `127.0.0.1:30004`**，理由：

- 已实测可用（`robot_dog_sdk` 那套验证过）
- 机器狗自己的 `golai` 网页遥控（8088）也走这条
- ROS2 那条路走不通：**`drdds` 消息包里没有 `NavCmd`**（文档 §2.3.1 写的类型不存在）

⚠️ **不要和其他客户端同时控制** —— 文档 §1.5 的 `0xE006` 要求轴指令 2 秒内同源，
`golai` 遥控器和本程序同时跑会互相踢掉。

---

## 7. 待办 / 已知问题

### 7.1 阻塞项：相机里看不到有效 tag

之前相机拍到的是一张**网格状小标记的纸**，但：

- 单个标记太小（1 米外像宽可能只有十几像素）
- **纸是卷曲的**（包在横杆上）
- 拍出来很糊

试过 **4 个 tag 家族 × 12 组鱼眼畸变矫正 × 多组检测参数**，**全部 0 命中**。
判断：**那张纸上的图案不是可解码的 AprilTag**。

**→ 需要用官方原图重新打印一张，贴平、贴大。**

### 7.2 tag 位姿话题是"空壳"

`.102` / `.103` 上都有这两条话题，但**没有任何数据**：

| 话题 | 类型 | 状态 |
|---|---|---|
| `/pose_in_apriltag_corrected` | `geometry_msgs/msg/PoseStamped` | 有发布者，**无数据** |
| `/tag_status` | `drdds/msg/StdMsgInt32` | 有发布者，**无数据** |

**原因**：发布它们的 `s10_sdk_deploy`（`lg` 用户的自研 RL 部署节点）**崩溃了**：

```
/home/lg/goai_embodied_future_material/core    ← 285MB coredump，2026-09-18 18:33
from '/home/user/goai_embodied_future_material/install/s10_sdk_deploy/...'
```

`lg` 的 bash_history 显示他在跑 `ros2 run s10_sdk_deploy rl_deploy`。

**→ 如果要复用这套 tag 管线，得先让它别崩。但本项目的检测是独立实现的，不依赖它。**

### 7.3 其它

- **RealSense 历史上掉线 33 次**，低带宽配置目前稳；再掉线就重启节点
- `.103` 的 ROS 图像话题（`/usb_camera_1/image_raw` 等）**全部没有数据**
- `.102` 上有 **12 个 ROS 域**在跑（`mcast_relay` 搭的），但只有 **domain 0** 有真实话题
- `.102` 上还有第三个用户 `lg`，其 `~/goai_embodied_future_material` 是**自研工程**（从竞赛仓库改的），`ysc` 需要 `sudo` 才能读

---

## 8. 相关文件

| 文件 | 说明 |
|---|---|
| `tag_follower.py` | 追踪程序（自包含单文件，可直接拷到机器狗上跑） |
| `../robot_dog_sdk/` | ASDU 协议层 + Web 遥控 + 模拟器（协议实现参考） |
| `../robot_dog_sdk/docs/软件开发指南.pdf` | 厂商协议文档 |
