# S10 决赛最小交付代码

本目录是可单独复制到 NVIDIA AGX 的决赛代码工作区。它不依赖外层目录中的
`first_ok`、`upstream_repos` 或临时文件。

## 目录

```text
S10_final_delivery/
├── src/
│   ├── drdds/                 # S10 ROS 2 消息
│   ├── S10_sdk_deploy/        # RL 底盘控制和 300 ms /STEER watchdog
│   ├── s10_sensor_adapter/    # Airy IMU 坐标适配
│   ├── s10_route_nav/         # 路线、检查点、避障、健康和指令仲裁
│   ├── s10_nav_bringup/       # 配置、launch 和现场脚本
│   ├── lightning/             # 已打 S10 补丁的建图定位
│   ├── rslidar_sdk/           # 已启用 XYZIRT/变换/IMU 的雷达驱动
│   └── rslidar_msg/           # RoboSense 消息
├── data/
│   ├── maps/                  # 现场生成的最终地图
│   ├── routes/                # 定位模式下录制的最终路线
│   ├── bags/                  # 调试录包，不提交
│   └── logs/                  # 运行日志，不提交
└── patches/                   # 全新克隆上游仓库时使用
```

## AGX 构建

当前交付包只保留 ARM ONNX Runtime。必须在 Ubuntu 24.04、ROS 2 Jazzy 的 AGX
上构建，并指定 ARM 平台：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/nvidia/S10_final_delivery
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_PLATFORM=arm
source install/setup.bash
```

构建日志中必须确认：

```text
POINT_TYPE is XYZIRT
Enable transform
Enable imu data parse
```

## 路径配置

复制到 AGX 后，修改以下文件中的绝对路径：

- `src/s10_nav_bringup/config/s10_lightning_slam.yaml`
- `src/s10_nav_bringup/config/s10_lightning_loc.yaml`
- `src/s10_nav_bringup/scripts/*.sh`

正式数据建议放在：

```text
/home/nvidia/S10_final_delivery/data/maps/final_map_v01/
/home/nvidia/S10_final_delivery/data/routes/final_route_v01.yaml
```

## 启动顺序

建图：

```bash
ros2 launch s10_nav_bringup mapping.launch.py
```

加载最终地图后录制路线：

```bash
ros2 launch s10_nav_bringup record_route.launch.py \
  output_file:=/home/nvidia/S10_final_delivery/data/routes/final_route_v01.yaml
```

比赛导航：

```bash
ros2 launch s10_nav_bringup navigation.launch.py \
  route_file:=/home/nvidia/S10_final_delivery/data/routes/final_route_v01.yaml
```

启动后 `/STEER` 默认保持为零。确认 `/s10_nav/healthy=true`，并用手柄让机器人
站立、进入 RL 模式后，再显式启用自主导航：

```bash
ros2 service call /s10_nav/set_enabled std_srvs/srv/SetBool '{data: true}'
```

急停：

```bash
ros2 service call /s10_nav/emergency_stop std_srvs/srv/Trigger '{}'
```

## 重要说明

- 当前 `data/maps/` 没有正式地图，必须现场建图。
- 当前只有安全测试路线模板，必须在最终地图定位模式下重新录制比赛路线。
- `src/lightning` 和 `src/rslidar_sdk` 已经应用补丁，不要再次运行补丁脚本。
- D435i 相机不属于当前 P0 必需代码，未放入交付包。
- 完整 ROS 编译、雷达/IMU 轴向和实机运动测试必须在 AGX 上完成。
