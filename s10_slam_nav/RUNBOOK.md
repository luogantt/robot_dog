# RUNBOOK — S10 自主导航现场运行手册

适用本机(AGX,`10.21.33.102`)已部署工作区 `/home/lg/.project/S10_final_delivery`。
三个场景:**A 建图 → B 定位与路线示教 → C 比赛运行**。安全红线见 §5。

---

## 0. 前置检查

| 项 | 要求 |
|---|---|
| 机器人 | 已上电,站姿可用手柄遥控;进 SDK 模式需官方授权码(手柄操作) |
| 网络 | AGX `end0 = 10.21.33.102/24`,双雷达在 `10.21.33.201/202`,机器人 `10.21.33.103` |
| 磁盘 | ≥ 5 GB 空闲(`df -h /home`);建图录包约 13 MB/s |
| 测试域 | **台架/联调用 `ROS_DOMAIN_ID=2`**;比赛时导航栈与 `rl_deploy` 必须在**同一个域** |

首次运行先跑自检:

```bash
cd /home/lg/.project/S10_final_delivery
bash src/s10_nav_bringup/scripts/preflight_check.sh
```

## 1. 构建(已构建过可跳过)

```bash
source /opt/ros/jazzy/setup.bash
cd /home/lg/.project/S10_final_delivery
# 第三方(glog/Pangolin,免 sudo,已装于 .thirdparty/)
export MAKEFLAGS=-j3 LD_LIBRARY_PATH=$PWD/.thirdparty/lib:$LD_LIBRARY_PATH
colcon build --symlink-install --executor sequential --parallel-workers 3 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_PLATFORM=arm \
  -DCMAKE_PREFIX_PATH=$PWD/.thirdparty
```
构建日志须出现:`POINT_TYPE is XYZIRT` / `Enable Transform Fucntions` / `Enable imu data parse`。

**每次运行前**都要先设置环境:

```bash
cd /home/lg/.project/S10_final_delivery
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=2                                   # 比赛时改为与 rl_deploy 相同的域
export LD_LIBRARY_PATH=$PWD/.thirdparty/lib:$LD_LIBRARY_PATH
```

## A. 建图(一次性,赛前)

1. 机器人放到场地起点,静止等待 10~20 s(IMU 初始化);
2. 起建图栈(含驱动、IMU 适配、建图、录包,一条命令):
   ```bash
   bash data/logs/mapping_run.sh      # 输出 "IMU INIT DONE" 后即可行走
   ```
   或分步:`ros2 launch s10_nav_bringup mapping.launch.py`
3. 手柄遥控**低速平稳**走完全场(拐角放慢、别贴墙),**绕回起点停 10 s** 完成闭环;
4. 保存地图:
   ```bash
   ros2 service call /lightning/save_map lightning/srv/SaveMap '{}'
   # 写入 data/maps/final_map_v01/(0.pcd/1.pcd/… + index.txt + global.pcd)
   ```
5. 停止:向运行窗口 `Ctrl+C`(或 `kill -TERM` 对应进程);等录包写出 `metadata.yaml` 后再收工。

**验收标准**:日志有 `optimize finished`(回环优化);停止前定位仍连续;地图目录内 `index.txt` 非空。

## B. 定位与路线示教

1. 机器人**放回规定起点**摆正;
2. 起定位 + 录制栈:
   ```bash
   ros2 launch s10_nav_bringup record_route.launch.py
   ```
3. 确认定位稳定:`ros2 topic hz /lightning/odom`(≈10 Hz)、`ros2 topic echo /s10_nav/...` 看位姿无跳变;
4. 开始录制并遥控沿允许路线行走;每到**计分点停稳**后标记:
   ```bash
   ros2 service call /s10_route_recorder/set_recording std_srvs/srv/SetBool '{data: true}'
   ros2 service call /s10_route_recorder/mark_checkpoint std_srvs/srv/Trigger '{}'   # 每个计分点调一次
   ros2 service call /s10_route_recorder/save std_srvs/srv/Trigger '{}'
   # 输出 data/routes/final_route_v01.yaml
   ```
   (record_route.launch.py 已把输出路径参数 `output_file` 指向该文件;其他路径可用参数覆盖。)
5. 检查路线:RViz 叠加 `/lightning/global_map` 与 `/s10_nav/route_path`,确认不穿墙、不出界。

## C. 比赛运行

**控制通道已改为 ASDU UDP 直连**（`output_channel: asdu`），**不再需要 `rl_deploy`**。
导航栈把归一化速度直接通过 UDP 发给机器人本体 `10.21.33.103:30004`。

1. **机器人侧**:手柄进入 SDK 模式 → 站立(C)→ 进入 RL 控制(A)
   —— **这一步仍需手柄**，之后手柄可以放下，轴指令由本栈接管。
   轴指令只在 `MotionState=17`(RL控制) 下有效，栈里有状态门兜底
   （不在 17 会报 `ROBOT_NOT_RL` 并输出零指令）。
2. **起导航栈**（不需要起 rl_deploy）:
   ```bash
   ros2 launch s10_nav_bringup navigation.launch.py \
     route_file:=/home/lg/.project/S10_final_delivery/data/routes/final_route_v01.yaml
   ```
   启动日志应出现:
   `output_channel=asdu: 轴指令将发往 ASDU 10.21.33.103:30004（不经过 rl_deploy）`
3. 等 `/s10_nav/healthy = true` 后**显式使能自主**:
   ```bash
   ros2 topic echo /s10_nav/healthy --once     # data: true
   ros2 service call /s10_nav/set_enabled std_srvs/srv/SetBool '{data: true}'
   ```
4. 运行中监控:
   - `ros2 topic echo /s10_nav/state`（RUNNING / SLOW / AVOIDING / OBSTACLE_STOP / …）
   - `ros2 topic echo /s10_nav/checkpoint_event`（REACHED / FINISHED）
   - **终端每 5 秒打印一次机器人状态**（MotionState / Gait / Mode + 状态新鲜度）

### ⚠️ ASDU 通道的三条硬约束

| 约束 | 说明 | 表现 |
|---|---|---|
| **2 秒同源** | 文档 §1.5 的 `0xE006`:2s 内轴指令必须来自同一客户端 | **手机 App 或手柄同时控制会互相踢掉**。进比赛前务必关掉 App 的手动控制 |
| **需要 RL 控制** | 机器人不在 `MotionState=17` 时轴指令被**静默忽略** | 栈会报 `ROBOT_NOT_RL` 并输出零 |
| **无响应帧** | §1.2.5 明文规定轴指令不返回任何响应 | **丢包是静默的**，无法从协议层确认生效，只能看机器人实际状态 |

### 切换回 /STEER 通道（备用）

改 `navigation.yaml` 一行，重新起 launch 即可，**代码不用动**:

```yaml
s10_command_mux:
  ros__parameters:
    output_channel: dds      # 改回 /STEER，需要 rl_deploy 以 kDDS 模式订阅
```

## 5. 安全操作(红线)

- **急停**:`ros2 service call /s10_nav/emergency_stop std_srvs/srv/Trigger '{}'`
  → 闩锁停车,恢复需 `reset_emergency` 且健康正常。
- **手动接管**:任何时候用手柄 B(趴下)/D(阻尼)即接管;导航栈的 `/STEER` 只影响 RL 控制状态。
- **禁止**:在机器人在场、非 SDK 模式下直接起导航栈;未经检查把 `route_file` 指向非录制路线。
- 底层自带保护:0.3 s 收不到 `/STEER` 自动清零(实测验证过)。
- 上位保护(实测):健康/避障/循迹任一进程死亡或超时 → 立即零指令并取消使能,需人工重新使能。

## 6. 常见问题

| 现象 | 处理 |
|---|---|
| `healthy` 一直 false | 看 `/s10_nav/diagnostics` 的原因:定位未动/odom 超时 → 检查 `run_loc_online` 与地图路径;点云 schema → 检查驱动构建是否 XYZIRT |
| 收不到 `/IMU`(Lightning 无输入) | 确认适配器在跑且 QoS 为 reliable;`ros2 topic hz /IMU` 应 ≈200 Hz |
| 定位不收敛/跳变 | 机器人必须放在地图起点附近且朝向正确;检查 YAML `init_pos/init_quat` |
| 地图加载失败 | `map_path` 必须是绝对路径且目录含 `index.txt`;确认用的是本机路径 |
| 建图变慢/丢帧 | 建图 YAML 降低 `point_filter_num` 增大(当前 10)、`max_iteration`(当前 3);保证帧耗时 <100 ms |
| 磁盘满 | 停录包(先 `kill -TERM` 录包进程),清理 `data/bags` 旧包 |

## 7. 现场清单(打印版)

- [ ] 手柄满电;SDK 授权码在手
- [ ] `preflight_check.sh` 全绿
- [ ] 地图 `data/maps/final_map_v01/index.txt` 存在且为最终版
- [ ] 路线 `data/routes/final_route_v01.yaml` 为最终版且已 RViz 检查
- [ ] 机器人放起点 → healthy=true → set_enabled 成功
- [ ] 急停手段确认(手柄 B + 急停服务)
- [ ] 录包留档(`data/logs/record_bag.sh` 或自定)
