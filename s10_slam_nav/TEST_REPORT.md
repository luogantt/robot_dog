# S10 决赛代码部署测试报告(本机 AGX 静态/离线)

> 日期:2026-09-06
> 环境:NVIDIA AGX Orin(aarch64,Ubuntu 24.04,ROS 2 Jazzy,`end0=10.21.33.102`),
> 工作区 `/home/lg/.project/S10_final_delivery`,测试用 `ROS_DOMAIN_ID=2`(不动现场会话的 domain 0)。
> 依据:`S10_决赛现场建图定位导航技术方案.md` §13/§19 执行顺序。

## 1. 结论

对照技术方案 P0 列表:构建、传感器链路、IMU 适配、Lightning 建图/保存/重载定位、
安全门控与断链 fail-closed **全部在本机验证通过**;测试中发现并修复
**8 个部署级缺陷**(其中 4 个会直接阻断或破坏首次实机联调)。机器人运动类项目
(运动建图、路线录制、RL 模式 watchdog、实体障碍)需现场人工执行,清单见 §6。

## 2. 构建(阶段 2/3)

- 第三方库以**用户前缀免 sudo** 安装到 `.thirdparty/`(glog v0.6.0、Pangolin 0.9.x,
  注意:按 lightning README **不要** apt 装 `libgoogle-glog-dev`)。
  - 上游 Pangolin 快照缺 `pango_packetstream/include/`(vendored 拷贝不完整),
    已从上游 GitHub 补齐该目录;构建时加 `-DBUILD_PANGOLIN_PYTHON=OFF`。
- 全量 `colcon build --symlink-install --executor sequential --parallel-workers 3
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_PLATFORM=arm
  -DCMAKE_PREFIX_PATH=<ws>/.thirdparty`:8 包全部成功。
- rslidar 构建日志确认:`POINT_TYPE is XYZIRT` / `Enable Transform Fucntions` /
  `Enable imu data parse`。
- 运行期需 `export LD_LIBRARY_PATH=<ws>/.thirdparty/lib:$LD_LIBRARY_PATH`(本测试轮方案;
  正式决赛 AGX 建议按上游 README `sudo make install` 到 /usr/local,则无需该环境变量)。

## 3. 部署级缺陷与修复(全部已落在源码)

| # | 位置 | 问题 | 修复 |
|---|---|---|---|
| 1 | lightning `ui/pangolin_window_impl.cc:57` | ARM 分支 `boost::make_shared` 在 PCL≥1.12(std::shared_ptr)上**编译失败** | 改用 `std::make_shared` |
| 2 | lightning `run_slam_online.cc` / `run_loc_online.cc` | `ros2 launch` 追加的 `--ros-args` 被 gflags 拒绝 → **三个 launch 全挂**;直接在原 argv 上解析还会被 gflags 清空指针导致 rclcpp 崩溃 | 解析前截断 `--ros-args`,在私有 argv 副本上跑 gflags |
| 3 | lightning `slam.cc SaveMap` | 保存地图硬编码 `./data/<map_id>/`(**相对 CWD**、空 id 会 `remove_all` 误删目录),完全无视 YAML 绝对 `map_path` | 空 `map_id` 时保存到 YAML `system.map_path`(绝对路径) |
| 4 | `s10_sensor_adapter` | rs_driver/RSAIRY 加速度以 **g 为单位**发出(静止≈1.0 而非 9.8);原始 IMU “上”在 **−y**,与点云 z-up 不同向 | 增加 `accel_scale=9.80665`;旋转改实测值 `roll=-90°`(面内 yaw 静态不可观测,列运动验证项) |
| 5 | `s10_sensor_adapter` | 发布 best_effort,Lightning 用默认 **reliable** 订阅 → **收不到 /IMU** | pub/sub 改 reliable depth 10 |
| 6 | `s10_route_nav/health_monitor.py` | /IMU 用 best_effort depth10 订阅 200Hz 流,执行器繁忙即丢包 → “imu stale” 假故障刷屏 | 改 reliable depth 200 |
| 7 | `s10_route_nav/command_mux.py` | **安全缺陷**:`/s10_nav/healthy`、`/s10_nav/obstacle_state` 无新鲜度监控,发布者死亡后仍按最后值放行 | 新增 `feed_timeout_sec=0.5` 看门狗:任一上游停更 → 零指令 + 取消 enable + `FEED_TIMEOUT` 状态;补 6 条 pytest |
| 8 | 全线绝对路径 | `/home/nvidia/goai_ws|goai_data` 与实机不符 | scripts/launch/两个 lightning YAML 统一改本机路径 |

阈值调整(依据实测抖动):`health_monitor` 的 `imu_timeout_sec 0.2→0.5`、
`cloud_timeout_sec 0.5→0.8`(实测传输偶有 0.3~0.7s 级停顿;真失效仍在 0.5~0.8s 内捕获)。
`odom_timeout_sec` 保持 0.3s(方案 §6.3)。

## 4. 验证结果

| 项目 | 方法 | 结果 |
|---|---|---|
| 点云 XYZIRT schema | 字段类型实测 x/y/z/intensity=F32、ring=U16、timestamp=F64,step=26 | ✅ |
| 点云频率 | front/rear ≈9.1~10 Hz | ✅ |
| 点云坐标 | 静止地面法向 ≈ −z(z-up) | ✅ |
| IMU 原始流 | 200 Hz,时间戳连续 | ✅ |
| `/IMU` 输出 | 200 Hz,reliable,frame=lidar_link,covariance[0]=−1,静止 (≈0,≈0,**+9.81**) m/s² | ✅ |
| IMU 静止初始化 | Lightning `imu init done`,bias 正常 | ✅ |
| Lightning 建图 | 参数实际加载(lidar_type 4 / scan_line 32 / point_filter_num 6 / use_imu_orient false),连续运行不崩溃,odom 10 Hz + TF `map→lidar_link` | ✅ |
| 地图保存 | `save_map` 服务写入 YAML 绝对路径 `data/maps/final_map_v01/`(0.pcd/global.pcd/index.txt) | ✅ |
| 定位模式 | `loaded chunks: 1`、`use_init_pose` 生效(init 0,0,0)、odom 10 Hz + TF 稳定 | ✅ |
| 健康门控 | 全真实数据下 `healthy=true`;默认不可 enable;需显式 `set_enabled` | ✅ |
| `/STEER` 唯一发布者 | `topic info --verbose` 仅 `s10_command_mux` | ✅ |
| 断链(自检脚本 `data/logs/kill_chain_test.sh`,全 PASS) | kill health_monitor → `FEED_TIMEOUT`+零指令+拒绝再使能;kill obstacle_guard → 同上;kill route_follower → `TRACKING_TIMEOUT`+零;`emergency_stop` 闩锁+拒绝使能 | ✅ |
| route 缺失 | follower 启动即退出(无 tracking → mux 超时归零,等效 fail-closed) | ✅(行为记录) |
| 单元测试 | `pytest src/s10_route_nav/test src/s10_sensor_adapter/test`:16 passed(含新增 watchdog 6 条) | ✅ |
| **实机运动验证**(2026-09-13,人工遥控:原地左右转各约 1 圈 + 慢走往返;包 `data/bags/motion_204750`,分析脚本 `data/logs/analyze_motion.py`) | **IMU 面内 yaw 复核通过**:339/339 帧间 ICP 全部有效,6 个转向段点云 ICP 累计转角与 IMU 积分**同号且幅度一致**(-82.8/-73.0°、-90.4/-84.9°、-107.7/-87.7°、+19.6/+26.0°、+32.0/+31.9°、+85.7/+85.8°)→ `imu_adapter.yaml` 的 `yaw:0.0` 正确 | ✅ |

已知非阻断现象(记录):
- 定位在静止台上偶发 DR 时间戳回跳警告(`pgo.cc: W…时间戳…相减得负值`),与静态无观测的
  NDT 更新有关,运动后复测;`odom_timeout 0.3s` 下健康可能因定位停顿短暂翻 false 并
  自动断开自主 → 现场需按实测抖动复核该阈值。
- 200Hz IMU 链路上偶有 0.3~0.7s 级传输停顿(与共享机器的多用户负载相关),已通过
  reliable 订阅 + 放宽阈值覆盖。

## 4.1 实机建图与定位复验(2026-09-13,人工遥控,domain 2)

- **建图**:全程 199 s、路径 141 m、覆盖约 30×20 m;**成功闭环**(回到起点最近 0.39 m);
  回环检测触发 6 组并完成位姿图优化;地图 76,196 网格格点;SLAM 帧耗时经参数调整
  (`point_filter_num 6→10`、`max_iteration 4→3`)由 104 ms 降至 **36 ms/帧(0% 超预算)**。
- **地图保存**:`data/maps/final_map_v01/`(3 分块 + `index.txt` + `global.pcd`,6.8 MB)。
- **定位复验**:**5/5 次重启加载通过**——每次 `loaded chunks: 3`、YAML 初始位姿生效、
  `/lightning/odom` ≈10 Hz(`data/logs/loc_verify.sh`,日志 `data/logs/lv_loc_*.log`)。
- **原始包归档**:`data/bags/mapping_0913_205737`(zstd,1.0 GB,metadata 完整),可离线重跑建图。
- 注意:定位已跑通即可进入路线示教(§7.3);比赛运行前把机器人放回规定起点再录制路线。

## 5. 本机运行方式(测试轮)

```bash
cd /home/lg/.project/S10_final_delivery
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=2 LD_LIBRARY_PATH=$PWD/.thirdparty/lib:$LD_LIBRARY_PATH

# 自检脚本(自起自清,不依赖已在跑的进程)
bash data/logs/quiet_test.sh          # 90s 健康稳定性
bash data/logs/kill_chain_test.sh     # 断链 fail-closed 全项

# 真实链路(navigation.launch 需 lightning 已构建;start_driver:=false 复用外部驱动)
ros2 launch s10_nav_bringup navigation.launch.py start_driver:=false
```

## 6. 仍需现场人工完成(P0 尾项)

1. **运动建图**:人工遥控走完全赛道→闭环→`save_map`(位置:YAML `map_path`);
   静止数据无法产生可观测量,本轮只能验证进程/参数/保存/加载。
2. **路线录制**:加载最终地图后 `record_route.launch.py`,在各计分点调用
   `mark_checkpoint`;RViz 叠加检查穿墙/越界。
3. ~~IMU 面内 yaw 复核~~ **已完成(2026-09-13 实机原地转体验证通过,yaw=0 保持)**。
   运动验证同步结论:Lightning 里程计在转向/行走中与 IMU 一致、无发散;走动往返后回原点
   水平残差 0.33 m(含人工回位误差);odom 最大单步跳变 0.301 m(低于 0.5 m 阈值,未误触发)。
4. **RL 底盘 watchdog**:`rl_deploy` 300 ms 清零需 SDK 模式实机验证(裁定项)。
5. **实体障碍测试**:近距停车 0.6 m / 减速 1.2 m / 走廊绕行按现场标定。
6. **定位入口阈值**:按现场定位抖动复核 `odom_timeout_sec`(0.3s)与
   `max_position_jump_m`(0.5 m)/`max_yaw_jump_deg`(15°)。
7. 台上测试用 `data/logs/fake_odom.py`(合成 20Hz odom)仅为隔离定位噪声,**不可用于比赛**。

## 7. 交付物变更索引

- 源码补丁:lightning 5 文件见 `patches/lightning_bench_fixes.patch`(在原有
  `patches/lightning_s10.patch` 基础上追加);rslidar 未改动(交付版已含补丁)。
- Python 节点:`command_mux.py`(新鲜度看门狗)、`health_monitor.py`(IMU 订阅 QoS)、
  `imu_adapter.py`(量纲/旋转/QoS)、`config/imu_adapter.yaml`、新增
  `test/test_feed_watchdog.py`。
- 配置:两个 lightning YAML 与 `navigation.yaml`(阈值)、scripts/launch 本机绝对路径。
- 测试资产:`data/logs/{quiet_test.sh,kill_chain_test.sh,fake_odom.py}`(日志已加入
  .gitignore)。git 改动尚未提交,由作者决定提交与拆分。
