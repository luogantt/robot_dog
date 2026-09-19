# robot_dog

山猫 S10 机器狗的控制程序集合。

## 内容

| 目录 | 说明 |
|---|---|
| [`robot_dog_sdk/`](robot_dog_sdk/) | **ASDU 协议 + Web 遥控** —— 遥控界面、模拟器、命令行工具、测试 |
| [`follow/`](follow/) | **AprilTag 追踪** —— 让机器狗跟着 tag 走，保持在 1 米距离 |
| [`s10_slam_nav/`](s10_slam_nav/) | **SLAM 建图与自主导航** —— Lightning-LM 定位 + Pure Pursuit 循迹 + 避障 + 指令仲裁 |

各项目的用法、安全注意事项、实机验证记录，看各自的 README：

- [robot_dog_sdk/README.md](robot_dog_sdk/README.md)
- [follow/README.md](follow/README.md)
- [s10_slam_nav/RUNBOOK.md](s10_slam_nav/RUNBOOK.md) ← 现场运行手册，**最常用**
- [s10_slam_nav/README.md](s10_slam_nav/README.md)

### s10_slam_nav 的控制通道说明

该项目原本通过 `/STEER`（ROS 2 话题）经 `rl_deploy` 控制机器人；
**现已改为 ASDU UDP 直连**（`output_channel: asdu`），不再需要 `rl_deploy`。

两个通道共用同一组归一化速度值（官方 SDK 里两者写入同一个字段），
改 `src/s10_nav_bringup/config/navigation.yaml` 里一行即可切回。详见
[s10_slam_nav/RUNBOOK.md](s10_slam_nav/RUNBOOK.md) 的 C 节。

> **本仓库不含现场数据**：录包（`data/bags`）、地图（`data/maps`）、路线
> （`data/routes`）都未入库，只有目录占位。第三方二进制（onnxruntime、
> 官方 App APK）、eigen、MuJoCo mesh 也未入库。

## 快速开始

```bash
cd robot_dog_sdk

# Windows
.\start.bat control          # 控制真机
.\start.bat sim              # 本机仿真（不碰真机）

# Linux（机器狗主机）
./start.sh control
```

浏览器打开 **http://127.0.0.1:8000**

> ⚠️ 三条必读事项在 [robot_dog_sdk/README.md](robot_dog_sdk/README.md) 第 4 节：
> 急停是软件零速不是硬件急停、用网页时要关手机 App、文档里没有"上高台"步态。

## 环境

- Python 3.10+（Windows 上用 Anaconda 自带的即可）
- 依赖：fastapi / uvicorn / websockets（见 `robot_dog_sdk/requirements.txt`）
- 连接真机前需要在机器人上关闭 30004 端口的 DTLS 加密（文档 §1.1.2）
