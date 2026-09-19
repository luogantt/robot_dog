# robot_dog

山猫 S10 机器狗的控制程序集合。

## 内容

| 目录 | 说明 |
|---|---|
| [`robot_dog_sdk/`](robot_dog_sdk/) | **主项目** —— ASDU/UDP 协议实现 + Web 遥控界面 + 模拟器 + 测试 |

具体用法、安全注意事项、实机验证记录，看 [robot_dog_sdk/README.md](robot_dog_sdk/README.md)。

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
