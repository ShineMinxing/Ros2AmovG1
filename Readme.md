# Ros2AmovG1 🎯

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Ros2AmovG1** 是在 **ROS 2 Humble / Ubuntu 22.04** 环境下驱动 **Amov G1 三轴云台** 的控制仓库，基于官方 **C++ SDK** 封装成 ROS Node，提供串口通讯、话题控制与状态发布功能。

---

## ✨ 功能特性

| 类别               | 说明                                                            |
| ---------------- | ------------------------------------------------------------- |
| **即插即用**         | 仅需 USB‑UART 连接；`sudo chmod 777 /dev/ttyUSB0` 后即可运行            |
| **标准 ROS Topic** | 控制指令 → `SMX/SportCmd` ；云台状态 ← `SMX/GimbalState`（可 TF 与其它模块复用） |
| **参数化配置**        | 关键话题名 / 串口设备 / Gimbal ID 均由 `config.yaml` 管理                  |
| **SDK 原生封装**     | 调用 Amov 官方 SDK，稳定可靠，支持后续固件功能扩展                                |

---
## 🏗️ 生态仓库一览

| 范畴       | 仓库                                                                                                   | 功能简介                             |
| -------- | ---------------------------------------------------------------------------------------------------- | -------------------------------- |
| **底层驱动** | [https://github.com/ShineMinxing/Ros2Go2Base](https://github.com/ShineMinxing/Ros2Go2Base)                                                                                 | DDS 桥、Unitree SDK2 控制、点云→Scan、TF |
| 里程计      | [Ros2Go2Estimator](https://github.com/ShineMinxing/Ros2Go2Estimator)  | 纯运动学多传感器融合                       |
| 语音 / LLM | [https://github.com/ShineMinxing/Ros2Chat](https://github.com/ShineMinxing/Ros2Chat)                 | 离线 ASR + OpenAI Chat + 语音合成      |
| 图像处理     | [https://github.com/ShineMinxing/Ros2ImageProcess](https://github.com/ShineMinxing/Ros2ImageProcess) | 相机、光点/人脸/无人机检测                   |
| 吊舱跟随     | [https://github.com/ShineMinxing/Ros2AmovG1](https://github.com/ShineMinxing/Ros2AmovG1)             | Amov G1 吊舱控制、目标跟踪                |
| 工具集      | **Ros2Go2Estimator (本仓库)**               | 蓝牙 IMU、手柄映射、吊舱闭环、数据采集            |

> ⚠️ 按需克隆：如仅需云台功能，可单独编译本仓库即可。

---

## 📂 本仓库结构

```
Ros2AmovG1/
├── g1_gimbal/                     # ROS2 package
│   ├── launch/                    # 示例 launch 文件
│   └── src/g1_gimbal_node.cpp     # 封装 Amov SDK
├── config.yaml                    # 话题 / 串口等参数
└── Readme.md                      # ← 当前文件
```

---

## ⚙️ 参数速查 `config.yaml`

| 参数名                  | 默认值               | 说明                              |
| -------------------- | ----------------- | ------------------------------- |
| `GIMBAL_STATE_TOPIC` | `SMX/GimbalState` | 发布云台姿态 (`Float64MultiArray[6]`) |
| `GIMBAL_CMD_TOPIC`   | `SMX/SportCmd`    | 订阅控制数组（同运动指令复用）                 |
| `UART_PORT`          | `/dev/ttyUSB0`    | 串口设备路径                          |
| `GIMBAL_ID`          | `G1`              | 设备 ID（多云台场景可区分）                 |

---

## 🛠️ 安装与运行

```bash
# 1. 依赖（若未安装）
sudo apt install libserialport-dev

# 2. clone 到工作空间
cd ~/ros2_ws/LeggedRobot/src
git clone --recursive https://github.com/ShineMinxing/Ros2AmovG1.git

# 3. 赋予串口权限
sudo chmod 777 /dev/ttyUSB0      # 或 udev 规则持久化

# 4. 编译
cd .. && colcon build --packages-select g1_gimbal
source install/setup.bash

# 5. 运行示例
ros2 run g1_gimbal g1_gimbal_node
```

> 🛈 若同时使用 **sport\_control** 包，可直接通过 `SMX/SportCmd` 数组控制云台。

---

## 🎥 视频演示

| 主题               | 点击图片观看                                                                                                                                |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| 纯里程计建图 (站立/四足切换) | [![img](https://i1.hdslb.com/bfs/archive/4f60453cb37ce5e4f593f03084dbecd0fdddc27e.jpg)](https://www.bilibili.com/video/BV1UtQfYJExu)  |
| 行走误差 0.5 %‑1 %   | [![img](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/) |
| 爬楼梯高度误差 < 5 cm   | [![img](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/) |
| 380 m 距离偏差 3.3 % | [![img](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/) |
| 语音交互 + 地图导航      | [![img](https://i2.hdslb.com/bfs/archive/5b95c6eda3b6c9c8e0ba4124c1af9f3da10f39d2.jpg)](https://www.bilibili.com/video/BV1HCQBYUEvk/) |
| 吊舱协同光点/人脸跟踪      | [![img](https://i0.hdslb.com/bfs/archive/5496e9d0b40915c62b69701fd1e23af7d6ffe7de.jpg)](https://www.bilibili.com/video/BV1faG1z3EFF/) |

---

## 📄 深入阅读

* 技术原理笔记：[https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22](https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22)
* ROS1 版本参考：[https://github.com/ShineMinxing/FusionEstimation](https://github.com/ShineMinxing/FusionEstimation)

---

## 📨 联系我们

| 邮箱                                          | 单位           |
| ------------------------------------------- | ------------ |
| [401435318@qq.com](mailto:401435318@qq.com) | 中国科学院光电技术研究所 |

> 📌 **本仓库仍在持续开发中** — 欢迎 Issue / PR 交流、贡献！