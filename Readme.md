# Ros2AmovG1 🎯

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Ros2AmovG1** 是在 **ROS 2 Humble / Ubuntu 22.04** 环境下驱动 **Amov G1 三轴云台** 的控制仓库，基于官方 **C++ SDK** [https://docs.amovlab.com/gimbalwiki/#/src/G1/doc/AmovGimbalStudio](https://docs.amovlab.com/gimbalwiki/#/src/G1/doc/AmovGimbalStudio)封装成 ROS Node，提供串口通讯、话题控制与状态发布功能。

---

## ✨ 功能特性

| 类别               | 说明                                                            |
| ---------------- | ------------------------------------------------------------- |
| **即插即用**         | 仅需 USB‑UART 连接；`sudo chmod 777 /dev/ttyUSB0` 后即可运行            |
| **标准 ROS Topic** | 控制指令 → `SMX/SportCmd` ；云台状态 ← `SMX/GimbalState`（可 TF 与其它模块复用） |
| **参数化配置**        | 关键话题名 / 串口设备 / Gimbal ID 均由 `config.yaml` 管理                  |
| **SDK 原生封装**     | 调用 Amov 官方 SDK，稳定可靠，支持后续固件功能扩展                                |

---

## 🏗️ 生态仓库详细信息参见

[https://github.com/ShineMinxing/Ros2Go2Estimator](https://github.com/ShineMinxing/Ros2Go2Estimator)

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

## 📄 深入阅读

* 技术原理笔记：[https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22](https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22)
* ROS1 版本参考：[https://github.com/ShineMinxing/FusionEstimation](https://github.com/ShineMinxing/FusionEstimation)

---

## 📨 联系我们

| 邮箱                                          | 单位           |
| ------------------------------------------- | ------------ |
| [401435318@qq.com](mailto:401435318@qq.com) | 中国科学院光电技术研究所 |

> 📌 **本仓库仍在持续开发中** — 欢迎 Issue / PR 交流、贡献！