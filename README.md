
# AgiBot-D1-EgoPlanner2D-TEB

**基于 EGO-Planner-2D 与 TEB 局部规划器的智元（Agibot）四足机器人集成轨迹优化与控制框架。**

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![Platform](https://img.shields.io/badge/Platform-AgiBot--D1-orange.svg)](https://github.com/AgibotTech)
[![Architecture](https://img.shields.io/badge/Arch-x86--64-lightgrey.svg)](#系统架构)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 🖥 系统架构

本项目采用**分布式控制架构**：
*   **计算平台**：算法运行于独立的上位机（如笔记本电脑，**x86_64** 架构）。
*   **通信链路**：上位机通过局域网（UDP 协议）与智元四足机器人底层嵌入式系统通信。
*   **核心模块依赖**：
    *   **局部规划器Ego-Planner-2D：** [Ego-Planner-2D-ROS2](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2) —— 负责生成平滑且避障的轨迹。
    *   **局部控制器TEB：** [AgiBot_D1_Navigation_ROS2](https://github.com/JackJu-HIT/AgiBot_D1_Navigation_ROS2) —— 负责轨迹跟踪控制。
    *   **硬件通信层：** [AgiBot_D1_Controller_ROS2](https://github.com/JackJu-HIT/AgiBot_D1_Controller_ROS2) —— 驱动层，封装了与智元 SDK 的交互。

---

## 🚀 快速上手

### 1. 环境准备
*   **操作系统：** Ubuntu 22.04
*   **ROS 版本：** ROS 2 Humble
*   **硬件依赖：** 智元机器人底层 SDK (AgiBot SDK)

### 2. 解决网络干扰 (⚠️ 关键步骤)
**背景：** ROS 2 默认的 DDS 组播流量极易干扰机器狗底层 SDK 的 UDP 实时通信包，导致控制延迟或指令丢失。

**操作：** 在运行 ROS 节点的每一个终端中，必须执行以下命令以强制使用 **共享内存（SHM）** 通信：
```bash
export FASTDDS_BUILTIN_TRANSPORTS=SHM
```

### 3. 编译安装
```bash
mkdir -p ~/agibot_ws/src
cd ~/agibot_ws/src
# 克隆本项目及核心模块
git clone https://github.com/JackJu-HIT/AgiBot-D1-EgoPlanner2D-TEB.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

---

## 💻 运行流程与调试

请严格按照以下顺序操作，确保实机运行安全：

### 第一步：启动硬件驱动
建立上位机与机器狗硬件的 UDP 通信链路。
```bash
ros2 run dog_controller dog_driver_node
```

### 第二步：启动状态管理节点
用于控制机器人进入预备状态。
```bash
ros2 run dog_controller dog_keyboard_node
```
*   **操作：** 在该终端按键，控制机器人进入 **Stand（站立）** 状态。

### 第三步：启动规划与控制核心
运行轨迹优化与跟踪控制算法。
```bash
ros2 run ego-planner motion_plan
```

### 第四步：交互与避障验证 (RViz2)
打开 RViz2 订阅相关话题，并使用顶部的工具栏进行交互：

1.  **设置轨迹：** 使用 **"Publish Point"** 工具依次点击两点：
    *   **第 1 点：** 机器人当前位置（Current Position）。
    *   **第 2 点：** 期望到达的目标点（Goal Position）。
    *   *完成后，机器人将自动生成直线初轨并开始移动。*
2.  **避障效果验证：** 
    由于 AgiBot-D1 原厂硬件暂不提供激光雷达点云，若需验证自主绕障功能，可以通过 RViz2 中的 **"Goal Pose"**（或自定义插件）手动发布模拟障碍物点云信息，观察 Ego-Planner 的实时重规划效果。

---

## 📊 可视化指标

| 话题名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `/run_path` | `nav_msgs/Path` | **实际轨迹**：红色线段，记录机器人实际位姿历史 |
| `/visual_teb_trajectory` | `nav_msgs/Path` | **TEB 轨迹**：控制器实时计算的预测/跟踪轨迹 |
| `/visual_ego_trajectory` | `nav_msgs/Path` | **Ego 优化轨迹**：规划器输出的高阶连续平滑轨迹 |
| `/visual_global_path` | `nav_msgs/Path` | **全局基准**：起始点到终点的原始目标直线 |
| `/visual_local_opbstacles` | `sensor_msgs/PointCloud2` | **局部障碍物**：用于规划避障的点云可视化 |
| `/odom` | `nav_msgs/Odometry` | **里程计反馈**：用于闭环控制的状态估计数据 |

---

## ℹ️ 其他信息

*   **智元 SDK 官方仓库：** [AgibotTech/agibot_D1_Edu-Ultra](https://github.com/AgibotTech/agibot_D1_Edu-Ultra)
*   **项目贡献：** 特别感谢 **智元机器人 (Agibot)** 提供的 `agibot_D1_Edu` 硬件平台及技术支持，本项目完成了 C++ 核心算法在 x86 上位机上的深度优化与实机部署。

---

## 📡 关注我们

获取更多关于四足机器人运动控制、路径规划的干货分享，请关注微信公众号：

**🔍 机器人规划与控制研究所**

[📄 点击此处阅读本项目相关的技术专栏文章](https://mp.weixin.qq.com/s/fBVtQuWLc2vMCbczovkCiw)

---
*© 2026 JackJu-HIT. Licensed under the MIT License.*
