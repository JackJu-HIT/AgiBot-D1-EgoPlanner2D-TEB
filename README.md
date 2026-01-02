
# AgiBot-D1-EgoPlanner2D-TEB

**基于 EGO-Planner-2D 与 TEB 局部规划器的智元（Agibot）四足机器人集成轨迹优化与控制框架。**

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![Platform](https://img.shields.io/badge/Platform-AgiBot--D1-orange.svg)](https://github.com/AgibotTech)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 🛠 项目架构

本项目是智元四足机器人自主导航方案的核心集成仓库，其功能依赖于以下模块：

*   **局部规划器：** [Ego-Planner-2D-ROS2](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2) —— 负责生成平滑且避障的轨迹。
*   **局部控制器：** [AgiBot_D1_Navigation_ROS2](https://github.com/JackJu-HIT/AgiBot_D1_Navigation_ROS2) —— 负责轨迹跟踪控制。
*   **硬件通信层：** [AgiBot_D1_Controller_ROS2](https://github.com/JackJu-HIT/AgiBot_D1_Controller_ROS2) —— 负责与智元机器人底层 SDK 进行 UDP 通信。

---

## 🚀 快速上手

### 1. 环境准备
*   **操作系统：** Ubuntu 22.04
*   **ROS 版本：** ROS 2 Humble
*   **依赖：** 确保已安装智元机器人底层 SDK。

### 2. 解决网络干扰 (⚠️ 重要)
**背景：** ROS 2 默认的 DDS 流量可能会阻塞或干扰机器狗底层 SDK 的 UDP 通信包，导致控制延迟或掉线。

**操作：** 在运行 ROS 节点的每一个终端中，必须执行以下命令以强制使用**共享内存（SHM）**通信：
```bash
export FASTDDS_BUILTIN_TRANSPORTS=SHM
```

### 3. 编译安装
```bash
mkdir -p ~/agibot_ws/src
cd ~/agibot_ws/src
# 克隆本项目及相关依赖
git clone https://github.com/JackJu-HIT/AgiBot-D1-EgoPlanner2D-TEB.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

---

## 💻 运行流程

请严格按照以下步骤操作，以确保规划与执行的安全：

### 第一步：启动驱动模块
负责建立与机器狗硬件的 UDP 通信链路。
```bash
ros2 run dog_controller dog_driver_node
```

### 第二步：启动遥控节点
用于切换机器人状态（如从趴下切换到站立）。
```bash
ros2 run dog_controller dog_keyboard_node
```
*   **操作：** 在该终端按键，使机器人进入 **Stand（站立）** 状态。

### 第三步：启动规划与控制核心
```bash
ros2 run ego-planner motion_plan
```

### 第四步：RViz2 交互与轨迹下发
打开 RViz2 并加载配置文件，使用顶部的 **"Publish Point"** 工具：

1.  **打点 1（起点）：** 必须点在机器人当前的**脚下位置**（Current Position）。
2.  **打点 2（终点）：** 点在期望到达的**目标位置**（Goal Position）。
3.  **自动执行：** 算法将自动生成连接两点的直线初始轨迹，并经由 Ego-Planner 优化后由 TEB 驱动机器人自主移动。

---

## 📊 可视化监控

为了直观观测算法表现，请在 RViz2 中订阅以下话题：

| 话题名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `/run_path` | `nav_msgs/Path` | **实际轨迹**：机器人走过的历史红色路径 |
| `/visual_teb_trajectory` | `nav_msgs/Path` | **TEB 轨迹**：控制器实时输出的预测轨迹 |
| `/visual_ego_trajectory` | `nav_msgs/Path` | **Ego 优化轨迹**：局部规划器平滑后的轨迹 |
| `/visual_global_path` | `nav_msgs/Path` | **全局目标**：起点到终点的初始直线 |
| `/visual_local_opbstacles` | `sensor_msgs/PointCloud2` | **局部障碍物**：点云可视化 |
| `/odom` | `nav_msgs/Odometry` | **里程计**：机器人当前位姿定位反馈 |

---

## ℹ️ 更多信息

*   **官方 SDK 仓库：** [AgibotTech/agibot_D1_Edu-Ultra](https://github.com/AgibotTech/agibot_D1_Edu-Ultra)
*   **致谢：** 特别感谢 **智元机器人 (Agibot)** 提供的 `agibot_D1_Edu` 硬件平台及技术支持，助力本项目顺利完成从仿真到实机的 C++ 代码迁移与调试。

---

## 📡 关注我们

更多关于四足机器人规划与控制的深度技术文章、视频演示，欢迎关注我们的微信公众号：

**🔍 机器人规划与控制研究所**

[📄 点击此处阅读相关技术专栏文章](你的文章链接)

---
*© 2026 JackJu-HIT. 基于 MIT 协议开源。*