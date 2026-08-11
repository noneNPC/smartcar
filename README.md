# Origincar

本项目是Origincar自主导航系统，实现了从环境感知、定位、导航控制到视觉任务处理的完整流程。

系统基于 ROS 2 通信框架，结合 Nav2 Navigation Stack、SLAM Toolbox、MPPI Controller 以及任务管理模块，实现针对 Origincar 的自主导航。

---
# 队伍信息

队伍：志飞一队
# 模块文档

- [导航](docs/navigation.md)
- [路径发布](docs/path_planner.md)
- [二维码识别](docs/qr_code.md)
- [大模型视觉理解](docs/image_understanding.md)
- [信息显示](docs/display_info.md)

# 功能介绍

## 导航

支持：

- 激光雷达环境感知
- 地图定位
- 自主路径规划
- 局部避障
- 轨迹跟踪
- Origincar控制

## 定位

采用：

- SLAM Toolbox

实现：

- 环境建图
- 已知地图定位
- 位姿估计

## 控制

基于：

- ROS 2 Nav2
- MPPI Controller

实现：

- 路径跟踪
- 速度控制
- 动态避障

## 固定路径管理

包含：

- 路径文件读取
- 路径切换
- FollowPath Action 调用

## 视觉

包含：

- 二维码识别
- 图像处理
- 大模型识别模块

---

# 软件&硬件环境

RDK x5

Ubuntu 22.04 LTS

ROS 2 Humble

---

# 依赖安装

## ROS依赖

```bash
sudo apt install \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-slam-toolbox \
ros-humble-rviz2 \
ros-humble-tf2-tools \
ros-humble-cv-bridge \
ros-humble-image-transport
```

## Python依赖

```bash
pip3 install opencv-python requests
```

# 编译

```bash
cd ~/npc_ws

colcon build --symlink-install
```

加载环境：

```bash
source install/setup.bash
```

---

# 项目结构

```
npc_ws
 └── src
      ├── origincar_nav
      │       │
      │       ├── launch
      │       │     ├── navigation2.launch.py                     # Nav2导航启动文件
      │       │               ├── pure_nav.launch.py              # 纯定位导航启动文件
      │       │               └── slam_toolbox.launch.py          # SLAM
      │       │
      │       ├── config
      │       │     ├── nav2_params.yaml                          # Nav2参数配置
      │       │     ├── mapper_params_localization.yaml           # SLAM Toolbox定位参数
      │       │     └── mapper_params_online_async.yaml           # SLAM Toolbox建图参数
      │       │
      │       └── map                                             # 栅格地图及元数据
      │            ├── race_map.data
      │            ├── race_map.posegraph
      │            ├── race_map.yaml                            
      │            └── race_map.pgm                             
      │
      ├── pub_plan
      │       ├── config
      │       │      └── config.yaml                              # 固定路径发布配置
      │       │
      │       ├── path
      │       │     └── *.json                                    # 固定路径文件
      │       │
      │       └── launch
      │              └── pub_plan.launch.py                       # 路径发布
      │
      ├── qr_code_recognition
      │       └── launch
      │               └── qrcode_detect.launch.py                 # 二维码识别
      │
      ├── display_info
      │       └── launch
      │               └── display_info.launch.py                  # 信息显示
      │
      ├── ollama_image_understanding
      │       └── launch
      │               └── image_understanding.launch.py           # 大模型识别
      │
      ├── LSLIDAR_X_ROS2                                          # 激光雷达驱动
      │
      └── tools
            ├── record_path
            │      ├── record_path.py                             # 路径记录
            │      └── path_editor.html                           # 路径编辑                     
            └── race.launch.py                                    # 比赛总启动入口


```

---

# 运行方式

## 一键启动

比赛运行入口：

```bash

ros2 launch /src/tools/race.launch.py
```

启动内容：

- Origincar底盘
- 激光雷达
- slamtoolbox_localization
- Nav2_controller
- 固定路径发布
- 大模型识别
- 信息显示

---

## SLAM建图

```bash
ros2 launch origincar_nav slam_toolbox.launch.py
```

## Nav2_Controller+Slamtoolbox_localization

```bash
ros2 launch origincar_nav pure_nav.launch.py
```

## 固定路径发布

```bash
ros2 launch pub_plan pub_plan.launch.py
```


## 二维码检测识别

```bash
ros2 launch qr_code_recognition qrcode_detect.launch.py
```

## 大模型调用

```bash
ros2 launch ollama_image_understanding image_understanding.launch.py
```

---
