# OriginCar Navigation

基于 ROS2 Humble + Nav2 的自主导航功能包。

该功能包负责机器人地图定位、路径规划、运动控制以及导航系统启动管理。

---

## 功能

- SLAM 建图
- 基于已有地图定位
- Nav2 自主导航
- 局部路径规划与控制
- RViz 可视化配置
- 导航参数统一管理

---

## 文件结构

```
origincar_nav
    ├── config
    │       ├── nav2_params.yaml
    │       ├── mapper_params_online_async.yaml
    │       └── mapper_params_localization.yaml
    │
    ├── launch
    │       ├── navigation2.launch.py
    │       ├── pure_nav.launch.py
    │       └── slam_toolbox.launch.py
    │
    ├── map
    │    ├── race_map.yaml
    │    ├── race_map.pgm
    │    ├── race_map.data
    │    └── race_map.posegraph
    │
    └── rviz
         └── npc.rviz
```

---

## Launch

### 完整导航

启动：

```bash
ros2 launch origincar_nav pure_nav.launch.py
```
包含：

- Map Server
- Slamtoolbox_Localization
- Controller
- Lifecycle_manager


编译
```bash
colcon build --packages-select origincar_nav
source install/setup.bash
```
说明

作为机器人导航核心模块，
负责连接：

```
传感器
  |
  v
定位(SLAM/Localization)
  |
  v
Nav2 Navigation Stack
  |
  v
Controller
  |
  v
底盘运动
```