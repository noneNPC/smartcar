# File Path Planner

基于 ROS2 Nav2 FollowPath Action 的自定义路径发布节点。

读取预先生成的 JSON 路径文件，根据车辆当前定位状态动态截取局部路径，并发送给 Nav2 Controller 进行路径跟踪。

---

## 功能

- 支持任务一以及任务二、三顺逆时针路径切换
- 从 JSON 文件加载路径
- 根据 TF 获取车辆当前位置
- 自动搜索最近路径点
- 动态裁剪局部路径
- 支持路径前瞻距离配置
- 通过 Nav2 FollowPath Action 控制车辆运动
- 支持路径异常自动恢复

---

## 输入

### 路径切换信号

Topic:

/sign_switch


类型:

origincar_msg/msg/Sign


根据 `sign_data` 选择不同路径：

|sign_data|路径|任务|
|-|-|-|
|2|path 1|任务一|
|3|path 2|任务二、三（顺时针）|
|4|path 3|任务二、三（逆时针）|

---

## 输出

### 可视化路径

Topic:

/plan

类型:

nav_msgs/msg/Path

用于 RViz 显示当前发送给控制器的局部路径。

---

## Nav2接口

调用：

/follow_path

Action:

nav2_msgs/action/FollowPath

控制器：

FollowPath

---

## 路径文件

路径存放：
```
pub_plan/
└── path/
      ├── 1.json
      ├── 2.json
      └── 3.json
```

JSON 格式：

```json
[
  {
    "x": 1.0,
    "y": 2.0,
    "w": 1.0
  }
]
```

# 编译
```bash
cd ~/npc_ws
colcon build --packages-select pub_plan
source install/setup.bash
```

# 运行
```bash
ros2 launch pub_plan pub_plan.launch.py
```
# 流程
```
JSON Path
    |
    |
    v
FilePathPlanner
    |
    |
获取 TF(map -> base_link)
    |
    |
寻找最近路径点
    |
    |
裁剪局部 Path
    |
    |
Nav2 FollowPath Controller
    |
    |
车辆执行
```