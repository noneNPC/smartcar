# Display_info

基于 ROS2 + SPI ST7796S LCD 的信息显示节点。

订阅系统中的文本信息，并通过 ST7796S 液晶屏实时显示，用于任务状态、视觉识别结果等信息展示。

---

## 功能

- 支持 ST7796S SPI LCD 显示
- ROS2 Topic 实时更新显示内容
- 支持中文字体显示
- 自动调整字体大小
- 自动换行和居中显示
- RGB888 转 RGB565 高速刷新
- SPI 高速传输

---

## 输入

### 显示信息

Topic:

/display_info

类型:

std_msgs/msg/String

显示内容来源：

- 二维码识别结果
- 图像理解结果

---

## 硬件接口

屏幕：

ST7796S 480x320

通信：

四线SPI

| 屏幕引脚 | RDK X5引脚 | 功能 |
|---|---|---|
| VCC | 5V | 电源 |
| GND | GND | 地 |
| SCL | Pin 23 | SPI Clock |
| SDA | Pin 19 | SPI MOSI |
| CS | Pin 24 | SPI CS |
| DC | Pin 18 | Data/Command |
| RST | Pin 22 | Reset |
| BL | 3.3V | 背光 |
---

## 工作流程

```
ROS2 Topic
|
v
/display_info
|
v
Display Node
|
v
PIL 图像渲染
|
v
RGB888 -> RGB565
|
v
SPI发送
|
v
ST7796S显示

```

---

## 显示特性

- 自动根据文本长度选择字号
- 支持中文显示
- 支持多行文本
- 避免重复内容刷新

---

## 编译

```bash
cd ~/npc_ws
colcon build --packages-select display_info
source install/setup.bash
```

# 说明

机器人人机交互显示模块

```
二维码识别
      |
图像理解
      |
机器人状态
      |
      v
/display_info
      |
      v
ST7796S Display
```
