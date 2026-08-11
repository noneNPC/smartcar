# QR Code Recognition

基于 ROS2 和 OpenCV WeChatQRCode 的二维码识别节点。

## 功能

- 订阅图像分割话题/yolo_cropped_image
- 使用 WeChatQRCode 进行二维码识别
- 根据二维码数字控制任务切换
- 发布识别结果信息
- 支持重复识别过滤

## 输入

Topic:
/yolo_cropped_image

类型:
sensor_msgs::msg::Image

---

## 输出

### 显示信息

Topic:
/display_info

类型:
std_msgs/msg/String

示例：
5 顺时针

### 控制信号

Topic:
/sign_switch

类型:
origincar_msg/msg/Sign

规则：

|二维码数字|动作|sign_data|
|-|-|-|
|奇数|顺时针|3|
|偶数|逆时针|4|

---

## 模型文件

```
qr_code_recognition/
└── model/
      ├── detect.prototxt
      ├── detect.caffemodel
      ├── sr.prototxt
      └── sr.caffemodel
```
---

## 编译

```bash
cd ~/npc_ws
colcon build --packages-select qr_code_recognition
source install/setup.bash
```
运行
```bash
ros2 launch qr_code_recognition qrcode_detect.launch.py
```