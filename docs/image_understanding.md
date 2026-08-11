# Image Understanding

基于 ROS2 + Ollama 多模态视觉模型的图像理解节点。

根据yolo识别结果，在指定位置自动采集图像，并调用视觉大模型进行内容分析，输出自然语言描述。

---

## 功能

- 订阅摄像头压缩图像
- 根据 TF 判断车辆位置
- 订阅/model_inference_data
- 调用 Ollama Vision 模型分析图片
- 发布识别结果

---

## 输入

### 图像

/image

类型：

sensor_msgs/msg/CompressedImage

### 检测结果

/model_inference_data

类型

ai_msgs::msg::PerceptionTargets
---

## 输出

/display_info

类型：

std_msgs/msg/String

发布视觉模型返回的文字描述。

---


默认模型：

qwen2.5vl:3b

---
## 编译
```bash
colcon build --packages-select ollama_image_understanding
source install/setup.bash
```

## 工作流程

```
订阅/model_inference_data
|
v
判断是否为人形立牌
|
v
采集图像
|
v
Ollama视觉理解
|
v
发布结果
```