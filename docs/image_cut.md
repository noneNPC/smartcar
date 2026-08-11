# Image Cut

基于 ROS2 + Horizon BPU 的 YOLOv11 目标检测与图像裁剪节点。

利用地平线 BPU 硬件加速完成实时目标检测，并根据检测结果裁剪目标区域，为二维码识别、图像理解等视觉任务提供输入。


---

## 功能

- YOLOv11 BPU硬件推理
- YOLOv11输出解析与NMS后处理
- NV12图像输入
- ROI区域快速裁剪
- 检测结果发布
- 目标图像发布


---

## 技术特点

### BPU推理加速

基于：


hobot dnn_node


调用BPU进行YOLOv11推理，降低CPU占用。


### NV12零拷贝处理

直接使用：


hbm_img_msgs/msg/HbmMsg1080P


获取NV12图像数据，减少图像复制。


### 实时性优化

- 推理任务数量限制
- 自动丢弃堆积帧
- ROI区域单独转换


相比整图：

NV12 -> BGR -> Crop

优化为：

NV12 -> Crop -> BGR

降低视觉处理延迟。

---

## 输入


### 摄像头图像

Topic:


/hbmem_img

类型:

hbm_img_msgs/msg/HbmMsg1080P

格式:

NV12

---

## 输出


### 检测结果

Topic:

/model_inference_data

类型:

ai_msgs/msg/PerceptionTargets


包含：

- 目标类别
- 置信度
- ROI坐标


---

### 裁剪图像

Topic:


/yolo_cropped_image


类型:

sensor_msgs/msg/Image

格式:

bgr8


---

## 模型


模型文件：

model/yolo11m.bin

支持：

- YOLOv11
- 4类目标检测

---

## 编译


```bash
cd ~/npc_ws

colcon build --packages-select image_cut

source install/setup.bash
```

选择：
```
/yolo_cropped_image
```
数据流程
```
Camera

  |

  v

/hbmem_img

  |

  v

YOLOv11 BPU

  |

  +------------+

  |            |

  v            v

Detection    Crop Image

Result          |

                v

       QR Recognition /
       Image Understanding

```
