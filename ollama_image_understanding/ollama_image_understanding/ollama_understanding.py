#!/usr/bin/env python3
import time
import base64
import cv2
import requests

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

from ai_msgs.msg import PerceptionTargets


class ImageUnderstandingNode(Node):

    def __init__(self):
        super().__init__('image_understanding_node')
        self.bridge = CvBridge()

        self.understanding_running = False
        self.last_finish_time = 0
        self.cool_down = 3.0  # API 处理完成后，强制冷却 3 秒

        # 状态控制：记录当前目标是否已经触发过识别
        self.has_triggered_for_current_target = False

        # ------------------ 配置参数 ------------------
        self.ollama_url = "http://192.168.100.173:11434/api/chat"
        self.model_name = "qwen2.5vl:3b"

        self.latest_image_msg = None
        # ----------------------------------------------

        # 1. 订阅图像数据
        self.create_subscription(
            CompressedImage, '/image', self.image_callback, 10
        )

        # 2. 订阅视觉模型推理数据话题
        self.create_subscription(
            PerceptionTargets, '/model_inference_data', self.inference_callback, 10
        )

        # 3. 发布识别结果到 /display_info
        self.publisher_ = self.create_publisher(String, '/display_info', 10)

        self.get_logger().info("✅ 图像理解节点已启动，正在监听 /model_inference_data 与 /image...")

    def image_callback(self, msg):
        """实时缓存最新帧图像"""
        self.latest_image_msg = msg

    def inference_callback(self, msg):
        """推理数据回调"""
        has_class_3 = False
        for target in msg.targets:
            if target.type == 'Class_3':
                has_class_3 = True
                break

        # 1. 如果视野里没有 Class_3，重置触发状态（等待下一次出现）
        if not has_class_3:
            if self.has_triggered_for_current_target:
                self.get_logger().info("ℹ️ Class_3 目标离开视野，重置触发状态")
                self.has_triggered_for_current_target = False
            return

        # 2. 如果检测到 Class_3，判断是否满足触发条件：
        #    - 当前目标尚未触发过
        #    - 当前没有正在运行的请求
        #    - 距离上次请求完成已超过冷却时间 cool_down
        current_time = time.time()
        if (not self.has_triggered_for_current_target and 
            not self.understanding_running and 
            (current_time - self.last_finish_time >= self.cool_down)):

            self.get_logger().info("🎯 首次/重新检测到 Class_3 目标，触发图像理解任务！")
            self.has_triggered_for_current_target = True  # 标记为已触发，避免同个目标重复调用
            self.process_and_send_image()

    def process_and_send_image(self):
        """处理图像并发送 HTTP 请求"""
        if self.latest_image_msg is None:
            self.get_logger().warn("⚠️ 满足触发条件，但 /image 话题尚未接收到图像数据！")
            return

        self.understanding_running = True

        try:
            # 1. 图像转换与编码
            cv_image = self.bridge.compressed_imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (320, 240))

            success, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if not success:
                raise RuntimeError("图像编码失败")

            image_base64 = base64.b64encode(buffer.tobytes()).decode('utf-8')

            # 2. 构建 API 请求 payload
            payload = {
                "model": self.model_name,
                "stream": False,
                "messages": [{
                    "role": "user",
                    "content": "描述一下这张图片里的主要内容,50字以内,返回纯文本内容,不要有多余的文字",
                    "images": [image_base64]
                }]
            }

            self.get_logger().info("🚀 正在请求 Ollama 模型进行图像理解...")

            # 3. 发送网络请求
            res = requests.post(self.ollama_url, json=payload, timeout=30)

            if res.status_code != 200:
                self.get_logger().error(f"❌ Ollama 返回异常状态码 {res.status_code}: {res.text}")
                result_str = f"错误: Ollama 返回状态码 {res.status_code}"
            else:
                response_data = res.json()
                result_str = ""
                if "message" in response_data and "content" in response_data["message"]:
                    result_str = response_data["message"]["content"]
                elif "response" in response_data:
                    result_str = response_data["response"]

                result_str = result_str.strip() if result_str else "模型未输出描述内容"

            # 4. 发布识别文本到 /display_info
            msg = String()
            msg.data = result_str
            self.publisher_.publish(msg)
            
            self.get_logger().info("📤 图像理解结果已发布至 /display_info")
            self.get_logger().info(f"🖼️ 图像描述: {result_str}")

        except requests.exceptions.Timeout:
            self.get_logger().error("❌ 连接 Ollama 超时（30秒）")
        except Exception as e:
            self.get_logger().error(f"❌ 图像处理/网络调用失败: {e}")
        finally:
            self.understanding_running = False
            self.last_finish_time = time.time()  # 重点：在请求完全结束后才开始计算冷却时间


def main(args=None):
    rclpy.init(args=args)
    node = ImageUnderstandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 节点接收中断信号退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()