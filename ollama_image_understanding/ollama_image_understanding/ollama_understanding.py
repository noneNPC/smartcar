#!/usr/bin/env python3
import math
import time
import base64
import cv2
import requests

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from origincar_msg.msg import Sign
from cv_bridge import CvBridge

import tf2_ros
from tf2_ros import TransformException


class ImageUnderstandingNode(Node):

    def __init__(self):
        super().__init__('image_understanding_node')
        self.bridge = CvBridge()

        self.understanding_running = False
        self.last_sent_time = 0
        self.send_interval = 2.0  # 最小上传间隔（秒）

        # ------------------ 配置参数 ------------------
        # 上位机 Ollama 接口地址
        self.ollama_url = "http://192.168.100.173:11434/api/chat"
        self.model_name = "qwen2.5vl:3b"

        # 标志位 3 对应的目标点 (x, y) 坐标 (顺时针)
        self.target_pt_3 = (3.98, 4.38)

        # 标志位 4 对应的目标点 (x, y) 坐标 (逆时针)
        self.target_pt_4 = (1.06, 4.38)

        # 触发距离阈值（单位：米）
        self.trigger_radius = 0.8

        # 状态变量
        self.current_sign_data = None
        self.latest_image_msg = None
        self.has_triggered_in_zone = False
        self.is_task_completed = False  # 标志：当前任务已完成（离开区域后停止检测）
        self.log_counter = 0
        # ----------------------------------------------

        # 初始化 TF2 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 1. 订阅图像数据
        self.create_subscription(
            CompressedImage, '/image', self.image_callback, 10
        )

        # 2. 实时订阅标志位 /sign_switch 话题
        self.create_subscription(
            Sign, '/sign_switch', self.sign_callback, 10
        )

        # 3. 发布识别结果到 /display_info
        self.publisher_ = self.create_publisher(String, '/display_info', 10)

        # 4. 创建 10Hz (0.1秒) 定时器，持续轮询小车位置
        self.timer = self.create_timer(0.1, self.check_location_timer)

        self.get_logger().info("✅ 图像理解节点已启动（单节点直连版），正在监听 Sign 与 TF...")

    def sign_callback(self, msg):
        """实时接收标志位数据"""
        if self.current_sign_data != msg.sign_data:
            self.get_logger().info(f"📩 收到新的 Sign 标志位: {msg.sign_data}")
            self.current_sign_data = msg.sign_data
            self.has_triggered_in_zone = False
            self.is_task_completed = False  # 重置状态，开始新一轮目标位置检测

    def image_callback(self, msg):
        """实时缓存最新帧图像"""
        self.latest_image_msg = msg

    def get_current_pose(self):
        """通过 TF 获取小车当前在 map 坐标系下的 (x, y) 位置"""
        for frame_id in ['base_footprint', 'base_link']:
            try:
                t = self.tf_buffer.lookup_transform(
                    'map',
                    frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                return (t.transform.translation.x, t.transform.translation.y)
            except TransformException:
                continue

        if self.log_counter % 20 == 0:
            self.get_logger().warn("⚠️ TF 查找失败！请确认是否有 map -> base_link 或 base_footprint 坐标变换。")
        return None

    def check_location_timer(self):
        """定时器回调：持续监控小车位置并匹配当前 Sign 要求"""
        # 如果当前标志位的识别拍照任务已完成并离场，直接跳过测算
        if self.is_task_completed:
            return

        self.log_counter += 1

        if self.current_sign_data is None:
            if self.log_counter % 30 == 0:
                self.get_logger().warn("⏳ 未接收到 /sign_switch 消息，等待中...")
            return

        target_pt = None
        if self.current_sign_data == 3:
            target_pt = self.target_pt_3
        elif self.current_sign_data == 4:
            target_pt = self.target_pt_4
        else:
            if self.log_counter % 30 == 0:
                self.get_logger().info(f"ℹ️ 当前 Sign={self.current_sign_data}，无需拍照。")
            return

        current_pose = self.get_current_pose()
        if current_pose is None:
            return

        x, y = current_pose
        distance = math.hypot(x - target_pt[0], y - target_pt[1])

        # 定期打印实时距离
        if self.log_counter % 10 == 0:
            self.get_logger().info(
                f"📍 [位置] x:{x:.2f}, y:{y:.2f} | 目标: {target_pt} | 距离: {distance:.2f}m | Sign: {self.current_sign_data}"
            )

        # 进入目标半径内 & 当前区域内还没拍过照
        if distance <= self.trigger_radius:
            if not self.has_triggered_in_zone:
                self.get_logger().info(
                    f"🎯 到达目标点附近 (Sign={self.current_sign_data}, 距离={distance:.2f}m)，触发拍照识别！"
                )
                self.has_triggered_in_zone = True
                self.process_and_send_image()

        # 驶离目标点区域后停止检测
        elif distance > (self.trigger_radius + 0.3):
            if self.has_triggered_in_zone:
                self.get_logger().info("🚗 小车已离开目标点区域，停止当前点的测量与监控")
                self.has_triggered_in_zone = False
                self.is_task_completed = True  # 彻底挂起测算，直到下一个 Sign 触发

    def process_and_send_image(self):
        """处理图像并直接发送 HTTP 请求给上位机 Ollama"""
        if self.latest_image_msg is None:
            self.get_logger().warn("⚠️ 满足拍照条件，但 /image 话题尚未接收到图像数据！")
            return

        current_time = time.time()
        if self.understanding_running or (current_time - self.last_sent_time < self.send_interval):
            self.get_logger().warn("⏳ 请求频率限制或上一次分析尚未结束，跳过本次发送")
            return

        self.understanding_running = True
        self.last_sent_time = current_time

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

            self.get_logger().info("🚀 正在直接请求上位机 Ollama 模型进行图像理解...")

            # 3. 直接发送网络请求
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