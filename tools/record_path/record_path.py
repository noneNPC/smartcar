#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import json
import math
import os
from geometry_msgs.msg import TransformStamped

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        # ==========================================
        # 路径与保存配置（更密集采点配置）
        # ==========================================
        self.save_filename = '2.json'  # 导出文件名
        
        # 1. 降低步长阈值：从 5cm (0.05m) 缩短到 1.5cm (0.015m)，获取更密集的路径数据
        self.dist_interval = 0.005     
        
        self.target_dir = './'
        if not os.path.exists(self.target_dir):
            os.makedirs(self.target_dir)
        self.full_save_path = os.path.join(self.target_dir, self.save_filename)

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_poses = []
        self.last_x = None
        self.last_y = None

        # 2. 提高检查频率：从 20Hz (0.05s) 提升到 50Hz (0.02s)，防止车速稍快时漏掉密集点
        self.timer = self.create_timer(0.02, self.record_callback)
        
        self.get_logger().info("==> [High Precision Mode] Path Recorder Initialized!")
        self.get_logger().info(f"==> Sampling Distance Interval: {self.dist_interval * 100:.1f} cm")
        self.get_logger().info(f"==> Export Target: {self.full_save_path}")
        self.get_logger().info("==> Drive smoothly. Press Ctrl+C to complete recording.")

    def record_callback(self):
        try:
            # 获取 map 坐标系下机器人当前 base_link 的位置和姿态
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            
            x = tf.transform.translation.x
            y = tf.transform.translation.y

            # 录制第一个点
            if self.last_x is None or self.last_y is None:
                self.save_point(tf)
                return

            # 计算欧氏距离
            distance = math.sqrt((x - self.last_x) ** 2 + (y - self.last_y) ** 2)

            # 达到设定的步长则录制当前点
            if distance >= self.dist_interval:
                self.save_point(tf)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def save_point(self, tf: TransformStamped):
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        
        self.last_x = x
        self.last_y = y

        # ==========================================================
        # 核心：严格对齐 C++ FilePathPublisher::load_path 提取的 Key 映射
        # ==========================================================
        pose_data = {
            "x": float(x),                               # 对齐 C++ 的 p.value("x", 0.0)
            "y": float(y),                               # 对齐 C++ 的 p.value("y", 0.0)
            "z": float(tf.transform.translation.z),      # 补全保留项
            "w": float(tf.transform.rotation.w),         # 对齐 C++ 的 p.value("w", 1.0)
            "x_ori": float(tf.transform.rotation.x),     # 对齐 C++ 的 p.value("x_ori", 0.0)
            "y_ori": float(tf.transform.rotation.y),     # 对齐 C++ 的 p.value("y_ori", 0.0)
            "z_ori": float(tf.transform.rotation.z)      # 对齐 C++ 的 p.value("z_ori", 0.0)
        }

        self.path_poses.append(pose_data)
        
        # 频率较高时，每录制 10 个点打印一次日志，避免刷屏
        if len(self.path_poses) % 10 == 0:
            self.get_logger().info(f"Recorded point #{len(self.path_poses)} -> x: {x:.3f}, y: {y:.3f}")

        # 实时写入文件，防止异常强退丢失数据
        try:
            with open(self.full_save_path, 'w') as f:
                json.dump(self.path_poses, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to write file: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Recording stopped. Total generated points: {len(node.path_poses)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()