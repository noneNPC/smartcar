#!/usr/bin/env python3

import time
import textwrap
import os
import spidev
import numpy as np
import Hobot.GPIO as GPIO
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# --- 物理引脚定义 (BOARD 编码) ---
DC_PIN  = 18   # Physical Pin 18 (GPIO24)
RST_PIN = 22   # Physical Pin 22 (GPIO25)
CS_PIN  = 24   # Physical Pin 24 (SPI1_CSN)

class ST7796SDisplayNode(Node):
    def __init__(self):
        super().__init__('st7796s_display_node')

        # 1. GPIO 初始化与片选接管
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DC_PIN, GPIO.OUT)
        GPIO.setup(RST_PIN, GPIO.OUT)
        GPIO.setup(CS_PIN, GPIO.OUT)
        GPIO.output(CS_PIN, GPIO.HIGH)

        # 2. SPI 高速初始化 (提升至 32MHz，传输速率翻 4 倍)
        self.spi = spidev.SpiDev()
        self.spi.open(1, 0)
        self.spi.max_speed_hz = 32000000  # 32 MHz
        self.spi.mode = 0

        # 3. 硬件唤醒与横屏初始化
        self.init_st7796s()

        # 4. 寻找中文字体文件
        font_paths = [
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",       # 文泉驿微米黑
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", # Noto CJK
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"   # 兜底英文字体
        ]

        self.font_path = None
        for path in font_paths:
            if os.path.exists(path):
                self.font_path = path
                break

        if self.font_path:
            self.get_logger().info(f"成功锁定字体: {self.font_path}")
            self.title_font = ImageFont.truetype(self.font_path, 20)
        else:
            self.get_logger().warn("未找到矢量字体文件，使用默认字体")
            self.title_font = ImageFont.load_default()

        # 用于避免重复刷新的缓存变量
        self.last_rendered_text = ""

        # 5. 创建 ROS 2 订阅者
        self.subscription = self.create_subscription(
            String,
            '/display_info',
            self.listener_callback,
            10
        )
        self.get_logger().info("屏幕显示启动")

        # 初始界面
        self.render_and_display("OK")
        time.sleep(2)
        self.render_and_display("")

    # --- 底层 SPI 传输 ---
    def write_cmd(self, c):
        GPIO.output(DC_PIN, GPIO.LOW)
        GPIO.output(CS_PIN, GPIO.LOW)
        self.spi.xfer2([c])
        GPIO.output(CS_PIN, GPIO.HIGH)

    def write_data(self, d):
            GPIO.output(DC_PIN, GPIO.HIGH)
            GPIO.output(CS_PIN, GPIO.LOW)
            if isinstance(d, int):
                self.spi.xfer2([d])
            else:
                # 严格限制每次 xfer2 最多传输 4096 字节，防止 spidev 溢出
                # 同时使用 list(d[i:i+4096]) 确保兼容性
                for i in range(0, len(d), 4096):
                    self.spi.xfer2(list(d[i:i+4096]))
            GPIO.output(CS_PIN, GPIO.HIGH)
    def init_st7796s(self):
        """ST7796S 官方标准初始化 (480x320 横屏)"""
        GPIO.output(RST_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(RST_PIN, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(RST_PIN, GPIO.HIGH)
        time.sleep(0.12)

        self.write_cmd(0x01); time.sleep(0.12)
        self.write_cmd(0x11); time.sleep(0.12)

        self.write_cmd(0xF0); self.write_data(0xC3)
        self.write_cmd(0xF0); self.write_data(0x96)

        self.write_cmd(0x36); self.write_data(0x28) # 横屏方向

        self.write_cmd(0x3A); self.write_data(0x55)
        self.write_cmd(0xB4); self.write_data(0x01)
        self.write_cmd(0xB6); self.write_data([0x80, 0x02, 0x3B])
        self.write_cmd(0xC1); self.write_data(0x06)
        self.write_cmd(0xC2); self.write_data(0xA7)
        self.write_cmd(0xC5); self.write_data(0x18)

        self.write_cmd(0xE0); self.write_data([0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B])
        self.write_cmd(0xE1); self.write_data([0xF0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2D, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B])

        self.write_cmd(0x29)
        time.sleep(0.05)

    def listener_callback(self, msg: String):
        text_content = msg.data
        # 内容无变化时跳过刷新，防止浪费系统资源
        if text_content == self.last_rendered_text:
            return
        
        self.last_rendered_text = text_content
        self.render_and_display(text_content)

    def calculate_auto_font(self, text: str, max_w: int, max_h: int):
            if not self.font_path:
                return ImageFont.load_default(), [text], 12

            # 尝试从大字号 (120px) 递减尝试，寻找最能填满空间的大小
            for font_size in range(120, 13, -2):
                font = ImageFont.truetype(self.font_path, font_size)
                
                wrapped_lines = []
                # 遍历段落，逐字测量像素宽度，完美贴合右边界
                for raw_line in text.split('\n'):
                    if not raw_line:
                        wrapped_lines.append('')
                        continue
                    
                    current_line = ""
                    for char in raw_line:
                        test_line = current_line + char
                        # 精确获取加上当前字符后的实际像素宽度
                        bbox = font.getbbox(test_line)
                        line_w = bbox[2] - bbox[0]
                        
                        if line_w <= max_w:
                            current_line = test_line
                        else:
                            # 超过边界，这一行结算，开启下一行
                            wrapped_lines.append(current_line)
                            current_line = char
                    
                    if current_line:
                        wrapped_lines.append(current_line)

                # 计算这套排版的总高度
                line_height = int(font_size * 1.25)
                total_height = len(wrapped_lines) * line_height

                # 如果垂直高度也装得下，说明找到了最佳字号！
                if total_height <= max_h:
                    return font, wrapped_lines, line_height

            # 降级兜底字号
            fallback_font = ImageFont.truetype(self.font_path, 14)
            return fallback_font, [text], 18

    def render_and_display(self, text: str):
        img = Image.new("RGB", (480, 320), (0, 0, 0))
        draw = ImageDraw.Draw(img)

        # 1. 绘制 UI 框架
        draw.rectangle([(5, 5), (474, 314)], outline=(0, 255, 0), width=3)
        draw.text((15, 12), "Origincar Display", fill=(0, 255, 0), font=self.title_font)
        draw.line([(10, 40), (470, 40)], fill=(100, 100, 100), width=2)

        # 2. 计算最佳适应字号和折行
        max_width = 445
        max_height = 255
        font, lines, line_height = self.calculate_auto_font(text, max_width, max_height)

        # 3. 垂直居中
        total_text_h = len(lines) * line_height
        y_start = 50 + (max_height - total_text_h) // 2

        # 4. 绘制文本
        y_offset = max(50, y_start)
        for line in lines:
            draw.text((18, y_offset), line, fill=(255, 255, 255), font=font)
            y_offset += line_height

        # 5. 设置 ST7796S 显存写窗口
        self.write_cmd(0x2A); self.write_data([0x00, 0x00, 0x01, 0xDF]) # 0~479
        self.write_cmd(0x2B); self.write_data([0x00, 0x00, 0x01, 0x3F]) # 0~319
        self.write_cmd(0x2C)

        # 🚀 6. 极速 RGB888 -> RGB565 转换（NumPy 矩阵并行化处理）
        img_np = np.array(img, dtype=np.uint16)
        r = img_np[:, :, 0]
        g = img_np[:, :, 1]
        b = img_np[:, :, 2]

        # 位运算矩阵加速
        rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        
        # 转化为大端高低字节高效率 bytearray
        bytes_high = (rgb565 >> 8).astype(np.uint8)
        bytes_low = (rgb565 & 0xFF).astype(np.uint8)
        
        raw_bytes = bytearray(np.stack((bytes_high, bytes_low), axis=-1).tobytes())

        # 7. 一次性推送字节数据
        self.write_data(raw_bytes)

    def destroy_node(self):
        self.spi.close()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ST7796SDisplayNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # 捕获 Ctrl+C 及 ROS2 的外部关闭信号，静默忽略
        pass
    except Exception as e:
        # 其他未预期的严重错误依然打印 Log，方便排查问题
        print(f"\n[INFO] 节点异常退出: {e}")
    finally:
        # 确保节点优雅清理并关闭
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
