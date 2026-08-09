#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import keyboard
from threading import Thread
from time import sleep
import sys

class Ros2KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('ros2_keyboard_teleop')

        # 默认速度配置 (对应原脚本全局变量)
        self.linear = 0.25      # 线速度 (m/s)
        self.angular = 3.14     # 角速度 (rad/s)
        self.dert = 0.3         # 调速步长

        # 创建 ROS 2 原生 cmd_vel 发布者
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 初始化键盘映射
        self.init_keymap()

        # 启动键盘监听线程
        self.is_active = False
        self.init_keyboard()

        self.get_logger().info("==> ROS 2 Direct Teleop Node Initialized!")
        self.help_tip()

    def init_keymap(self):
        self.key_mapping = {
            'up':         {'linear': self.linear,  'angular': 0.0},
            'down':       {'linear': -self.linear, 'angular': 0.0},
            'left':       {'linear': 0.0,          'angular': self.angular},
            'right':      {'linear': 0.0,          'angular': -self.angular},
            'up_left':    {'linear': self.linear,  'angular': self.angular},
            'up_right':   {'linear': self.linear,  'angular': -self.angular},
            'down_left':  {'linear': -self.linear, 'angular': self.angular},
            'down_right': {'linear': -self.linear, 'angular': -self.angular},
            'stop':       {'linear': 0.0,          'angular': 0.0}
        }

    def update_vel(self):
        self.key_mapping['up']['linear']          =  self.linear
        self.key_mapping['down']['linear']        = -self.linear
        self.key_mapping['left']['angular']       =  self.angular
        self.key_mapping['right']['angular']      = -self.angular
        self.key_mapping['up_left']['linear']     =  self.linear
        self.key_mapping['up_left']['angular']    =  self.angular
        self.key_mapping['up_right']['linear']    =  self.linear
        self.key_mapping['up_right']['angular']   = -self.angular
        self.key_mapping['down_left']['linear']   = -self.linear
        self.key_mapping['down_left']['angular']  =  self.angular
        self.key_mapping['down_right']['linear']  = -self.linear
        self.key_mapping['down_right']['angular'] = -self.angular

    def publish_twist(self, cmd_data):
        twist = Twist()
        twist.linear.x = float(cmd_data['linear'])
        twist.angular.z = float(cmd_data['angular'])
        self.vel_pub.publish(twist)

    def init_keyboard(self):
        self.keyboard_thread = Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def keyboard_listener(self):
        print("\n------------------------------------")
        print("**** 按下   [ r ]   开始控制小车 ****")
        print("------------------------------------")
        
        # 等待激活
        while rclpy.ok():
            sleep(0.05)
            if keyboard.is_pressed('r'):
                self.is_active = True
                print("\n正在继续键盘监听.")
                break

        # 核心监听循环
        while rclpy.ok():
            sleep(0.05)

            # 退出/暂停激活键盘控制
            if keyboard.is_pressed('p'):
                self.is_active = False
                self.publish_twist(self.key_mapping['stop']) # 暂停时先停车
                print("\n已退出键盘监听.")
                print("按下 [ r ] 继续键盘监听.")
                while rclpy.ok():
                    if keyboard.is_pressed('r'):
                        self.is_active = True
                        print("\n正在继续键盘监听.")
                        break
                    sleep(0.05)

            if not self.is_active:
                continue

            # ---------------- WASD 方向与组合键控制 ----------------
            if keyboard.is_pressed('w'):                                  # 前进
                if keyboard.is_pressed('a'):                              # 前进加左转
                    self.publish_twist(self.key_mapping['up_left'])
                elif keyboard.is_pressed('d'):                            # 前进加右转
                    self.publish_twist(self.key_mapping['up_right'])
                else:
                    self.publish_twist(self.key_mapping['up'])
            elif keyboard.is_pressed('s'):                                # 后退
                if keyboard.is_pressed('a'):                              # 后退加左转（保留原脚本逻辑映射）
                    self.publish_twist(self.key_mapping['down_right'])
                elif keyboard.is_pressed('d'):                            # 后退加右转
                    self.publish_twist(self.key_mapping['down_left'])
                else:
                    self.publish_twist(self.key_mapping['down'])
            elif keyboard.is_pressed('a'):                                # 仅左转
                self.publish_twist(self.key_mapping['left'])
            elif keyboard.is_pressed('d'):                                # 仅右转
                self.publish_twist(self.key_mapping['right'])

            # ---------------- 方向键调整速度/转角 ----------------
            elif keyboard.is_pressed("up"):
                self.linear += self.dert
                print(f"线速度设置为: {self.linear:.2f} m/s.")
                self.update_vel()
                sleep(0.2)
            elif keyboard.is_pressed("down"):
                if self.linear - self.dert < 0:
                    print(f"速度设置失败！(线速度为: {self.linear:.2f}).")
                else:
                    self.linear -= self.dert
                    print(f"线速度设置为: {self.linear:.2f} m/s.")
                self.update_vel()
                sleep(0.2)
            elif keyboard.is_pressed("left"):
                if self.angular - self.dert < 0:
                    print(f"角度设置失败！(转角为: {self.angular:.2f} rad).")
                else:
                    self.angular -= self.dert
                    print(f"角度设置为: {self.angular:.2f} rad.")
                self.update_vel()
                sleep(0.2)
            elif keyboard.is_pressed("right"):
                self.angular += self.dert
                print(f"转角设置为: {self.angular:.2f} rad.")
                self.update_vel()
                sleep(0.2)

            # ---------------- 提示 ----------------
            elif keyboard.is_pressed('t'):
                self.help_tip()
                sleep(0.5)
            
            # 松开按键发送 stop 零速
            else:
                self.publish_twist(self.key_mapping['stop'])

    def help_tip(self):
        print("\n\n提示：")
        print("按   [ p ] 退出键盘控制.")
        print("按   [ r ] 回到键盘控制.")
        print("按   [ t ] 显示按键帮助.", end="\n\n")

        print("控制：")
        print(f"--- [ w ] --- 前进: {self.linear:.2f} m/s.")
        print(f"--- [ a ] --- 左转: {self.angular:.2f} rad.")
        print(f"--- [ d ] --- 右转: {-self.angular:.2f} rad.")
        print(f"--- [ s ] --- 后退: {-self.linear:.2f} m/s.")

        print("调整速度(使用键盘的方向键):")
        print("---  [   up  ]  --- 增加线速度.")
        print("---  [  left ]  --- 减小转角.")
        print("---  [ right ]  --- 增加转角.")
        print("---  [  down ]  --- 减小线速度.\n")

def main(args=None):
    rclpy.init(args=args)
    node = Ros2KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发零速确保安全停车
        stop_twist = Twist()
        node.vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()