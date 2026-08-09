#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():

    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params','lidar_uart_ros2', 'lsn10.yaml')
                     
    driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		#设置激光数据topic名称
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver_dir],
                                )
    base_link_to_laser_tf_node = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='base_link_to_base_laser_ld19',
                                arguments=['0','0','0.18','0','0','0','base_link','base_laser']
                                )

    return LaunchDescription([
        driver_node,
        base_link_to_laser_tf_node
    ])

