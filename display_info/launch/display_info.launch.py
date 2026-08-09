from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='display_info',
             executable='display_info',
             name='display_info',
             output='screen'),
    ])
