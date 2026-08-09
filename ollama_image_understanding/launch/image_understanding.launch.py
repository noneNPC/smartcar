from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ollama_image_understanding',
             executable='ollama_understanding',
             name='ollama_understanding',
             output='screen'),
    ])
