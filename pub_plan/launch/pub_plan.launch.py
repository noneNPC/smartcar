from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory("pub_plan")

    path_file = os.path.join(pkg_share, "path", "path.json")
    config_file = os.path.join(pkg_share, "config", "config.yaml")

    return LaunchDescription([
        Node(
            package="pub_plan",
            executable="pub_plan",
            name="path_publisher",
            output="screen",
            parameters=[
                config_file,
                {
                    "global_path_file": path_file
                }
            ]
        )
    ])