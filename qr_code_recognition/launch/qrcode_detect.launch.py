import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 复制 origincar 依赖的配置文件（保留原逻辑）
    os.system("cp -r /userdata/dev_ws/src/origincar/origincar_bringup/config .")

    launch_args = [
        DeclareLaunchArgument(
            "device",
            default_value=TextSubstitution(text="/dev/video0"),
            description="USB Camera Device"
        ),
    ]

    # 1. 启动 USB 相机（必须保留：负责提供 /image 图像数据）
    usb_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_usb_cam"),
                "launch",
                "hobot_usb_cam.launch.py"
            )
        ),
        launch_arguments={
            "usb_image_width": "640",
            "usb_image_height": "480",
            "usb_video_device": LaunchConfiguration("device"),
            "log_level": "error",
        }.items(),
    )

    # 2. 启动二维码识别节点（必须保留：订阅 /image，发布 /sign_switch 和 /display_info）
    qr_code_recognition_node = Node(
        package="qr_code_recognition",
        executable="qr_code_recognition_node",
        name="qr_code_recognition_node",
        output="screen",
    )

    return LaunchDescription(
        launch_args + [
            usb_node,
            qr_code_recognition_node,
        ]
    )