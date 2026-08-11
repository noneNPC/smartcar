from ament_index_python.packages import get_package_share_directory
import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # 获取各个包的路径
    shm_launch_file = os.path.join(get_package_share_directory('hobot_shm'), 'launch', 'hobot_shm.launch.py')
    usb_cam_launch_file = os.path.join(get_package_share_directory('hobot_usb_cam'), 'launch', 'hobot_usb_cam.launch.py')
    codec_launch_file = os.path.join(get_package_share_directory('hobot_codec'), 'launch', 'hobot_codec_decode.launch.py')
    websocket_launch_file = os.path.join(get_package_share_directory('websocket'), 'launch', 'websocket.launch.py')

    launch_elements = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(shm_launch_file)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(usb_cam_launch_file),
            launch_arguments={
                'usb_image_width': '640',
                'usb_image_height': '480',
                'usb_video_device': '/dev/video0'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(codec_launch_file),
            launch_arguments={
                'codec_in_mode': 'ros',
                'codec_out_mode': 'shared_mem',
                'codec_sub_topic': '/image',
                'codec_pub_topic': '/hbmem_img'
            }.items()
        ),

        launch_ros.actions.Node(
            package='yolov11_detector',
            executable='yolov11_detector_node',
            name='yolov11_detector_node',
            output='log',
            arguments=['--ros-args', '--log-level', 'fatal'],
#            parameters=[{'log_level': LaunchConfiguration('log_level')}]
        ),
    ]

    # 只有在 websocket_enable 为 true 时才启动 WebSocket 显示节点
    if LaunchConfiguration('websocket_enable').perform(context) == 'true':
        launch_elements.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(websocket_launch_file),
                launch_arguments={
                    'websocket_image_topic': '/image',
                    'websocket_image_type': 'mjpeg',
                    'websocket_smart_topic': 'model_inference_data'
                }.items()
            )
        )

    return launch_elements

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='warning', description='Logging level'),
        DeclareLaunchArgument('websocket_enable', default_value='false', description='Enable WebSocket display node (true/false)'),
        OpaqueFunction(function=launch_setup)
    ])
