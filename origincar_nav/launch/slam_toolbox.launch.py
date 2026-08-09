import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 声明与获取 LaunchConfiguration 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # 包路径获取
    nav_pkg_share = get_package_share_directory('origincar_nav')
    laser_pkg_share = get_package_share_directory('lslidar_driver')

    # 2. 声明 Launch 参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            nav_pkg_share, 'config', 'mapper_params_online_async.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # 4. 定义包含的雷达驱动 Launch 文件
    launch_laser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_pkg_share, 'launch', 'lsn10_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. 定义 SLAM Toolbox 节点
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 延时 2 秒启动 SLAM 节点，等待雷达和 TF 稳定
    delayed_slam = TimerAction(
        period=2.0,
        actions=[start_async_slam_toolbox_node]
    )

    # 6. 构建并返回 LaunchDescription 容器
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(launch_laser)
    ld.add_action(delayed_slam)

    return ld