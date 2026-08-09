import os
import launch
import launch_ros
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    origincar_nav_dir = get_package_share_directory('origincar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    laser_pkg_share = get_package_share_directory('lslidar_driver')

    #  use_sim_time 设置为 false
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    rviz_config = launch.substitutions.LaunchConfiguration(
        'rviz', default=os.path.join(origincar_nav_dir, 'rviz', 'npc.rviz'))

    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(origincar_nav_dir, 'map', 'map_1784359523.yaml'))

    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(origincar_nav_dir, 'config', 'nav2_full_params.yaml'))

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        launch.actions.DeclareLaunchArgument(
            'map',
            default_value=map_yaml_path,
            description='Full path to map file'
        ),

        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_path,
            description='Full path to param file'
        ),

        
        # 启动雷达
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [laser_pkg_share, '/launch', '/lsn10_launch.py']
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 启动nav2核心

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']
            ),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'log_level' : 'error',
            }.items()
        ),

        # RVIZ 不使用仿真时间
        # launch_ros.actions.Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'
        # ),
    ])