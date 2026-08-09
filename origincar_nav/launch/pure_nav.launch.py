#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_share = get_package_share_directory("origincar_nav")
    laser_pkg_share = get_package_share_directory('lslidar_driver')

    params_file = os.path.join(
        pkg_share,
        "config",
        "nav2_params.yaml"
    )
    slamtoolbox_params = os.path.join(
        pkg_share,
        "config",
        "mapper_params_localization.yaml"
    )

    map_file = os.path.join(
        pkg_share,
        "map",
        "race_map.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                "yaml_filename": map_file,
                "use_sim_time": use_sim_time
            },
            convert_types=True
        ),
        allow_substs=True
    )

    lifecycle_nodes = [
        "map_server",
        "controller_server"
    ]
    
    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time"
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup lifecycle nodes"
        ),

        #
        # Map Server
        #
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'error'],
        ),

        #
        # slamtoolbox
        #
        Node(
            package="slam_toolbox",
            executable="localization_slam_toolbox_node",
            name="slam_toolbox",
            output="log",
            parameters=[
                slamtoolbox_params,
                {
                    "use_sim_time": use_sim_time
                }
            ],
            arguments=['--ros-args', '--log-level', 'error'],
        ),
        #
        # Controller Server (MPPI)
        #
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="log",
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'error'],
        ),

        #
        # Lifecycle Manager
        #
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": lifecycle_nodes,
                }
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    laser_pkg_share,
                    "launch",
                    "lsn10_launch.py"
                )
            ),
            launch_arguments={
                "use_sim_time": use_sim_time
            }.items()
        ),


    ])