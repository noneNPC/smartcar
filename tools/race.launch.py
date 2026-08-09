#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    SetLaunchConfiguration
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # =========================
    # 1. 底盘
    # =========================

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('origincar_base'),
                'launch',
                'origincar_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'log_level': 'error'
        }.items()
    )


    base_started = LogInfo(
        msg='底盘节点启动'
    )


    # =========================
    # 2. 导航
    # =========================

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('origincar_nav'),
                'launch',
                'pure_nav.launch.py'
            ])
        ),
        launch_arguments={
            'log_level': 'error'
        }.items()
    )


    nav_started = LogInfo(
        msg='导航节点启动'
    )


    # =========================
    # 3. 路径发布
    # =========================

    plan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pub_plan'),
                'launch',
                'pub_plan.launch.py'
            ])
        ),
        launch_arguments={
            'log_level': 'error'
        }.items()
    )


    plan_started = LogInfo(
        msg='路径发布节点启动'
    )


    # =========================
    # 4. 二维码
    # =========================

    qrcode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('qr_code_recognition'),
                'launch',
                'qrcode_detect.launch.py'
            ])
        ),
        launch_arguments={
            'log_level': 'error'
        }.items()
    )

    qrcode_started = LogInfo(
        msg='二维码检测节点启动'
    )


    # =========================
    # 5. 大模型
    # =========================


    # ollama_understanding_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('ollama_image_understanding'),
    #             'launch',
    #             'image_understanding.launch.py'
    #         ])
    #     ),
    # )
    
    # ollama_started = LogInfo(
    #     msg='大模型节点启动'
    # )

    # =========================
    # 6. 显示信息
    # =========================


    display_info_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('display_info'),
                'launch',
                'display_info.launch.py'
            ])
        ),
    )
    
    display_info_started = LogInfo(
        msg='显示信息节点启动'
    )


    return LaunchDescription([


        # base
        base_launch,

        TimerAction(
            period=3.0,
            actions=[
                base_started
            ]
        ),


        # nav
        TimerAction(
            period=5.0,
            actions=[
                nav_launch,
                nav_started
            ]
        ),


        # plan
        TimerAction(
            period=8.0,
            actions=[
                plan_launch,
                plan_started
            ]
        ),


        # qrcode
        TimerAction(
            period=10.0,
            actions=[
                qrcode_launch,
                qrcode_started
            ]
        ),
        # ollama
        # TimerAction(
        #     period=12.0,
        #     actions=[
        #         ollama_understanding_launch,
        #         ollama_started
        #     ]
        # ),
        # display
        TimerAction(
            period=12.0,
            actions=[
                display_info_launch,
                display_info_started
            ]
        ),

    ])
