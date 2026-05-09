#!/usr/bin/env python3
"""
Flight launch configuration for SAE 2026 — Mission 2.
Launches the drone node, FSM node, and supporting services.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_mission_2 = get_package_share_directory('mission_2')
    flight_params = os.path.join(pkg_mission_2, "config", "flight.yaml")

    # Declare the mission executable argument
    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="mission_2",
        description="Executable that implements the mission FSM")

    # System health monitor (from drone_lib)
    system_health_node = Node(
        
        package='drone_lib',
        executable='system_health',
        parameters=[flight_params],
        output='screen'
    )

    # Mission FSM node (delayed start to let other nodes initialize)
    fsm_node = Node(
        package='mission_2',
        executable=LaunchConfiguration("mission"),
        parameters=[flight_params],
        output='screen'
    )

    # Ball Detector (Computer Vision Node)
    ball_detector_node = Node(
        package='ball_detector',
        executable='ball_detector_node',
        output='screen'
    )

    # Hose (Mangueira) Detector
    mangueira_detector_config = os.path.join(pkg_mission_2, "config", "mangueira_detector.yaml")
    mangueira_detector_node = Node(
        package='mangueira_detector',
        executable='mangueira_detector_node',
        parameters=[mangueira_detector_config],
        output='screen'
    )

    vertical_camera_node = Node(
        package='camera_publisher',
        executable='webcam',
        output='screen',
        parameters=[{'camera_name': 'vertical', 'use_compressed': True}]
    )

    horizontal_camera_node = Node(
        package='camera_publisher',
        executable='webcam',
        output='screen',
        parameters=[{'camera_name': 'horizontal', 'use_compressed': True}]
    )

    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])

    return LaunchDescription([
        exec_arg,
        system_health_node,
        ball_detector_node,
        mangueira_detector_node,
        vertical_camera_node,
        horizontal_camera_node,
        delayed_fsm_node
    ])
