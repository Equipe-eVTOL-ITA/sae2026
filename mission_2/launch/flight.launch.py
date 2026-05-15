#!/usr/bin/env python3
"""
Flight launch configuration for SAE 2026 — Mission 2.
Launches the drone node, FSM node, and supporting services.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
import datetime
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_mission_2 = get_package_share_directory('mission_2')
    flight_params = os.path.join(pkg_mission_2, "config", "flight.yaml")

    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/mission_2_{timestamp}')
    bag_topics = [
        '/rosout',
        '/telemetry/logs',
        '/telemetry/drone_status',
        '/telemetry/position',
        '/telemetry/system_health',
        '/drone_trajectory',
        '/ball_detection',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_status',
        '/fmu/out/battery_status',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/vehicle_command',
    ]
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics,
        output='screen'
    )

    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="mission_2",
        description="Executable that implements the mission FSM")

    vertical_camera_node = Node(
        package='camera_publisher',
        executable='webcam',
        output='screen',
        parameters=[{'camera_name': 'vertical', 'use_compressed': True, 'video_source': '/dev/video0'}]
    )

    horizontal_camera_node = Node(
        package='camera_publisher',
        executable='webcam',
        output='screen',
        parameters=[{'camera_name': 'horizontal', 'use_compressed': True, 'video_source': '/dev/video2'}]
    )

    system_health_node = Node(
        package='drone_lib',
        executable='system_health',
        parameters=[flight_params],
        output='screen'
    )

    fsm_node = Node(
        package='mission_2',
        executable=LaunchConfiguration("mission"),
        parameters=[flight_params],
        output='screen'
    )

    ball_detector_node = Node(
        package='ball_detector',
        executable='ball_detector_node',
        output='screen'
    )

    mangueira_detector_config = os.path.join(pkg_mission_2, "config", "mangueira_detector.yaml")
    mangueira_detector_node = Node(
        package='mangueira_detector',
        executable='mangueira_detector_node',
        parameters=[mangueira_detector_config],
        output='screen'
    )

    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])

    return LaunchDescription([
        exec_arg,
        bag_record,
        vertical_camera_node,
        horizontal_camera_node,
        system_health_node,
        ball_detector_node,
        mangueira_detector_node,
        delayed_fsm_node,
    ])
