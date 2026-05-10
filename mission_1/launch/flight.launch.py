#!/usr/bin/env python3
"""
Real flight launch configuration for SAE 2026 — Mission 1.
Launches the drone node, FSM node, and supporting services.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_mission_1 = get_package_share_directory('mission_1')
    flight_params = os.path.join(pkg_mission_1, "config", "flight.yaml")

    # Declare the mission executable argument
    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="mission_1",
        description="Executable that implements the mission FSM")

    webcam_publisher_node = Node(
        package='camera_publisher',
        executable='webcam',
        output='screen'
    )
    # System health monitor (from drone_lib)
    system_health_node = Node(
        
        package='drone_lib',
        executable='system_health',
        parameters=[flight_params],
        output='screen'
    )

    # Mission FSM node (delayed start to let other nodes initialize)
    fsm_node = Node(
        package='mission_1',
        executable=LaunchConfiguration("mission"),
        parameters=[flight_params],
        output='screen'
    )

    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])

    # Vision node

    bouncing_cv_node = Node(
        package='bouncing_detector',
        executable='bouncing_detector_node',
        output='screen'
    )

    # Webcam publisher node

    return LaunchDescription([
        exec_arg,
        system_health_node,
        bouncing_cv_node,
        webcam_publisher_node,
        delayed_fsm_node
    ])
