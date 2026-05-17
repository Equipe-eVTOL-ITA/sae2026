#!/usr/bin/env python3
"""
FLight launch configuration for SAE 2026 — Mission 3.
Launches the drone node, FSM node, and supporting services.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


RVIZ_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', 'sae2026', 'rviz', 'trajectory.rviz'
)

def generate_launch_description():

    pkg_mission_3 = get_package_share_directory('mission_3')
    flight_params = os.path.join(pkg_mission_3, "config", "flight.yaml")

    pkg_manometro = get_package_share_directory('manometro_detector')
    manometro_params = os.path.join(pkg_manometro, "config", "manometro_detector.yaml")

    # Declare the mission executable argument
    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="mission_3",
        description="Executable that implements the mission FSM")

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 for trajectory visualization")

    camera_publisher = Node(
        package='camera_publisher',
        executable='webcam',
        parameters=[{
            'video_source': '/dev/video2',
            'camera_name': 'vertical',   # publishes to /vertical_camera/compressed
        }],
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
        package='mission_3',
        executable=LaunchConfiguration("mission"),
        parameters=[flight_params],
        output='screen'
    )


    vision_node = Node(
        package='manometro_detector',
        executable='manometro_detector',
        parameters=[manometro_params],  # detector-specific YAML (image_topic, debug, calibration)
        output='screen'
    )

    
    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', RVIZ_CONFIG],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output='screen'
    )

    return LaunchDescription([
        exec_arg,
        rviz_arg,
        camera_publisher,
        system_health_node,
        vision_node,
        delayed_fsm_node,
        rviz_node,
    ])