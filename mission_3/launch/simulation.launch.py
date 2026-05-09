#!/usr/bin/env python3
"""
Simulation launch configuration for SAE 2026 — Mission 3.
Launches the drone node, FSM node, and supporting services.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_mission_3 = get_package_share_directory('mission_3')
    simulation_params = os.path.join(pkg_mission_3, "config", "simulation.yaml")

    # Declare the mission executable argument
    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="mission_3",
        description="Executable that implements the mission FSM")

    # System health monitor (from drone_lib)
    system_health_node = Node(
        
        package='drone_lib',
        executable='system_health',
        parameters=[simulation_params],
        output='screen'
    )

    # Mission FSM node (delayed start to let other nodes initialize)
    fsm_node = Node(
        package='mission_3',
        executable=LaunchConfiguration("mission"),
        parameters=[simulation_params],
        output='screen'
    )


    vision_node = Node(
        package='manometro_detector',   
        executable='manometro_detector',       
        output='screen'
    )

    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])











    return LaunchDescription([
        exec_arg,
        system_health_node,
        vision_node,
        delayed_fsm_node
    ])