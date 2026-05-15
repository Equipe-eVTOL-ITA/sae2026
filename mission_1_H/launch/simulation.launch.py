#!/usr/bin/env python3
"""
Simulation launch configuration for SAE 2026 — Mission 1.
Launches the drone node, FSM node, and supporting services.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
import datetime
from ament_index_python.packages import get_package_share_directory


RVIZ_CONFIG = os.path.expanduser('~/evtol/dev/src/sae2026/rviz/trajectory.rviz')

def generate_launch_description():

    pkg_mission_1 = get_package_share_directory('mission_1_H')
    simulation_params = os.path.join(pkg_mission_1, "config", "simulation.yaml")

    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/mission_1_H_{timestamp}')
    bag_topics = [
        '/rosout',
        '/telemetry/logs',
        '/telemetry/drone_status',
        '/telemetry/position',
        '/telemetry/system_health',
        '/drone_trajectory',
        '/bouncing_detection',
        '/discovered_bases',
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

    # Declare the mission executable argument
    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="mission_1_H",
        description="Executable that implements the mission FSM")

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 for trajectory visualization")

    # System health monitor (from drone_lib)
    system_health_node = Node(
        
        package='drone_lib',
        executable='system_health',
        parameters=[simulation_params],
        output='screen'
    )

    # Mission FSM node (delayed start to let other nodes initialize)
    fsm_node = Node(
        package='mission_1_H',
        executable=LaunchConfiguration("mission"),
        parameters=[simulation_params],
        output='screen'
    )

    delayed_fsm_node = TimerAction(period=5.0, actions=[fsm_node])

    # Vision node
    bouncing_cv_node = Node(
        package='RDPformas',#'bouncing_detector',
        executable='RDPformas',#'bouncing_detector_node',
        output='screen'
    )


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
        bag_record,
        system_health_node,
        bouncing_cv_node,
        delayed_fsm_node,
        rviz_node,
    ])
