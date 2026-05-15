#!/usr/bin/env python3
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

    pkg = get_package_share_directory('mission_teste')
    params = os.path.join(pkg, 'config', 'simulation.yaml')

    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/mission_teste_{timestamp}')
    bag_topics = [
        '/rosout',
        '/telemetry/logs',
        '/telemetry/drone_status',
        '/telemetry/position',
        '/telemetry/system_health',
        '/drone_trajectory',
        '/bouncing_detection',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_status',
        '/fmu/out/battery_status',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/vehicle_command',
    ]

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2')

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics,
        output='screen'
    )

    system_health_node = Node(
        package='drone_lib',
        executable='system_health',
        parameters=[params],
        output='screen'
    )

    vision_node = Node(
        package='RDPformas',
        executable='RDPformas',
        output='screen'
    )

    fsm_node = Node(
        package='mission_teste',
        executable='mission_teste',
        parameters=[params],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', RVIZ_CONFIG],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        bag_record,
        system_health_node,
        vision_node,
        TimerAction(period=5.0, actions=[fsm_node]),
        rviz_node,
    ])
