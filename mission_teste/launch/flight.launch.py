#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os
import datetime
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('mission_teste')
    params = os.path.join(pkg, 'config', 'flight.yaml')

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

    camera_node = Node(
        package='camera_publisher',
        executable='webcam',
        output='screen'
    )

    vision_node = Node(
        package='bouncing_detector',
        executable='bouncing_detector_node',
        output='screen'
    )

    fsm_node = Node(
        package='mission_teste',
        executable='mission_teste',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([
        bag_record,
        system_health_node,
        camera_node,
        vision_node,
        TimerAction(period=5.0, actions=[fsm_node]),
    ])
