#!/usr/bin/env python3
"""
Real flight launch configuration for SAE 2026 — Mission 1.
Launches the drone node, FSM node, and supporting services.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
import datetime
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_mission_1 = get_package_share_directory('mission_1_H')
    flight_params = os.path.join(pkg_mission_1, "config", "flight.yaml")

    pkg_rdpformas = get_package_share_directory('RDPformas')
    rdpformas_params = os.path.join(pkg_rdpformas, "config", "rdpformas.yaml")

    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.expanduser(f'~/evtol/mission_logs/mission_1_H_{timestamp}')
    bag_topics = [
        '/rosout',
        '/telemetry/logs',
        '/telemetry/drone_status',
        '/telemetry/position',
        '/telemetry/system_health',
        '/bouncing_detection',
        '/discovered_bases',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_status',
        '/fmu/out/battery_status',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/vehicle_command',
    ]

    # Incluir /drone_trajectory no bag apenas se bag_record_trajectory: true no YAML
    with open(flight_params, 'r') as _f:
        _p = yaml.safe_load(_f)
    if _p.get('mission_1_H_node', {}).get('ros__parameters', {}).get('bag_record_trajectory', False):
        bag_topics.append('/drone_trajectory')

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics,
        output='screen'
    )

    # Declare the mission executable argument
    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="mission_1_H",
        description="Executable that implements the mission FSM")

    webcam_publisher_node = Node(
        package='camera_publisher',
        executable='webcam',
        output='screen',
        parameters=[{
            'video_source': '/dev/video2',
            'camera_name': 'vertical',   # publishes to /vertical_camera/compressed
        }]
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
        package='mission_1_H',
        executable=LaunchConfiguration("mission"),
        parameters=[flight_params],
        output='screen'
    )

    # Delays vêm do YAML (camera_startup_delay, cv_startup_delay).
    # Permite ajustar sem reconstruir.
    _params = _p.get('mission_1_H_node', {}).get('ros__parameters', {})
    cam_delay = float(_params.get('camera_startup_delay', 8.0))
    cv_delay  = float(_params.get('cv_startup_delay',     8.0))

    # FSM starts immediately (Drone::waitForOdometry handles sync internally).
    bouncing_cv_node = Node(
        package='RDPformas',
        executable='RDPformas',
        parameters=[rdpformas_params],
        output='screen'
    )
    delayed_webcam_node = TimerAction(period=cam_delay, actions=[webcam_publisher_node])
    delayed_cv_node     = TimerAction(period=cv_delay,  actions=[bouncing_cv_node])

    return LaunchDescription([
        exec_arg,
        bag_record,
        system_health_node,
        fsm_node,               # FSM + Drone first → owns CPU during takeoff
        delayed_webcam_node,    # camera at camera_startup_delay
        delayed_cv_node,        # CV at cv_startup_delay
    ])
