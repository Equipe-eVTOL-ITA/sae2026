#!/usr/bin/env python3
"""
Flight launch configuration for SAE 2026 — Mission 2.

Launches the drone node, FSM node, computer-vision detectors and the
ground-station telemetry pipeline (ros2 bag).

All image-related topics are individually controllable from the command line
so the operator can trade detection quality for telemetry bandwidth:

    Camera topics (raw stream feeding detectors):
        vertical_camera:=true|false        # /vertical_camera/compressed
        horizontal_camera:=true|false      # /horizontal_camera/compressed
        Resolution / framerate live in:
            config/vertical_camera.yaml
            config/horizontal_camera.yaml

    Debug overlays (annotations sent to the ground station):
        ball_debug_image:=true|false       # /ball_detection_image/compressed
        ball_debug_mask:=true|false        # /ball_detector/mask/compressed
        mangueira_debug_image:=true|false  # /mangueira_detector/image/compressed
        mangueira_debug_mask:=true|false   # /mangueira_detector/mask/compressed
        Throttling / downscale / JPEG quality live in:
            config/ball_detector.yaml      (debug_publish_rate, debug_max_width,
                                            debug_jpeg_quality)
            config/mangueira_detector.yaml (idem)

    Telemetry bag recording (optional):
        record_bag:=true|false             # default true
        record_camera_topics:=true|false   # include raw camera streams in bag

Example — low bandwidth ground-station run:
    ros2 launch mission_2 flight.launch.py \\
        ball_debug_mask:=false mangueira_debug_mask:=false \\
        record_camera_topics:=false
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(ctx, name):
    return LaunchConfiguration(name).perform(ctx).strip().lower() in (
        '1', 'true', 'yes', 'on'
    )


def _launch_setup(context, *args, **kwargs):
    pkg_mission_2 = get_package_share_directory('mission_2')
    cfg = lambda fname: os.path.join(pkg_mission_2, 'config', fname)

    flight_params = cfg('flight.yaml')
    ball_detector_config = cfg('ball_detector.yaml')
    mangueira_detector_config = cfg('mangueira_detector.yaml')
    vertical_camera_config = cfg('vertical_camera.yaml')
    horizontal_camera_config = cfg('horizontal_camera.yaml')

    enable_vertical = _truthy(context, 'vertical_camera')
    enable_horizontal = _truthy(context, 'horizontal_camera')

    ball_debug_image = _truthy(context, 'ball_debug_image')
    ball_debug_mask = _truthy(context, 'ball_debug_mask')
    mangueira_debug_image = _truthy(context, 'mangueira_debug_image')
    mangueira_debug_mask = _truthy(context, 'mangueira_debug_mask')

    record_bag = _truthy(context, 'record_bag')
    record_camera_topics = _truthy(context, 'record_camera_topics')

    actions = []

    # ----- ros2 bag recording (telemetry log) -----
    if record_bag:
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
            '/mangueira/position',
            '/mangueira/angle',
            '/mangueira/detections',
            '/fmu/out/vehicle_local_position',
            '/fmu/out/vehicle_status',
            '/fmu/out/battery_status',
            '/fmu/in/trajectory_setpoint',
            '/fmu/in/vehicle_command',
        ]
        if record_camera_topics:
            if enable_vertical:
                bag_topics.append('/vertical_camera/compressed')
            if enable_horizontal:
                bag_topics.append('/horizontal_camera/compressed')
            if ball_debug_image:
                bag_topics.append('/ball_detection_image/compressed')
            if ball_debug_mask:
                bag_topics.append('/ball_detector/mask/compressed')
            if mangueira_debug_image:
                bag_topics.append('/mangueira_detector/image/compressed')
            if mangueira_debug_mask:
                bag_topics.append('/mangueira_detector/mask/compressed')

        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics,
            output='screen',
        ))

    # ----- Cameras -----
    if enable_vertical:
        actions.append(Node(
            package='camera_publisher',
            executable='webcam',
            name='vertical_camera',
            output='screen',
            parameters=[vertical_camera_config, {'video_source': '/dev/video2'}],
        ))
    if enable_horizontal:
        actions.append(Node(
            package='camera_publisher',
            executable='webcam',
            name='horizontal_camera',
            output='screen',
            parameters=[horizontal_camera_config, {'video_source': '/dev/video0'}],
        ))

    # ----- System health -----
    actions.append(Node(
        package='drone_lib',
        executable='system_health',
        parameters=[flight_params],
        output='screen',
    ))

    # ----- Detectors -----
    # Launch-arg overrides take precedence over YAML defaults for the
    # debug_image / debug_mask switches, so the operator can fully silence a
    # debug topic with a single CLI flag.
    actions.append(Node(
        package='ball_detector',
        executable='ball_detector_node',
        parameters=[
            ball_detector_config,
            {'debug_image': ball_debug_image, 'debug_mask': ball_debug_mask},
        ],
        output='screen',
    ))
    actions.append(Node(
        package='mangueira_detector',
        executable='mangueira_detector_node',
        parameters=[
            mangueira_detector_config,
            {'debug_image': mangueira_debug_image,
             'debug_mask': mangueira_debug_mask},
        ],
        output='screen',
    ))

    # ----- FSM (delayed so detectors/cameras are warm) -----
    fsm_node = Node(
        package='mission_2',
        executable=LaunchConfiguration('mission'),
        parameters=[flight_params],
        output='screen',
    )
    actions.append(TimerAction(period=5.0, actions=[fsm_node]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission', default_value='mission_2',
            description='Executable that implements the mission FSM',
        ),

        # Camera streams
        DeclareLaunchArgument(
            'vertical_camera', default_value='true',
            description='Publish /vertical_camera/compressed'),
        DeclareLaunchArgument(
            'horizontal_camera', default_value='true',
            description='Publish /horizontal_camera/compressed'),

        # Detector debug overlays (telemetry)
        DeclareLaunchArgument(
            'ball_debug_image', default_value='true',
            description='Publish /ball_detection_image/compressed'),
        DeclareLaunchArgument(
            'ball_debug_mask', default_value='true',
            description='Publish /ball_detector/mask/compressed'),
        DeclareLaunchArgument(
            'mangueira_debug_image', default_value='true',
            description='Publish /mangueira_detector/image/compressed'),
        DeclareLaunchArgument(
            'mangueira_debug_mask', default_value='true',
            description='Publish /mangueira_detector/mask/compressed'),

        # Telemetry bag
        DeclareLaunchArgument(
            'record_bag', default_value='true',
            description='Record a ros2 bag with mission telemetry'),
        DeclareLaunchArgument(
            'record_camera_topics', default_value='true',
            description='Include image topics in the telemetry bag '
                        '(disable to save disk on low-bandwidth runs)'),

        OpaqueFunction(function=_launch_setup),
    ])
