import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('audio_alert')

    audio_above = os.path.join(pkg_share, 'audio', 'above.mp3')
    audio_below = os.path.join(pkg_share, 'audio', 'below.mp3')

    return LaunchDescription([
        Node(
            package='audio_alert',
            executable='audio_alert',
            name='audio_alert',
            parameters=[{
                'audio_above_limit': audio_above if os.path.isfile(audio_above) else '',
                'audio_below_limit': audio_below if os.path.isfile(audio_below) else '',
            }],
            output='screen',
        ),
    ])
