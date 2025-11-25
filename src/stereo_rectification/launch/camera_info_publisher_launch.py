from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_rectification',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen',
            emulate_tty=True,
        ),
    ])

