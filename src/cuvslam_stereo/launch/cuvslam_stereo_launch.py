from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cuvslam_stereo',
            executable='cuvslam_stereo_node',
            name='cuvslam_stereo_node',
            output='screen'  # Show logs in terminal
        )
    ])