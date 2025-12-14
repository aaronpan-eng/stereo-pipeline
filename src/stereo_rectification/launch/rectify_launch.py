from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config file argument for cmd line
    config_file_arg = DeclareLaunchArgument(
        'config_yaml',
        default_value='UAS1_fitness_trail_2025-10-15.yaml',
        description='YAML filename with .yaml extension with camera calibration parameters'
    )

    # Get config file path
    config_file_path = PathJoinSubstitution([
        FindPackageShare('stereo_rectification'),
        'config',
        LaunchConfiguration('config_yaml')
    ])

    # Rectify node
    rectify_node = Node(
        package='stereo_rectification',
        executable='rectify',
        name='rectify_stereo_imgs',
        output='screen',
        emulate_tty=True,
        parameters=[config_file_path],
    )

    return LaunchDescription([
        config_file_arg,
        rectify_node
    ])

