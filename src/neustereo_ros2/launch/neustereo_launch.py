from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config file argument
    config_file_arg = DeclareLaunchArgument(
        'config_yaml',
        default_value='neustereo_ros2.yaml',
        description='YAML config file for NeuStereo node'
    )

    # Get config file path
    config_file_path = PathJoinSubstitution([
        FindPackageShare('neustereo_ros2'),
        'config',
        LaunchConfiguration('config_yaml')
    ])

    # NeuStereo node
    neustereo_node = Node(
        package='neustereo_ros2',
        executable='neu_stereo_node',
        name='neu_stereo_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file_path],
    )

    return LaunchDescription([
        config_file_arg,
        neustereo_node
    ])

