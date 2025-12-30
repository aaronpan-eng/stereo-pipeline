from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get config file path
    config_file_path = PathJoinSubstitution([
        FindPackageShare('rerun_visualizer'),
        'config',
        'config.yaml'
    ])

    # Rerun initilaization node
    # sets up rerun blueprint and views for all nodes
    rerun_node = Node(
        package='rerun_visualizer',
        executable='rerun_visualizer',
        name='rerun_visualizer',
        output='screen',
        emulate_tty=True, # Show logs in terminal
        parameters=[config_file_path],
    )

    return LaunchDescription([
        rerun_node,
    ])

