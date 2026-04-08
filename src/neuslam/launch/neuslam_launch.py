from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config file argument
    neuslam_config_arg = DeclareLaunchArgument(
        'neuslam_config',
        default_value='neuslam_config.yaml',
        description='Neuslam node ros parameter config file'
    )

    neuslam_config_path = PathJoinSubstitution([
        FindPackageShare('neuslam'),
        'config',
        LaunchConfiguration('neuslam_config')
    ])

    # Override parameters with launch arguments
    # arg = DeclareLaunchArgument(
    #     '', 
    #     description='',
    # )

    # neuslam node
    neuslam_node = Node(
        package='neuslam',
        executable='neuslam',
        name='neuslam',
        output='screen',
        emulate_tty=True, # Show logs in terminal
        parameters=[
            neuslam_config_path,
        ],
    )

    return LaunchDescription([
        neuslam_config_arg,
        neuslam_node
    ])

