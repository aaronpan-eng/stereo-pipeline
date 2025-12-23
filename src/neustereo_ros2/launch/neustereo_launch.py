from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config file argument
    model_config_file_arg = DeclareLaunchArgument(
        'model_config_yaml',
        default_value='neustereo_ros2.yaml',
        description='YAML config file for NeuStereo model'
    )

    # Get config file path
    config_file_path = PathJoinSubstitution([
        FindPackageShare('neustereo_ros2'),
        'config',
        LaunchConfiguration('model_config_yaml')
    ])

    # Override parameters with launch arguments
    model_filename_arg = DeclareLaunchArgument(
        'model_filename', 
        description='Override model filename in config file',
    )
    display_disparity_arg = DeclareLaunchArgument(
        'display_disparity', 
        description='Override display disparity flag in config file',
    )
    display_stereo_resized_arg = DeclareLaunchArgument(
        'display_stereo_resized', 
        description='Override display stereo resized flag in config file',
    )
    display_stereo_arg = DeclareLaunchArgument(
        'display_stereo', 
        description='Override display stereo flag in config file',
    )

    # NeuStereo node
    neustereo_node = Node(
        package='neustereo_ros2',
        executable='neu_stereo_node',
        name='neu_stereo_node',
        output='screen',
        emulate_tty=True, # Show logs in terminal
        parameters=[
            config_file_path,
            {
                'model_filename': LaunchConfiguration('model_filename'),
                'display_disparity': LaunchConfiguration('display_disparity'),
                'display_stereo_resized': LaunchConfiguration('display_stereo_resized'),
                'display_stereo': LaunchConfiguration('display_stereo'),
                'model_config_file': LaunchConfiguration('model_config_file'),
            }
        ],
    )

    return LaunchDescription([
        model_config_file_arg,
        model_filename_arg,
        display_disparity_arg,
        display_stereo_resized_arg,
        display_stereo_arg,
        neustereo_node
    ])

