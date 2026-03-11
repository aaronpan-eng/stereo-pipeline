from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config file argument
    neustereo_ros2_config_arg = DeclareLaunchArgument(
        'neustereo_ros2_config',
        default_value='neustereo_ros2.yaml',
        description='YAML config file for NeuStereo model'
    )

    # Get config file path
    neustereo_ros2_config_path = PathJoinSubstitution([
        FindPackageShare('neustereo_ros2'),
        'config',
        LaunchConfiguration('neustereo_ros2_config')
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
    rerun_visualization_arg = DeclareLaunchArgument(
        'rerun_visualization', 
        description='Override rerun visualization flag in config file',
    )

    # NeuStereo node
    neustereo_node = Node(
        package='neustereo_ros2',
        executable='neu_stereo_node',
        name='neu_stereo_node',
        output='screen',
        emulate_tty=True, # Show logs in terminal
        parameters=[
            neustereo_ros2_config_path,
            {
                'model_filename': LaunchConfiguration('model_filename'),
                'display_disparity': LaunchConfiguration('display_disparity'),
                'display_stereo_resized': LaunchConfiguration('display_stereo_resized'),
                'display_stereo': LaunchConfiguration('display_stereo'),
                'rerun_visualization': LaunchConfiguration('rerun_visualization'),
            }
        ],
    )

    return LaunchDescription([
        neustereo_ros2_config_arg,
        model_filename_arg,
        display_disparity_arg,
        display_stereo_resized_arg,
        display_stereo_arg,
        rerun_visualization_arg,
        neustereo_node
    ])

