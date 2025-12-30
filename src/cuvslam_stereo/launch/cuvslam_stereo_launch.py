from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch arg for cuvslam
    cuvslam_stereo_config_arg = DeclareLaunchArgument(
        'cuvslam_stereo_config',
        default_value='cuvslam_config.yaml',
        description='Cuvslam stereo config file'
    )

    # Get the path
    cuvslam_stereo_config_path = PathJoinSubstitution([
        FindPackageShare('cuvslam_stereo'),
        'config',
        LaunchConfiguration('cuvslam_stereo_config')
    ])

    # Override parameters with launch arguments
    rerun_visualization_arg = DeclareLaunchArgument(
        'rerun_visualization',
        description='Override rerun visualization flag in config file'
    )
    save_trajectory_tum_arg = DeclareLaunchArgument(
        'save_trajectory_tum',
        description='Override save trajectory tum flag in config file'
    )

    # Cuvslam stereo node
    cuvslam_stereo_node = Node(
        package='cuvslam_stereo',
        executable='cuvslam_stereo_node',
        name='cuvslam_stereo_node',
        output='screen',  # Show logs in terminal
        parameters=[
            cuvslam_stereo_config_path,
            {
                'rerun_visualization': LaunchConfiguration('rerun_visualization'),
                'save_trajectory_tum': LaunchConfiguration('save_trajectory_tum'),
            },
        ],
    )


    return LaunchDescription([
        cuvslam_stereo_config_arg,
        rerun_visualization_arg,
        save_trajectory_tum_arg,
        cuvslam_stereo_node
    ])