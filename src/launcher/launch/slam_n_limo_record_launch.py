
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch arg for rectify config (just the filename, child builds the full path)
    rectify_config_arg = DeclareLaunchArgument(
        'rectify_config_yaml',
        default_value='payload1_20250828.yaml',
        description='REQUIRED: YAML filename with camera calibration parameters (e.g. payload1_20250828.yaml)'
    )

    # Rectify raw stereo images based on camera intrinsics/extrinsics
    rectify = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stereo_rectification'),
                'launch',
                'rectify_launch.py'
            ])
        ),
        launch_arguments={
            'config_yaml': LaunchConfiguration('rectify_config_yaml'), # Pass just the filename - child launch builds the full path
        }.items()
        # Can replace with other neuroam payloads by changing the config_yaml argument
    )

    # Cuvslam stereo node
    cuvslam_stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cuvslam_stereo'),
                'launch',
                'cuvslam_stereo_launch.py'
            ])
        ),
        launch_arguments={
            'rerun_visualization': 'true',
            'save_trajectory_tum': 'true',
        }.items()
    )

    # Fast LIMO node
    fast_LIMO = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('fast_limo'), 'launch'),
            '/fast_limo.launch.py'])
    )

    # Fast LIMO odometry saver node
    fast_LIMO_odometry_saver = Node(
        package='fast_limo',
        executable='odometry_saver',
        name='odometry_saver',
        output='screen',
    )

    return LaunchDescription([
        rectify_config_arg,
        rectify,
        cuvslam_stereo,
        fast_LIMO,
        fast_LIMO_odometry_saver,
    ])


