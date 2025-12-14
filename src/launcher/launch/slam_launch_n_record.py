
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # TODO: add any parameters like:: calibration yaml, results directory, bag file to play

    # Rectify raw stereo images based on camera intrinsics/extrinsics
    rectify = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('stereo_rectification'), 'launch'),
            '/rectify_launch.py'])
    )

    # Cuvslam stereo node
    cuvslam_stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('cuvslam_stereo'), 'launch'),
            '/cuvslam_stereo_launch.py'])
    )

    # Fast LIMO node
    fast_LIMO = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('fast_limo'), 'launch'),
            '/fast_limo.launch.py'])
    )

    # Fast LIMO odometry saver node
    fast_LIMO_odometry_saver = Node(
        package='fast_limo'
        executable='odometry_saver'
        name='odometry_saver'
        output='screen'
    )

    return LaunchDescription([
        rectify,
        cuvslam_stereo,
        fast_LIMO,
        fast_LIMO_odometry_saver,
    ])


