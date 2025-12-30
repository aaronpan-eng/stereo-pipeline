from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Rerun visualization
    rerun_visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rerun_visualizer'),
                'launch',
                'rerun_visualizer_launch.py'
            ])
        )
    )

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
            'save_trajectory_tum': 'false',
        }.items()
    )

    # Neustereo ros2 node
    neustereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('neustereo_ros2'),
                'launch',
                'neustereo_launch.py'
            ])
        ),
        launch_arguments={
            'model_filename': 'models/NeuStereo_FSD_250924_checkpoints_epoch_30.pth',
            'display_disparity': 'true',
            'display_stereo_resized': 'true',
            'display_stereo': 'false',
            'model_config_file': 'config.yaml',
        }.items()
    )

    return LaunchDescription([
        rerun_visualizer,
        rectify_config_arg,
        rectify,
        cuvslam_stereo,
        neustereo,
    ])