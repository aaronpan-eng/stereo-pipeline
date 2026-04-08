from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arg for rectify config (just the filename, child builds the full path)
    rectify_config_arg = DeclareLaunchArgument(
        'rectify_config_yaml',
        default_value='frl0_drone.yaml',
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
        }.items()
    )

    # Setting the frame rate via bash terminal
    set_frame_rate = ExecuteProcess(
        cmd=['v4l2-ctl', '-d', '/dev/video0', '--set-parm=65'],
        output='screen',
    )

    # Launching the cameras
    left_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='/left',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'video_device': '/dev/video0',
            'output_encoding': 'mono8',
            'image_size': [1232, 1024],
            'trigger_mode': False,
            'reverse_x': True,
            'reverse_y': True,
            'frame_id': 'left_camera',
            'exposure_active_line_selector': 0,
            'exposure_active_line_mode': True,
            'pixel_format': 'GREY',
        }],
    )

    right_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='/right',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'video_device': '/dev/video1',
            'output_encoding': 'mono8',
            'image_size': [1232, 1024],
            'trigger_mode': True,
            'trigger_source': 0,
            'reverse_x': True,
            'reverse_y': True,
            'frame_id': 'right_camera',
            'pixel_format': 'GREY',
        }],
    )

    neuslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('neuslam'),
                'launch',
                'neuslam_launch.py'
            ])
        ),
    )

    return LaunchDescription([
        rectify_config_arg,
        set_frame_rate,
        left_camera,
        right_camera,
        rectify,
        cuvslam_stereo,
        neustereo,
        neuslam,
    ])
