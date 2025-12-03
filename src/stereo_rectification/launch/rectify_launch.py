from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # Get the path to the config file
    pkg_share = get_package_share_directory('stereo_rectification')
    config_file = os.path.join(pkg_share, 'config', 'payload1_20250828_1257_calibBefore_ros1_debayered-camchain-imucam.yaml')
    
    # Load the YAML file
    with open(config_file, 'r') as f:
        config_data = yaml.safe_load(f)
    
    # Extract parameters from cuvslam_stereo_node section
    cuvslam_params = config_data['cuvslam_stereo_node']['ros__parameters']
    
    # Map to rectify node parameters format
    rectify_params = {
        'cam0.intrinsics': cuvslam_params['cam0']['intrinsics'],
        'cam0.resolution': cuvslam_params['cam0']['resolution'],
        'cam0.distortion_coeffs': cuvslam_params['cam0']['distortion_coeffs'],
        'cam0.distortion_model': cuvslam_params['cam0']['distortion_model'],
        'cam1.intrinsics': cuvslam_params['cam1']['intrinsics'],
        'cam1.resolution': cuvslam_params['cam1']['resolution'],
        'cam1.distortion_coeffs': cuvslam_params['cam1']['distortion_coeffs'],
        'cam1.distortion_model': cuvslam_params['cam1']['distortion_model'],
        'cam1.T_cn_cnm1': cuvslam_params['cam1']['T_cn_cnm1'],
    }
    
    return LaunchDescription([
        Node(
            package='stereo_rectification',
            executable='rectify',
            name='rectify_stereo_imgs',
            output='screen',
            emulate_tty=True,
            parameters=[rectify_params],
        ),
    ])

