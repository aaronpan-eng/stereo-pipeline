#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
import glob
import os
from typing import List, Tuple


class StereoImagePlayback(Node):
    def __init__(self, playback_rate: float = 1.0):
        """
        Args:
            playback_rate: Rate in Hz to publish images (default: 1.0 Hz)
        """
        super().__init__('stereo_image_playback')
        
        self.playback_rate = playback_rate
        self.period = 1.0 / playback_rate if playback_rate > 0 else 1.0
        
        # Input directory
        self.input_dir = Path('/workspace/research/stereo_images')
        
        # Camera calibration parameters (from camera_info_publisher.py)
        # Camera0 (left) calibration intrinsics
        self.K0 = np.array([[1465.68012087966, 0.0, 614.3963941599402],
                            [0.0, 1467.0112924106957, 502.41166600878574],
                            [0.0, 0.0, 1.0]])
        self.D0 = np.array([-0.2276610738329907, 0.1660478844099269, 
                            -0.0004548029386917339, -0.00101001689804351])
        
        # Camera1 (right) calibration intrinsics
        self.K1 = np.array([[1467.8129423414975, 0.0, 621.2933466671502],
                            [0.0, 1470.2125944454904, 506.2026058803627],
                            [0.0, 0.0, 1.0]])
        self.D1 = np.array([-0.22786734473467452, 0.1762437097407578, 
                            -0.00018181733061175195, -0.0014607816751857803])
        
        # Image dimensions (will be set from first image)
        self.image_width = None
        self.image_height = None
        
        # CvBridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()
        
        # Load images by globbing directories
        cam0_dir = self.input_dir / 'cam0'
        cam1_dir = self.input_dir / 'cam1'
        
        # Glob all jpg images and sort them
        cam0_images = sorted(glob.glob(str(cam0_dir / '*.jpg')))
        cam1_images = sorted(glob.glob(str(cam1_dir / '*.jpg')))
        
        # Use minimum length to ensure we can pair all images
        self.num_pairs = min(len(cam0_images), len(cam1_images))
        self.cam0_images = cam0_images[:self.num_pairs]
        self.cam1_images = cam1_images[:self.num_pairs]
        
        self.get_logger().info(f'Found {len(cam0_images)} cam0 images')
        self.get_logger().info(f'Found {len(cam1_images)} cam1 images')
        self.get_logger().info(f'Will publish {self.num_pairs} image pairs')
        
        # Create publishers
        self.cam0_pub = self.create_publisher(Image, '/left/image_raw', 10)
        self.cam1_pub = self.create_publisher(Image, '/right/image_raw', 10)
        self.cam0_info_pub = self.create_publisher(CameraInfo, '/left/camera_info', 10)
        self.cam1_info_pub = self.create_publisher(CameraInfo, '/right/camera_info', 10)
        
        self.get_logger().info(f'Publishing at {playback_rate} Hz')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /left/image_raw')
        self.get_logger().info('  - /right/image_raw')
        self.get_logger().info('  - /left/camera_info')
        self.get_logger().info('  - /right/camera_info')
        
        # Playback state
        self.current_index = 0
        self.start_time = None
        
        # Create timer for playback
        self.timer = self.create_timer(self.period, self.publish_next_pair)
    
    def _extract_timestamp_from_filename(self, filepath: str) -> int:
        """Extract timestamp from filename (assumes format: {timestamp}_{left/right}.jpg)."""
        filename = os.path.basename(filepath)
        timestamp_str = filename.split('_')[0]
        return int(timestamp_str)
        
    def _create_camera_info(self, K, D, width, height, frame_id, stamp):
        """Create a CameraInfo message from calibration parameters."""
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = frame_id
        
        camera_info.width = width
        camera_info.height = height
        camera_info.distortion_model = 'plumb_bob'
        
        # Flatten and convert to list
        camera_info.d = D.flatten().tolist()
        camera_info.k = K.flatten().tolist()
        
        # For unrectified images, R is identity and P is K with zero translation
        R = np.eye(3)
        P = np.zeros((3, 4))
        P[:3, :3] = K
        
        camera_info.r = R.flatten().tolist()
        camera_info.p = P.flatten().tolist()
        
        return camera_info
    
    def _publish_image(self, image_path: str, stamp_msg, publisher, frame_id: str = 'cam'):
        """Load and publish a single image as Image message."""
        # Read image
        img = cv2.imread(image_path)
        if img is None:
            self.get_logger().error(f'Failed to load image: {image_path}')
            return None
        
        # Set image dimensions from first image
        if self.image_width is None or self.image_height is None:
            self.image_height, self.image_width = img.shape[:2]
        
        # Convert OpenCV image to ROS Image message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header.stamp = stamp_msg
            img_msg.header.frame_id = frame_id
            publisher.publish(img_msg)
            return img
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return None
    
    def publish_next_pair(self):
        """Publish the next pair of stereo images and camera info."""
        if self.current_index >= self.num_pairs:
            self.get_logger().info('Finished playback - all images published')
            self.timer.cancel()
            return
        
        # Get images by index (paired sequentially)
        cam0_path = self.cam0_images[self.current_index]
        cam1_path = self.cam1_images[self.current_index]
        
        # Extract timestamps from filenames for each camera
        cam0_timestamp_ns = self._extract_timestamp_from_filename(cam0_path)
        cam1_timestamp_ns = self._extract_timestamp_from_filename(cam1_path)
        
        # Create timestamps using individual image timestamps
        cam0_stamp = rclpy.time.Time(seconds=cam0_timestamp_ns // 1_000_000_000,
                                     nanoseconds=cam0_timestamp_ns % 1_000_000_000)
        cam1_stamp = rclpy.time.Time(seconds=cam1_timestamp_ns // 1_000_000_000,
                                     nanoseconds=cam1_timestamp_ns % 1_000_000_000)
        cam0_stamp_msg = cam0_stamp.to_msg()
        cam1_stamp_msg = cam1_stamp.to_msg()
        
        # Publish both images with their respective timestamps
        cam0_img = self._publish_image(cam0_path, cam0_stamp_msg, self.cam0_pub, 'left')
        cam1_img = self._publish_image(cam1_path, cam1_stamp_msg, self.cam1_pub, 'right')
        
        # Publish camera info if images were loaded successfully
        if cam0_img is not None and cam1_img is not None:
            # Ensure image dimensions are set
            if self.image_width is None or self.image_height is None:
                self.image_height, self.image_width = cam0_img.shape[:2]
            
            # Create and publish camera info for left camera (using left timestamp)
            cam0_info = self._create_camera_info(
                self.K0, self.D0, self.image_width, self.image_height,
                'left', cam0_stamp_msg
            )
            self.cam0_info_pub.publish(cam0_info)
            
            # Create and publish camera info for right camera (using right timestamp)
            cam1_info = self._create_camera_info(
                self.K1, self.D1, self.image_width, self.image_height,
                'right', cam1_stamp_msg
            )
            self.cam1_info_pub.publish(cam1_info)
        
        if self.current_index % 10 == 0:
            self.get_logger().info(f'Published image pair {self.current_index + 1}/{self.num_pairs}')
        
        self.current_index += 1


def main(args=None):
    import sys
    
    # Parse playback rate from command line (default: 1.0 Hz)
    playback_rate = 1.0
    if len(sys.argv) > 1:
        try:
            playback_rate = float(sys.argv[1])
            if playback_rate <= 0:
                print("Error: Playback rate must be positive")
                return
        except ValueError:
            print(f"Error: Invalid playback rate '{sys.argv[1]}'. Using default: 1.0 Hz")
            playback_rate = 1.0
    
    rclpy.init(args=args)
    node = StereoImagePlayback(playback_rate=playback_rate)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

