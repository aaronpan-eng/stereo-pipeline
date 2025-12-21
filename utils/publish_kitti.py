#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import argparse
from pathlib import Path
import numpy as np


class KittiStereoPublisher(Node):
    def __init__(self, dataset_path, sequence, rate, use_color=False):
        super().__init__('kitti_stereo_publisher')
        
        self.bridge = CvBridge()
        self.rate = rate
        self.use_color = use_color
        
        # Setup publishers
        self.pub_left = self.create_publisher(
            Image, 
            '/cam_sync/cam0/image_rect', 
            10
        )
        self.pub_right = self.create_publisher(
            Image, 
            '/cam_sync/cam1/image_rect', 
            10
        )
        
        # Load image paths
        self.left_images, self.right_images = self._load_image_paths(
            dataset_path, sequence, use_color
        )
        
        if len(self.left_images) == 0:
            self.get_logger().error('No images found in dataset!')
            return
            
        self.get_logger().info(f'Loaded {len(self.left_images)} stereo pairs from sequence {sequence}')
        self.get_logger().info(f'Publishing at {rate} Hz')
        
        # Start publishing
        self.current_idx = 0
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        
    def _load_image_paths(self, dataset_path, sequence, use_color):
        """Load image paths from KITTI dataset directory."""
        base_path = Path(dataset_path) / 'sequences' / sequence
        
        # KITTI uses image_2 (left color), image_3 (right color)
        # or image_0 (left grayscale), image_1 (right grayscale)
        if use_color:
            left_dir = base_path / 'image_2'
            right_dir = base_path / 'image_3'
        else:
            left_dir = base_path / 'image_0'
            right_dir = base_path / 'image_1'
            
        if not left_dir.exists() or not right_dir.exists():
            self.get_logger().error(f'Image directories not found: {left_dir} or {right_dir}')
            return [], []
        
        # Get sorted list of image files
        left_images = sorted([str(f) for f in left_dir.glob('*.png')])
        right_images = sorted([str(f) for f in right_dir.glob('*.png')])
        
        # Verify matching pairs
        if len(left_images) != len(right_images):
            self.get_logger().warn(
                f'Mismatch in stereo pairs: {len(left_images)} left, {len(right_images)} right'
            )
            min_len = min(len(left_images), len(right_images))
            left_images = left_images[:min_len]
            right_images = right_images[:min_len]
            
        return left_images, right_images
    
    def timer_callback(self):
        """Publish stereo pair at specified rate."""
        if self.current_idx >= len(self.left_images):
            self.get_logger().info('Finished publishing all images. Looping...')
            self.current_idx = 0
            
        # Read images
        left_img = cv2.imread(self.left_images[self.current_idx])
        right_img = cv2.imread(self.right_images[self.current_idx])
        
        if left_img is None or right_img is None:
            self.get_logger().error(f'Failed to read images at index {self.current_idx}')
            self.current_idx += 1
            return
        
        # Convert to ROS Image messages
        # KITTI images are BGR by default if color, grayscale if grayscale
        if self.use_color:
            left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
            right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')
        else:
            # Convert to grayscale if loaded as color
            if len(left_img.shape) == 3:
                left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
                right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
            left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding='mono8')
            right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding='mono8')
        
        # Set timestamps (synchronized)
        timestamp = self.get_clock().now().to_msg()
        left_msg.header.stamp = timestamp
        right_msg.header.stamp = timestamp
        left_msg.header.frame_id = 'cam0'
        right_msg.header.frame_id = 'cam1'
        
        # Publish
        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)
        
        self.get_logger().info(
            f'Published frame {self.current_idx}/{len(self.left_images)}',
            throttle_duration_sec=1.0
        )
        
        self.current_idx += 1


def main():
    parser = argparse.ArgumentParser(description='Publish KITTI stereo dataset to ROS2')
    parser.add_argument(
        'dataset_path',
        type=str,
        help='Path to KITTI dataset root (containing sequences/)'
    )
    parser.add_argument(
        '--sequence',
        type=str,
        default='00',
        help='KITTI sequence number (default: 00)'
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=10.0,
        help='Publishing rate in Hz (default: 10.0)'
    )
    parser.add_argument(
        '--color',
        action='store_true',
        help='Use color images (image_2/3) instead of grayscale (image_0/1)'
    )
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = KittiStereoPublisher(
            args.dataset_path,
            args.sequence,
            args.rate,
            args.color
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()