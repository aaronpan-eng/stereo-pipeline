#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
import os
from pathlib import Path
from datetime import datetime
from cv_bridge import CvBridge


class StereoImageSaver(Node):
    def __init__(self):
        super().__init__('stereo_image_saver')
        
        # Create output directories
        self.output_dir = Path('stereo_images_ros2')
        self.cam0_dir = self.output_dir / 'cam0'
        self.cam1_dir = self.output_dir / 'cam1'
        
        self.cam0_dir.mkdir(parents=True, exist_ok=True)
        self.cam1_dir.mkdir(parents=True, exist_ok=True)
        
        # Open timestamp files
        self.cam0_timestamps_file = open(self.output_dir / 'cam0_timestamps.txt', 'w')
        self.cam1_timestamps_file = open(self.output_dir / 'cam1_timestamps.txt', 'w')
        
        # Write headers
        self.cam0_timestamps_file.write('# timestamp_ns filename\n')
        self.cam1_timestamps_file.write('# timestamp_ns filename\n')
        
        # Image counters
        self.cam0_count = 0
        self.cam1_count = 0
        
        # Create subscribers
        self.cam0_sub = self.create_subscription(
            Image,
            '/left/image_rect',
            self.cam0_callback,
            10
        )
        
        self.cam1_sub = self.create_subscription(
            Image,
            '/right/image_rect',
            self.cam1_callback,
            10
        )
        self.bridge = CvBridge()
        
        self.get_logger().info(f'Stereo Image Saver started. Saving to: {self.output_dir.absolute()}')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  - /cam_sync/cam0/image_raw/compressed')
        self.get_logger().info('  - /cam_sync/cam1/image_raw/compressed')
    
    def cam0_callback(self, msg):
        self.save_image(msg, 'cam0', self.cam0_dir, 
                       self.cam0_timestamps_file, self.cam0_count)
        self.cam0_count += 1
        
        if self.cam0_count % 10 == 0:
            self.get_logger().info(f'Saved {self.cam0_count} images from cam0')
    
    def cam1_callback(self, msg):
        self.save_image(msg, 'cam1', self.cam1_dir, 
                       self.cam1_timestamps_file, self.cam1_count)
        self.cam1_count += 1
        
        if self.cam1_count % 10 == 0:
            self.get_logger().info(f'Saved {self.cam1_count} images from cam1')
    
    def save_image(self, msg, cam_name, output_dir, timestamp_file, count):
        # Get timestamp in nanoseconds
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        # Generate filename with left/right designation
        side = 'left' if cam_name == 'cam0' else 'right'
        cam_count = self.cam0_count if cam_name == 'cam0' else self.cam1_count
        filename = f'{cam_count}_{side}.jpg'
        filepath = output_dir / filename
        
        # Decode compressed image
        # np_arr = np.frombuffer(msg.data, np.uint8)
        # img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        if img is not None:
            # Save image
            cv2.imwrite(str(filepath), img)
            
            # Write timestamp to file
            timestamp_file.write(f'{timestamp_ns} {filename}\n')
            timestamp_file.flush()
        else:
            self.get_logger().error(f'Failed to decode image from {cam_name}')
    
    def destroy_node(self):
        # Close timestamp files
        self.cam0_timestamps_file.close()
        self.cam1_timestamps_file.close()
        self.get_logger().info(f'Total images saved - cam0: {self.cam0_count}, cam1: {self.cam1_count}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoImageSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()