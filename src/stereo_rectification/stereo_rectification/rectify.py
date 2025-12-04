import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import glob
import os
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend


class RectifyStereoImgs(Node):
    def __init__(self):
        super().__init__('rectify_stereo_imgs')

        # left and right camera parameters
        self.declare_parameter('cam0.intrinsics', [0.0])      # [fx, fy, cx, cy]
        self.declare_parameter('cam0.resolution', [0])      # [width, height]
        self.declare_parameter('cam0.distortion_coeffs', [0.0])
        self.declare_parameter('cam0.distortion_model', '')
        self.declare_parameter('cam1.intrinsics', [0.0])
        self.declare_parameter('cam1.resolution', [0])
        self.declare_parameter('cam1.distortion_coeffs', [0.0])
        self.declare_parameter('cam1.distortion_model', '')
        self.declare_parameter('cam1.T_cn_cnm1', [0.0])      # 4x4 extrinsic matrix
        
        # extract values from params
        cam0_intrinsics = self.get_parameter('cam0.intrinsics').value
        cam0_resolution = self.get_parameter('cam0.resolution').value
        cam0_distortion = self.get_parameter('cam0.distortion_coeffs').value
        cam0_dist_model = self.get_parameter('cam0.distortion_model').value
        cam1_intrinsics = self.get_parameter('cam1.intrinsics').value
        cam1_resolution = self.get_parameter('cam1.resolution').value
        cam1_distortion = self.get_parameter('cam1.distortion_coeffs').value
        cam1_dist_model = self.get_parameter('cam1.distortion_model').value
        T_cam1_cam0 = self.get_parameter('cam1.T_cn_cnm1').value  # flattened 4x4 matrix


        # camera0 and 1 calibration instrinsics
        self.K0 = np.array([[cam0_intrinsics[0], 0, cam0_intrinsics[2]],
                             [0, cam0_intrinsics[1], cam0_intrinsics[3]],
                             [0, 0, 1]], dtype=np.float64)
        self.D0 = np.array(cam0_distortion, dtype=np.float64)
  
        self.K1 = np.array([[cam1_intrinsics[0], 0, cam1_intrinsics[2]],
                             [0, cam1_intrinsics[1], cam1_intrinsics[3]],
                             [0, 0, 1]], dtype=np.float64)
        self.D1 = np.array(cam1_distortion, dtype=np.float64)
        self.T_cn_cnm1 = np.array(T_cam1_cam0, dtype=np.float64).reshape(4, 4) 

        # Use resolution from parameters (width, height)
        self.image_size = tuple(cam0_resolution)
        self._compute_maps()
        # self._compute_fundamental_matrix()
        
        # flag to track if plotted epipolar lines for first image
        self.epipolar_plotted = False

        # publishing rectified images
        self.pub_left_rect = self.create_publisher(Image, '/cam_sync/cam0/image_rect', 10)
        self.pub_right_rect = self.create_publisher(Image, '/cam_sync/cam1/image_rect', 10)
        self.pub_left_info = self.create_publisher(CameraInfo, '/cam_sync/cam0/rect_info', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/cam_sync/cam1/rect_info', 10)

        self.visualization = False

        # TODO: change this into a separate python script or break off into another function
        # image_directory = "/home/aaron/test_bags/stereo_images"

        # self.bridge = CvBridge()
        # cam0_dir = os.path.join(image_directory, "cam0")
        # cam1_dir = os.path.join(image_directory, "cam1")

        # # Get lists of left and right images, sorted by filename
        # cam0_images = sorted(glob.glob(os.path.join(cam0_dir, "*.jpg")))
        # cam1_images = sorted(glob.glob(os.path.join(cam1_dir, "*.jpg")))

        # # Make sure to only process the min length to stay in sync
        # num_pairs = min(len(cam0_images), len(cam1_images))
        # self.get_logger().info(f"Found {num_pairs} image pairs")

        # self.save_directory = "/home/aaron/test_bags/stereo_images_rectified"
        # os.makedirs(self.save_directory, exist_ok=True)

        # for i in range(num_pairs):
        #     left_path = cam0_images[i]
        #     right_path = cam1_images[i]

        #     left_img = cv2.imread(left_path)
        #     right_img = cv2.imread(right_path)

        #     #extract timestamp from filename
        #     left_filename = os.path.basename(left_path)
        #     right_filename = os.path.basename(right_path)

        #     left_timestamp_ns = int(left_filename.split('_')[0])
        #     right_timestamp_ns = int(right_filename.split('_')[0])

        #     self.rectify_from_files(left_img, right_img, left_timestamp_ns, right_timestamp_ns)

        
        # grab left and right images and calibration data
        self.left_img = Subscriber(self, CompressedImage, '/cam_sync/cam0/image_raw/compressed')
        self.right_img = Subscriber(self, CompressedImage, '/cam_sync/cam1/image_raw/compressed')

        # sync messages
        self.sync = ApproximateTimeSynchronizer([self.left_img, self.right_img],queue_size=20,slop=0.01)
        self.sync.registerCallback(self.rectify)
        self.bridge = CvBridge()

    def recalibrate(self, left, right):
        NotImplementedError()

    def _compute_fundamental_matrix(self):
        # rotation and translation 
        R = self.T_cn_cnm1[:3, :3]
        T = self.T_cn_cnm1[:3, 3]
        
        # skew-symmetric matrix of translation vector
        T_skew = np.array([
            [0, -T[2], T[1]],
            [T[2], 0, -T[0]],
            [-T[1], T[0], 0]
        ])
        
        # Fundamental matrix: F = (K1^-T) * [T]_x * R * K0^-1
        K0_inv = np.linalg.inv(self.K0)
        K1_inv_T = np.linalg.inv(self.K1).T
        self.F = K1_inv_T @ T_skew @ R @ K0_inv

        return self.F

    def _plot_epipolar_lines(self, left_orig, right_orig, left_rect, right_rect, left_timestamp_ns):
        """Plot epipolar lines from 3 consistent points across all 4 images."""
        # Select 3 points in the left image (well-distributed)
        h, w = left_orig.shape[:2]
        points_left = np.array([
            [w * 0.25, h * 0.33],  # Top-left region
            [w * 0.5, h * 0.5],    # Center
            [w * 0.75, h * 0.67]   # Bottom-right region
        ], dtype=np.float32)
        
        # Colors for the 3 points
        colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255)]  # Green, Red, Blue
        
        # Create copies for drawing
        left_orig_vis = left_orig.copy()
        right_orig_vis = right_orig.copy()
        left_rect_vis = left_rect.copy()
        right_rect_vis = right_rect.copy()
        
        # For unrectified images: compute epipolar lines using fundamental matrix
        lines_right = cv2.computeCorrespondEpilines(points_left.reshape(-1, 1, 2), 1, self.F)
        lines_right = lines_right.reshape(-1, 3)
        
        # Draw points and epipolar lines on unrectified images
        for i, (pt, line, color) in enumerate(zip(points_left, lines_right, colors)):
            # Draw point in left image
            pt_int = tuple(map(int, pt))
            cv2.circle(left_orig_vis, pt_int, 8, color, -1)
            cv2.circle(left_orig_vis, pt_int, 8, (255, 255, 255), 1)
            
            # Draw epipolar line in right image
            a, b, c = line
            if abs(b) > 1e-6:
                x0, y0  = 0, int(-c / b)
                x1, y1 = w, int(-(c + a * w) / b)
            else:
                x0, y0 = int(-c / a), 0
                x1, y1 = int(-c / a), h
            
            # Clip to image bounds
            x0 = max(0, min(w, x0))
            y0 = max(0, min(h, y0))
            x1 = max(0, min(w, x1))
            y1 = max(0, min(h, y1))
            
            cv2.line(right_orig_vis, (x0, y0), (x1, y1), color, 2)
        
        # For rectified images: epipolar lines are horizontal
        # Map the points from original to rectified coordinates using undistortPoints
        # When P is provided, undistortPoints returns coordinates directly in the rectified image pixel space
        points_left_2d = points_left.reshape(-1, 1, 2).astype(np.float32)
        points_left_rect = cv2.undistortPoints(points_left_2d, self.K0, self.D0, R=self.R1, P=self.P1)
        points_left_rect = points_left_rect.reshape(-1, 2)
        
        # Draw points and horizontal epipolar lines on rectified images
        for i, (pt, color) in enumerate(zip(points_left_rect, colors)):
            # Draw point in left rectified image
            pt_int = tuple(map(int, pt))
            cv2.circle(left_rect_vis, pt_int, 8, color, -1)
            cv2.circle(left_rect_vis, pt_int, 8, (255, 255, 255), 2)
            
            # Draw horizontal epipolar line in both rectified images
            y = int(pt[1])
            cv2.line(left_rect_vis, (0, y), (w, y), color, 2)
            cv2.line(right_rect_vis, (0, y), (w, y), color, 2)
        
        # Convert BGR to RGB for matplotlib
        left_orig_vis = cv2.cvtColor(left_orig_vis, cv2.COLOR_BGR2RGB)
        right_orig_vis = cv2.cvtColor(right_orig_vis, cv2.COLOR_BGR2RGB)
        left_rect_vis = cv2.cvtColor(left_rect_vis, cv2.COLOR_BGR2RGB)
        right_rect_vis = cv2.cvtColor(right_rect_vis, cv2.COLOR_BGR2RGB)
        
        # Create side-by-side figure for unrectified images
        fig1, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        ax1.imshow(left_orig_vis)
        ax1.set_title('Left Original (Unrectified)', fontsize=14)
        ax1.axis('off')
        ax2.imshow(right_orig_vis)
        ax2.set_title('Right Original (Unrectified)', fontsize=14)
        ax2.axis('off')
        plt.tight_layout()
        
        # Save unrectified epipolar lines figure
        unrect_path = os.path.join(self.save_directory, f"{left_timestamp_ns}_epipolar_unrectified.png")
        plt.savefig(unrect_path, dpi=150, bbox_inches='tight')
        plt.close(fig1)
        self.get_logger().info(f"Saved unrectified epipolar lines: {unrect_path}")
        
        # Create side-by-side figure for rectified images
        fig2, (ax3, ax4) = plt.subplots(1, 2, figsize=(16, 8))
        ax3.imshow(left_rect_vis)
        ax3.set_title('Left Rectified', fontsize=14)
        ax3.axis('off')
        ax4.imshow(right_rect_vis)
        ax4.set_title('Right Rectified', fontsize=14)
        ax4.axis('off')
        plt.tight_layout()
        
        # Save rectified epipolar lines figure
        rect_path = os.path.join(self.save_directory, f"{left_timestamp_ns}_epipolar_rectified.png")
        plt.savefig(rect_path, dpi=150, bbox_inches='tight')
        plt.close(fig2)
        self.get_logger().info(f"Saved rectified epipolar lines: {rect_path}")

    def _compute_maps(self):
        # decompose T_cn_cnm1
        R = self.T_cn_cnm1[:3, :3]
        T = self.T_cn_cnm1[:3, 3]

        # cv2 to stereo rect
        self.R1, self.R2, self.P1, self.P2, self.Q, validPixROI1, validPixROI2 = cv2.stereoRectify(self.K0, self.D0, self.K1, self.D1, self.image_size, R, T)
        
        # compute maps - used later in cv2.remap
        self.map1_x, self.map1_y = cv2.initUndistortRectifyMap(self.K0, self.D0, self.R1, self.P1, self.image_size, cv2.CV_32FC1)
        self.map2_x, self.map2_y = cv2.initUndistortRectifyMap(self.K1, self.D1, self.R2, self.P2, self.image_size, cv2.CV_32FC1)

    def rectify_from_files(self, left, right, left_timestamp_ns, right_timestamp_ns):
        # rectify left and right images
        rect_l = cv2.remap(left, self.map1_x, self.map1_y, cv2.INTER_LINEAR)
        rect_r = cv2.remap(right, self.map2_x, self.map2_y, cv2.INTER_LINEAR)

        # Plot epipolar lines for the first image pair only
        if not self.epipolar_plotted:
            self._plot_epipolar_lines(left, right, rect_l, rect_r, left_timestamp_ns)
            self.epipolar_plotted = True

        # convert timestamp nanoseconds to rclpy time
        left_timestamp = rclpy.time.Time(seconds=0, nanoseconds=left_timestamp_ns)
        right_timestamp = rclpy.time.Time(seconds=0, nanoseconds=right_timestamp_ns)

        # save rectified images
        left_rect_path = os.path.join(self.save_directory, f"{left_timestamp_ns}_left_rect.jpg")
        right_rect_path = os.path.join(self.save_directory, f"{right_timestamp_ns}_right_rect.jpg")
        cv2.imwrite(left_rect_path, rect_l)
        cv2.imwrite(right_rect_path, rect_r)

    def rectify(self, left, right):
        callback_start = time.perf_counter()
        
        # grab left and right images
        left_img = self.bridge.compressed_imgmsg_to_cv2(left)
        right_img = self.bridge.compressed_imgmsg_to_cv2(right)
        
        # rectify left and right images
        rect_l = cv2.remap(left_img, self.map1_x, self.map1_y, cv2.INTER_LINEAR)
        rect_r = cv2.remap(right_img, self.map2_x, self.map2_y, cv2.INTER_LINEAR)

        # grab left time stamp to sync publishing
        time_sync_stamp_left = left.header.stamp
        time_sync_stamp_right = right.header.stamp

        # publish left & right rectified images
        # specify encoding for other camera inputs - for supporting other cameras
        encoding = 'bgr8' if len(rect_l.shape) == 3 else 'mono8'
        left_rect_msg = self.bridge.cv2_to_imgmsg(rect_l, encoding=encoding)
        left_rect_msg.header.stamp = time_sync_stamp_left
        left_rect_msg.header.frame_id = 'cam0_rect'
        self.pub_left_rect.publish(left_rect_msg)

        right_rect_msg = self.bridge.cv2_to_imgmsg(rect_r, encoding=encoding)
        right_rect_msg.header.stamp = time_sync_stamp_right
        right_rect_msg.header.frame_id = 'cam1_rect'
        self.pub_right_rect.publish(right_rect_msg)

        # publish left and right camera info
        rect_info_left = CameraInfo()
        rect_info_left.width = self.image_size[0]
        rect_info_left.height = self.image_size[1]
        rect_info_left.p = self.P1.flatten().tolist()
        rect_info_left.r = self.R1.flatten().tolist()
        rect_info_left.k = self.P1[:3, :3].flatten().tolist()
        rect_info_left.d = [0.0, 0.0, 0.0, 0.0, 0.0] # distortion ideally zero after rectification
        rect_info_left.distortion_model = 'plumb_bob'
        rect_info_left.header.stamp = time_sync_stamp_left
        rect_info_left.header.frame_id = 'cam0_rect'
        self.pub_left_info.publish(rect_info_left)

        rect_info_right = CameraInfo()
        rect_info_right.width = self.image_size[0]
        rect_info_right.height = self.image_size[1]
        rect_info_right.p = self.P2.flatten().tolist()
        rect_info_right.r = self.R2.flatten().tolist()
        rect_info_right.k = self.P2[:3, :3].flatten().tolist()
        rect_info_right.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        rect_info_right.distortion_model = 'plumb_bob'
        rect_info_right.header.stamp = time_sync_stamp_right
        rect_info_right.header.frame_id = 'cam1_rect'
        self.pub_right_info.publish(rect_info_right)

        if self.visualization:
            # display unrectified pair
            unrect_stereo_pair = np.hstack((left_img, right_img))
            cv2.imshow('Unrectified Stereo Pair', unrect_stereo_pair)
            cv2.waitKey(1)
            
            # display rectified pair
            stereo_pair = np.hstack((rect_l, rect_r))
            cv2.imshow('Rectified Stereo Pair', stereo_pair)
            cv2.waitKey(1)

        # Log callback execution time
        callback_elapsed_ms = (time.perf_counter() - callback_start) * 1000
        self.get_logger().info(f"Rectify callback time: {callback_elapsed_ms:.2f} ms")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = RectifyStereoImgs()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  