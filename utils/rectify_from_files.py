import cv2
import numpy as np
import rclpy
import os

# TODO: test this script!!!!

class RectifyFromFiles():
    '''
    Batch-rectifies stereo image pairs from disk using precomputed rectification maps.

    Expects a directory structure with cam0/ and cam1/ subdirectories containing
    matching .jpg images named with nanosecond timestamps (e.g., 1234567890_left.jpg).
    Outputs rectified images and epipolar line verification plots to a save directory.
    '''
    def __init__(self, map1_x, map1_y, map2_x, map2_y, K0, K1, D0, D1):
        self.map1_x = map1_x
        self.map1_y = map1_y
        self.map2_x = map2_x
        self.map2_y = map2_y
        self.K0 = K0
        self.K1 = K1
        self.D0 = D0
        self.D1 = D1

    def rectify_all(self, save_directory, image_directory):
        self.save_directory = save_directory

        cam0_dir = os.path.join(image_directory, "cam0")
        cam1_dir = os.path.join(image_directory, "cam1")

        # Get lists of left and right images, sorted by filename
        cam0_images = sorted(glob.glob(os.path.join(cam0_dir, "*.jpg")))
        cam1_images = sorted(glob.glob(os.path.join(cam1_dir, "*.jpg")))

        # Make sure to only process the min length to stay in sync
        num_pairs = min(len(cam0_images), len(cam1_images))
        self.get_logger().info(f"Found {num_pairs} image pairs")

        self.save_directory = "/home/aaron/test_bags/stereo_images_rectified"
        os.makedirs(self.save_directory, exist_ok=True)

        for i in range(num_pairs):
            left_path = cam0_images[i]
            right_path = cam1_images[i]

            left_img = cv2.imread(left_path)
            right_img = cv2.imread(right_path)

            #extract timestamp from filename
            left_filename = os.path.basename(left_path)
            right_filename = os.path.basename(right_path)

            left_timestamp_ns = int(left_filename.split('_')[0])
            right_timestamp_ns = int(right_filename.split('_')[0])

            self.rectify_from_files(left_img, right_img, left_timestamp_ns, right_timestamp_ns)


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

