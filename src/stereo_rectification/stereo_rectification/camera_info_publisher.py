import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

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

        # Transformation from cam1 to cam0 (extrinsics)
        self.T_cn_cnm1 = np.array([
            [0.999987780014801, -0.004197569715124891, 0.0026115569225214365, -0.22201818214496324],
            [0.004192243832357699, 0.9999891286372057, 0.0020414943191810224, 3.57306524280388e-05],
            [-0.002620097846066464, -0.0020305210887493633, 0.9999945060206022, 0.001029531535409571],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Extract rotation and translation
        self.R = self.T_cn_cnm1[:3, :3]
        self.T = self.T_cn_cnm1[:3, 3]

        # Image dimensions (will be set from first image)
        self.image_width = None
        self.image_height = None

        # Publishers for raw images and CameraInfo
        self.pub_left_image = self.create_publisher(Image, '/cam_sync/cam0/image_raw', 10)
        self.pub_left_info = self.create_publisher(CameraInfo, '/cam_sync/cam0/camera_info', 10)
        self.pub_right_image = self.create_publisher(Image, '/cam_sync/cam1/image_raw', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/cam_sync/cam1/camera_info', 10)
        
        self.get_logger().info('Publishing on topics:')
        self.get_logger().info('  /cam_sync/cam0/image_raw')
        self.get_logger().info('  /cam_sync/cam0/camera_info')
        self.get_logger().info('  /cam_sync/cam1/image_raw')
        self.get_logger().info('  /cam_sync/cam1/camera_info')

        # Subscribers for images (to sync CameraInfo with images)
        self.left_img_sub = Subscriber(self, CompressedImage, '/cam_sync/cam0/image_raw/compressed')
        self.right_img_sub = Subscriber(self, CompressedImage, '/cam_sync/cam1/image_raw/compressed')

        # Synchronize messages
        self.sync = ApproximateTimeSynchronizer(
            [self.left_img_sub, self.right_img_sub], 
            queue_size=10, 
            slop=0.02
        )
        self.sync.registerCallback(self.image_callback)
        
        self.bridge = CvBridge()

        self.get_logger().info('Camera Info Publisher started')

    def create_camera_info_msg(self, K, D, R, P, width, height, frame_id, stamp):
        """
        Create a CameraInfo message from calibration parameters.
        
        Args:
            K: 3x3 camera intrinsic matrix
            D: distortion coefficients (4 or 5 elements for plumb_bob)
            R: 3x3 rectification matrix (identity for unrectified)
            P: 3x4 projection matrix
            width: image width
            height: image height
            frame_id: camera frame ID
            stamp: timestamp from image
        """
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = frame_id
        
        camera_info.width = width
        camera_info.height = height
        
        # Distortion model (using plumb_bob for radial-tangential distortion)
        camera_info.distortion_model = 'plumb_bob'
        
        # Flatten and convert to list
        camera_info.d = D.flatten().tolist()
        camera_info.k = K.flatten().tolist()
        camera_info.r = R.flatten().tolist()
        camera_info.p = P.flatten().tolist()
        
        # Binning (no binning)
        camera_info.binning_x = 0
        camera_info.binning_y = 0
        
        # ROI (region of interest, use full image)
        camera_info.roi.do_rectify = False
        
        return camera_info

    def image_callback(self, left_msg, right_msg):
        """
        Callback when synchronized left and right compressed images are received.
        Decompresses images and publishes both raw images and CameraInfo.
        """
        # Decompress images
        left_img = self.bridge.compressed_imgmsg_to_cv2(left_msg)
        right_img = self.bridge.compressed_imgmsg_to_cv2(right_msg)
        
        # Get image dimensions from first image
        if self.image_width is None or self.image_height is None:
            self.image_height, self.image_width = left_img.shape[:2]
            self.get_logger().info(f'Image dimensions: {self.image_width}x{self.image_height}')

        # Determine encoding
        encoding = 'bgr8' if len(left_img.shape) == 3 else 'mono8'

        # Convert to ROS Image messages
        left_image_msg = self.bridge.cv2_to_imgmsg(left_img, encoding=encoding)
        left_image_msg.header.stamp = left_msg.header.stamp
        left_image_msg.header.frame_id = 'left_camera'
        
        right_image_msg = self.bridge.cv2_to_imgmsg(right_img, encoding=encoding)
        right_image_msg.header.stamp = right_msg.header.stamp
        right_image_msg.header.frame_id = 'right_camera'

        # For unrectified images, R is identity and P is [K | 0]
        R0 = np.eye(3)
        P0 = np.zeros((3, 4))
        P0[:3, :3] = self.K0
        
        R1 = np.eye(3)
        P1 = np.zeros((3, 4))
        P1[:3, :3] = self.K1

        # Create CameraInfo messages for both cameras
        left_info = self.create_camera_info_msg(
            self.K0, self.D0, R0, P0,
            self.image_width, self.image_height,
            'left_camera', left_msg.header.stamp
        )
        
        right_info = self.create_camera_info_msg(
            self.K1, self.D1, R1, P1,
            self.image_width, self.image_height,
            'right_camera', right_msg.header.stamp
        )

        # Publish raw images and CameraInfo
        self.pub_left_image.publish(left_image_msg)
        self.pub_left_info.publish(left_info)
        self.pub_right_image.publish(right_image_msg)
        self.pub_right_info.publish(right_info)
        # self.get_logger().info('Published images and CameraInfo')


def main(args=None):
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisher()
    
    try:
        rclpy.spin(camera_info_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_info_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

