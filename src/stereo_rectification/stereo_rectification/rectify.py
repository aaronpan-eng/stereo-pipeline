import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
import rerun as rr
import rerun.blueprint as rrb
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
        self.declare_parameter('cam0.rostopic', '')
        self.declare_parameter('cam0.topic_type', '')

        self.declare_parameter('cam1.intrinsics', [0.0])
        self.declare_parameter('cam1.resolution', [0])
        self.declare_parameter('cam1.distortion_coeffs', [0.0])
        self.declare_parameter('cam1.distortion_model', '')
        self.declare_parameter('cam1.T_cn_cnm1', [0.0])      # 4x4 extrinsic matrix
        self.declare_parameter('cam1.rostopic', '')
        self.declare_parameter('cam1.topic_type', '')

        # extract values from params
        self.cam0_intrinsics = self.get_parameter('cam0.intrinsics').value
        self.cam0_resolution = self.get_parameter('cam0.resolution').value
        self.cam0_distortion = self.get_parameter('cam0.distortion_coeffs').value
        self.cam0_dist_model = self.get_parameter('cam0.distortion_model').value
        self.cam0_rostopic = self.get_parameter('cam0.rostopic').value
        self.cam0_topic_type = self.get_parameter('cam0.topic_type').value

        self.cam1_intrinsics = self.get_parameter('cam1.intrinsics').value
        self.cam1_resolution = self.get_parameter('cam1.resolution').value
        self.cam1_distortion = self.get_parameter('cam1.distortion_coeffs').value
        self.cam1_dist_model = self.get_parameter('cam1.distortion_model').value
        self.cam1_rostopic = self.get_parameter('cam1.rostopic').value
        self.cam1_topic_type = self.get_parameter('cam1.topic_type').value
        self.T_cam1_cam0 = self.get_parameter('cam1.T_cn_cnm1').value  # flattened 4x4 matrix


        # camera0 and 1 calibration instrinsics
        self.K0 = np.array([[self.cam0_intrinsics[0], 0, self.cam0_intrinsics[2]],
                             [0, self.cam0_intrinsics[1], self.cam0_intrinsics[3]],
                             [0, 0, 1]], dtype=np.float64)
        self.D0 = np.array(self.cam0_distortion, dtype=np.float64)
  
        self.K1 = np.array([[self.cam1_intrinsics[0], 0, self.cam1_intrinsics[2]],
                             [0, self.cam1_intrinsics[1], self.cam1_intrinsics[3]],
                             [0, 0, 1]], dtype=np.float64)
        self.D1 = np.array(self.cam1_distortion, dtype=np.float64)
        self.T_cn_cnm1 = np.array(self.T_cam1_cam0, dtype=np.float64).reshape(4, 4) 

        # Use resolution from parameters (width, height)
        self.image_size = tuple(self.cam0_resolution)
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
        if self.visualization:
            self._initialize_rerun()
        
        # Dictionary to map the strings from the config to the actual message type
        topic_map = {
            'CompressedImage': CompressedImage,
            'Image': Image,
        }

        # grab left and right images and calibration data
        self.left_img = Subscriber(self, topic_map[self.cam0_topic_type], self.cam0_rostopic)
        self.right_img = Subscriber(self, topic_map[self.cam1_topic_type], self.cam1_rostopic)

        # sync messages
        self.sync = ApproximateTimeSynchronizer([self.left_img, self.right_img],queue_size=20,slop=0.01)
        self.sync.registerCallback(self.rectify)
        self.bridge = CvBridge()

    def recalibrate(self, left, right):
        NotImplementedError()

    def _initialize_rerun(self):
        rr.init('stereo_rectification', strict=True, spawn=True)
        rr.send_blueprint(rrb.Blueprint(
            rrb.TimePanel(state="collapsed"),
            rrb.Vertical(
                row_shares=[0.5, 0.5],
                contents=[
                    rrb.Spatial2DView(
                        name='unreftified stereo pair', 
                        origin='/stereo_rectification/unrectified'
                    ),
                    rrb.Spatial2DView(
                        name='rectified stereo pair', 
                        origin='/stereo_rectification/rectified'
                    ),
                ]
            )
        ))
    
    def _parse_pckl(self, file):
        NotImplementedError() #TODO: add support format for intrinsic format in .pckl to convert to ros_parameters file

    def _parse_kalibr(self, file):
        NotImplementedError() #TODO: add support format for intrinsic format in kalibr .yaml to convert to ros_parameters file

    def _compute_maps(self):
        # decompose T_cn_cnm1
        R = self.T_cn_cnm1[:3, :3]
        T = self.T_cn_cnm1[:3, 3]

        # cv2 to stereo rect
        self.R1, self.R2, self.P1, self.P2, self.Q, validPixROI1, validPixROI2 = cv2.stereoRectify(self.K0, self.D0, self.K1, self.D1, self.image_size, R, T)
        
        # compute maps - used later in cv2.remap
        self.map1_x, self.map1_y = cv2.initUndistortRectifyMap(self.K0, self.D0, self.R1, self.P1, self.image_size, cv2.CV_32FC1)
        self.map2_x, self.map2_y = cv2.initUndistortRectifyMap(self.K1, self.D1, self.R2, self.P2, self.image_size, cv2.CV_32FC1)

        self.map1_x_gpu = cv2.cuda_GpuMat()
        self.map1_x_gpu.upload(self.map1_x)
        self.map1_y_gpu = cv2.cuda_GpuMat()
        self.map1_y_gpu.upload(self.map1_y)
        self.map2_x_gpu = cv2.cuda_GpuMat()
        self.map2_x_gpu.upload(self.map2_x)
        self.map2_y_gpu = cv2.cuda_GpuMat()
        self.map2_y_gpu.upload(self.map2_y)

    def rectify_cv2_cuda(self, left, right):
        # Upload images to GPU
        left_gpumat = cv2.cuda_GpuMat()
        left_gpumat.upload(left)
        right_gpumat = cv2.cuda_GpuMat()
        right_gpumat.upload(right)

        # Rectify using CUDA remap with GPU maps
        rect_l_gpumat = cv2.cuda.remap(left_gpumat, self.map1_x_gpu, self.map1_y_gpu, cv2.INTER_LINEAR)
        rect_r_gpumat = cv2.cuda.remap(right_gpumat, self.map2_x_gpu, self.map2_y_gpu, cv2.INTER_LINEAR)

        # Download results from GPU
        rect_l = rect_l_gpumat.download()
        rect_r = rect_r_gpumat.download()

        return rect_l, rect_r

    def rectify(self, left, right):
        callback_start = time.perf_counter()
        
        # grab left and right images
        if self.cam0_topic_type == 'CompressedImage':
            left_img = self.bridge.compressed_imgmsg_to_cv2(left)
            right_img = self.bridge.compressed_imgmsg_to_cv2(right)
        elif self.cam1_topic_type == 'Image':
            left_img = self.bridge.imgmsg_to_cv2(left)
            right_img = self.bridge.imgmsg_to_cv2(right)
        else:
            self.get_logger().error(f"Unsupported topic type: {self.cam0_topic_type} or {self.cam1_topic_type}")
            return

        # rectify using cv2 with cuda support
        rect_l, rect_r = self.rectify_cv2_cuda(left_img, right_img)

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
            # display unrectified pair to rerun
            unrect_stereo_pair = np.hstack((left_img, right_img))
            rr.log('/stereo_rectification/unrectified', rr.Image(unrect_stereo_pair).compress(jpeg_quality=80))
            
            # display rectified pair to rerun
            stereo_pair = np.hstack((rect_l, rect_r))
            rr.log('/stereo_rectification/rectified', rr.Image(stereo_pair).compress(jpeg_quality=80))

        # Log callback execution time
        callback_elapsed_ms = (time.perf_counter() - callback_start) * 1000
        # self.get_logger().info(f"Rectify callback time: {callback_elapsed_ms:.2f} ms")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = RectifyStereoImgs()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  