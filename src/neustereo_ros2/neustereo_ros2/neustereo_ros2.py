import cv2
import numpy as np
import torch
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo
from rclpy.node import Node
from cv_bridge import CvBridge
from NeuStereo.neustereo import NeuStereo


class NeuStereoNode(Node):
    def __init__(self):
        super().__init__('neu_stereo_node')

        # Declare parameters from config yaml file
        self.declare_parameter('display', False)
        self.declare_parameter('model_filename', '')

        # Grab parameters from file
        self.model_filename = self.get_parameter('model_filename').value
        self.display = self.get_parameter('display').value

        # Set device
        self._set_device()

        # Set publishers for disparity and depth
        self.disparity_pub = self.create_publisher(Image, '/neustereo/disparity', 10)
        self.depth_pub = self.create_publisher(Image, '/neustereo/depth', 10)

        # Initialize subscribers to rectified images
        self.left_sub = self.Subscriber(self, Image, '/cam_sync/cam0/image_rect')
        self.right_sub = self.Subscriber(self, Image, '/cam_sync/cam1/image_rect')
        self.left_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam0/rect_info')
        self.right_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam1/rect_info')

        # Use approximate time sync to initiate callback
        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub, self.left_info_sub, self.right_info_sub], queue_size=10, slop=0.025
        )
        self.sync.registerCallback(self.inference_callback)

        # Initialize cvbridge
        self.bridge = CvBridge()

    def _set_device(self):
        """
        Setting device based on if cuda is detected
        """
        # Set device for inference
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
            self.get_logger().info("CUDA detected. Using GPU for inference.")
        else:
            self.device = torch.device('cpu')
            self.get_logger().info("CUDA not detected. Using CPU for inference.")


    def _preprocess_images(self, left, right):
        """
        Preprocess images to get them ready for NeuStereo inference
        """
        # Convert from BGR --> RGB for pytorch
        left_rgb = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
        right_rgb = cv2.cvtColor(right, cv2.COLOR_BGR2RGB)

        # Convert to tensor
        left_tensor = torch.from_numpy(left_rgb).float().permute(2, 0, 1) # permute changes from (h,w,ch) --> (ch,w,h)
        right_tensor = torch.from_numpy(right_rgb).float().permute(2, 0, 1)

        left_tensor = left_tensor.unsqueeze(0) # unsqueeze to change to (ch,w,h) --> (1,ch,w,h)
        right_tensor = right_tensor.unsqueeze(0)

        return left_tensor, right_tensor


    def inference_callback(self, left, right, left_info, right_info):
        """
        Run inference on NeuStereo from callback
        """
        # Load images from msg
        left_img = self.bridge.imgmsg_to_cv2(left)
        right_img = self.bridge.imgmsg_to_cv2(right)

        # Convert image to tensors
        left_tensor, right_tensor = self._preprocess_images(left_img, right_img)

        # Feed into NeuStereo
        with torch.no_grad():
            flow_list = self.model(left_tensor.to(self.device), right_tensor.to(self.device))

        # Grab the most refined disparity from the list (last one)
        # TODO: check the output shape (maybe, Shape: [1, 1, H, W] -> [H, W])
        disparity = flow_list[-1].squeeze().cpu().numpy()
        
        # Convert disparity to depth using camera intrinsics
        depth = self.depth_from_disparity(disparity, left_info, right_info)
        
        # Publish disparity image (convert to 16-bit for precision)
        # TODO: double check this stuff below
        disparity_msg = self.bridge.cv2_to_imgmsg(
            (disparity * 256).astype(np.uint16), encoding='mono16'
        )
        disparity_msg.header = left.header
        self.disparity_pub.publish(disparity_msg)
        
        # Publish depth image (in meters, as 32-bit float)
        depth_msg = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding='32FC1')
        depth_msg.header = left.header
        self.depth_pub.publish(depth_msg)
        
        # if self.display:
        #     # Normalize disparity for visualization (assuming max ~128 pixels)
        #     disp_vis = np.clip(disparity, 0, 128)
        #     disp_vis = (disp_vis / 128.0 * 255).astype(np.uint8)
        #     disp_colormap = cv2.applyColorMap(disp_vis, cv2.COLORMAP_MAGMA)
            
        #     # Normalize depth for visualization (clip to 50m max)
        #     depth_vis = np.clip(depth, 0, 50)
        #     depth_vis = (depth_vis / 50.0 * 255).astype(np.uint8)
        #     depth_colormap = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
            
        #     cv2.imshow("Disparity", disp_colormap)
        #     cv2.imshow("Depth", depth_colormap)
        #     cv2.waitKey(1)

    def depth_from_disparity(self, disparity, left_info, right_info):
        """
        Convert disparity map to depth map using camera intrinsics.

        """
        # Extract focal length from left camera intrinsic
        fx = left_info.k[0]
        
        # Extract baseline from right camera projection matrix P
        # For right camera: Tx = -fx * baseline, so baseline = -Tx / fx
        tx = right_info.p[3] 
        baseline = -tx / right_info.p[0]  # Use right camera's fx from P
        
        # Avoid division by zero - set minimum disparity
        disparity_safe = np.maximum(disparity, 0.1)
        
        # Compute depth: Z = (f * B) / d
        depth = (fx * baseline) / disparity_safe
        
        # Set invalid regions (zero/negative disparity) to zero depth
        depth[disparity <= 0] = 0.0
        
        return depth


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = NeuStereoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
