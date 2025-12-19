import cv2
import sys
import numpy as np
import torch
from pathlib import Path
from easydict import EasyDict
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo
from rclpy.node import Node
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

# NeuStereo is an external git submodule - slight modification
# to pull it in from the submodules directory
workspace_root = Path(__file__).resolve().parents[3]
neustereo_path = workspace_root / 'submodules' / 'NeuStereo'
sys.path.insert(0, str(neustereo_path))

from NeuStereo.neustereo import NeuStereo
from utils.utils import load_config


class NeuStereoNode(Node):
    def __init__(self):
        super().__init__('neu_stereo_node')

        # Declare parameters from config yaml file
        self.declare_parameter('display', False)
        self.declare_parameter('model_filename', '')
        self.declare_parameter('config_file','')

        # Grab parameters from file
        model_filename_param = self.get_parameter('model_filename').value
        self.display = self.get_parameter('display').value
        config_file = self.get_parameter('config_file').value
        self.config_path = neustereo_path / 'configs' / config_file
        
        # TODO: see if this block can be simplified below
        # Resolve model filename path
        # If it's a relative path, resolve it relative to the package share directory
        if model_filename_param and not Path(model_filename_param).is_absolute():
            pkg_share = Path(get_package_share_directory('neustereo_ros2'))
            self.model_filename = str(pkg_share / model_filename_param)
        else:
            self.model_filename = model_filename_param

        # Set device
        self._set_device()

        # Load weights from the .pth file
        self._load_model()

        # Set publishers for disparity and depth
        self.disparity_pub = self.create_publisher(Image, '/neustereo/disparity', 10)
        self.depth_pub = self.create_publisher(Image, '/neustereo/depth', 10)

        # Initialize subscribers to rectified images
        self.left_sub = Subscriber(self, Image, '/cam_sync/cam0/image_rect')
        self.right_sub = Subscriber(self, Image, '/cam_sync/cam1/image_rect')
        # self.left_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam0/rect_info')
        # self.right_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam1/rect_info')

        # Use approximate time sync to initiate callback
        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=10, slop=0.025
        )
        self.sync.registerCallback(self.inference_callback)

        # Initialize cvbridge
        self.bridge = CvBridge()

    def _load_model(self):
        """
        Load the model with the desired weights
        """
        # Create the model using NeuStereo class
        # loading the config and init the model can be seen in these files
        ### https://github.com/aniket-gupta1/NeuStereo/blob/main/main.py#L185               # model loading
        ### https://github.com/aniket-gupta1/NeuStereo/blob/main/main.py#L252               # model loading
        ### https://github.com/aniket-gupta1/NeuStereo/blob/main/utils.py#L37               # config loading
        ### https://github.com/aniket-gupta1/NeuStereo/blob/main/NeuStereo/neustereo.py     # NeuStereo network
        ### https://github.com/aniket-gupta1/NeuStereo/tree/main/configs                    # config files

        # Load the model
        cfg = EasyDict(load_config(self.config_path))
        self.model = NeuStereo(cfg)
        
        # Load checkpoint 
        # See how NeuStereo is saved here: https://github.com/aniket-gupta1/NeuStereo/blob/main/train.py#L105-L107
        if self.model_filename:
            self.get_logger().info(f"Loading weights from: {self.model_filename}")
            checkpoint = torch.load(self.model_filename, map_location=self.device, weights_only=True)
            self.model.load_state_dict(checkpoint['model'])
        else:
            self.get_logger().warn("No weights file specified!")
        
        #TODO: if not a checkpoint - load appropriately
        
        # Move model to device and set eval mode for inference
        self.model.to(self.device)
        self.model.eval()
        
        # Flag to track if model dimensions have been initialized
        self.model_initialized = False


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
        Preprocess images to get them ready for NeuStereo inference.
        
        Reference: https://github.com/aniket-gupta1/NeuStereo
          - datasets/__init__.py: crop_size = (384, 768)
          - NeuStereo/neustereo.py: model expects 0-255, does /255 internally
        """
        # Orig size to upscale later
        self.original_size = (left.shape[0], left.shape[1]) # (H,W)
        self.get_logger().info(f"Original size: {self.original_size}")

        # Specify target size (H,W)
        target_size = (384, 768) #TODO: change this to pull from the NeuStereo yaml file or have it specified in the ros2 config

        # # Handle grayscale images - tile to 3 channels
        # # Reference: https://github.com/aniket-gupta1/NeuStereo/blob/main/datasets/default.py#L77-L79
        # if len(left.shape) == 2:
        #     left = np.tile(left[..., None], (1, 1, 3))
        # if len(right.shape) == 2:
        #     right = np.tile(right[..., None], (1, 1, 3))

        # Convert from BGR --> RGB for pytorch
        left_rgb = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
        right_rgb = cv2.cvtColor(right, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f"Image dtype: {left_rgb.dtype}")

        # Resize images
        left_resized = cv2.resize(left_rgb, (target_size[1], target_size[0]), interpolation=cv2.INTER_LINEAR) #TODO: check interpolation
        right_resized = cv2.resize(right_rgb, (target_size[1], target_size[0]), interpolation=cv2.INTER_LINEAR)

        # Convert to tensor
        left_tensor = torch.from_numpy(left_resized).permute(2, 0, 1).float() # permute changes from (h,w,ch) --> (ch,w,h)
        right_tensor = torch.from_numpy(right_resized).permute(2, 0, 1).float()

        left_tensor = left_tensor.unsqueeze(0) # unsqueeze to change to (ch,w,h) --> (1,ch,w,h)
        right_tensor = right_tensor.unsqueeze(0)

        combined_img = np.hstack((left, right))
        cv2.imshow("Combined Image", combined_img)
        combined_resized = np.hstack((left_resized, right_resized))
        cv2.imshow("Combined Resized Image", combined_resized)
        cv2.waitKey(1)

        return left_tensor, right_tensor


    def inference_callback(self, left, right):
        """
        Run inference on NeuStereo from callback
        """
        # Load images from msg
        left_img = self.bridge.imgmsg_to_cv2(left)
        right_img = self.bridge.imgmsg_to_cv2(right)

        # Convert image to tensors
        left_tensor, right_tensor = self._preprocess_images(left_img, right_img)

        # Initialize model dimensions on first run
        if not self.model_initialized:
            batch_size = left_tensor.shape[0]
            height = left_tensor.shape[2]
            width = left_tensor.shape[3]
            # amp=True uses half precision (float16) internally
            self.model.init_bhwd(batch_size, height, width, self.device, amp=True)
            self.model_initialized = True
            self.get_logger().info(f"Initialized model with dims: batch={batch_size}, H={height}, W={width}")

        # Feed into NeuStereo with half precision (matching evaluation code)
        # Convert to half precision and use autocast for proper AMP inference
        # Reference: https://github.com/aniket-gupta1/NeuStereo/blob/experimental/evaluate_stereo.py#L294-L298
        left_half = left_tensor.to(self.device).half()
        right_half = right_tensor.to(self.device).half()
        
        # reference: https://github.com/aniket-gupta1/NeuStereo/blob/experimental/evaluate_stereo.py#L306-L313
        with torch.no_grad(), torch.amp.autocast('cuda', enabled=True):
            flow_list = self.model(left_half, right_half)

        # Grab the most refined disparity from the list (last one)
        # Shape: [1, 1, H, W] -> [H, W]
        disparity = flow_list[-1].squeeze().cpu().numpy()  # Keep as float!
        self.get_logger().info(f"Disparity range: {disparity.min()} to {disparity.max()}")
        
        # Convert disparity to depth using camera intrinsics
        # depth = self.depth_from_disparity(disparity, left_info, right_info)
        
        # Publish disparity image (convert to 16-bit for precision)
        # TODO: double check this stuff below
        disparity_msg = self.bridge.cv2_to_imgmsg(
            (disparity * 256).astype(np.uint16), encoding='mono16'
        )
        disparity_msg.header = left.header
        self.disparity_pub.publish(disparity_msg)
        
        # Publish depth image (in meters, as 32-bit float)
        # depth_msg = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding='32FC1')
        # depth_msg.header = left.header
        # self.depth_pub.publish(depth_msg)

        
        if self.display:
            # Normalize disparity for visualization
            # KITTI max disparity is ~256, adjust based on your data
            # max_disp = 256.0
            # disp_vis = np.clip(disparity, 0, max_disp)
            # disp_vis = (disp_vis / max_disp * 255).astype(np.uint8)
            disp_vis = disparity.astype(np.uint8)
            disp_colormap = cv2.applyColorMap(disp_vis, cv2.COLORMAP_VIRIDIS)
            
        #     # Normalize depth for visualization (clip to 50m max)
        #     depth_vis = np.clip(depth, 0, 50)
        #     depth_vis = (depth_vis / 50.0 * 255).astype(np.uint8)
        #     depth_colormap = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
            
            cv2.imshow("Disparity", disp_colormap)
        #     cv2.imshow("Depth", depth_colormap)
            cv2.waitKey(1)

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
