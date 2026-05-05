import sys
import yaml
import threading
from pathlib import Path

import cv2
import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, Imu
from rclpy.time import Time
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory

workspace_root = Path(__file__).resolve().parents[3]
neuslam_path = workspace_root / 'submodules' / 'Dynamic_SLAM'
sys.path.insert(0, str(neuslam_path))

# inside Dynamic_SLAM submodule
from core.slam.slam_mp import SLAM_MP
from core.slam.slam_main import SLAM_Main
from munch import munchify

class NeuSLAM(Node):
    def __init__(self):
        super().__init__('neuslam_node')

        # parameters
        # Declare parameters from config yaml file
        self.declare_parameter('rerun_visualization', False)
        self.declare_parameter('left_cam_topic_rerun', '')
        
        # Grab parameters from file
        self.rerun_visualization = self.get_parameter('rerun_visualization').value
        self.left_cam_topic_rerun = self.get_parameter('left_cam_topic_rerun').value

        # initilaize subscribers
        self.left_sub = Subscriber(self, Image, '/cam_sync/cam0/image_rect')
        self.right_sub = Subscriber(self, Image, '/cam_sync/cam1/image_rect')
        self.left_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam0/rect_info')
        self.right_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam1/rect_info')

        # publisher
        self.pose_pub = self.create_publisher(Odometry, '/neuslam/odometry', 10)

        # sync
        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub, self.left_info_sub, self.right_info_sub], queue_size=10, slop=0.025
        )
        self.sync.registerCallback(self.slam_callback)

        # bridge for cv2
        self.bridge = CvBridge()

        # SLAM config file
        # See /slam_config/slam_cfg.yaml for more info
        pkg_share = Path(get_package_share_directory('neuslam'))
        slam_cfg_path = pkg_share / 'slam_config' / 'slam_cfg.yaml'
        with open(slam_cfg_path) as f:
            slam_cfg = yaml.safe_load(f)
        self.slam_cfg = munchify(slam_cfg)

        self._set_model_path()
        self._apply_runtime_visualization_overrides()
        
        # SLAM initialization (use slam.mp.enabled from config to select backend)
        slam_out = workspace_root / 'output' / 'slam_mc'
        slam_out.mkdir(parents=True, exist_ok=True)
        self.use_mp = bool(self.slam_cfg.slam.mp.enabled)

        if self.use_mp:
            self.get_logger().info("Using SLAM_MP (multiprocessing)")
            self.slam = SLAM_MP(self.slam_cfg, exp_dir=str(slam_out))
            self.slam.start()
        else:
            self.get_logger().info("Using SLAM_Main (single process)")
            self.slam = SLAM_Main(self.slam_cfg, exp_dir=str(slam_out))

        # image pair previous
        self.prev_pair = None

    def _set_model_path(self):
        """Resolve relative model paths against the installed models/ directory."""
        models_dir = Path(get_package_share_directory('neuslam')) / 'models'

        self.slam_cfg.model.weight_path = str(models_dir / self.slam_cfg.model.weight_path)
        self.slam_cfg.model.trt_engine_path = str(models_dir / self.slam_cfg.model.trt_engine_path)
        self.slam_cfg.slam.loop_closure.bow.vocab_path = str(models_dir / self.slam_cfg.slam.loop_closure.bow.vocab_path)

    def _apply_runtime_visualization_overrides(self):
        """Bridge ROS params to Dynamic_SLAM rerun config."""
        rerun_cfg = self.slam_cfg.slam.vis.rerun
        rerun_cfg.enabled = bool(self.rerun_visualization)
        self.get_logger().info(
            f"Dynamic_SLAM rerun logging enabled={rerun_cfg.enabled} "
            f"(serve={getattr(rerun_cfg, 'serve', False)}, spawn={getattr(rerun_cfg, 'spawn', False)})"
        )

    # def _initialize_rerun_visualization():
    #     rr.init()      

    # TODO: get rid of buffer methods if not used in code later
    def _is_buffer_initialized(self):
        # Check if image_buffer attribute exists
        # if not, create it as an empty list
        if not hasattr(self, "image_buffer"):
            self.image_buffer = []
 
        if len(self.image_buffer) == 2:
            return True

        if len(self.image_buffer) > 2:
            self.get_logger().error("Unexpected behavior: Image buffer has more than 2 elements.")
            raise RuntimeError("Image buffer overflow: more than 2 elements present.")
        return False

    # TODO: get rid of buffer methods if not used in code later
    def _add_to_buffer(self, left, right):
        if len(buffer) < 2:
            self.image_buffer.append((left, right))
        else:
            self.image_buffer.pop(0)
            self.image_buffer.append((left, right))

    def _result_reader(self):
        """Background thread: blocks on the SLAM backend output queue and
        publishes pose results as they arrive."""
        while rclpy.ok():
            try:
                result = self.slam.q_backend_out.get(timeout=1.0)
                self.get_logger().info("result drained from slam")
            except Exception:
                continue

            if not isinstance(result, dict) or result.get('cmd') != 'result':
                continue

            self._publish_pose(result)

    def _publish_pose(self, result):
        T = result['T_world_cam']   # 4x4 numpy array
        ts = result['timestamp']

        # rotation matrix to quaternion (w, x, y, z)
        R = T[:3, :3]
        t = T[:3, 3]
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'camera'
        odom.pose.pose.position.x = float(t[0])
        odom.pose.pose.position.y = float(t[1])
        odom.pose.pose.position.z = float(t[2])
        odom.pose.pose.orientation.w = float(w)
        odom.pose.pose.orientation.x = float(x)
        odom.pose.pose.orientation.y = float(y)
        odom.pose.pose.orientation.z = float(z)
        self.pose_pub.publish(odom)

    TARGET_W, TARGET_H = 768, 368

    def _resize_if_needed(self, img):
        """Resize image to (TARGET_W, TARGET_H) if dimensions don't match."""
        h, w = img.shape[:2]
        if w != self.TARGET_W or h != self.TARGET_H:
            img = cv2.resize(img, (self.TARGET_W, self.TARGET_H), interpolation=cv2.INTER_LINEAR)
        return img

    def _img_to_tensor(self, img):
        """Convert a numpy image (H,W) or (H,W,3) uint8 to a (1,3,H,W) float tensor in [0,1]."""
        import torch
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        elif img.shape[2] == 4:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        t = torch.from_numpy(img).permute(2, 0, 1).contiguous().float() / 255.0
        return t.unsqueeze(0)  # (1, 3, H, W)

    def slam_callback(self, left, right, left_info, right_info):
        self.get_logger().info("slam_callback called")

        # image buffer has to be initialized
        # if not _is_buffer_initialized:
        #     self._add_to_buffer(left, right)
        #     return

        # convert images: ROS msg -> numpy (resize) -> torch tensor (1,3,H,W) float [0,1]
        left_t = self._img_to_tensor(self._resize_if_needed(self.bridge.imgmsg_to_cv2(left)))
        right_t = self._img_to_tensor(self._resize_if_needed(self.bridge.imgmsg_to_cv2(right)))

        # if previous pair not initialized - initialize and skip callback
        if not self.prev_pair:
            self.prev_pair = (left_t, right_t, left_info, right_info)
            return

        # get prev image data
        left_prev, right_prev, left_info_prev, right_info_prev = self.prev_pair

        # timestamp
        ts = left.header.stamp.sec + left.header.stamp.nanosec * 1e-9

        # Extract K and baseline from CameraInfo as torch tensors
        import torch
        K = torch.tensor(left_info.k, dtype=torch.float32).reshape(3, 3)
        baseline = None
        if right_info.p[3] != 0.0 and right_info.p[0] != 0.0:
            baseline = torch.tensor(abs(right_info.p[3] / right_info.p[0]), dtype=torch.float32)

        if self.use_mp:
            batch = {
                "timestamp": float(ts),
                "left_t0": left_prev,
                "right_t0": right_prev,
                "left_t1": left_t,
            # use SLAM multiprocessing (runs parallel)
                "K": K,
                "baseline": baseline,
            }
            self.slam.submit(batch)

            for msg in self.slam.drain_results(non_blocking=True):
                if isinstance(msg, dict) and msg.get("cmd") == "result":
                    self._publish_pose(msg)
                    self.get_logger().info(
                        f"ts={msg.get('timestamp', 0.0):.3f} keyframe={msg.get('is_keyframe', False)} "
                        f"map_pts={msg.get('num_map_points', 0)} tracks={msg.get('num_tracks', 0)} "
                        f"fe={msg.get('frontend_stats', {})}"
                    )
                elif isinstance(msg, dict) and msg.get("cmd") == "error":
                    self.get_logger().error(f"[SLAM_MP] {msg}")
        else:
            out = self.slam(
                ts, left_prev, right_prev, left_t,
                K=K, baseline=baseline,
            )
            self._publish_pose(out)
            self.get_logger().info(
                f"ts={out['timestamp']:.3f} keyframe={out['is_keyframe']} "
                f"map_pts={out['num_map_points']} tracks={out['num_tracks']} "
                f"fe={out['frontend_stats']}"
            )

        # update previous pair
        self.prev_pair = (left_t, right_t, left_info, right_info)



def main(args=None):
    rclpy.init(args=args)
    neuslam_node = None
    try:
        neuslam_node = NeuSLAM()
        rclpy.spin(neuslam_node)
    finally:
        if neuslam_node is not None:
            neuslam_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()