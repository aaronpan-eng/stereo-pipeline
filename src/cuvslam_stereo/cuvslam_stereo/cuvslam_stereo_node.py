import numpy as np
import cuvslam
import rclpy
import time
import rerun as rr
import rerun.blueprint as rrb
import datetime
import cv2
import matplotlib.pyplot as plt

from pathlib import Path
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, Imu
from rclpy.time import Time
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from cuvslam_stereo.utils import combine_poses, transform_landmarks
import numpy as np


class CuvslamStereo(Node):
    def __init__(self):
        super().__init__('cuvslam_stereo_node')

        # Declare parameters from config yaml file
        self.declare_parameter('rerun_visualization', False)
        self.declare_parameter('left_cam_topic_rerun', '')
        self.declare_parameter('save_trajectory_tum', False)
        self.declare_parameter('visualization_2d', False)
        self.declare_parameter('use_imu', False)
        # Grab parameters from file
        self.rerun_visualization = self.get_parameter('rerun_visualization').value
        self.left_cam_topic_rerun = self.get_parameter('left_cam_topic_rerun').value
        self.save_trajectory_tum = self.get_parameter('save_trajectory_tum').value
        self.visualization_2d = self.get_parameter('visualization_2d').value
        self.use_imu = self.get_parameter('use_imu').value
        # TODO: add VIO to this node

        # Setup rerun visualization
        # like in kitti demo, the left camera is chosen for visualization
        if self.rerun_visualization:
            self._initialize_rerun_visualization(self.left_cam_topic_rerun)

        # To save trajectory data to csv upon node shutdown
        self.trajectory = []
        self.trajectory_slam = []
        self.odom_pose_estimates = []
        self.slam_poses = []
        self.loop_closure_poses = []
        
        # To initialize tracker once upon first image pair callback
        self.initialize_tracker = False

        # parameters for trakcer
        self.use_imu = False
        
        # initilaize subscribers
        self.left_sub = Subscriber(self, Image, '/cam_sync/cam0/image_rect')
        self.right_sub = Subscriber(self, Image, '/cam_sync/cam1/image_rect')
        self.left_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam0/rect_info')
        self.right_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam1/rect_info')
        
        # self.imu_sub = Subscriber(self, Imu, '/imu/data', 10)

        # initialize publishers
        self.pose_pub = self.create_publisher(Odometry, '/cuvslam/odometry', 10)

        # synchronize the left and right image streams and feed them to slam_callback
        # i chose slop to be 0.025 because that is half the publishing freq 20hz, can be slightly higher or lower
        # but shouldnt exceed publishing time gap (0.05) or else messages could get mixed
        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub, self.left_info_sub, self.right_info_sub], queue_size=10, slop=0.025
        )
        self.sync.registerCallback(self.slam_callback)

        self.bridge = CvBridge()

        if self.visualization_2d:
            plt.ion()
            self.traj_fig, self.traj_ax = plt.subplots()
            self.traj_ax.set_title('Cuvslam Trajectory')
            self.traj_ax.set_xlabel('X (m)')
            self.traj_ax.set_ylabel('Z (m)')
            self.traj_ax.set_aspect('equal', adjustable='datalim')
            self.traj_line, = self.traj_ax.plot([], [], 'b-')
            self.traj_x = []
            self.traj_z = []
    

    def _color_from_id(self, identifier): 
        return [(identifier * 17) % 256, (identifier * 31) % 256, (identifier * 47) % 256]


    def _initialize_rerun_visualization(self, cam_name):
        # Setup rerun visualizer
        rr.init('cuvslam_stereo_node', strict=True, spawn=True)  # launch re-run instance

        # Setup rerun views
        rr.send_blueprint(rrb.Blueprint(
            rrb.TimePanel(state="collapsed"),
            rrb.Vertical(
                row_shares=[0.6, 0.4],
                contents=[rrb.Spatial3DView(), rrb.Spatial2DView(origin=cam_name)]
            )
        ))

        # Setup coordinate basis for root, cuvslam uses right-hand system with X-right, Y-down, Z-forward
        rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

        # Draw arrays in origin X-red, Y-green, Z-blue
        rr.log("xyz", rr.Arrows3D(
            vectors=[[50, 0, 0], [0, 50, 0], [0, 0, 50]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            labels=['[x]', '[y]', '[z]']
        ), static=True)
        
        self.get_logger().info("Rerun visualization initialized")

    def _initialize_cuvslam_from_camera_info(self, left_info, right_info):
        # grab camera width and height
        width_l = left_info.width
        height_l = left_info.height
        # note: if the left and right cameras arent identical you would add a different for right
        
        # Store for visualization
        self.width = width_l
        self.height = height_l
        
        # grab camera k matrix from rectification
        P1 = np.array(left_info.p).reshape(3, 4)
        P2 = np.array(right_info.p).reshape(3, 4)
        K1 = P1[:3, :3]
        K2 = P2[:3, :3]
        self.cam0_intrinsics_matrix = K1
        fx_l, fy_l, cx_l, cy_l = K1[0, 0], K1[1, 1], K1[0, 2], K1[1, 2]
        fx_r, fy_r, cx_r, cy_r = K2[0, 0], K2[1, 1], K2[0, 2], K2[1, 2]
        
        # After rectification, cameras are aligned and rotation is the identity matrix
        # only horizontal baseline remains: P2[0,3] = -fx * baseline
        baseline = -P2[0, 3] / P2[0, 0]
        
        # For rectified stereo, the relative pose has:
        # identity rotation (cameras are aligned)
        # translation along X-axis only (horizontal baseline)
        rect_rotation_quat = np.array([0.0, 0.0, 0.0, 1.0])  # identity quaternion [qx, qy, qz, qw]
        rect_translation = np.array([baseline, 0.0, 0.0])
        
        self.get_logger().info(f"Rectified stereo baseline: {baseline:.6f} meters")
        self.get_logger().info(f"Rectified translation: {rect_translation}")
        
        # initialize cameras
        left_cam = cuvslam.Camera(
            size=[width_l, height_l],
            principal=[cx_l, cy_l],
            focal=[fx_l, fy_l]
        )
        right_cam = cuvslam.Camera(
            size=[width_l, height_l],
            principal=[cx_r, cy_r],
            focal=[fx_r, fy_r],
            rig_from_camera=cuvslam.Pose(
                rect_rotation_quat,
                rect_translation
            )
        )

        # initialize the rig
        rig = cuvslam.Rig(cameras=[left_cam, right_cam])
        self.get_logger().info(f"Rig initialized with cameras: {rig.cameras}")

        # initialize the cuvslam tracker
        odom_cfg = cuvslam.Tracker.OdometryConfig(
            async_sba=False,
            enable_final_landmarks_export=True,
            horizontal_stereo_camera=True,
            use_gpu = True
        )
        slam_cfg = cuvslam.Tracker.SlamConfig(
            sync_mode = False,
            use_gpu = True
        )

        # initialize the tracker
        self.tracker = cuvslam.Tracker(rig, odom_cfg, slam_cfg)
    
    def slam_callback(self, left, right, left_info, right_info):
        callback_start = time.perf_counter()
        
        # Initialize tracker
        if self.initialize_tracker is False:
            self._initialize_cuvslam_from_camera_info(left_info, right_info)
            self.initialize_tracker = True

        # grab the left and right images
        left_img = self.bridge.imgmsg_to_cv2(left)
        right_img = self.bridge.imgmsg_to_cv2(right)

        # grab timestamp
        left_stamp = left.header.stamp
        timestamp = Time.from_msg(left_stamp).nanoseconds

        # feed the images to the cuvslam tracker
        odom_pose_estimate, slam_pose = self.tracker.track(timestamp, [left_img, right_img])

        # error check odometry estimate
        if odom_pose_estimate.world_from_rig is None:
            self.get_logger().info(f"Failed to track frame {timestamp}")
            return

        odom_pose = odom_pose_estimate.world_from_rig.pose

        # get loop closure poses
        current_lc_poses = self.tracker.get_loop_closure_poses()
        if (current_lc_poses and 
            (not self.loop_closure_poses or 
            not np.array_equal(current_lc_poses[-1].pose.translation, self.loop_closure_poses[-1]))):
            self.loop_closure_poses.append(current_lc_poses[-1].pose.translation)  

        # save to odometry trajectory csv
        ros_time_ns = self.get_clock().now().nanoseconds
        self.odom_pose_estimates.append({
            'timestamp': ros_time_ns,               # currenlty set to grab current time because comparing based on playing rosbags
            'x': float(odom_pose.translation[0]),   # might have to change back to image timestamp later
            'y': float(odom_pose.translation[1]),
            'z': float(odom_pose.translation[2]),
            'qx': float(odom_pose.rotation[0]),
            'qy': float(odom_pose.rotation[1]),
            'qz': float(odom_pose.rotation[2]),
            'qw': float(odom_pose.rotation[3]),
        })

        # save slam pose
        self.slam_poses.append({
            'timestamp': ros_time_ns,
            'x': float(slam_pose.translation[0]),
            'y': float(slam_pose.translation[1]),
            'z': float(slam_pose.translation[2]),
            'qx': float(slam_pose.rotation[0]),
            'qy': float(slam_pose.rotation[1]),
            'qz': float(slam_pose.rotation[2]),
            'qw': float(slam_pose.rotation[3]),
        })
        

        # publish the odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'world'
        odom_msg.pose.pose.position.x = float(odom_pose.translation[0])
        odom_msg.pose.pose.position.y = float(odom_pose.translation[1])
        odom_msg.pose.pose.position.z = float(odom_pose.translation[2])
        odom_msg.pose.pose.orientation.x = float(odom_pose.rotation[0])
        odom_msg.pose.pose.orientation.y = float(odom_pose.rotation[1])
        odom_msg.pose.pose.orientation.z = float(odom_pose.rotation[2])
        odom_msg.pose.pose.orientation.w = float(odom_pose.rotation[3])
        self.pose_pub.publish(odom_msg)

        # visualization data for rerun visualization
        observations = self.tracker.get_last_observations(0)  # get observation from left camera
        landmarks = self.tracker.get_last_landmarks()
        final_landmarks = self.tracker.get_final_landmarks()

        # Prepare visualization data
        observations_uv = [[o.u, o.v] for o in observations]
        observations_colors = [self._color_from_id(o.id) for o in observations]
        landmark_xyz = [l.coords for l in landmarks]
        landmarks_colors = [self._color_from_id(l.id) for l in landmarks]
        self.trajectory.append(odom_pose.translation)
        self.trajectory_slam.append(slam_pose.translation)

        # Visualize with rerun gui
        if self.rerun_visualization:
            # Send results to rerun for visualization
            rr.set_time_nanos("frame", timestamp)
            rr.log('trajectory', rr.LineStrips3D(self.trajectory))
            rr.log('trajectory_slam', rr.LineStrips3D(self.trajectory_slam))
            rr.log('final_landmarks', rr.Points3D(list(final_landmarks.values()), radii=0.1))
            rr.log('loop_closure_poses', rr.Points3D(
                self.loop_closure_poses, radii=1.2, colors=[[255, 0, 0]]
            ))
            rr.log('cam_sync', rr.Transform3D(
                translation=odom_pose.translation,
                quaternion=odom_pose.rotation
            ))
            rr.log('cam_sync/body', rr.Boxes3D(centers=[[0, 1.65 / 2, 0]], sizes=[[1.6, 1.65, 2.71]]))
            rr.log('cam_sync/landmarks_center', rr.Points3D(
                landmark_xyz, radii=0.25, colors=landmarks_colors
            ))
            rr.log('cam_sync/landmarks_lines', rr.Arrows3D(
                vectors=landmark_xyz, radii=0.05, colors=landmarks_colors
            ))
            rr.log('cam_sync/cam0', rr.Pinhole(
                image_from_camera=self.cam0_intrinsics_matrix, # image_plane_distance=1.68, from before
                width=self.width,
                height=self.height
            ))
            rr.log('cam_sync/cam0/image', rr.Image(left_img).compress(jpeg_quality=80))
            rr.log('cam_sync/cam0/observations', rr.Points2D(
                observations_uv, radii=5, colors=observations_colors
            ))

        # Visualize stereo pair video output and trajectory 2D X-Z live plot
        if self.visualization_2d:
            stereo_pair = np.hstack((left_img, right_img))
            cv2.imshow('Stereo Pair - Cuvslam Node', stereo_pair)
            cv2.waitKey(1)

            self.traj_x.append(float(odom_pose.translation[0]))
            self.traj_z.append(float(odom_pose.translation[2]))
            self.traj_line.set_data(self.traj_x, self.traj_z)
            self.traj_ax.relim()
            self.traj_ax.autoscale_view()
            self.traj_fig.canvas.draw()
            self.traj_fig.canvas.flush_events()

        # Log callback execution time
        callback_elapsed_ms = (time.perf_counter() - callback_start) * 1000
        self.get_logger().info(f"Callback time: {callback_elapsed_ms:.2f} ms")


    def visualize_trajectory(self):

        # TODO rerun trajectory recording
        NotImplementedError


def create_tum_filename(filename):
    """Create a file name with date and time as filename_YYYYMMDD_HHMMSS.txt (TUM format)"""
    now = datetime.datetime.now()
    date_time = now.strftime('%Y%m%d_%H%M%S')

    return f"{filename}_{date_time}.txt"


def save_trajectory_tum(filepath, poses):
    """Save trajectory in TUM format: timestamp tx ty tz qx qy qz qw"""
    with open(filepath, 'w') as f:
        for pose in poses:
            timestamp_sec = pose['timestamp'] / 1e9
            line = f"{timestamp_sec:.9f} {pose['x']} {pose['y']} {pose['z']} {pose['qx']} {pose['qy']} {pose['qz']} {pose['qw']}\n"
            f.write(line)


def main(args=None):
    rclpy.init(args=args)
    cuvslam_stereo = CuvslamStereo()

    try:
        rclpy.spin(cuvslam_stereo)
    except KeyboardInterrupt:
        pass
    finally:
        # Before shutting down, save the trajectory in TUM format
        results_dir = Path(__file__).resolve().parent.parent.parent.parent / 'output' / 'trajectories'
        results_dir.mkdir(parents=True, exist_ok=True)

        if cuvslam_stereo.save_trajectory_tum:
            odom_filename = results_dir / create_tum_filename('odom_trajectory')
            save_trajectory_tum(odom_filename, cuvslam_stereo.odom_pose_estimates)

            slam_filename = results_dir / create_tum_filename('slam_trajectory')
            save_trajectory_tum(slam_filename, cuvslam_stereo.slam_poses)
            cuvslam_stereo.get_logger().info(f"Saved trajectory to {results_dir} directory")

        # shutdown node
        cuvslam_stereo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()  