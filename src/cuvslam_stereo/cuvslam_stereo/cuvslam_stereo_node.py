import numpy as np
import cuvslam
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from rclpy.time import Time
import cv2
import matplotlib.pyplot as plt
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import pandas as pd
import rerun as rr
import rerun.blueprint as rrb


class CuvslamStereo(Node):
    def __init__(self):
        super().__init__('cuvslam_stereo_node')

        # Setup rerun visualizer
        rr.init('neuroam', strict=True, spawn=True)  # launch re-run instance

        # Setup rerun views
        rr.send_blueprint(rrb.Blueprint(
            rrb.TimePanel(state="collapsed"),
            rrb.Vertical(
                row_shares=[0.6, 0.4],
                contents=[rrb.Spatial3DView(), rrb.Spatial2DView(origin='cam_sync/cam0')]
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

        self.trajectory = []
        
        self.initialize_tracker = False
        
        # initilaize subscribers
        self.left_sub = Subscriber(self, Image, '/cam_sync/cam0/image_rect')
        self.right_sub = Subscriber(self, Image, '/cam_sync/cam1/image_rect')
        self.left_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam0/rect_info')
        self.right_info_sub = Subscriber(self, CameraInfo, '/cam_sync/cam1/rect_info')

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

        # to save trajectory data to csv upon node shutdown
        self.trajectory_data = []

        self.visualization = False
        if self.visualization:
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


    def _initialize_cvslam_from_camera_info(self, left_info, right_info):
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
        cfg = cuvslam.Tracker.OdometryConfig(
            async_sba=False,
            enable_final_landmarks_export=True,
            horizontal_stereo_camera=True
        )

        # initialize the tracker
        self.tracker = cuvslam.Tracker(rig, cfg)

    
    def slam_callback(self, left, right, left_info, right_info):
        if self.initialize_tracker is False:
            self._initialize_cvslam_from_camera_info(left_info, right_info)
            self.initialize_tracker = True

        # grab the left and right images
        left_img = self.bridge.imgmsg_to_cv2(left)
        right_img = self.bridge.imgmsg_to_cv2(right)

        # grab timestamp
        left_stamp = left.header.stamp
        timestamp = Time.from_msg(left_stamp).nanoseconds

        # feed the images to the cuvslam tracker
        odom_pose_estimate,_ = self.tracker.track(timestamp, [left_img, right_img])

        odom_pose = odom_pose_estimate.world_from_rig.pose

        # save to trajectory csv
        self.trajectory_data.append({
            'timestamp': timestamp,
            'x': float(odom_pose.translation[0]),
            'y': float(odom_pose.translation[1]),
            'z': float(odom_pose.translation[2]),
            'qx': float(odom_pose.rotation[0]),
            'qy': float(odom_pose.rotation[1]),
            'qz': float(odom_pose.rotation[2]),
            'qw': float(odom_pose.rotation[3]),
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

        # Send results to rerun for visualization
        rr.set_time_nanos("frame", timestamp)
        rr.log('trajectory', rr.LineStrips3D(self.trajectory))
        rr.log('final_landmarks', rr.Points3D(list(final_landmarks.values()), radii=0.1))
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

        if self.visualization is True:
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


    def visualize_trajectory(self):

        # TODO rerun trajectory recording
        NotImplementedError



def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = CuvslamStereo()

    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Before shutting down, save the trajectory to a csv
        trajectory_df = pd.DataFrame(odometry_publisher.trajectory_data)
        trajectory_df.to_csv('trajectory.csv', index=False)

        # shutdown node
        odometry_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()  