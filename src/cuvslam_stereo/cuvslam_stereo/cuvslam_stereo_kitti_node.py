import os
import rclpy 
from rclpy.node import Node
from numpy import loadtxt, asarray
import numpy as np
from PIL import Image
import rerun as rr
import rerun.blueprint as rrb
import cuvslam
from nav_msgs.msg import Odometry 

class KittiCuVSLAMNode(Node):
    def __init__(self):
        super().__init__('kitti_cuvslam_node')
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 10)
        
        # Set up dataset path
        self.sequence_path = "/home/aaron/stereo_pipeline_stuff/src/cuvslam_stereo/cuvslam_stereo/dataset/sequences/06"
        
        # Setup rerun visualizer (save to file)
        rr.init('kitti', strict=True)
        rr.save('/tmp/kitti_cuvslam_recording.rrd')
        self.get_logger().info("Saving visualization to /tmp/kitti_cuvslam_recording.rrd")
        
        # Setup rerun views
        rr.send_blueprint(rrb.Blueprint(
            rrb.TimePanel(state="collapsed"),
            rrb.Vertical(
                row_shares=[0.6, 0.4],
                contents=[rrb.Spatial3DView(), rrb.Spatial2DView(origin='car/cam0')]
            )
        ))
        
        # Setup coordinate basis
        rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)
        
        # Draw axes
        rr.log("xyz", rr.Arrows3D(
            vectors=[[50, 0, 0], [0, 50, 0], [0, 0, 50]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            labels=['[x]', '[y]', '[z]']
        ), static=True)
        
        # Load KITTI dataset calibration and initialize cameras
        intrinsics = loadtxt(
            os.path.join(self.sequence_path, 'calib.txt'),
            usecols=range(1, 13)
        )[:4].reshape(4, 3, 4)
        
        size = Image.open(os.path.join(self.sequence_path, 'image_0', '000001.png')).size
        
        cameras = [cuvslam.Camera(), cuvslam.Camera()]
        for i in [0, 1]:
            cameras[i].size = size
            cameras[i].principal = [intrinsics[i][0][2], intrinsics[i][1][2]]
            cameras[i].focal = [intrinsics[i].diagonal()[0], intrinsics[i].diagonal()[1]]
        cameras[1].rig_from_camera.translation[0] = -intrinsics[1][0][3] / intrinsics[1][0][0]
        
        # Initialize the cuvslam tracker
        cfg = cuvslam.Tracker.OdometryConfig(
            async_sba=False,
            enable_final_landmarks_export=True,
            horizontal_stereo_camera=True
        )
        self.tracker = cuvslam.Tracker(cuvslam.Rig(cameras), cfg)
        
        # Get timestamps
        self.timestamps = [
            int(10 ** 9 * float(sec_str))
            for sec_str in open(os.path.join(self.sequence_path, 'times.txt')).readlines()
        ]
        
        self.intrinsics = intrinsics
        self.size = size
        self.trajectory = []
        self.frame = 0
        
        # Create timer to process frames at 10Hz
        self.timer = self.create_timer(0.1, self.process_frame)
        
        self.get_logger().info(f"Initialized with {len(self.timestamps)} frames")
    
    def color_from_id(self, identifier):
        return [(identifier * 17) % 256, (identifier * 31) % 256, (identifier * 47) % 256]
    
    def process_frame(self):
        if self.frame >= len(self.timestamps):
            self.get_logger().info("Finished processing all frames")
            self.timer.cancel()
            return
        
        # Load images
        images = [
            asarray(Image.open(os.path.join(self.sequence_path, f'image_{cam}', f'{self.frame:0>6}.png')))
            for cam in [0, 1]
        ]
        
        # Do tracking
        odom_pose_estimate, _ = self.tracker.track(self.timestamps[self.frame], images)
        
        if odom_pose_estimate.world_from_rig is None:
            self.get_logger().warn(f"Failed to track frame {self.frame}")
            self.frame += 1
            return
        
        # Get current pose
        odom_pose = odom_pose_estimate.world_from_rig.pose
        
        # Publish ROS2 Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = float(odom_pose.translation[0])
        odom_msg.pose.pose.position.y = float(odom_pose.translation[1])
        odom_msg.pose.pose.position.z = float(odom_pose.translation[2])
        
        # Orientation (quaternion)
        odom_msg.pose.pose.orientation.x = float(odom_pose.rotation[0])
        odom_msg.pose.pose.orientation.y = float(odom_pose.rotation[1])
        odom_msg.pose.pose.orientation.z = float(odom_pose.rotation[2])
        odom_msg.pose.pose.orientation.w = float(odom_pose.rotation[3])
        
        self.odom_pub.publish(odom_msg)
        
        # Get visualization data
        observations = self.tracker.get_last_observations(0)
        landmarks = self.tracker.get_last_landmarks()
        final_landmarks = self.tracker.get_final_landmarks()
        
        # Prepare visualization data
        observations_uv = [[o.u, o.v] for o in observations]
        observations_colors = [self.color_from_id(o.id) for o in observations]
        landmark_xyz = [l.coords for l in landmarks]
        landmarks_colors = [self.color_from_id(l.id) for l in landmarks]
        self.trajectory.append(odom_pose.translation)
        
        # Send results to rerun for visualization
        rr.set_time_sequence('frame', self.frame)
        rr.log('trajectory', rr.LineStrips3D(self.trajectory))
        rr.log('final_landmarks', rr.Points3D(list(final_landmarks.values()), radii=0.1))
        rr.log('car', rr.Transform3D(
            translation=odom_pose.translation,
            quaternion=odom_pose.rotation
        ))
        rr.log('car/body', rr.Boxes3D(centers=[0, 1.65 / 2, 0], sizes=[[1.6, 1.65, 2.71]]))
        rr.log('car/landmarks_center', rr.Points3D(
            landmark_xyz, radii=0.25, colors=landmarks_colors
        ))
        rr.log('car/landmarks_lines', rr.Arrows3D(
            vectors=landmark_xyz, radii=0.05, colors=landmarks_colors
        ))
        rr.log('car/cam0', rr.Pinhole(
            image_plane_distance=1.68,
            image_from_camera=self.intrinsics[0][:3, :3],
            width=self.size[0],
            height=self.size[1]
        ))
        rr.log('car/cam0/image', rr.Image(images[0]).compress(jpeg_quality=80))
        rr.log('car/cam0/observations', rr.Points2D(
            observations_uv, radii=5, colors=observations_colors
        ))
        
        if self.frame % 10 == 0:
            self.get_logger().info(f"Processed frame {self.frame}/{len(self.timestamps)}")
        
        self.frame += 1


def main(args=None):
    rclpy.init(args=args)
    node = KittiCuVSLAMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory = np.array(node.trajectory)
        np.savetxt('trajectory.txt', trajectory)
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()