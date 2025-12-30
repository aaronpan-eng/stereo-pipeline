from io import open_code
import rclpy
from rclpy.node import Node
import rerun as rr
import rerun.blueprint as rrb
from std_msgs.msg import String


class RerunVisualizer(Node):

    def __init__(self):
        super().__init__('rerun_visualizer')
        
        self.declare_parameter('rerun_app_id', 'stereo_pipeline')
        self.declare_parameter('recording_id', 'stereo_pipeline_session')

        self.rerun_app_id = self.get_parameter('rerun_app_id').value
        self.recording_id = self.get_parameter('recording_id').value

        self._initialize_rerun()

    def _initialize_rerun(self):
        # Setup rerun visualizer w ith recording_id for shared session
        rr.init(self.rerun_app_id, recording_id=self.recording_id)
        rr.spawn(port=9876)
        
        self.get_logger().info(f"Rerun initialized: app_id='{self.rerun_app_id}', recording_id='{self.recording_id}'")

        # Setup rerun views
        rr.send_blueprint(rrb.Blueprint(
            rrb.TimePanel(state="collapsed"),
                rrb.Vertical(                       # Cuvslam view
                row_shares=[0.6, 0.4],
                contents=[
                    rrb.Horizontal(
                        contents=[ 
                            rrb.Spatial2DView(
                                origin='/cam_sync/cam0'
                            ),
                            rrb.Spatial3DView(
                                name = "cuvslam map"            # origin namespace at root "/" by default
                            ),
                        ]
                    ),
                    rrb.Horizontal(
                        contents=[
                            rrb.Spatial2DView(                  # Rectification view
                                name = 'stereo resized images',
                                origin = '/neustereo_ros2/stereo/resized'
                            ),
                            rrb.Spatial2DView(                  # Neustereo disparity map view
                                name = 'disparity map',
                                origin = '/neustereo_ros2/disparity'
                            ),     
                        ]
                    )
                ]),
                                             # Add extra views to rerun here using rrb
        ))

        # Setup coordinate basis for root, cuvslam uses right-hand system with X-right, Y-down, Z-forward
        rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

        # Draw arrays in origin X-red, Y-green, Z-blue
        rr.log("xyz", rr.Arrows3D(                              #"xyz" goes to root view which is the cuvslam viz
            vectors=[[50, 0, 0], [0, 50, 0], [0, 0, 50]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            labels=['[x]', '[y]', '[z]']
        ), static=True)
        
        self.get_logger().info("Rerun visualization initialized")
    

def main(args=None):
    rclpy.init(args=args)
    rerun_visualizer = RerunVisualizer()
    rclpy.spin(rerun_visualizer)
    rerun_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()