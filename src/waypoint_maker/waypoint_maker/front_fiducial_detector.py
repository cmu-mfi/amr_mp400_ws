import rclpy
import rclpy.duration
import numpy as np
import tf2_geometry_msgs
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from tf2_ros import Buffer, TransformListener 
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import quat2mat, mat2quat
from tf2_geometry_msgs import do_transform_pose



class FiducialDetector(Node):

    def __init__(self):
        super().__init__('fiducial_detector')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_sent = False


        self.sub = self.create_subscription(
            MarkerArray,
            '/robot1/marker_publisher/markers', 
            self.sub_callback,
            10)
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot1/map_pose',
            self.pose_callback,
            10
        )

        self.latest_pose = None
        
        self.client = ActionClient(
                self, NavigateToPose, "/robot1/navigate_to_pose"
            )

        self.distance = 0.5 # Change distance from marker if needed

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = msg.pose.pose

    def sub_callback(self, markers: MarkerArray):
        if self.goal_sent:
            return

        if len(markers.markers) == 0:
            self.get_logger().info('No Markers Found')
            return
        
        # There should only be one marker in the array for a docking station
        if len(markers.markers) > 1:
            self.get_logger().info('More than one marker found, cant dock')
            return
        
        # Now we only have one marker
        marker: Marker = markers.markers[0]
        self.get_logger().info(f"Marker with id {marker.id} found!")

        try: 
            
            map_to_odom = self.tf_buffer.lookup_transform(
                'map',
                'robot1/odom',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            odom_to_camera = self.tf_buffer.lookup_transform(
                'robot1/odom',
                pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )


        except Exception as e:
            self.get_logger().info(f"Transform error with {e}")

        pose_stamped = PoseStamped()
        pose_stamped.header = marker.header
        pose_stamped.pose = marker.pose.pose

        marker_pose = marker.pose.pose

        desired_pose = marker_pose
        desired_pose.position.z += self.distance

        # Get point pose in map frame
        point_pose = do_transform_pose(do_transform_pose(desired_pose, odom_to_camera), map_to_odom) # assume we have the transform from robot to marker


        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position = self.latest_pose.position
        goal_msg.pose.orientation = point_pose.orientation

        self.get_logger().info('Sent goal pose to nav2')
        self.goal_sent = True
        self.sub = None

        goal_future = self.client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.goal_response)


    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
    
        self.get_logger().info('Goal accepted, waiting for result')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future):
        result = future.result().result
        if result:
            self.get_logger().info(f"Nav2 completed")
        else:
            self.get_logger().error("Nav2 failed to complete!")

        self.goal_sent = False
        self.sub = self.create_subscription(
            MarkerArray,
            '/robot/marker_publisher/markers', 
            self.sub_callback,
            10)
        
def main():
    rclpy.init()

    fiducial_detector = FiducialDetector()
    rclpy.spin(fiducial_detector)
    fiducial_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
