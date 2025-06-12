import rclpy
import rclpy.duration
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_ros import Buffer, TransformListener 
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import quat2mat, mat2quat
import numpy as np


class FiducialDetector(Node):

    def __init__(self):
        super().__init__('fiducial_detector')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_sent = False


        self.sub = self.create_subscription(
            MarkerArray,
            '/robot/marker_publisher/markers', 
            self.sub_callback,
            10)
        
        self.client = ActionClient(
                self, NavigateToPose, "/robot1/navigate_to_pose"
            )

        self.distance = 0.5 # Change distance from marker if needed

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

        marker = markers.markers[0]
    
        # Create proper PoseStamped for transformation
        pose_stamped = PoseStamped()
        pose_stamped.header = marker.header  # Critical: maintain original frame_id
        pose_stamped.pose = marker.pose.pose  # Extract geometry_msgs/Pose
    
        try:
            # Transform to map frame
            transformed_pose = self.tf_buffer.transform(
                pose_stamped,
                'map',
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
                
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {str(e)}")
            return

        q = transformed_pose.pose.orientation
        q_np = [q.w, q.x, q.y, q.z]

        R = quat2mat(q_np)
        z_axis = R[:3][2]
        z_x, z_y = z_axis[0], z_axis[1]

        x_goal = transformed_pose.pose.position.x - self.distance * z_x
        y_goal = transformed_pose.pose.position.y - self.distance * z_y

        _, _, yaw_marker = quat2euler(q_np)
        yaw_goal = yaw_marker + np.pi  # 180° rotation (π radians)
        q_goal = euler2quat(0, 0, yaw_goal)  # Convert back to quaternion

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  # Or 'map' if localized
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x_goal
        goal_msg.pose.pose.position.y = y_goal
        goal_msg.pose.pose.orientation.x = q_goal[1]  # transforms3d uses [w, x, y, z]
        goal_msg.pose.pose.orientation.y = q_goal[2]
        goal_msg.pose.pose.orientation.z = q_goal[3]
        goal_msg.pose.pose.orientation.w = q_goal[0]


        self.get_logger().info('Sent goal pose to nav2')
        self.goal_sent = True
        self.sub.destroy()

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
            self.get_logger().info(f"Nav2 completed with status: {result.status}")
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
