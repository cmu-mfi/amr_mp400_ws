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

        if marker.id > 20:
            self.get_logger().info(f"Marker id too high: {marker.id}")
            return
        
        self.get_logger().info(f"Marker with id {marker.id} found!")

        try: 

            map_to_marker = self.tf_buffer.lookup_transform(
                'robot1/base_link',
                marker.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )


        except Exception as e:
            self.get_logger().info(f"Transform error with {e}")
            exit(1)

        pose_stamped = PoseStamped()
        pose_stamped.header = marker.header
        pose_stamped.pose = marker.pose.pose

        marker_pose = marker.pose.pose

        # desired_pose = marker_pose
        # desired_pose.position.z += self.distance

        # Get point pose in map frame
        point_pose = do_transform_pose(marker_pose, map_to_marker) # assume we have the transform from robot to marker

        q = point_pose.orientation
        q_np = [q.w, q.x, q.y, q.z]

        _, _, yaw_marker = quat2euler(q_np)
        yaw_goal = yaw_marker + np.pi

        q_goal = euler2quat(0, 0, yaw_goal)  # Convert back to quaternion

        R = quat2mat(q_np)
        z_axis = R[:3, 2]  # Forward vector of the marker

        
        # Calculate desired position: move forward along marker's forward vector
        # Dont think this makes too much sense but yeah :) (Might also just be = not +=)
        desired_position = point_pose.position
        desired_position.x += z_axis[0] * self.distance
        desired_position.y += z_axis[1] * self.distance
        desired_position.z = 0.0
    

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = desired_position
        goal_msg.pose.pose.orientation.x = q_goal[1]  # transforms3d uses [w, x, y, z]
        goal_msg.pose.pose.orientation.y = q_goal[2]
        goal_msg.pose.pose.orientation.z = q_goal[3]
        goal_msg.pose.pose.orientation.w = q_goal[0]


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

        # self.goal_sent = False
        # self.sub = self.create_subscription(
        #     MarkerArray,
        #     '/robot/marker_publisher/markers', 
        #     self.sub_callback,
        #     10)
        
def main():
    rclpy.init()

    fiducial_detector = FiducialDetector()
    rclpy.spin(fiducial_detector)
    fiducial_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
