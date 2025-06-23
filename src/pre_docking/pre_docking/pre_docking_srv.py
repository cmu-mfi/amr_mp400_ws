import rclpy
import rclpy.duration
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Trigger
from aruco_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Twist
from tf2_ros import Buffer, TransformListener 
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import quat2mat, mat2quat
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation as R




class PreDockingSrv(Node):

    def __init__(self):

        super().__init__('pre_docking_srv')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_sent = False
        self.distance = 0.5

        self.latest_pose = None
        self.marker = None

        self.client = ActionClient(
            self, NavigateToPose, "/robot1/navigate_to_pose"
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot1/map_pose',
            self.pose_callback,
            10
        )

        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/front_camera/marker_publisher/markers',
            self.marker_callback,
            10
        )

        self.srv = self.create_service(Trigger, 'pre_docker', self.docker_callback)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = msg.pose.pose

    def marker_callback(self, markers: MarkerArray):
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
        
        self.marker: Marker = marker
    
    def docker_callback(self, request: Trigger.Request, response: Trigger.Response):
        
        if self.marker == None:
            self.get_logger().info("No marker found, cant do pre-docking")
            response.message = "Failed to Find Marker"
            response.success = False
            return response
        
        marker = self.marker
        
        self.get_logger().info(f"Marker with id {marker.id} found!")

        try:

            map_to_marker = self.tf_buffer.lookup_transform(
                'map',
                marker.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        
        except Exception as e:
            self.get_logger().info(f"Transform error with {e}")
            response.message = f"Transform Failure: {e}"
            response.success = False
            return response

        marker_pose = marker.pose.pose
        
        # cmd = Twist()
        # self.fix_orientation(cmd)

        # Need point pose in map frame
        point_pose = do_transform_pose(marker_pose, map_to_marker) 

        q = point_pose.orientation
        q_np = [q.w, q.x, q.y, q.z]

        _, _, yaw_marker = quat2euler(q_np)
        yaw_goal = yaw_marker + np.pi

        quat_goal = euler2quat(0, 0, yaw_goal)

        R = quat2mat(q_np)
        z_axis = R[:3, 2]

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
        goal_msg.pose.pose.orientation.x = quat_goal[1]  # transforms3d uses [w, x, y, z]
        goal_msg.pose.pose.orientation.y = quat_goal[2]
        goal_msg.pose.pose.orientation.z = quat_goal[3]
        goal_msg.pose.pose.orientation.w = quat_goal[0]

        self.get_logger().info('Sent goal pose to nav2')
        self.goal_sent = True
        self.sub = None

        response.message = "Sent goal request to Nav2"
        response.success = True

        goal_future = self.client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.goal_response)
        
        return response


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
            # Need to send trigger to the other interns code and do the backup
            success = self.send_client_request()
            if not success:
                self.get_logger().info('Docking Failed')
            else:
                self.get_logger().info('Docking Completed')

        else:
            self.get_logger().error("Nav2 failed to complete!")
    
    def send_client_request(self):
        self.docking_offsets_client = self.create_client(Trigger, '/robot1/get_docking_offsets')
        self.docking_client = self.create_client(Trigger, '/robot1/docking_with_markers')

        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Docking service not available, waiting again...')

        while not self.docking_offsets_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Offsets service not available, waiting again...')
    
        self.docking_request = Trigger.Request()
        self.offsets_request = Trigger.Request()

        self.offsets_future = self.docking_offsets_client.call_async(self.offsets_request)
        self.docking_future = self.docking_client.call_async(self.docking_request)

        rclpy.spin_until_future_complete(self, self.offsets_future) 
        if not self.offsets_future.result():
            self.get_logger().info("Failed to get marker offset")
            return self.offsets_future.result()



        rclpy.spin_until_future_complete(self, self.docking_future)

        return self.docking_future.result()

'''
    def fix_orientation(self, cmd):
        rot = self.marker.pose.pose.orientation
        R_mat = R.from_quat([rot.x, rot.y, rot.z, rot.w])
        yaw = R_mat.as_euler('xyz', degrees=True)[1]

        while abs(yaw) >= 0.3:
            self.get_logger().info(f"Fixing Orientation. Yaw: {yaw}")
            if yaw > 0:
                cmd.angular.z = -0.1
            else:
                cmd.angular.z = 0.1
            
            rot = self.marker.pose.pose.orientation
            R_mat = R.from_quat([rot.x, rot.y, rot.z, rot.w])
            yaw = R_mat.as_euler('xyz', degrees=True)[1]
            
            


        cmd.angular.z = 0.0
        self.get_logger().info(f"Orientation fixed. Yaw: {yaw}")
       
        return cmd, success
'''




def main(args=None):
    rclpy.init(args=args)

    pre_docking_srv = PreDockingSrv()

    rclpy.spin(pre_docking_srv)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


