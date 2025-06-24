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
        self.front_marker: Marker = None
        self.back_marker: Marker = None

        self.client = ActionClient(
            self, NavigateToPose, "/robot1/navigate_to_pose"
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot1/map_pose',
            self.pose_callback,
            10
        )

        self.front_marker_sub = self.create_subscription(
            MarkerArray,
            '/front_camera/marker_publisher/markers',
            self.front_marker_callback,
            10
        )

        self.back_marker_sub = self.create_subscription(
            MarkerArray,
            "/robot1/marker_publisher/markers",
            self.back_marker_callback,
            10
        )

        self.offset_srv = self.create_service(Trigger, 'pre_docker_offset', self.offset_callback)
        self.docking_srv = self.create_service(Trigger, 'pre_docker_docking', self.docking_callback)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = msg.pose.pose

    def front_marker_callback(self, markers: MarkerArray):
        if self.goal_sent:
            self.front_marker = None
            return

        if len(markers.markers) == 0:
            self.get_logger().info('No Markers Found')
            self.front_marker = None
            return
        
        # There should only be one marker in the array for a docking station
        if len(markers.markers) > 1:
            self.get_logger().info('More than one marker found, cant dock')
            self.front_marker = None
            return
        
        
        # Now we only have one marker
        marker: Marker = markers.markers[0]

        if marker.id > 20:
            self.get_logger().info(f"Marker id too high: {marker.id}")
            self.front_marker = None
            return
        
        self.front_marker = marker
    
    def back_marker_callback(self, markers: MarkerArray):
        if self.goal_sent:
            self.back_marker = None
            return
        
        if len(markers.markers) == 0:
            self.get_logger().info("Back Camera sees no markers")
            self.back_marker = None
            return
        
        if len(markers.markers) > 1:
            self.get_logger().info("Back Camera sees more than one marker")
            self.back_marker = None
            return
        
        marker: Marker = markers.markers[0]

        if marker.id > 20:
            self.get_logger().info(f'Back Marker id too high: {marker.id}')
            self.back_marker = None
            return
        
        self.back_marker = marker


    def offset_callback(self, request: Trigger.Request, response: Trigger.Response):
        
        if self.back_marker == None:
            if self.front_marker == None:
                self.get_logger().info('No marker found, cant dock')
                response.success = False
                response.message = 'No markers'

                return response
            
            # We know we see front and not back, so we can do the spin
            success = self.turn_away(response)
            if not success:
                response.success = False
                response.message = 'Failed to Turn Around'

                return response
            
            
        # Back camera is detecting a marker
        success = self.send_offset_request()
        if not success:
            response.success = False
            response.message = 'Failed to Save Marker Offset'

            return response
        
        # Success
        self.get_logger().info("Succesfully got Offsets")
        response.success = True
        response.message = 'Saved Marker Offset'

        return response
    
    def docking_callback(self, request: Trigger.Request, response: Trigger.Response):
        # Robot should already be at nav2 position
        if self.back_marker == None:
            success = self.turn_around(response)
            if not success:
                response.success = False
                response.message = 'Failed to Turn Around'

                return response
        
        # Robot should see marker from the back
        success = self.send_docking_request()
        if not success:
            response.success = False
            response.message = 'Docking Failed'
            return response
        
        self.get_logger().info("Succesfully got Offsets")
        response.success = True
        response.message = 'Docking Success'

        return response

    def send_offset_request(self):
        self.docking_offsets_client = self.create_client(Trigger, '/robot1/get_docking_offsets')

        while not self.docking_offsets_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Offsets service not available, waiting again...')
        
        self.offsets_request = Trigger.Request()
        self.offsets_future = self.docking_offsets_client.call_async(self.offsets_request)

        rclpy.spin_until_future_complete(self, self.offsets_future) 
        return self.offsets_future.result()


    def send_docking_request(self, future):
        self.docking_client = self.create_client(Trigger, '/robot1/docking_with_markers')

        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Docking service not available, waiting again...')
    
        self.docking_request = Trigger.Request()
        self.docking_future = self.docking_client.call_async(self.docking_request)

        rclpy.spin_until_future_complete(self, self.docking_future)
        return self.docking_future.result()
    

    def turn_away(self, response: Trigger.Response):
        if self.front_marker == None:
            self.get_logger().info("No marker found, cant do pre-docking")
            response.message = "Failed to Find Marker"
            response.success = False
            return response
        
        marker = self.front_marker
        
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

        # Calculate desired position: move forward along marker's forward vector
        # Dont think this makes too much sense but yeah :) (Might also just be = not +=)
        # Might also just not need this as you can just turn where you saved the position -> Make sure to save position facing just ahaead of the marker
        '''
        R = quat2mat(q_np)
        z_axis = R[:3, 2]
        desired_position = point_pose.position
        desired_position.x += z_axis[0] * self.distance
        desired_position.y += z_axis[1] * self.distance
        desired_position.z = 0.0
        '''


        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = self.latest_pose.position
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
            success = self.send_docking_request()
            if not success:
                self.get_logger().info('Docking Failed')
            else:
                self.get_logger().info('Docking Completed')

        else:
            self.get_logger().error("Nav2 failed to complete!")
    
    


def main(args=None):
    rclpy.init(args=args)

    pre_docking_srv = PreDockingSrv()

    rclpy.spin(pre_docking_srv)

    rclpy.shutdown()

if __name__ == '__main__':
    main()



'''
# if self.front_marker == None:
        #     self.get_logger().info("No marker found, cant do pre-docking")
        #     response.message = "Failed to Find Marker"
        #     response.success = False
        #     return response
        
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
        # Might also just not need this as you can just turn where you saved the position -> Make sure to save position facing just ahaead of the marker
        
        desired_position = point_pose.position
        desired_position.x += z_axis[0] * self.distance
        desired_position.y += z_axis[1] * self.distance
        desired_position.z = 0.0
        


        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = self.latest_pose.position
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
'''
