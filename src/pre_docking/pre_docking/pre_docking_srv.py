import rclpy
import rclpy.duration
import numpy as np
from rclpy.node import Node
import time
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
from amr_mp400_interfaces.srv import GroundTruth




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
            self._pose_callback,
            10
        )

        self.front_marker_sub = self.create_subscription(
            MarkerArray,
            '/front_camera/marker_publisher/markers',
            self._front_marker_callback,
            10
        )

        self.back_marker_sub = self.create_subscription(
            MarkerArray,
            "/robot1/marker_publisher/markers",
            self._back_marker_callback,
            10
        )

        self.offset_srv = self.create_service(Trigger, 'pre_docker_offset', self.offset_callback)
        self.docking_srv = self.create_service(GroundTruth, 'pre_docker_docking', self.docking_callback)

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = msg.pose.pose

    def _front_marker_callback(self, markers: MarkerArray):
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
    
    def _back_marker_callback(self, markers: MarkerArray):
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
        self.get_logger().info("OFFSET CALLBACK METHOD CALLED")

        if self.back_marker == None:
            self.get_logger().info("NO BACK MARKER FOUND")
            if self.front_marker == None:
                self.get_logger().info("NO FRONT MARKER FOUND ALSO")
                self.get_logger().info('No marker found, cant dock')
                response.success = False
                response.message = 'No markers'

                return response
            
            # We know we see front and not back, so we can do the spin
            self.get_logger().info("FRONT MARKER FOUND")
            success = self.__turn_around()
            if not success:
                response.success = False
                response.message = 'Failed to Turn Around'

                return response            
            
        # Back camera is detecting a marker
        self.get_logger().info("BACK MARKER FOUND")
        success = self.__send_offset_request()
        if not success:
            response.success = False
            response.message = 'Failed to Save Marker Offset'

            return response
        
        # Success
        self.get_logger().info("Succesfully got Offsets")
        response.success = True
        response.message = 'Saved Marker Offset'

        return response
    
    def docking_callback(self, request: GroundTruth.Request, response: GroundTruth.Response):
        self.lgt = request.last_ground_truth
        self.goal = request.goal
        self.curr_pose = request.curr_pose


        # Robot should already be at nav2 position
        if self.back_marker == None:
            self.get_logger().info("NO BACK MARKER FOUND")
            if self.front_marker == None:
                self.get_logger().info("NO FRONT MARKER FOUND")
                response.success = False
                # Do recovery policy here

                self.recover(self.lgt, self.goal, self.curr_pose)

                response.message = 'No markers found for docking'
                return response
            
            self.get_logger().info("FRONT MARKER FOUND")
            success = self.__turn_around()
            if not success:
                response.success = False
                response.message = 'Failed to Turn Around'

                return response

        # Robot should see marker from the back
        self.get_logger().info("BACK MARKER FOUND")
        self.get_logger().info('Sending Docking Request')
        success = self.__send_docking_request()
        if not success:
            response.success = False
            response.message = 'Docking Failed'
            return response
        
        self.get_logger().info("Succesfully got Offsets")
        response.success = True
        response.message = 'Docking Success'

        return response

    def __send_offset_request(self):
        self.docking_offsets_client = self.create_client(Trigger, '/robot1/get_docking_offsets')

        while not self.docking_offsets_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Offsets service not available, waiting again...')
        
        self.offsets_request = Trigger.Request()
        self.offsets_future = self.docking_offsets_client.call_async(self.offsets_request)

        return True


    def __send_docking_request(self):
        self.docking_client = self.create_client(Trigger, '/robot1/docking_with_markers')

        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Docking service not available, waiting again...')
    
        self.docking_request = Trigger.Request()
        self.docking_client.call_async(self.docking_request)

        return True
    

    def __turn_around(self):
        self.get_logger().info("TURN AROUND REQUEST RECEIVED")
        if self.front_marker == None:
            self.get_logger().info("No marker found, cant do pre-docking")
            return False
        
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
            return False

        marker_pose = marker.pose.pose

        # Need point pose in map frame
        point_pose = do_transform_pose(marker_pose, map_to_marker) 

        q = point_pose.orientation
        q_np = [q.w, q.x, q.y, q.z]

        _, _, yaw_marker = quat2euler(q_np)
        yaw_goal = yaw_marker + np.pi

        quat_goal = euler2quat(0, 0, yaw_goal)

        self.get_logger().info("Turning Around")

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

        self.get_logger().info("Sent goal request to Nav2")

        goal_future = self.client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.__goal_response)
        
        return True


    def __goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
    
        self.get_logger().info('Goal accepted, waiting for result')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.__goal_result)

    def __goal_result(self, future):
        result = future.result().result
        if result:
            self.get_logger().info(f"Nav2 completed")
        else:
            self.get_logger().error("Nav2 failed to complete!")

    def recover(self, lgt: PoseWithCovarianceStamped, goal: PoseStamped, curr_pose: PoseWithCovarianceStamped):
        error = 0.05 # 5 % error
        remainder = 1 - error

        # Get positional error
        x = (curr_pose.pose.position.x - lgt.pose.position.x) * remainder
        y = (curr_pose.pose.position.y - lgt.pose.position.y) * remainder

        # Relocalize robot with new assumed position
        pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = curr_pose.header
        pose_msg.pose = curr_pose.pose
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y

        pose_pub.publish(pose_msg)

        pose_pub.destroy()

        # Try going to goal again 

        action_client = ActionClient(
            self, NavigateToPose, "/robot1/navigate_to_pose"
        )

        goal_pose = NavigateToPose.Goal()
        goal_pose.pose = goal

        future = action_client.send_goal_async(goal_pose)
        
        # Recheck for markers, and if not, do recovery again
        future.add_done_callback(self.__recovery_response)
        

    def __recovery_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
            
        self.get_logger().info('Goal accepted, waiting for result')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.__recovery_result)
    
    def __recovery_result(self, future):
        result = future.result().result
        if result:
            if self.back_marker == None and self.front_marker == None: # Still no markers found
                self.get_logger().info("Calling recovery again, no markers found")
                self.recover(self.lgt, self.goal, self.latest_pose)
            else: 
                self.get_logger().info("Recovery completed successfully")
            
        
        

    
    


def main(args=None):
    rclpy.init(args=args)

    pre_docking_srv = PreDockingSrv()

    rclpy.spin(pre_docking_srv)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

