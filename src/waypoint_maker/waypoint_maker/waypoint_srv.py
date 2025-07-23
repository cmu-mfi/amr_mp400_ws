import os
import rclpy
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from amr_mp400_interfaces.srv import SetFlag
from std_msgs.msg import Int32

class WaypointSrv(Node):

    def __init__(self):
        super().__init__('waypoint_srv')
        self.srv = self.create_service(SetFlag, 'waypoint_maker', self.waypoint_callback)

        subscriber_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            'robot1/map_pose',
            self.sub_callback,
            qos_profile=subscriber_qos
        )

        self.timer_period = 0.1

        self.pub = self.create_publisher(
            Int32, 
            '/waypoint_amount', 
            10
        )

        self.timer = self.create_timer(self.timer_period, self.publish_amount)
        self.action_client = ActionClient(
            self, NavigateToPose, "/robot1/navigate_to_pose"
        )
        
        path = "/home/neobotix/mp_400_workspace/src/amr_mp400_ws/src/waypoint_maker/waypoint_maker/robot_poses.txt"
        self.file_path = path
        self.file = None

        self.latest = None
        self.docking = None

    def sub_callback(self, msg: PoseWithCovarianceStamped):
        self.latest = msg.pose.pose

    def waypoint_callback(self, request, response):
        # Flags for operations -> a - Add / o - overwrite / w - delete all and add new / d - delete / g - go
        flag = chr(request.flag)
        index = str(request.index)
        dock = chr(request.docking)
        
        if not self.latest:
            response.success = False
            response.msg = "Couldn't get robot pose"
            return response
        
        if dock == 'd':
            self.docking = True
        else:
            self.docking = False

        pose = self.create_pose_array(self.latest)
        with open(self.file_path, 'r') as file:
            self.pose_dict = self.create_pose_dict(file)
        # ONLY NEED TO OPEN FILE ONCE AND GET THE POSE DICT AND WORK ON THAT AND THEN WRITE TO THE FILE IN THE END 

        match flag:

            case 'h':
                # Help message for new users/reminders
                pass

            case "a":

                # Dont need index to add, need robot pose
                response.success = self.append_write(pose)
                response.msg = "Wrote Robot Pose!"


            case "o":
                # Need index to overwrite, need new robot pose
                response.success = self.overwrite_waypoint(index, pose)
                if response.success:
                    response.msg = "Waypoint Overwritten!"
                else:
                    response.msg = "Couldnt overwrite waypoint, give good index"

            case "w":
                # Dont need index to overwrite all, need robot pose
                response.success = self.overwrite_all(pose)
                response.msg = "Deleted All And Wrote Robot Pose!"


            case "d":
                # Need index to delete, dont need new robot pose
                response.success = self.delete_waypoint(index)
                if not response.success:
                    response.msg = "Waypoint couldnt be deleted"
                else:
                    response.msg = "Waypoint Deleted!"


            case "g":
                # Need index to go to, dont need robot pose
                self.get_logger().info(f"Go given with {dock}")
                response.success = self.publish_waypoint(index)
                if response.success: 
                    response.msg = "Waypoint Published!"
                else:
                    response.msg = "Waypoint couldnt be published, give a good index"

            case "s":
                self.get_logger().info("Saving marker pose")
                self.save_marker_pose()
                response.success = True
                response.msg = 'Saved Marker Pose'

            case _:
                self.get_logger().info("Bad Flag Given: Try 'h' for help, or one of ['a', 'o', 'w', 'd', 'g'] for functionality")
                response.success = True
                response.msg = "Bad Flag Given"
               
        with open(self.file_path, 'w') as file:
            dict_str = json.dumps(self.pose_dict, indent=4)
            file.write(dict_str)
        return response 
    

    def save_marker_pose(self):
        if self.docking:
            client = self.create_client(Trigger, 'pre_docker_offset')

            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Pre Docking Offset Service not available, trying again...')
        
            client_request = Trigger.Request()
            self.get_logger().info('Calling Pre-Docking Offset')
            client_future = client.call_async(client_request)

    def append_write(self, pose, ind=None):
        self.get_logger().info(f'Ind = {ind}')
        if ind == None:
            ind = len(self.pose_dict.keys())
        self.get_logger().info(f'Ind = {ind}')
        self.pose_dict[ind] = pose
        return True
    
    def overwrite_all(self, pose):
        self.pose_dict.clear()
        return self.append_write(pose)

    def overwrite_waypoint(self, index, pose):
        if not self.good_index(index):
            return False
        
        return self.append_write(pose, ind=index)

    def delete_waypoint(self, index):
        if not self.good_index(index):
            return False

        print(self.pose_dict)
        del self.pose_dict[index]

        self.restructure() # Makes indices of pose_dict make sense
        return True

    def publish_waypoint(self, index):
        if not self.good_index(index):
            return False
        

        pose = self.pose_dict[index]

        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.pose.position.x = float(pose[0])
        goal_pose.pose.pose.position.z = float(pose[2])
        goal_pose.pose.pose.position.y = float(pose[1])
        goal_pose.pose.pose.orientation.x = float(pose[3])
        goal_pose.pose.pose.orientation.y = float(pose[4])
        goal_pose.pose.pose.orientation.z = float(pose[5])
        goal_pose.pose.pose.orientation.w = float(pose[6])

        goal_future = self.action_client.send_goal_async(goal_pose)
        goal_future.add_done_callback(self.goal_response)

        return True
    
    def publish_amount(self):
        # This function is used to publish the amount of waypoints in the dict
        with open(self.file_path, 'r') as file:
            pose_dict = self.create_pose_dict(file)
        
        msg = Int32()
        msg.data = len(pose_dict.keys())
        self.pub.publish(msg)

############################################### Helper Functions ##################################################################################
    
    def goal_response(self, future):
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future):
        result = future.result().result
        if result:
            # Nav2 completed, can do predocking
            if self.docking:
                client = self.create_client(Trigger, 'pre_docker_docking')

                while not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Pre Docking Docking Service not available, trying again...')
                
                client_request = Trigger.Request()
                self.get_logger().info('Calling Pre-Docking Docking')
                client_future = client.call_async(client_request)

            self.get_logger().error("Nav2 complete!")
        
        else:
            self.get_logger().error("Nav2 failed to complete!")
    
    def pre_docker_response(self, future):
        future_handle = future.result
        result_future = future_handle.get_result_async()
        result_future.add_done_callback(self.pre_docker_result)
    
    def pre_docker_result(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Pre Docking Complete')
        else:
            self.get_logger().info("Pre Docker Error, Couldn't Complete")
    
    def good_index(self, index):
        if str(index) not in list(self.pose_dict.keys()):
            return False
        return True

    def post_help_msg(self):
        # Used in h flag and bad flag to get people up to speed
        pass
    
    def create_pose_dict(self, file):
        content = file.read()
        if os.path.getsize(self.file_path) == 0:
            return {}
        pose_dict = json.loads(content)
        return pose_dict
    
    def create_pose_array(self, pose):
        pose_array = [pose.position.x, pose.position.y, pose.position.z, 
                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return pose_array
    
    def restructure(self):
        # This function is used to make sure that the indices of the pose_dict are sequential
        # for safety
        new_dict = {}
        for i, key in enumerate(sorted(self.pose_dict.keys(), key=int)):
            new_dict[str(i)] = self.pose_dict[key]
        self.pose_dict = new_dict

####################################################### End Helpers ##################################################################################

def main(args=None):
    rclpy.init(args=args)

    waypoint_srv = WaypointSrv()

    rclpy.spin(waypoint_srv)
    
    rclpy.shutdown()

if __name__ == '__main__':
     main()
