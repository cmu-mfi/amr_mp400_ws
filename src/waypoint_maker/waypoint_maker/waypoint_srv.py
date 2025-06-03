import os
import rclpy
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from amr_mp400_interfaces.srv import SetFlag

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

        self.pub = self.create_publisher(PoseStamped, 'robot1/goal_pose', 10)
        dir = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.join(dir, "robot_poses.txt")
        self.file = None

        self.latest = None

    def sub_callback(self, msg: PoseWithCovarianceStamped):
        self.latest = msg.pose.pose

    def waypoint_callback(self, request, response):
        # What do we need to do here? 
        # - Add a marker for docking stations TODO
        # Flags for operations -> a - Add / o - overwrite / w - delete all and add new / d - delete / g - go
        flag = chr(request.flag)
        index = request.index
        
        if not self.latest:
            response.success = False
            response.msg = "Couldn't get robot pose"
            return response

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
                response.msg = "Waypoint Deleted!"


            case "g":
                # Need index to go to, dont need robot pose
                response.success = self.publish_waypoint(index)
                if response.success: 
                    response.msg = "Waypoint Published!"
                else:
                    response.msg = "Waypoint couldnt be published, give a good index"


            case _:
                self.get_logger().info("Bad Flag Given: Try 'h' for help, or one of ['a', 'o', 'w', 'd', 'g'] for functionality")
                response.success = True
                response.msg = "Bad Flag Given"
                
        with open(self.file_path, 'w') as file:
            dict_str = json.dumps(self.pose_dict, indent=4)
            file.write(dict_str)
        return response 
    
    def append_write(self, pose):
        ind = len(self.pose_dict.keys())
        self.pose_dict[ind] = pose
        return True
    
    def overwrite_all(self, pose):
        self.pose_dict.clear()
        self.pose_dict[0] = pose
        return True

    def overwrite_waypoint(self, index, pose):
        if not self.good_index(index):
            return False
        
        self.pose_dict[index] = pose

        return True

    def delete_waypoint(self, index):
        if not self.good_index(index):
            return False
        
        del self.pose_dict[index]

        return True

    def publish_waypoint(self, index):
        lines = self.file.readlines()
        if not self.good_index(index, lines):
            return False
        

        pose = self.pose_dict[index]

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(pose[0])
        goal_pose.pose.position.z = float(pose[2])
        goal_pose.pose.position.y = float(pose[1])
        goal_pose.pose.orientation.x = float(pose[3])
        goal_pose.pose.orientation.y = float(pose[4])
        goal_pose.pose.orientation.z = float(pose[5])
        goal_pose.pose.orientation.w = float(pose[6])

        self.pub.publish(goal_pose)

############################################### Helper Functions ##################################################################################
    def good_index(self, index):
        if index not in self.pose_dict.keys():
            return False
        return True

    def post_help_msg(self):
        # Used in h flag and bad flag to get people up to speed
        pass
    
    def create_pose_dict(self, file):
        content = file.read()
        pose_dict = json.loads(content)
        return pose_dict
    
    def create_pose_array(self, pose):
        pose_array = [pose.position.x, pose.position.y, pose.position.z, 
                                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return pose_array

####################################################### End Helpers ##################################################################################

def main(args=None):
    rclpy.init(args=args)

    waypoint_srv = WaypointSrv()

    rclpy.spin(waypoint_srv)
    
    rclpy.shutdown()

if __name__ == '__main__':
     main()