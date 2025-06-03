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

        pose = self.latest
        pose_arr = self.create_pose_array(pose)

        match flag:

            case 'h':
                # Help message for new users/reminders
                pass

            case "a":
                # Dont need index to add, need robot pose
                self.open_file(flag)
                response.success = self.simple_write(pose_arr)
                response.msg = "Wrote Robot Pose!"


            case "o":
                # Need index to overwrite, need new robot pose
                flag = 'r+' # r/w to file, file pointer at start of file
                self.open_file(flag)
                response.success = self.overwrite_waypoint(index, pose_arr)
                if response.success:
                    response.msg = "Waypoint Overwritten!"
                else:
                    response.msg = "Couldnt overwrite waypoint, give good index"

            case "w":
                # Dont need index to overwrite all, need robot pose
                self.open_file(flag)
                response.success = self.simple_write(pose_arr)
                response.msg = "Deleted All And Wrote Robot Pose!"


            case "d":
                flag = 'r+' # r/w to file, file pointer at start of file
                # Need index to delete, dont need new robot pose
                self.open_file(flag)
                response.success = self.delete_waypoint(index)
                response.msg = "Waypoint Deleted!"


            case "g":
                flag = 'r' # read file, file pointer at start of file
                # Need index to go to, dont need robot pose
                self.open_file(flag)
                response.success = self.publish_waypoint(index)
                if response.success: 
                    response.msg = "Waypoint Published!"
                else:
                    response.msg = "Waypoint couldnt be published, give a good index"


            case _:
                self.get_logger().info("Bad Flag Given: Try 'h' for help, or one of ['a', 'o', 'w', 'd', 'g'] for functionality")
                response.success = True
                response.msg = "Bad Flag Given"
                

        return response
    
    def simple_write(self, pose):
        # Appending and overwriting all are the same process internally, but just with different file opening methods
        self.file.write(f'{pose} \n')
        return True

    def overwrite_waypoint(self, index, pose):
        success = self.write_to_line(index, 'o', pose=pose)
        return success

    def delete_waypoint(self, index):
        success = self.write_to_line(index, 'd')
        return success

    def publish_waypoint(self, index):
        lines = self.file.readlines()
        if not self.good_index(index, lines):
            return False
        

        pose = self.create_arr_from_string(lines, index)

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
    def good_index(self, index, lines):
        len = len(lines)
        if index < 0 or index >= len:
            return False
        return True

    def open_file(self, flag):
        # Current Working Directory when run is wherever you run it, but want relative to program (the directory it is in)
        dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(dir, "robot_poses.txt")
        print(file_path)
        if not os.path.exists(file_path):
            self.file = open(file_path, 'w')
        else: 
            self.file = open(file_path, flag)

        return
    
    def write_to_line(self, index, flag, pose = None):
        # Used for both overwrite and delete as they are basically the same process, just different things are written to the line
        lines = self.file.readlines()
        if not self.good_index(index, lines):
            return False

        if flag == 'd':
            text = ""
        elif flag == 'o':
            text = f"{pose} \n"
        else:
            return False
        
        lines[index] = text

        self.file.writelines(lines)

        return True

    def post_help_msg(self):
        # Used in h flag and bad flag to get people up to speed
        pass
    
    def create_pose_array(self, pose):
        pose_array = [pose.position.x, pose.position.y, pose.position.z, 
                                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return pose_array

    def create_arr_from_string(self, lines, index):
        arr = []
        for num in lines[index].strip()[1:-2].split(','):
            arr.append(float(num))

        return arr

####################################################### End Helpers ##################################################################################

def main(args=None):
    rclpy.init(args=args)

    waypoint_srv = WaypointSrv()

    rclpy.spin(waypoint_srv)
    
    rclpy.shutdown()

if __name__ == '__main__':
     main()