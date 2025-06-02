import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

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

        self.pub = None
        self.file = None

        self.latest = None

    def sub_callback(self, msg: PoseWithCovarianceStamped):
        self.latest = msg.pose.pose

    def waypoint_callback(self, request, response):
        # What do we need to do here? 
        # - Differentiate between wanting to add/overwrite/delete/go
        # - Add a marker for docking stations
        # Flags for operations -> a - Add / o - overwrite / w - delete all and add new / d - delete / g - go
        flag = request.flag
        index = request.index
        

        # Have to create a subscriber to PoseStampedWithCovariance (?) that gets the pose of the robot to use
        # How do I do the subscriber thing / publisher thing <- Just start writing code bruh
        # From the subscriber need to get the pose that basically all functions use

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
                response.success = self.add_waypoint(pose_arr)

                pass

            case "o":
                # Need index to overwrite, need new robot pose
                flag = 'r+' # r/w to file, file pointer at start of file
                self.open_file(flag)
                response.success = self.overwrite_waypoint(index, pose_arr)
                pass

            case "w":
                # Dont need index to overwrite all, need robot pose
                self.open_file(flag)
                response.success = self.overwrite_all(pose_arr)

                pass

            case "d":
                flag = 'r+' # r/w to file, file pointer at start of file
                # Need index to delete, dont need new robot pose
                self.open_file(flag)
                response.success = self.delete_waypoint(index)

                pass

            case "g":
                flag = 'r+' # r/w to file, file pointer at start of file
                # Need index to go to, dont need robot pose
                self.open_file(flag)
                response.success = self.publish_waypoint(index)

                pass

            case _:
                self.get_logger().info("Bad Flag Given: Try 'h' for help, or one of ['a', 'o', 'w', 'd', 'g'] for functionality")
                response.success = True
                
                pass

        pass

    def add_waypoint(self, pose):
        self.file.write(f'{pose} \n')
        return True

    def overwrite_waypoint(self, index, pose):
        success = self.write_to_line(index, 'o', pose=pose)
        return success

    def overwrite_all(self, pose):
        pass

    def delete_waypoint(self, index):
        success = self.write_to_line(index, 'd')
        return success

    def publish_waypoint(self, index):
        pass

    def good_index(self, index):
        pass

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
        if index < 0 or index >= len(lines):
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




def main(args=None):
    rclpy.init(args=args)

    waypoint_srv = WaypointSrv()

    rclpy.spin(waypoint_srv)
    
    rclpy.shutdown()

if __name__ == '__main__':
     main()