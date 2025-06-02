import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from amr_mp400_interfaces.srv import SetFlag

class WaypointSrv(Node):

    def __init__(self):
         super().__init__('waypoint_srv')
         self.srv = self.create_service(SetFlag, 'waypoint_maker', self.waypoint_callback)


    def waypoint_callback(self, request, response):
        # What do we need to do here? 
        # - Differentiate between wanting to add/overwrite/delete/go
        # - Add a marker for docking stations
        # Flags for operations -> a - Add / o - overwrite / w - delete all and add new / d - delete / g - go

        pass



def main(args=None):
    rclpy.init(args=args)

    waypoint_srv = WaypointSrv()

    rclpy.spin(waypoint_srv)
    
    rclpy.shutdown()

if __name__ == '__main__':
     main()