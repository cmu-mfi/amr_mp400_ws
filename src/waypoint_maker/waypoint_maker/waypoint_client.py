import sys
import rclpy

from rclpy.node import Node
from amr_mp400_interfaces.srv import SetFlag

class WaypointClientAsync(Node):

    def __init__(self):
        super().__init__('waypoint_client_async')

        self.client = self.create_client(SetFlag, 'waypoint_maker')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = SetFlag.Request()
    
    def send_request(self, flag, index, docking):
        self.request.flag = flag
        self.request.index = index
        self.request.docking = docking

        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    


def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WaypointClientAsync()

    response = waypoint_client.send_request(ord(sys.argv[1]), int(sys.argv[2]), ord(sys.argv[3]))

    waypoint_client.get_logger().info(
        'Success: %d \nReturn Message: %s' % (response.success, response.msg)
    )

    waypoint_client.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
