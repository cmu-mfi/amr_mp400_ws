import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# Publish the state of the robot
class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.state = "Idle"
        path = "/home/neobotix/mp_400_workspace/src/amr_mp400_ws/src/robot_state_publisher/text/dependencies.txt"
        self.file_path = path

        with open(self.file_path, 'r') as file:
            self.dependencies: dict = json.load(file)

        self.create_service('/robot_state_publishing', String, self.get_new_state)
        
        self.pub = self.create_publisher(String, '/robot_state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_new_state(self, request: String, response):
        new_state = request.data
        if new_state not in self.dependencies[f"{self.state}"]:
            self.state = f"Error: Cant transition from {self.state} to {new_state}"
            return
        
        self.state = new_state
        return

    def timer_callback(self):
        msg = String()
        msg.data = self.state
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing robot state: {self.state}')
        