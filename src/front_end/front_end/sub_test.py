#!/usr/bin/env python3
import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ROS2Node(Node):
    def __init__(self):
        super().__init__('qt_gui_node')
        self.subscription = self.create_subscription(
            Int32,
            '/waypoint_amount',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.latest_message = "No message yet"

    def listener_callback(self, msg):
        self.latest_message = str(msg.data)
        self.get_logger().info(f'I heard: "{msg.data}"')



class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        
        self.setWindowTitle("ROS 2 Qt GUI")
        self.setGeometry(100, 100, 400, 300)
        
        # Create central widget and layout
        central_widget = QWidget()
        layout = QVBoxLayout()
        
        # Add widgets
        self.label = QLabel("ROS 2 Messages will appear here")
        
        layout.addWidget(self.label)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        
        # Timer to update GUI with ROS messages
        self.timer = self.startTimer(100)  # Update every 100ms


    def timerEvent(self, event):
        # Update GUI with latest ROS message
        self.label.setText(f"Latest ROS message: {self.ros_node.latest_message}")

def main():
    # Initialize ROS 2
    rclpy.init()
    
    # Create ROS 2 node
    ros_node = ROS2Node()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create and show main window
    window = MainWindow(ros_node)
    window.show()
    
    # Use a single executor for both Qt and ROS 2
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ros_node)
    
    # Main loop
    while True:
        # Process Qt events
        app.processEvents()
        
        # Process ROS 2 events with timeout
        executor.spin_once(timeout_sec=0.01)
        
        # Exit if window is closed
        if not window.isVisible():
            break
    
    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()