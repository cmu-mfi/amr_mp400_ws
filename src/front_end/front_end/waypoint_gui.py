from front_end.classes.GUI_class import DockingGUI
import rclpy
from PySide6.QtWidgets import QApplication

def main():
    rclpy.init()
    ros_node = rclpy.create_node('robot_control_gui')
    
    app = QApplication([])
    gui = DockingGUI(ros_node)
    
    # ROS executor in a separate thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ros_node)
    
    import threading
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    app.exec()
    
    # Cleanup
    executor.shutdown()
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()