import os
from classes.GUI_class import RobotControlApp  # Renamed from DockingGUI
import rclpy
from PySide6.QtWidgets import QApplication

def main():
    rclpy.init()
    ros_node = rclpy.create_node('robot_control_gui')

    os.environ["ROS_DISTRO"] = "humble"  # or your distro
    os.environ["WORKSPACE"] = "~/mp_400_ws/" 
    
    app = QApplication([])
    gui = RobotControlApp(ros_node)
    
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