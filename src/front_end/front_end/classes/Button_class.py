import os, subprocess, threading
from pathlib import Path
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QRect, Signal, QObject
from PySide6.QtGui import QPainter, QColor, QPen, QBrush
from std_msgs.msg import Int32
from waypoint_maker.waypoint_client import WaypointClientAsync
import rclpy
from rclpy.node import Node

class WaypointButton(QWidget):
    go_section_clicked = Signal()
    rviz_closed = Signal()

    def __init__(self, parent=None, btn_id=0, name="", x=0, y=0, width=400, height=80, ros_node=None):
        super().__init__(parent)
        self.ros_node = ros_node
        self.btn_id = btn_id
        self.name = name
        self.setGeometry(x, y, width, height)
        self.active = False
        self.setMouseTracking(True)
        
        # Initialize ROS 2 subscriber for active state
        self.active_sub = self.ros_node.create_subscription(
            Int32,
            '/waypoint_amount',
            self.active_state_callback,
            10
        )
        
        # Section definitions (different when active/inactive)
        self.active_sections = [
            {"name": "Go", "rect": QRect(0, 0, width//3, height), 
             "color": QColor(144, 238, 144), "action": "g"},
            {"name": "Overwrite", "rect": QRect(width//3, 0, width//3, height), 
             "color": QColor(255, 255, 153), "action": "o"},
            {"name": "Delete", "rect": QRect(2*width//3, 0, width//3, height), 
             "color": QColor(255, 182, 193), "action": "d"}
        ]
        
        self.inactive_section = {
            "name": "Append", 
            "rect": QRect(0, 0, width, height), 
            "color": QColor(173, 216, 230), 
            "action": "a"
        }
        
        self.hovered_action = None

        self.status_sub = None
        self.last_status = None
        self.robot_namespace = "robot1"
    
    def active_state_callback(self, msg):
        """Update active state based on ROS topic"""
        self.active = (self.btn_id < msg.data)
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        
        # Draw button background
        if self.active:
            # Active state - three sections
            for section in self.active_sections:
                brush = QBrush(section["color"])
                if self.hovered_action == section["action"]:
                    brush.setColor(section["color"].lighter(120))
                painter.setBrush(brush)
                painter.drawRect(section["rect"])
                painter.drawText(section["rect"], Qt.AlignCenter, section["name"])
        else:
            # Inactive state - single section with dotted border
            pen.setStyle(Qt.DotLine)
            painter.setPen(pen)
            brush = QBrush(self.inactive_section["color"])
            if self.hovered_action == self.inactive_section["action"]:
                brush.setColor(self.inactive_section["color"].lighter(120))
            painter.setBrush(brush)
            painter.drawRect(self.inactive_section["rect"])
            painter.drawText(self.inactive_section["rect"], Qt.AlignCenter, 
                           f"{self.name}\n(Append)" if self.name else "Append")
        
        # Draw button ID and name at the top
        painter.setPen(Qt.black)
        painter.drawText(5, 15, f"ID: {self.btn_id}")
        
        painter.end()
    
    def mouseMoveEvent(self, event):
        pos = event.position().toPoint()
        self.hovered_action = None
        
        if self.active:
            for section in self.active_sections:
                if section["rect"].contains(pos):
                    self.hovered_action = section["action"]
                    break
        else:
            if self.inactive_section["rect"].contains(pos):
                self.hovered_action = self.inactive_section["action"]
        
        self.update()
    
    def mousePressEvent(self, event):
        if not self.hovered_action:
            return
        
        client = WaypointClientAsync()
        self.call_ros_service(client)
        
    
    def call_ros_service(self, client: WaypointClientAsync):
        future = client.send_request(
            flag=ord(self.hovered_action),
            index=self.btn_id,
            docking=0  # Assuming docking is not used here, set to 0
        )
        
        if not future.success:
            print(f'Action {self.hovered_action} failed with response {future.msg}')
        else:
            print(f'Action {self.hovered_action} succeded with response {future.msg}')
        if self.hovered_action == "g":
            self.launch_rviz()
    
    def launch_rviz(self):
        try:
            # Create a temporary launch script
            launch_script = """#!/bin/bash
unset QT_QPA_PLATFORM_PLUGIN_PATH
unset QT_PLUGIN_PATH
export QT_QPA_PLATFORM=xcb
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/mp_400_workspace/install/setup.bash
exec ros2 launch neo_nav2_bringup rviz_launch.py use_namespace:=True namespace:=$ROBOT_NAMESPACE
"""
            script_path = Path("/tmp/launch_rviz.sh")
            script_path.write_text(launch_script)
            script_path.chmod(0o755)
            
            # Launch RViz in a separate process
            env = os.environ.copy()
            env["ROBOT_NAMESPACE"] = self.robot_namespace  # Make sure this is set
            self.rviz_process = subprocess.Popen(
                [str(script_path)],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print("Launched rviz")

            
        except Exception as e:
            print(f"RViz launch error: {str(e)}")

        self._setup_status_monitoring()

    def _setup_status_monitoring(self):
        from action_msgs.msg import GoalStatusArray
        
        self.status_sub = self.ros_node.create_subscription(
            GoalStatusArray,
            f'/{self.robot_namespace}/navigate_to_pose/_action/status',
            self._status_callback,
            10
        )

    def _status_callback(self, msg):
        """Handle status updates from main thread's spin"""
        for status in msg.status_list:
            if status.status == 4 or status.status == 0:  # SUCCEEDED
                print(f"Process finished with status: {status.status}")
                self.terminate_rviz()
                break

    def terminate_rviz(self):
        if hasattr(self, 'rviz_process') and self.rviz_process:
            try:
                # Terminate the process
                self.rviz_process.terminate()
                
                # Wait briefly for clean shutdown
                try:
                    self.rviz_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.rviz_process.kill()  # Force kill if needed
                    
                # Clean up
                self.rviz_process = None
                if hasattr(self, 'status_sub'):
                    self.ros_node.destroy_subscription(self.status_sub)
                    self.status_sub = None
                
                print("RViz terminated successfully")
                self.rviz_closed.emit()  # Notify other components
                
            except Exception as e:
                print(f"Error terminating RViz: {str(e)}")
            

    def closeEvent(self, event):
        self.terminate_rviz()
        super().closeEvent(event)