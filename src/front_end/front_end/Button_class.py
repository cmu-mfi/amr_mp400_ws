from cProfile import label
import os, subprocess
from pathlib import Path
from PySide6.QtWidgets import QWidget, QInputDialog 
from PySide6.QtCore import Qt, QRect, Signal
from PySide6.QtGui import QPainter, QColor, QPen, QBrush
from std_msgs.msg import Int32
from waypoint_maker.waypoint_client import WaypointClientAsync
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node

class WaypointButton(QWidget):
    go_section_clicked = Signal()
    rviz_closed = Signal()

    def __init__(self, parent=None, btn_id=0, name="", x=0, y=0, width=400, height=80, ros_node=None):
        super().__init__(parent)
        self.ros_node: Node = ros_node
        self.btn_id = btn_id
        self.name = name
        self.setGeometry(x, y, width, height)
        self.active = False
        self.setMouseTracking(True)
        self.timer = None
        self.parent = parent
        
        # Initialize ROS 2 subscriber for active state
        self.active_sub = self.ros_node.create_subscription(
            Int32,
            '/waypoint_amount',
            self.active_state_callback,
            10
        )
        
        # Section definitions (different when active/inactive)
        self.active_sections = [
            {"name": "Go", "rect": QRect(0, 0, width//4, height), 
             "color": QColor(144, 238, 144), "action": "g"},
            {"name": "Overwrite", "rect": QRect(width//4, 0, width//4, height), 
             "color": QColor(255, 255, 153), "action": "o"},
            {"name": "Delete", "rect": QRect(2*width//4, 0, width//4, height), 
             "color": QColor(255, 182, 193), "action": "d"},
             {"name": "Localize", "rect": QRect(3*width//4, 0, width//4, height),
             "color": QColor(173, 216, 230), "action": "l"
             }
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

                section_rect = section["rect"]
                painter.drawText(
                    QRect(section_rect.x(), section_rect.y(), 
                        section_rect.width(), section_rect.height()//2),
                    Qt.AlignCenter, 
                    section["name"]
                )
                if self.name:
                    painter.drawText(
                        QRect(section_rect.x(), section_rect.y() + section_rect.height()//2,
                            section_rect.width(), section_rect.height()//2),
                        Qt.AlignCenter, 
                        self.name
                    )
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
        
        if self.hovered_action in ['a', 'o']:
            label, ok = QInputDialog.getText(
                self,
                f"Waypoint {self.btn_id} Label",
                "Enter waypoint label:",
                text=self.name if self.name else f"Waypoint {self.btn_id}"
            )
            if not ok:  # User cancelled
                self.name = ""
            else: 
                self.name = label
            self.update()  # Refresh the button display

        client = WaypointClientAsync()
        self.call_ros_service(client, self.hovered_action)
        
    
    def call_ros_service(self, client: WaypointClientAsync, action):
        future = client.send_request(
            flag=ord(action),
            index=self.btn_id,
            docking=ord(self.parent.docking),
            label=self.name
        )
        
        if not future.success:
            print(f'Action {action} failed with response {future.msg}')
        else:
            print(f'Action {action} succeded with response {future.msg}')
        if action == "g":
            self.launch_rviz()
    
    def launch_rviz(self):
        try:
           
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

    #     self._setup_status_monitoring()

    # def _setup_status_monitoring(self):
    #     self.status_sub = self.ros_node.create_subscription(
    #         GoalStatusArray,
    #         f'/{self.robot_namespace}/navigate_to_pose/_action/status',
    #         self.handle_status,
    #         10
    #     )

    # def handle_status(self, msg: GoalStatusArray):
    #     self.ros_node.get_logger().info("Checking latest Rviz status")
    #     status = msg.status_list[len(msg.status_list) - 1]
    #     self.ros_node.get_logger().info(f'Status: {status.status}')
    #     if status.status == 4 or status.status == 0:  # SUCCEEDED
    #         print(f"Process finished with status: {status.status}")
    #         self.ros_node.get_logger().info("Terminating Rviz")
    #         self.terminate_rviz()

    # def terminate_rviz(self):
    #     if hasattr(self, 'rviz_process') and self.rviz_process:
    #         try:
    #             # Terminate the process
    #             self.rviz_process.kill()
    #             self.ros_node.get_logger().info("Post Kill")
                    
    #             # Clean up
    #             self.rviz_process = None
    #             # if hasattr(self, 'status_sub'):
    #             #     self.ros_node.destroy_subscription(self.status_sub)
    #             #     self.ros_node.get_logger().info("Destroyed Status Sub")
    #             #     self.status_sub = None
                
    #             print("RViz terminated successfully")
    #             self.rviz_closed.emit()  # Notify other components

    #             # self.ros_node.destroy_timer(self.timer)
    #             # self.ros_node.get_logger().info("Destroyed Timer")
                
    #         except Exception as e:
    #             print(f"Error terminating RViz: {str(e)}")