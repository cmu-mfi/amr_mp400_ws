from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QRect
from PySide6.QtGui import QPainter, QColor, QPen, QBrush
from std_msgs.msg import Int32
from std_srvs.srv import Trigger


class WaypointButton(QWidget):
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
             "color": QColor(144, 238, 144), "action": "go"},
            {"name": "Overwrite", "rect": QRect(width//3, 0, width//3, height), 
             "color": QColor(255, 255, 153), "action": "overwrite"},
            {"name": "Delete", "rect": QRect(2*width//3, 0, width//3, height), 
             "color": QColor(255, 182, 193), "action": "delete"}
        ]
        
        self.inactive_section = {
            "name": "Append", 
            "rect": QRect(0, 0, width, height), 
            "color": QColor(173, 216, 230), 
            "action": "append"
        }
        
        self.hovered_action = None
    
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
            
        service_name = f"/waypoint_{self.hovered_action}"
        self.call_ros_service(service_name, self.btn_id, self.name)
    
    def call_ros_service(self, service_name, waypoint_id, waypoint_name):
        client = self.ros_node.create_client(Trigger, service_name)
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().warn(f"Service {service_name} not available")
            return
            
        req = Trigger.Request()
        # Add your custom request fields here
        # req.waypoint_id = waypoint_id
        # req.waypoint_name = waypoint_name
        
        future = client.call_async(req)
        future.add_done_callback(
            lambda future: self.ros_node.get_logger().info(
                f"Called {service_name} for waypoint {waypoint_id} ({waypoint_name})"
            )
        )