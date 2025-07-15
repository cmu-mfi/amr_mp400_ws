from PySide6.QtWidgets import (QWidget, QApplication, QVBoxLayout, QHBoxLayout, 
                              QLabel, QPushButton, QGraphicsView, QGraphicsScene, QMenu)
from PySide6.QtCore import Qt, QRect, QPoint, QTimer
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QImage, QPixmap
import rclpy
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import numpy as np
from waypoint_maker.publishPoint import WaypointClientAsync

class DockingGUI(QWidget):
    def __init__(self, ros_node=None):
        super().__init__()
        self.ros_node = ros_node
        self.bridge = CvBridge()
        self.setWindowTitle("Robot Control Panel")
        self.showFullScreen()
        
        # Main layout
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)
         
        # Left panel - Camera views
        self.camera_panel = QVBoxLayout()
        self.front_cam_view = CameraView("Front Camera")
        self.back_cam_view = CameraView("Rear Camera")
        self.camera_panel.addWidget(self.front_cam_view)
        self.camera_panel.addWidget(self.back_cam_view)
        
        # Right panel - Controls
        self.control_panel = QVBoxLayout()
        
        # Docking controls
        self.docking_control = QHBoxLayout()
        self.docking_toggle = QPushButton("Docking Mode: OFF")
        self.docking_toggle.setCheckable(True)
        self.docking_toggle.setStyleSheet("""
            QPushButton {
                background-color: #FF6B6B;
                font-size: 24px;
                padding: 15px;
            }
            QPushButton:checked {
                background-color: #6BFF6B;
            }
        """)
        self.docking_toggle.toggled.connect(self.toggle_docking_mode)
        
        self.save_dock_button = QPushButton("Save Docking Position")
        self.save_dock_button.setStyleSheet("""
            QPushButton {
                background-color: #6B6BFF;
                font-size: 24px;
                padding: 15px;
            }
        """)
        
        self.save_dock_button.clicked.connect(self.save_docking_position)
        
        self.docking_control.addWidget(self.docking_toggle)
        self.docking_control.addWidget(self.save_dock_button)
        
        # Waypoint buttons
        self.waypoint_panel = QVBoxLayout()
        self.waypoint_buttons = []
        for i in range(5):  # Create 5 waypoint buttons
            btn = WaypointButton(btn_id=i, name=f"Waypoint {i}", ros_node=ros_node)
            self.waypoint_buttons.append(btn)
            self.waypoint_panel.addWidget(btn)
        
        # Add all to control panel
        self.control_panel.addLayout(self.docking_control)
        self.control_panel.addLayout(self.waypoint_panel)
        
        # Add panels to main layout
        main_layout.addLayout(self.camera_panel, 60)  # 60% width
        main_layout.addLayout(self.control_panel, 40)  # 40% width
        self.setup_escape_controls()
        
        # ROS subscriptions
        self.docking_state_sub = self.ros_node.create_subscription(
            Bool, '/docking_mode', self.docking_state_callback, 10)
        
        self.front_cam_sub = self.ros_node.create_subscription(
            Image, '/front_camera/marker_publisher/result', self.front_cam_callback, 10)
            
        self.back_cam_sub = self.ros_node.create_subscription(
            Image, '/robot1/marker_publisher/result', self.back_cam_callback, 10)
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_views)
        self.update_timer.start(30)  # ~30 FPS

    def setup_escape_controls(self):
        # Escape key to exit fullscreen
        self.setFocusPolicy(Qt.StrongFocus)
        
        # Right-click menu
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_context_menu)
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.showNormal()  # Or self.close()
        super().keyPressEvent(event)
    
    def show_context_menu(self, pos):
        menu = QMenu()
        
        fullscreen_action = menu.addAction(
            "Exit Fullscreen" if self.isFullScreen() else "Fullscreen"
        )
        minimize_action = menu.addAction("Minimize")
        exit_action = menu.addAction("Exit")
        
        action = menu.exec_(self.mapToGlobal(pos))
        
        if action == fullscreen_action:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        elif action == minimize_action:
            self.showMinimized()
        elif action == exit_action:
            self.close()
    
    def closeEvent(self, event):
        # Cleanup code if needed
        super().closeEvent(event)
    
    def toggle_docking_mode(self, checked):
        state = "ON" if checked else "OFF"
        self.docking_toggle.setText(f"Docking Mode: {state}")
        self.call_ros_service("/set_docking_mode", checked)
    
    def save_docking_position(self):
        self.call_ros_service("/save_docking_position")
    
    def docking_state_callback(self, msg):
        self.docking_toggle.blockSignals(True)
        self.docking_toggle.setChecked(msg.data)
        state = "ON" if msg.data else "OFF"
        self.docking_toggle.setText(f"Docking Mode: {state}")
        self.docking_toggle.blockSignals(False)
    
    def front_cam_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            self.front_cam_view.set_image(QImage(
                cv_image.data, w, h, bytes_per_line, 
                QImage.Format_BGR888
            ))
        except Exception as e:
            self.ros_node.get_logger().error(f"Front cam error: {str(e)}")
    
    def back_cam_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            self.back_cam_view.set_image(QImage(
                cv_image.data, w, h, bytes_per_line, 
                QImage.Format_BGR888
            ))
        except Exception as e:
            self.ros_node.get_logger().error(f"Rear cam error: {str(e)}")
    
    def call_ros_service(self, service_name, *args):
        client = self.ros_node.create_client(Trigger, service_name)
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().warn(f"Service {service_name} not available")
            return
            
        req = Trigger.Request()
        # Add your custom request fields here if needed
        future = client.call_async(req)
        future.add_done_callback(
            lambda future: self.ros_node.get_logger().info(
                f"Service {service_name} call {'succeeded' if future.result().success else 'failed'}"
            )
        )
    
    def update_views(self):
        self.front_cam_view.update()
        self.back_cam_view.update()

class CameraView(QGraphicsView):
    def __init__(self, title):
        super().__init__()
        self.setScene(QGraphicsScene())
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setAlignment(Qt.AlignCenter)
        self.title = title
        self.current_image = None
        
        # Title label
        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 24px;
                background-color: rgba(0, 0, 0, 150);
                padding: 5px;
            }
        """)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setFixedWidth(self.width())
        self.title_label.move(10, 10)
        
    def set_image(self, qimage):
        self.current_image = qimage
        
    def update(self):
        if self.current_image:
            pixmap = QPixmap.fromImage(self.current_image)
            self.scene().clear()
            self.scene().addPixmap(pixmap)
            self.setSceneRect(pixmap.rect())
            self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)
            
            # Update title label position
            self.title_label.setFixedWidth(self.width())
            self.title_label.move(10, 10)

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
