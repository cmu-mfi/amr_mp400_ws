from urllib import response
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                              QPushButton, QMenu)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from classes.Camera_class import CameraView
from classes.Button_class import WaypointButton
from waypoint_maker.waypoint_client import WaypointClientAsync


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
        self.dock = 'n'
        
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
        if state == "ON":
            self.dock = 'd'
        else:
            self.dock = 'n'
        self.docking_toggle.setText(f"Docking Mode: {state}")
    
    def save_docking_position(self):
        self.call_ros_service("s")
    
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
        client = WaypointClientAsync()
        
        future = client.send_request(
            flag=ord(service_name[0]),  # Assuming service_name is a single character
            index=0,
            docking=ord(self.dock)
        )

        # Add your custom request fields here if needed
        if not future.success:
            print(f'Action {service_name[0]} failed with response {future.msg}')
        else:
            print(f'Action {service_name[0]} succeded with response {future.msg}')
    
    def update_views(self):
        self.front_cam_view.update()
        self.back_cam_view.update()