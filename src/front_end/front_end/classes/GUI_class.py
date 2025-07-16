from urllib import response
from PySide6.QtWidgets import (QLabel, QWidget, QVBoxLayout, QHBoxLayout, 
                              QPushButton, QMenu, QStackedWidget)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from classes.Camera_class import CameraView
from classes.Button_class import WaypointButton
from waypoint_maker.waypoint_client import WaypointClientAsync
from rqt_gui_py.embed import EmbedWidget

class BaseScreen(QWidget):
    back_requested = Signal()
    
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node
        self.bridge = CvBridge()
        
    def on_show(self):
        """Called when screen becomes visible"""
        pass
        
    def on_hide(self):
        """Called when screen is hidden"""
        pass

class HomeScreen(BaseScreen):
    def __init__(self, ros_node, parent=None):
        super().__init__(ros_node, parent)
        layout = QVBoxLayout()
        
        title = QLabel("Robot Control System")
        title.setStyleSheet("font-size: 24px; font-weight: bold;")
        layout.addWidget(title)
        
        # Navigation buttons
        btn_manual = QPushButton("Manual Control")
        btn_manual.clicked.connect(lambda: self.parent().navigate_to("manual"))
        btn_manual.setStyleSheet("font-size: 18px; padding: 15px;")
        
        btn_mapping = QPushButton("Mapping Mode")
        btn_mapping.clicked.connect(lambda: self.parent().navigate_to("mapping"))
        btn_mapping.setStyleSheet("font-size: 18px; padding: 15px;")
        
        layout.addWidget(btn_manual)
        layout.addWidget(btn_mapping)
        layout.addStretch()
        
        self.setLayout(layout)

class RVizScreen(BaseScreen):
    def __init__(self, ros_node, parent=None):
        super().__init__(ros_node, parent)
        layout = QVBoxLayout()
        
        # Header with back button
        header = QHBoxLayout()
        btn_back = QPushButton("← Home")
        btn_back.clicked.connect(self.back_requested.emit)
        header.addWidget(btn_back)
        header.addStretch()
        layout.addLayout(header)
        
        # RViz embed
        self.rviz_widget = EmbedWidget(self, "rviz")
        self.rviz_widget.setObjectName("RViz")
        layout.addWidget(self.rviz_widget)
        
        self.setLayout(layout)

class ManualControlScreen(BaseScreen):
    def __init__(self, ros_node, parent=None):
        super().__init__(ros_node, parent)
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
        
        # Header with back button
        header = QHBoxLayout()
        btn_back = QPushButton("← Home")
        btn_back.clicked.connect(self.back_requested.emit)
        header.addWidget(btn_back)
        header.addStretch()
        self.control_panel.addLayout(header)
        
        # Your existing manual controls
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
        for i in range(5):
            btn = WaypointButton(btn_id=i, name=f"Waypoint {i}", ros_node=ros_node)
            self.waypoint_buttons.append(btn)
            self.waypoint_panel.addWidget(btn)
        
        self.control_panel.addLayout(self.docking_control)
        self.control_panel.addLayout(self.waypoint_panel)
        
        # Add panels to main layout
        main_layout.addLayout(self.camera_panel, 60)
        main_layout.addLayout(self.control_panel, 40)
        
        # ROS subscriptions
        self.front_cam_sub = ros_node.create_subscription(
            Image, '/front_camera/marker_publisher/result', self.front_cam_callback, 10)
        self.back_cam_sub = ros_node.create_subscription(
            Image, '/robot1/marker_publisher/result', self.back_cam_callback, 10)
        
        # Timer for updates
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_views)
        self.update_timer.start(30)
    
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

class RobotControlApp(QWidget):
    def __init__(self, ros_node=None):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Robot Control System")
        self.showFullScreen()
        
        # Screen management
        self.stacked_widget = QStackedWidget()
        self.screens = {
            "home": HomeScreen(ros_node, self),
            "manual": ManualControlScreen(ros_node, self),
            "rviz": RVizScreen(ros_node, self)
        }
        
        for name, screen in self.screens.items():
            self.stacked_widget.addWidget(screen)
            screen.back_requested.connect(lambda: self.navigate_to("home"))
        
        self.setLayout(QHBoxLayout())
        self.layout().addWidget(self.stacked_widget)
        
        # Start with home screen
        self.navigate_to("home")
        
        # Escape controls
        self.setup_escape_controls()
    
    def navigate_to(self, screen_name):
        current_screen = self.screens.get(self.stacked_widget.currentWidget().objectName())
        if current_screen:
            current_screen.on_hide()
        
        self.stacked_widget.setCurrentWidget(self.screens[screen_name])
        self.screens[screen_name].on_show()
    
    def setup_escape_controls(self):
        self.setFocusPolicy(Qt.StrongFocus)
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_context_menu)
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.showNormal()
        super().keyPressEvent(event)
    
    def show_context_menu(self, pos):
        menu = QMenu()
        fullscreen_action = menu.addAction(
            "Exit Fullscreen" if self.isFullScreen() else "Fullscreen"
        )
        exit_action = menu.addAction("Exit")
        
        action = menu.exec_(self.mapToGlobal(pos))
        if action == fullscreen_action:
            self.showFullScreen() if not self.isFullScreen() else self.showNormal()
        elif action == exit_action:
            self.close()