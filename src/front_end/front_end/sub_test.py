import sys
from std_msgs.msg import Int32
from PySide6 import QtCore, QtWidgets, QtGui
from PySide6.QtWidgets import QApplication
import rclpy
from rclpy.node import Node

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            Int32,
            '/waypoint_amount',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.msg_count = 0

    def listener_callback(self, msg):
        print(f'Received: {msg.data}')
        self.msg_count = msg.data
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('ROS Subscriber Test')
        self.setGeometry(100, 100, 400, 300)
        self.label = QtWidgets.QLabel('Waiting for messages...', self)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.setCentralWidget(self.label)

        self.timer = self.startTimer(100)


    def update_label(self):
        self.label.setText(str(self.node.msg_count))

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    sub = SubscriberNode()
    window = MainWindow(sub)
    window.show()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(sub)

    while rclpy.ok():
        window.update_label()
        executor.spin_once(timeout_sec=0.1)

        if not window.isVisible():
            break

    sub.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()



