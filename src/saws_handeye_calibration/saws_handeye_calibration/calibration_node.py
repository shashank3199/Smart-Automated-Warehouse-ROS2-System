import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
import tf2_ros

from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget, QGridLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import numpy as np
from saws_handeye_calibration.axbsolver import AXBsolver


class Calibration(Node):
    """
    A ROS2 node for hand-eye calibration using ArUco markers. This node integrates ROS2 with a PyQt5 GUI to display images
    and record transformations between the camera, marker, base, and tool frames.
    """

    def __init__(self):
        """
        Initialize the Calibration node, set up subscriptions, transform listeners, and the GUI.
        """
        super().__init__('calibration')
        self.image_subscriber = self.create_subscription(Image, '/aruco/result', self.image_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        self.image = None
        self.transforms_camera_marker = []
        self.transforms_base_tool = []

        # GUI setup
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.layout = QVBoxLayout()
        self.grid_layout = QGridLayout()

        self.image_label = QLabel()
        self.layout.addWidget(self.image_label)

        self.button = QPushButton('Record TF')
        self.button.clicked.connect(self.record_tf)
        self.layout.addWidget(self.button)

        self.transform_label = QLabel("Recorded Transforms: 0")
        self.layout.addWidget(self.transform_label)
        self.layout.addLayout(self.grid_layout)

        self.window.setLayout(self.layout)
        self.window.setWindowTitle('Calibration')
        self.window.show()

        # Integrate ROS2 timer with Qt timer
        self.create_timer(0.1, self.update_gui)
        self.qt_timer = QTimer()
        self.qt_timer.timeout.connect(self.ros_spin_once)
        self.qt_timer.start(10)

    def image_callback(self, msg):
        """
        Callback function for image subscription. Converts ROS image messages to OpenCV images.
        """
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')

    def record_tf(self):
        """
        Record the current transformations between camera-marker and base-tool frames.
        """
        try:
            transform_camera_marker = self.tf_buffer.lookup_transform('camera_link', 'marker', rclpy.time.Time())
            transform_base_tool = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            self.transforms_camera_marker.append(transform_camera_marker)
            self.transforms_base_tool.append(transform_base_tool)
            self.get_logger().info('Transforms recorded.')

            self.transform_label.setText(f"Recorded Transforms: {len(self.transforms_camera_marker)}")

            if len(self.transforms_camera_marker) >= 5:
                self.display_transform_matrix()
        except tf2_ros.LookupException as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'Transform extrapolation failed: {e}')

    def display_transform_matrix(self):
        """
        Calculate and display the transformation matrix after recording sufficient transforms.
        """
        e_bh = np.array([self.transform_to_array(transform) for transform in self.transforms_base_tool])
        e_sc = np.array([self.transform_to_array(transform) for transform in self.transforms_camera_marker])
        solution = AXBsolver(e_bh, e_sc)
        matrix = solution.axxb()
        self.update_matrix_display(matrix)

    def transform_to_array(self, transform):
        """
        Convert a TransformStamped object to a list [x, y, z, qx, qy, qz, qw].
        """
        t = transform.transform.translation
        q = transform.transform.rotation
        return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]

    def update_matrix_display(self, matrix):
        """
        Update the GUI to display the transformation matrix.
        """
        # Clear previous matrix display
        for i in reversed(range(self.grid_layout.count())):
            widget = self.grid_layout.itemAt(i).widget()
            if widget is not None:
                widget.deleteLater()

        # Display the matrix
        for i in range(4):
            for j in range(4):
                label = QLabel(f"{matrix[i][j]:.2f}")
                self.grid_layout.addWidget(label, i, j)

    def update_gui(self):
        """
        Update the GUI with the latest image from the camera.
        """
        if self.image is not None:
            height, width, channel = self.image.shape
            bytes_per_line = 3 * width
            q_img = QImage(self.image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.image_label.setPixmap(QPixmap.fromImage(q_img))

    def ros_spin_once(self):
        """
        Spin the ROS2 node once to handle callbacks.
        """
        rclpy.spin_once(self, timeout_sec=0.1)

    def spin(self):
        """
        Start the Qt event loop.
        """
        self.app.exec_()


def main(args=None):
    """
    Main function to initialize the ROS2 node and start the application.
    """
    rclpy.init(args=args)
    node = Calibration()
    node.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
