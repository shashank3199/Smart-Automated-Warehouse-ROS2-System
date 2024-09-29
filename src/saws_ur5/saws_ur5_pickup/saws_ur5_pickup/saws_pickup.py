import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation as R
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
from rclpy.action import ActionClient
from saws_gripper_msgs.action import Gripper

class SawsPickup(Node):
    def __init__(self):
        super().__init__('saws_pickup')

        # Publishers
        self.publisher_ = self.create_publisher(Point, '/setpoint', 10)
        self.joint_publisher_ = self.create_publisher(JointState, '/set_ur5_joints', 10)
        self.gripper_status_publisher_ = self.create_publisher(Bool, '/gripper_picked_status', 10)

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.pickup_block_subscriber = self.create_subscription(Bool, '/pickup_block', self.pickup_block_callback, 10)

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Current joint state and flags
        self.current_joint_state = JointState()
        self.pickup_block_flag = False

        # Load configuration file
        package_share_directory = get_package_share_directory('saws_ur5_pickup')
        config_file_path = os.path.join(package_share_directory, 'config', 'handeye.yaml')

        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)
            translation = config['transform']['translation']
            rotation = config['transform']['rotation']

        self.config_transform = self.create_transform_matrix(translation, rotation)

        # Create the action client for the gripper
        self.gripper_client_ = ActionClient(self, Gripper, '/gripper_control')
        self.yaw_alignment_enabled = False

    def create_transform_matrix(self, translation, rotation):
        """Creates a 4x4 transformation matrix from translation and rotation (quaternion)."""
        translation_matrix_ = np.eye(4)
        translation_matrix_[:3, 3] = translation
        rotation_matrix_ = np.eye(4)
        rotation_matrix_[:3, :3] = R.from_quat(rotation).as_matrix()
        transform_matrix = translation_matrix_ @ rotation_matrix_
        return transform_matrix

    def joint_state_callback(self, msg):
        """Callback to update the current joint state."""
        self.current_joint_state = msg

    def pickup_block_callback(self, msg):
        """Callback to handle the pickup block status."""
        self.pickup_block_flag = msg.data
        if self.pickup_block_flag:
            self.perform_pickup_operations()

    def perform_pickup_operations(self):
        """Performs the pickup operations using the TF transforms and publishes commands."""
        while True:
            try:
                # Lookup transforms
                base_to_gripper = self.tf_buffer.lookup_transform('base_link', 'gripper_pick', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                self.get_logger().info("Base to Gripper transform found.")
                camera_to_marker = self.tf_buffer.lookup_transform('camera_link', 'marker', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                self.get_logger().info("Camera to Marker transform found.")

                # Create transformation matrices
                base_to_gripper_matrix = self.create_transform_matrix(
                    [base_to_gripper.transform.translation.x, base_to_gripper.transform.translation.y, base_to_gripper.transform.translation.z],
                    [base_to_gripper.transform.rotation.x, base_to_gripper.transform.rotation.y, base_to_gripper.transform.rotation.z, base_to_gripper.transform.rotation.w]
                )

                camera_to_marker_matrix = self.create_transform_matrix(
                    [camera_to_marker.transform.translation.x, camera_to_marker.transform.translation.y, camera_to_marker.transform.translation.z],
                    [camera_to_marker.transform.rotation.x, camera_to_marker.transform.rotation.y, camera_to_marker.transform.rotation.z, camera_to_marker.transform.rotation.w]
                )

                # Compute the resultant matrix
                resultant_matrix = base_to_gripper_matrix @ self.config_transform @ camera_to_marker_matrix

                g_x, g_y, g_z = base_to_gripper_matrix[:3, 3]
                x, y, z = resultant_matrix[:3, 3]

                # Check if the transform is within a threshold
                if np.linalg.norm([g_x - x, g_y - y, g_z - z]) < 0.05:
                    # Publish gripper status as picked
                    gripper_pick_status = Bool()
                    gripper_pick_status.data = True
                    self.gripper_status_publisher_.publish(gripper_pick_status)

                    # Send gripper close command
                    self.send_gripper_command()
                    return

                # Publish the setpoint
                point = Point()
                point.x = x
                point.y = y
                point.z = z
                self.publisher_.publish(point)

                # Align yaw if enabled
                if self.yaw_alignment_enabled:
                    r = R.from_matrix(resultant_matrix[:3, :3])
                    yaw = r.as_euler('zyx')[0]

                    joint_state = JointState()
                    joint_state.name = self.current_joint_state.name
                    joint_state.position = list(self.current_joint_state.position)

                    if 'wrist_3_joint' in joint_state.name:
                        index = joint_state.name.index('wrist_3_joint')
                        joint_state.position[index] = yaw
                    else:
                        joint_state.name.append('wrist_3_joint')
                        joint_state.position.append(yaw)

                    self.joint_publisher_.publish(joint_state)

            except TransformException as e:
                self.get_logger().warn(f'Failed to compute transform: {e}')

    def send_gripper_command(self):
        """Sends a command to close the gripper using an action client."""
        goal_msg = Gripper.Goal()
        goal_msg.command = "close"

        self.gripper_client_.wait_for_server()
        self.gripper_client_.send_goal_async(goal_msg)

def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    node = SawsPickup()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
