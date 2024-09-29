import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np

class SawsControlLoop(Node):
    def __init__(self):
        super().__init__('saws_control_loop')

        # Publishers
        self.pickup_block_publisher = self.create_publisher(Bool, '/pickup_block', 10)
        self.setpoint_publisher = self.create_publisher(Point, '/setpoint', 10)
        self.joint_publisher_ = self.create_publisher(JointState, '/set_ur5_joints', 10)

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.gripper_picked_status_subscriber = self.create_subscription(Bool, '/gripper_picked_status', self.gripper_picked_status_callback, 10)
        self.turtlebot_arrived_subscriber = self.create_subscription(Bool, '/turtlebot_arrived', self.turtlebot_arrived_callback, 10)

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State variables
        self.current_joint_state = JointState()
        self.gripper_picked_status = False
        self.turtlebot_arrived = False
        self.pickup_block_flag = False
        self.noted_transform = None

    def joint_state_callback(self, msg):
        """Callback to update the current joint state."""
        self.current_joint_state = msg

    def gripper_picked_status_callback(self, msg):
        """Callback to handle the gripper picked status."""
        self.gripper_picked_status = msg.data
        if self.gripper_picked_status and self.pickup_block_flag:
            self.publish_pickup_block(False)
            self.send_setpoint_to_noted_transform()
            self.move_shoulder_pan_joint(-1.57)

    def turtlebot_arrived_callback(self, msg):
        """Callback to handle the turtlebot arrival status."""
        self.turtlebot_arrived = msg.data
        if self.turtlebot_arrived:
            self.move_shoulder_pan_joint(0.4)
            self.note_transform()
            self.publish_pickup_block(True)

    def move_shoulder_pan_joint(self, angle):
        """Moves the shoulder pan joint to a specified angle."""
        joint_state = JointState()
        joint_state.name = self.current_joint_state.name
        joint_state.position = list(self.current_joint_state.position)

        if 'shoulder_pan_joint' in joint_state.name:
            index = joint_state.name.index('shoulder_pan_joint')
            joint_state.position[index] = angle
        else:
            joint_state.name.append('shoulder_pan_joint')
            joint_state.position.append(angle)

        self.joint_publisher_.publish(joint_state)

    def note_transform(self):
        """Notes the current transform from base_link to gripper_pick."""
        try:
            base_to_gripper = self.tf_buffer.lookup_transform('base_link', 'gripper_pick', rclpy.time.Time())
            base_to_gripper_matrix = self.create_transform_matrix(
                [base_to_gripper.transform.translation.x, base_to_gripper.transform.translation.y, base_to_gripper.transform.translation.z],
                [base_to_gripper.transform.rotation.x, base_to_gripper.transform.rotation.y, base_to_gripper.transform.rotation.z, base_to_gripper.transform.rotation.w]
            )
            self.noted_transform = base_to_gripper_matrix[:3, 3]
        except Exception as e:
            self.get_logger().warn(f'Failed to compute transform: {e}')

    def create_transform_matrix(self, translation, rotation):
        """Creates a transformation matrix from translation and rotation (quaternion)."""
        translation_matrix_ = np.eye(4)
        translation_matrix_[:3, 3] = translation
        rotation_matrix_ = np.eye(4)
        rotation_matrix_[:3, :3] = R.from_quat(rotation).as_matrix()
        transform_matrix = translation_matrix_ @ rotation_matrix_
        return transform_matrix

    def publish_pickup_block(self, flag):
        """Publishes the pickup block flag."""
        self.pickup_block_flag = flag
        msg = Bool()
        msg.data = flag
        self.pickup_block_publisher.publish(msg)

    def send_setpoint_to_noted_transform(self):
        """Sends the setpoint to the noted transform."""
        if self.noted_transform is not None:
            point = Point()
            point.x = self.noted_transform[0]
            point.y = self.noted_transform[1]
            point.z = self.noted_transform[2]
            self.setpoint_publisher.publish(point)
        else:
            self.get_logger().warn('No noted transform to send as setpoint')

def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    node = SawsControlLoop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
