#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class NavigateToGoalNode(Node):
    def __init__(self):
        super().__init__('navigate_to_goal_node')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_w', 1.0)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('goal_w', 1.0)

        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_w = self.get_parameter('initial_w').value
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        goal_w = self.get_parameter('goal_w').value

        # Create a navigator
        navigator = TurtleBot4Navigator()

        # Set initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.position.x = initial_x
        initial_pose.pose.position.y = initial_y
        initial_pose.pose.orientation.w = initial_w

        # Set goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation.w = goal_w

        navigator.setInitialPose(initial_pose)
        navigator.waitUntilNav2Active()

        # Navigate to the goal pose
        navigator.startToPose(goal_pose)

        # Wait until the goal is reached
        navigator.waitUntilGoalReached()

        self.get_logger().info('Goal reached')

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
