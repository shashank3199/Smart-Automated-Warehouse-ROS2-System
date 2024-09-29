#ifndef SAWS_TELEOP_JOY_ROBOTIQ_GRIPPER_CONTROL_HPP
#define SAWS_TELEOP_JOY_ROBOTIQ_GRIPPER_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <saws_teleop_joy/ps4_macros.hpp>

// Class to control the Robotiq gripper using a joystick
class JoyRobotiqGripperControl : public rclcpp::Node
{
public:
    // Constructor
    JoyRobotiqGripperControl();

private:
    // Callbacks for the joystick and joint state subscriptions
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    // Function to send a command to the gripper
    void send_gripper_command(double position);

    // Subscriptions and action client
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;

    // Variables to store the current position of the gripper
    double current_position_;
};

#endif
