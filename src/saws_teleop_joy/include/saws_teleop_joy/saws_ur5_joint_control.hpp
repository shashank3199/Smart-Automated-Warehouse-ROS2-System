#ifndef SAWS_TELEOP_JOY_UR5_JOINT_CONTROL_HPP
#define SAWS_TELEOP_JOY_UR5_JOINT_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <saws_teleop_joy/ps4_macros.hpp>

// Class to control the UR5 joints using a joystick
class JoyUR5JointControl : public rclcpp::Node
{
public:
    // Constructor
    JoyUR5JointControl();

private:
    // Callbacks for the joystick and joint state subscriptions
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // Subscriptions and publisher
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    // Variables to store the joint names and positions
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
};

#endif
