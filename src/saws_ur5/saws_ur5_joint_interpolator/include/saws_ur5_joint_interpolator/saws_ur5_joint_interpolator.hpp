#ifndef SAWS_UR5_JOINT_INTERPOLATOR_HPP
#define SAWS_UR5_JOINT_INTERPOLATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// Class to interpolate between two joint states for the UR5
class SawsUR5JointInterpolator : public rclcpp::Node
{
public:
    // Constructor
    SawsUR5JointInterpolator();

private:
    // Callbacks for the joint state subscriptions
    void target_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void current_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_trajectory();

    // Subscriptions and publisher
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

    // Timer to publish the trajectory
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables to store the target and current joint states
    sensor_msgs::msg::JointState target_joint_state_;
    sensor_msgs::msg::JointState current_joint_state_;

    // Flag to check if the target and current joint states have been received
    bool target_received_;
    bool current_received_;
};

#endif
