#ifndef SAWS_TELEOP_JOY_UR5_CARTESIAN_CONTROL_HPP
#define SAWS_TELEOP_JOY_UR5_CARTESIAN_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <saws_teleop_joy/ps4_macros.hpp>

// Class to control the UR5 end-effector position using a joystick
class JoyUR5CartesianControl : public rclcpp::Node
{
public:
    // Constructor
    JoyUR5CartesianControl();

private:
    // Callback for the joystick subscription
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    // Function to update the transform
    void update_transform();

    // Subscriptions and publisher
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr setpoint_publisher_;
    // TF2 buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Timer to update the transform
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables to store the setpoint and the current transform
    double x_, y_, z_;
};

#endif
