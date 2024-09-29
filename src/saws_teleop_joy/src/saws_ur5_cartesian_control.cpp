#include <saws_teleop_joy/saws_ur5_cartesian_control.hpp>
#include <iostream>
#include <thread>

JoyUR5CartesianControl::JoyUR5CartesianControl() : Node("saws_ur5_cartesian_control")
{
    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscription to joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",
                                                                         10,
                                                                         std::bind(&JoyUR5CartesianControl::joy_callback,
                                                                                   this,
                                                                                   std::placeholders::_1));

    // Publisher for setpoint topic
    setpoint_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/setpoint", 10);

    // Timer to update transform every few seconds
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&JoyUR5CartesianControl::update_transform,
                                               this));
}

void JoyUR5CartesianControl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Check if PS4_BUTTON_X is pressed
    if (msg->buttons[PS4_BUTTON_X] == 1)
    {
        // Update x, y, z based on joystick button input
        double old_x = x_;
        double old_y = y_;
        double old_z = z_;

        if (msg->axes[PS4_AXIS_LEFT_X] > 0)
        {
            x_ += 0.01;
        }
        else if (msg->axes[PS4_AXIS_LEFT_X] < 0)
        {
            x_ -= 0.01;
        }

        if (msg->axes[PS4_AXIS_LEFT_Y] > 0)
        {
            y_ += 0.01;
        }
        else if (msg->axes[PS4_AXIS_LEFT_Y] < 0)
        {
            y_ -= 0.01;
        }

        if (msg->buttons[PS4_BUTTON_UP] == 1)
        {
            z_ += 0.01;
        }
        else if (msg->buttons[PS4_BUTTON_DOWN] == 1)
        {
            z_ -= 0.01;
        }

        // Create and publish the updated Point message
        if (old_x != x_ || old_y != y_ || old_z != z_)
        {
            geometry_msgs::msg::Point setpoint;
            setpoint.x = x_;
            setpoint.y = y_;
            setpoint.z = z_;

            setpoint_publisher_->publish(setpoint);
            // Pause for 0.5 seconds
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

void JoyUR5CartesianControl::update_transform()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        // Lookup the transform from base_link to gripper_pick
        transformStamped = tf_buffer_->lookupTransform("base_link", "gripper_pick", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform base_link to gripper_pick: %s", ex.what());
        return;
    }

    // Update member variables with the new transform values
    x_ = transformStamped.transform.translation.x;
    y_ = transformStamped.transform.translation.y;
    z_ = transformStamped.transform.translation.z;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyUR5CartesianControl>());
    rclcpp::shutdown();
    return 0;
}
