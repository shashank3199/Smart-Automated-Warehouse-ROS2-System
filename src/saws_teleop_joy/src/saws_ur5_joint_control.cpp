#include <saws_teleop_joy/saws_ur5_joint_control.hpp>
#include <algorithm>
#include <iostream>

JoyUR5JointControl::JoyUR5JointControl() : Node("saws_ur5_joint_control")
{
    // Subscription to joint states topic
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
                                                                                        10,
                                                                                        std::bind(&JoyUR5JointControl::joint_state_callback,
                                                                                                  this,
                                                                                                  std::placeholders::_1));

    // Publisher for joint trajectory
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/set_ur5_joints", 10);

    // Subscription to joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",
                                                                         10,
                                                                         std::bind(&JoyUR5JointControl::joy_callback,
                                                                                   this,
                                                                                   std::placeholders::_1));

    // Initialize joint names
    joint_names_ = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"};

    // Initialize joint positions with zeroes
    joint_positions_.resize(joint_names_.size(), 0.0);
}

void JoyUR5JointControl::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Update joint positions based on the joint names
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
        if (it != joint_names_.end())
        {
            size_t index = std::distance(joint_names_.begin(), it);
            joint_positions_[index] = msg->position[i];
        }
    }
}

void JoyUR5JointControl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    double shoulder_pan = joint_positions_[0];
    double shoulder_lift = joint_positions_[1];
    double elbow = joint_positions_[2];
    double wrist_1 = joint_positions_[3];
    double wrist_2 = joint_positions_[4];
    double wrist_3 = joint_positions_[5];

    if (msg->buttons[PS4_BUTTON_L1] == 1)
    {
        wrist_3 += 0.75;
    }
    else if (msg->buttons[PS4_BUTTON_R1] == 1)
    {
        wrist_3 -= 0.75;
    }

    if (msg->axes[PS4_AXIS_L2] < 0)
    {
        wrist_2 += 0.05;
    }
    else if (msg->axes[PS4_AXIS_R2] < 0)
    {
        wrist_2 -= 0.05;
    }

    if (msg->axes[PS4_AXIS_LEFT_Y] > 0.05)
    {
        wrist_1 += 0.05;
    }
    else if (msg->axes[PS4_AXIS_LEFT_Y] < -0.05)
    {
        wrist_1 -= 0.05;
    }

    if (msg->axes[PS4_AXIS_RIGHT_Y] > 0.05)
    {
        elbow += 0.01;
    }
    else if (msg->axes[PS4_AXIS_RIGHT_Y] < -0.05)
    {
        elbow -= 0.01;
    }

    if (msg->axes[PS4_AXIS_LEFT_X] > 0.05)
    {
        shoulder_lift += 0.005;
    }
    else if (msg->axes[PS4_AXIS_LEFT_X] < -0.05)
    {
        shoulder_lift -= 0.005;
    }

    if (msg->axes[PS4_AXIS_RIGHT_X] > 0.05)
    {
        shoulder_pan += 0.01;
    }
    else if (msg->axes[PS4_AXIS_RIGHT_X] < -0.05)
    {
        shoulder_pan -= 0.01;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.name = joint_names_;
    joint_state.position = {shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3};
    std::cout << std::setw(6) << shoulder_pan << std::setw(6) << shoulder_lift << std::setw(6) << elbow << std::setw(6) << wrist_1 << std::setw(6) << wrist_2 << std::setw(6) << wrist_3 << std::endl;
    joint_state_publisher_->publish(joint_state);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyUR5JointControl>());
    rclcpp::shutdown();
    return 0;
}
