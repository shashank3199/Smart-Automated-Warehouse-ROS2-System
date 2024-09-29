#include <saws_teleop_joy/saws_gripper_control.hpp>

JoyRobotiqGripperControl::JoyRobotiqGripperControl() : Node("joy_robotiq_gripper_control"), current_position_(0.0)
{
    // Subscription to joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",
                                                                         10,
                                                                         std::bind(&JoyRobotiqGripperControl::joy_callback,
                                                                                   this,
                                                                                   std::placeholders::_1));

    // Subscription to joint states topic
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
                                                                                        10,
                                                                                        std::bind(&JoyRobotiqGripperControl::joint_state_callback,
                                                                                                  this,
                                                                                                  std::placeholders::_1));

    // Action client for gripper command
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(this,
                                                                                                "/robotiq_gripper_controller/gripper_cmd");
}

void JoyRobotiqGripperControl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Open the gripper
    if (msg->buttons[PS4_BUTTON_L1] == 1)
    {
        current_position_ = std::min(current_position_ + 0.05, 0.75);
        send_gripper_command(current_position_);
    }
    // Close the gripper
    else if (msg->buttons[PS4_BUTTON_R1] == 1)
    {
        current_position_ = std::max(current_position_ - 0.05, 0.0);
        send_gripper_command(current_position_);
    }
}

void JoyRobotiqGripperControl::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "robotiq_85_left_knuckle_joint")
        {
            current_position_ = msg->position[i];
            break;
        }
    }
}

void JoyRobotiqGripperControl::send_gripper_command(double position)
{
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    // Create goal message
    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 0.0;

    // Send goal
    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal canceled");
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
            break;
        }
    };

    // Send goal
    gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyRobotiqGripperControl>());
    rclcpp::shutdown();
    return 0;
}
