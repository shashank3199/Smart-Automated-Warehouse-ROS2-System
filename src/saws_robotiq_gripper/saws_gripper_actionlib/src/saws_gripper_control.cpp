#include <saws_gripper_actionlib/saws_gripper_control.hpp>

// Constructor
SawsGripperControl::SawsGripperControl() : Node("saws_gripper_control"), current_position_(0.0)
{
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<GripperControl>(this,
                                                                        "/gripper_control",
                                                                        std::bind(&SawsGripperControl::handle_goal, this, _1, _2),
                                                                        std::bind(&SawsGripperControl::handle_cancel, this, _1),
                                                                        std::bind(&SawsGripperControl::handle_accepted, this, _1));

    this->gripper_client_ = rclcpp_action::create_client<GripperCommand>(this,
                                                                         "/robotiq_gripper_controller/gripper_cmd");

    this->joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
                                                                                              10,
                                                                                              std::bind(&SawsGripperControl::joint_state_callback,
                                                                                                        this,
                                                                                                        _1));
}

// Callbacks for the action server
rclcpp_action::GoalResponse SawsGripperControl::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                            std::shared_ptr<const GripperControl::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with command %s", goal->command.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SawsGripperControl::handle_cancel(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SawsGripperControl::handle_accepted(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
{
    std::thread{std::bind(&SawsGripperControl::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void SawsGripperControl::execute(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GripperControl::Feedback>();
    auto result = std::make_shared<GripperControl::Result>();

    // Create a goal for the gripper action client
    auto gripper_goal = GripperCommand::Goal();
    if (goal->command == "open")
    {
        gripper_goal.command.position = OPEN_POSITION;
        gripper_goal.command.max_effort = 0.0;
    }
    else if (goal->command == "close")
    {
        gripper_goal.command.position = CLOSE_POSITION;
        gripper_goal.command.max_effort = 0.0;
    }
    else if (goal->command == "reset")
    {
        gripper_goal.command.position = 0.0;
        gripper_goal.command.max_effort = 0.0;
    }
    else
    {
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // Send goal to the gripper action client
    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = [this, goal_handle, result](const GripperCommandGoalHandle::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            result->success = true;
            goal_handle->succeed(result);
        }
        else
        {
            result->success = false;
            goal_handle->abort(result);
        }
    };

    this->gripper_client_->async_send_goal(gripper_goal, send_goal_options);

    // Continuously send feedback
    while (rclcpp::ok())
    {
        double percentage = (current_position_ - OPEN_POSITION) / (CLOSE_POSITION - OPEN_POSITION) * 100.0;
        feedback->status = static_cast<float>(percentage);
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void SawsGripperControl::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SawsGripperControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
