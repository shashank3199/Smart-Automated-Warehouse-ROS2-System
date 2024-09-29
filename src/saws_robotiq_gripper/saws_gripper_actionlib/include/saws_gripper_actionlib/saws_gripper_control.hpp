#ifndef SAWS_GRIPPER_CONTROL_HPP
#define SAWS_GRIPPER_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <saws_gripper_msgs/action/gripper.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Class to control the gripper using actionlib
class SawsGripperControl : public rclcpp::Node
{
public:
    // Define the types used by the action server and client
    using GripperControl = saws_gripper_msgs::action::Gripper;
    // GoalHandleGripperControl is a type that represents a handle to an active goal
    using GoalHandleGripperControl = rclcpp_action::ServerGoalHandle<GripperControl>;
    // GripperCommand is a type that represents the goal message for the gripper action client
    using GripperCommand = control_msgs::action::GripperCommand;
    // GripperCommandGoalHandle is a type that represents a handle to an active goal for the gripper action client
    using GripperCommandGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

    // Constructor
    SawsGripperControl();

private:
    // Constants for the gripper
    static constexpr double OPEN_POSITION = 0.3;
    static constexpr double CLOSE_POSITION = 0.4;

    // Action server and client
    rclcpp_action::Server<GripperControl>::SharedPtr action_server_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
    // Subscription to the joint state topic
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    // Variables to store the current position of the gripper
    double current_position_;

    // Callbacks for the action server
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const GripperControl::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGripperControl> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleGripperControl> goal_handle);
    void execute(const std::shared_ptr<GoalHandleGripperControl> goal_handle);

    // Callback for the joint state subscription
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

#endif
