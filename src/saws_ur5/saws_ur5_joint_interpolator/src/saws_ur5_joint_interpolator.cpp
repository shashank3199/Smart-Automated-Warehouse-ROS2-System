#include <saws_ur5_joint_interpolator/saws_ur5_joint_interpolator.hpp>

SawsUR5JointInterpolator::SawsUR5JointInterpolator() : Node("saws_ur5_joint_interpolator"),
                                                       target_received_(false),
                                                       current_received_(false)
{
    target_joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/set_ur5_joints",
                                                                                         10,
                                                                                         std::bind(&SawsUR5JointInterpolator::target_joint_callback,
                                                                                                   this,
                                                                                                   std::placeholders::_1));

    current_joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
                                                                                          10,
                                                                                          std::bind(&SawsUR5JointInterpolator::current_joint_callback,
                                                                                                    this,
                                                                                                    std::placeholders::_1));

    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/scaled_joint_trajectory_controller/joint_trajectory",
                                                                                          10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&SawsUR5JointInterpolator::publish_trajectory,
                                               this));
}

void SawsUR5JointInterpolator::target_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Store the target joint state
    target_joint_state_ = *msg;
    target_received_ = true;
}

void SawsUR5JointInterpolator::current_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Store the current joint state
    current_joint_state_ = *msg;
    current_received_ = true;
}

void SawsUR5JointInterpolator::publish_trajectory()
{
    if (target_received_ && current_received_)
    {
        // Create a joint trajectory message
        trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.header.stamp = this->now();
        joint_trajectory_msg.joint_names = current_joint_state_.name;

        // Align target joint positions with current joint names
        std::vector<double> aligned_target_positions(current_joint_state_.name.size(), 0.0);
        for (size_t i = 0; i < current_joint_state_.name.size(); ++i)
        {
            // Find the joint in the target state
            auto it = std::find(target_joint_state_.name.begin(), target_joint_state_.name.end(), current_joint_state_.name[i]);
            if (it != target_joint_state_.name.end())
            {
                // Get the index of the joint in the target state
                size_t index = std::distance(target_joint_state_.name.begin(), it);
                aligned_target_positions[i] = target_joint_state_.position[index];
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Joint %s not found in target state", current_joint_state_.name[i].c_str());
            }
        }

        // Calculate interpolated points based on 0.1 radian increments
        double increment = 0.1; // 0.1 radians per increment
        size_t num_joints = current_joint_state_.position.size();

        // Find the maximum number of increments required for any joint
        int max_increments = 0;
        for (size_t j = 0; j < num_joints; ++j)
        {
            double diff = std::abs(aligned_target_positions[j] - current_joint_state_.position[j]);
            int increments = static_cast<int>(std::ceil(diff / increment));
            if (increments > max_increments)
            {
                max_increments = increments;
            }
        }

        // Generate trajectory points
        for (int i = 0; i <= max_increments; ++i)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            for (size_t j = 0; j < num_joints; ++j)
            {
                double start = current_joint_state_.position[j];
                double end = aligned_target_positions[j];
                double direction = (end - start) > 0 ? 1.0 : -1.0;
                double interpolated_position = start + direction * std::min(std::abs(end - start), increment * i);
                point.positions.push_back(interpolated_position);
            }
            // 0.2 seconds per increment
            point.time_from_start = rclcpp::Duration::from_seconds(0.2 * i);
            joint_trajectory_msg.points.push_back(point);
        }

        // Publish the trajectory
        trajectory_publisher_->publish(joint_trajectory_msg);
        // Reset target_received_ to wait for the next target
        target_received_ = false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SawsUR5JointInterpolator>());
    rclcpp::shutdown();
    return 0;
}
