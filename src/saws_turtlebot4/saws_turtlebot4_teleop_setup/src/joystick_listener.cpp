#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <unistd.h> // for fork, execvp
#include <sys/types.h> // for pid_t
#include <sys/wait.h> // for waitpid
#include <cstring> // for strerror
#include <filesystem> // for std::filesystem
#include <iostream> // for std::cerr
#include <fstream> // for std::ofstream

class JoystickListener : public rclcpp::Node
{
public:
    JoystickListener()
    : Node("joystick_listener"), current_pose_received_(false)
    {
        this->declare_parameter<std::string>("map_name", "wyman160");
        this->get_parameter("map_name", map_name_);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickListener::joy_callback, this, std::placeholders::_1));
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&JoystickListener::pose_callback, this, std::placeholders::_1));

        // Get the package directory
        auto package_directory = ament_index_cpp::get_package_share_directory("saws_turtlebot4_teleop_setup");
        map_directory_ = (std::filesystem::path(package_directory) / "maps").string();
        config_directory_ = (std::filesystem::path(package_directory) / "config").string();
        std::filesystem::create_directories(map_directory_);
        std::filesystem::create_directories(config_directory_);
        config_file_path_ = (std::filesystem::path(config_directory_) / "positions.yaml").string();
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Assuming the X button is the first button (index 0)
        if (msg->buttons[0] == 1)
        {
            RCLCPP_INFO(this->get_logger(), "X button pressed, saving map");
            save_map();
        }
        // Assuming the box button is the second button (index 1)
        if (msg->buttons[1] == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Box button pressed, saving current position");
            save_position();
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = msg->pose.pose;
        current_pose_received_ = true;
    }

    void save_map()
    {
        pid_t pid = fork();
        if (pid == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to fork process: %s", std::strerror(errno));
            return;
        }

        if (pid == 0)
        {
            // Child process
            std::string map_path = map_directory_ + "/" + map_name_;
            std::string command = "ros2 run nav2_map_server map_saver_cli -f " + map_path + " --ros-args -p map_subscribe_transient_local:=true";
            char *argv[] = { (char*)"sh", (char*)"-c", (char*)command.c_str(), NULL };
            execvp(argv[0], argv);
            // If execvp returns, it must have failed
            RCLCPP_ERROR(this->get_logger(), "Failed to execvp: %s", std::strerror(errno));
            std::exit(EXIT_FAILURE);
        }
        else
        {
            // Parent process
            int status;
            waitpid(pid, &status, 0);
            if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Map saved successfully.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to save map.");
            }
        }
    }

    void save_position()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!current_pose_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Current pose not received yet, cannot save position");
            return;
        }

        std::ofstream outfile(config_file_path_, std::ios_base::app);
        if (!outfile.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open config file for writing: %s", config_file_path_.c_str());
            return;
        }

        outfile << "- position:\n";
        outfile << "    x: " << current_pose_.position.x << "\n";
        outfile << "    y: " << current_pose_.position.y << "\n";
        outfile << "    z: " << current_pose_.position.z << "\n";
        outfile << "  orientation:\n";
        outfile << "    x: " << current_pose_.orientation.x << "\n";
        outfile << "    y: " << current_pose_.orientation.y << "\n";
        outfile << "    z: " << current_pose_.orientation.z << "\n";
        outfile << "    w: " << current_pose_.orientation.w << "\n";
        outfile.close();

        RCLCPP_INFO(this->get_logger(), "Current position saved to config file");
    }

    std::string map_name_;
    std::string map_directory_;
    std::string config_directory_;
    std::string config_file_path_;
    geometry_msgs::msg::Pose current_pose_;
    bool current_pose_received_;
    std::mutex pose_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
