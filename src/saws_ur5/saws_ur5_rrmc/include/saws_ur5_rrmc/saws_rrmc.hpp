#ifndef SAWS_RESOLVE_RATE_MOTION_CONTROL_HPP
#define SAWS_RESOLVE_RATE_MOTION_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Eigen>

// Resolve Rate Motion Control
class SawsResolveRateMotionControl : public rclcpp::Node
{
public:
    // Constructor
    SawsResolveRateMotionControl(const std::string &name);

private:
    // Subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_setpoint;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_jointstate;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory;

    // Variables for joint states
    double shoulder_pan_joint;
    double shoulder_lift_joint;
    double elbow_joint;
    double wrist_1_joint;
    double wrist_2_joint;
    double wrist_3_joint;

    // Callbacks
    void JointState(const sensor_msgs::msg::JointState &jointstate);
    void SetPoint(const geometry_msgs::msg::Point &newgoal);

    // Calculate the inverse of a 6x6 matrix
    double Inverse(double A[6][6], double Ainverse[6][6]);

    // Forward kinematics, inverse kinematics, Jacobian, and adjoint transformation inverse
    void ForwardKinematics(double q1, double q2, double q3, double q4, double q5, double q6, double E[4][4]);
    void ForwardKinematicsInverse(double q1, double q2, double q3, double q4, double q5, double q6, double E[4][4]);
    void Jacobian(double q1, double q2, double q3, double q4, double q5, double q6, double J[6][6]);
    void AdjointTransformationInverse(double E[4][4], double Ad[6][6]);
};

#endif
