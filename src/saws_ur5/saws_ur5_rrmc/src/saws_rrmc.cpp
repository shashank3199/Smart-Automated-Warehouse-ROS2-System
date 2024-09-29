#include <saws_ur5_rrmc/saws_rrmc.hpp>

SawsResolveRateMotionControl::SawsResolveRateMotionControl(const std::string &name) : Node(name)
{
    pub_trajectory = create_publisher<trajectory_msgs::msg::JointTrajectory>("/scaled_joint_trajectory_controller/joint_trajectory", 1);
    sub_setpoint = create_subscription<geometry_msgs::msg::Point>("setpoint",
                                                                  1,
                                                                  std::bind(&SawsResolveRateMotionControl::SetPoint,
                                                                            this,
                                                                            std::placeholders::_1));
    sub_jointstate = create_subscription<sensor_msgs::msg::JointState>("joint_states",
                                                                       1,
                                                                       std::bind(&SawsResolveRateMotionControl::JointState,
                                                                                 this,
                                                                                 std::placeholders::_1));
}

void SawsResolveRateMotionControl::JointState(const sensor_msgs::msg::JointState &jointstate)
{
    for (size_t i = 0; i < jointstate.name.size(); i++)
    {
        if (jointstate.name[i] == "shoulder_pan_joint")
        {
            shoulder_pan_joint = jointstate.position[i];
        }
        if (jointstate.name[i] == "shoulder_lift_joint")
        {
            shoulder_lift_joint = jointstate.position[i];
        }
        if (jointstate.name[i] == "elbow_joint")
        {
            elbow_joint = jointstate.position[i];
        }
        if (jointstate.name[i] == "wrist_1_joint")
        {
            wrist_1_joint = jointstate.position[i];
        }
        if (jointstate.name[i] == "wrist_2_joint")
        {
            wrist_2_joint = jointstate.position[i];
        }
        if (jointstate.name[i] == "wrist_3_joint")
        {
            wrist_3_joint = jointstate.position[i];
        }
    }
}

void SawsResolveRateMotionControl::SetPoint(const geometry_msgs::msg::Point &newgoal)
{
    // The increment in the position
    double positionincrement = 1e-3;

    // The name of all the joints
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names.push_back("shoulder_pan_joint");
    trajectory.joint_names.push_back("shoulder_lift_joint");
    trajectory.joint_names.push_back("elbow_joint");
    trajectory.joint_names.push_back("wrist_1_joint");
    trajectory.joint_names.push_back("wrist_2_joint");
    trajectory.joint_names.push_back("wrist_3_joint");

    // The trajectory point. Initialized with current values
    trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.push_back(shoulder_pan_joint);
    trajectory_point.positions.push_back(shoulder_lift_joint);
    trajectory_point.positions.push_back(elbow_joint);
    trajectory_point.positions.push_back(wrist_1_joint);
    trajectory_point.positions.push_back(wrist_2_joint);
    trajectory_point.positions.push_back(wrist_3_joint);
    trajectory_point.time_from_start = rclcpp::Duration(0, 1000000);

    // The current position of the end effector
    tf2::Vector3 current;
    current.setX(0.0);
    current.setY(0.0);
    current.setZ(0.0);

    // The goal position of the end effector
    double E[4][4];
    ForwardKinematicsInverse(trajectory_point.positions[0],
                             trajectory_point.positions[1],
                             trajectory_point.positions[2],
                             trajectory_point.positions[3],
                             trajectory_point.positions[4],
                             trajectory_point.positions[5],
                             E);

    tf2::Vector3 goal;
    goal.setX(E[0][0] * newgoal.x + E[0][1] * newgoal.y + E[0][2] * newgoal.z + E[0][3]);
    goal.setY(E[1][0] * newgoal.x + E[1][1] * newgoal.y + E[1][2] * newgoal.z + E[1][3]);
    goal.setZ(E[2][0] * newgoal.x + E[2][1] * newgoal.y + E[2][2] * newgoal.z + E[2][3]);

    // The translation from the current position to the goal position
    tf2::Vector3 translation = goal - current;

    // Loop until the end effector reaches the goal position
    while (positionincrement < translation.length())
    {

        // Compute the joint velocities to reach the goal position
        tf2::Vector3 vb = (translation / translation.length()) * positionincrement;

        double Js[6][6], J[6][6], Ji[6][6];

        // Compute the Jacobian
        Jacobian(trajectory_point.positions[0],
                 trajectory_point.positions[1],
                 trajectory_point.positions[2],
                 trajectory_point.positions[3],
                 trajectory_point.positions[4],
                 trajectory_point.positions[5],
                 Js);

        // update the current position/orientation of the end effector
        ForwardKinematics(trajectory_point.positions[0],
                          trajectory_point.positions[1],
                          trajectory_point.positions[2],
                          trajectory_point.positions[3],
                          trajectory_point.positions[4],
                          trajectory_point.positions[5],
                          E);

        double Ad[6][6];
        AdjointTransformationInverse(E, Ad);

        // Compute the Body Jacobian
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                J[r][c] = 0.0;
                for (int k = 0; k < 6; k++)
                {
                    J[r][c] += Ad[r][k] * Js[k][c];
                }
            }
        }

        // Compute the inverse of the Jacobian
        if (fabs(Inverse(J, Ji)) < 1e-09)
        {
            std::cout << "Jacobian is near singular." << std::endl;
        }

        // Compute the joint velocity by multiplying the (Ji v)
        double qd[6];
        qd[0] = Ji[0][0] * vb[0] + Ji[0][1] * vb[1] + Ji[0][2] * vb[2] +
                Ji[0][3] * 0.0 + Ji[0][4] * 0.0 + Ji[0][5] * 0.0;
        qd[1] = Ji[1][0] * vb[0] + Ji[1][1] * vb[1] + Ji[1][2] * vb[2] +
                Ji[1][3] * 0.0 + Ji[1][4] * 0.0 + Ji[1][5] * 0.0;
        qd[2] = Ji[2][0] * vb[0] + Ji[2][1] * vb[1] + Ji[2][2] * vb[2] +
                Ji[2][3] * 0.0 + Ji[2][4] * 0.0 + Ji[2][5] * 0.0;
        qd[3] = Ji[3][0] * vb[0] + Ji[3][1] * vb[1] + Ji[3][2] * vb[2] +
                Ji[3][3] * 0.0 + Ji[3][4] * 0.0 + Ji[3][5] * 0.0;
        qd[4] = Ji[4][0] * vb[0] + Ji[4][1] * vb[1] + Ji[4][2] * vb[2] +
                Ji[4][3] * 0.0 + Ji[4][4] * 0.0 + Ji[4][5] * 0.0;
        qd[5] = Ji[5][0] * vb[0] + Ji[5][1] * vb[1] + Ji[5][2] * vb[2] +
                Ji[5][3] * 0.0 + Ji[5][4] * 0.0 + Ji[5][5] * 0.0;

        // Update the joint positions
        trajectory_point.positions[0] += (qd[0]);
        trajectory_point.positions[1] += (qd[1]);
        trajectory_point.positions[2] += (qd[2]);
        trajectory_point.positions[3] += (qd[3]);
        trajectory_point.positions[4] += (qd[4]);
        trajectory_point.positions[5] += (qd[5]);
        trajectory_point.time_from_start = rclcpp::Duration(trajectory_point.time_from_start) + rclcpp::Duration(0, 50000000);

        // Add the trajectory point to the trajectory
        trajectory.points.push_back(trajectory_point);

        // Update the current position of the end effector
        ForwardKinematicsInverse(trajectory_point.positions[0],
                                 trajectory_point.positions[1],
                                 trajectory_point.positions[2],
                                 trajectory_point.positions[3],
                                 trajectory_point.positions[4],
                                 trajectory_point.positions[5],
                                 E);

        goal.setX(E[0][0] * newgoal.x + E[0][1] * newgoal.y + E[0][2] * newgoal.z + E[0][3]);
        goal.setY(E[1][0] * newgoal.x + E[1][1] * newgoal.y + E[1][2] * newgoal.z + E[1][3]);
        goal.setZ(E[2][0] * newgoal.x + E[2][1] * newgoal.y + E[2][2] * newgoal.z + E[2][3]);
        translation = goal - current;
    }

    pub_trajectory->publish(trajectory);
}

// Function to compute the inverse of a 6x6 matrix
double SawsResolveRateMotionControl::Inverse(double A[6][6], double Ainverse[6][6])
{
    // Compute the determinant and inverse of a 6x6 matrix
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> mat(A[0]);
    double determinant = mat.determinant();
    // Check if the determinant is zero
    mat = mat.inverse();
    // Copy the inverse matrix to Ainverse
    std::copy(mat.data(), mat.data() + mat.size(), Ainverse[0]);
    // Check if the inverse is correct
    assert(Ainverse[5][5] == mat(5, 5));
    // Return the determinant
    return determinant;
}

// Function to compute forward kinematics
void SawsResolveRateMotionControl::ForwardKinematics(double q1, double q2, double q3, double q4, double q5, double q6, double E[4][4])
{
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            E[r][c] = 0.0;

    E[0][0] = cos(q6) * (sin(q1) * sin(q5) + cos(q2 + q3 + q4) * cos(q1) * cos(q5)) * -1.0 + sin(q2 + q3 + q4) * cos(q1) * sin(q6);
    E[0][1] = sin(q6) * (sin(q1) * sin(q5) + cos(q2 + q3 + q4) * cos(q1) * cos(q5)) + sin(q2 + q3 + q4) * cos(q1) * cos(q6);
    E[0][2] = cos(q5) * sin(q1) * -1.0 + cos(q2 + q3 + q4) * cos(q1) * sin(q5);
    E[0][3] = sin(q1) * -1.091499999999996E-1 + cos(q1) * cos(q2) * 4.250000000000114E-1 - cos(q5) * sin(q1) * 2.2229999999999E-1 + cos(q2 + q3 + q4) * cos(q1) * sin(q5) * 2.2229999999999E-1 - cos(q2 + q3) * cos(q1) * sin(q4) * 9.465000000000146E-2 - sin(q2 + q3) * cos(q1) * cos(q4) * 9.465000000000146E-2 + cos(q1) * cos(q2) * cos(q3) * 3.9224999999999E-1 - cos(q1) * sin(q2) * sin(q3) * 3.9224999999999E-1;
    E[1][0] = cos(q6) * (cos(q1) * sin(q5) - cos(q2 + q3 + q4) * cos(q5) * sin(q1) * 1.0) + sin(q2 + q3 + q4) * sin(q1) * sin(q6);
    E[1][1] = sin(q6) * (cos(q1) * sin(q5) - cos(q2 + q3 + q4) * cos(q5) * sin(q1) * 1.0) * -1.0 + sin(q2 + q3 + q4) * cos(q6) * sin(q1);
    E[1][2] = cos(q1) * cos(q5) + cos(q2 + q3 + q4) * sin(q1) * sin(q5);
    E[1][3] = cos(q1) * 1.091499999999996E-1 + cos(q1) * cos(q5) * 2.2229999999999E-1 + cos(q2) * sin(q1) * 4.250000000000114E-1 - sin(q1) * sin(q2) * sin(q3) * 3.9224999999999E-1 + cos(q2 + q3 + q4) * sin(q1) * sin(q5) * 2.2229999999999E-1 - cos(q2 + q3) * sin(q1) * sin(q4) * 9.465000000000146E-2 - sin(q2 + q3) * cos(q4) * sin(q1) * 9.465000000000146E-2 + cos(q2) * cos(q3) * sin(q1) * 3.9224999999999E-1;
    E[2][0] = cos(q2 + q3 + q4) * sin(q6) + sin(q2 + q3 + q4) * cos(q5) * cos(q6);
    E[2][1] = cos(q2 + q3 + q4) * cos(q6) - sin(q2 + q3 + q4) * cos(q5) * sin(q6) * 1.0;
    E[2][2] = sin(q2 + q3 + q4) * sin(q5) * -1.0;
    E[2][3] = sin(q2 + q3) * -3.9224999999999E-1 - sin(q2) * 4.250000000000114E-1 - sin(q5) * (cos(q2 + q3) * sin(q4) * 2.2229999999999E-1 + sin(q2 + q3) * cos(q4) * 2.2229999999999E-1) * 1.0 - cos(q2 + q3) * cos(q4) * 9.465000000000146E-2 + sin(q2 + q3) * sin(q4) * 9.465000000000146E-2 + 8.899999999999864E-2;
    E[3][3] = 1.0;
}

// Function to compute forward kinematics inverse
void SawsResolveRateMotionControl::ForwardKinematicsInverse(double q1, double q2, double q3, double q4, double q5, double q6, double E[4][4])
{
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            E[r][c] = 0.0;

    E[0][0] = ((sin(q2 + q3 + q4) * cos(q1) * pow(cos(q5), 2.0) * sin(q6) * -1.0 + pow(cos(q2 + q3 + q4), 2.0) * cos(q6) * sin(q1) * sin(q5) - sin(q2 + q3 + q4) * cos(q1) * pow(sin(q5), 2.0) * sin(q6) * 1.0 + pow(sin(q2 + q3 + q4), 2.0) * cos(q6) * sin(q1) * sin(q5) + cos(q2 + q3 + q4) * cos(q1) * cos(q5) * cos(q6)) * -1.0) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0));
    E[0][1] = ((cos(q2 + q3 + q4) * cos(q5) * cos(q6) * sin(q1) * -1.0 + pow(cos(q2 + q3 + q4), 2.0) * cos(q1) * cos(q6) * sin(q5) + pow(sin(q2 + q3 + q4), 2.0) * cos(q1) * cos(q6) * sin(q5) + sin(q2 + q3 + q4) * pow(cos(q5), 2.0) * sin(q1) * sin(q6) + sin(q2 + q3 + q4) * sin(q1) * pow(sin(q5), 2.0) * sin(q6)) * 1.0) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0));
    E[0][2] = ((sin(q2 + q3 + q4) * cos(q5) * cos(q6) + cos(q2 + q3 + q4) * pow(cos(q5), 2.0) * sin(q6) + cos(q2 + q3 + q4) * pow(sin(q5), 2.0) * sin(q6)) * 1.0) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0));
    E[0][3] = ((sin(q2 + q3 + q4) * cos(q5) * cos(q6) * -8.899999999999864E+32 - cos(q2 + q3 + q4) * pow(cos(q5), 2.0) * sin(q6) * 8.899999999999864E+32 - pow(cos(q2 + q3 + q4), 2.0) * cos(q6) * sin(q5) * 1.091499999999996E+33 - cos(q2 + q3 + q4) * pow(sin(q5), 2.0) * sin(q6) * 8.899999999999864E+32 - pow(sin(q2 + q3 + q4), 2.0) * cos(q6) * sin(q5) * 1.091499999999996E+33 + sin(q2 + q3 + q4) * cos(q5) * cos(q6) * sin(q2) * 4.250000000000114E+33 + cos(q2 + q3 + q4) * sin(q2 + q3) * pow(cos(q5), 2.0) * sin(q6) * 3.9224999999999E+33 + cos(q2 + q3 + q4) * sin(q2 + q3) * pow(sin(q5), 2.0) * sin(q6) * 3.9224999999999E+33 + cos(q2 + q3 + q4) * pow(cos(q5), 2.0) * sin(q2) * sin(q6) * 4.250000000000114E+33 - sin(q2 + q3 + q4) * cos(q2) * pow(cos(q5), 2.0) * sin(q6) * 4.250000000000114E+33 - pow(sin(q2 + q3 + q4), 2.0) * cos(q5) * cos(q6) * sin(q5) * 2.2229999999999E+33 + cos(q2 + q3 + q4) * sin(q2) * pow(sin(q5), 2.0) * sin(q6) * 4.250000000000114E+33 - sin(q2 + q3 + q4) * cos(q2) * pow(sin(q5), 2.0) * sin(q6) * 4.250000000000114E+33 - cos(q2 + q3 + q4) * sin(q2 + q3 + q4) * pow(sin(q5), 3.0) * sin(q6) * 2.2229999999999E+33 + sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q5) * cos(q6) * 3.9224999999999E+33 + cos(q2 + q3 + q4) * cos(q2) * cos(q5) * cos(q6) * 4.250000000000114E+33 + cos(q2 + q3 + q4) * cos(q2 + q3) * cos(q4) * pow(cos(q5), 2.0) * sin(q6) * 9.465000000000146E+32 + cos(q2 + q3 + q4) * cos(q2 + q3) * cos(q4) * pow(sin(q5), 2.0) * sin(q6) * 9.465000000000146E+32 - cos(q2 + q3 + q4) * sin(q2 + q3) * pow(cos(q5), 2.0) * sin(q4) * sin(q6) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * cos(q2 + q3) * pow(cos(q5), 2.0) * sin(q4) * sin(q6) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * pow(cos(q5), 2.0) * sin(q6) * 9.465000000000146E+32 + cos(q2 + q3 + q4) * cos(q2 + q3) * sin(q4) * pow(sin(q5), 3.0) * sin(q6) * 2.2229999999999E+33 + cos(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * pow(sin(q5), 3.0) * sin(q6) * 2.2229999999999E+33 - cos(q2 + q3 + q4) * sin(q2 + q3) * sin(q4) * pow(sin(q5), 2.0) * sin(q6) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * cos(q2 + q3) * sin(q4) * pow(sin(q5), 2.0) * sin(q6) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * pow(sin(q5), 2.0) * sin(q6) * 9.465000000000146E+32 - sin(q2 + q3 + q4) * cos(q2) * cos(q3) * pow(cos(q5), 2.0) * sin(q6) * 3.9224999999999E+33 - sin(q2 + q3 + q4) * cos(q2) * cos(q3) * pow(sin(q5), 2.0) * sin(q6) * 3.9224999999999E+33 + sin(q2 + q3 + q4) * pow(cos(q5), 2.0) * sin(q2) * sin(q3) * sin(q6) * 3.9224999999999E+33 + sin(q2 + q3 + q4) * sin(q2) * sin(q3) * pow(sin(q5), 2.0) * sin(q6) * 3.9224999999999E+33 - cos(q2 + q3 + q4) * sin(q2 + q3 + q4) * pow(cos(q5), 2.0) * sin(q5) * sin(q6) * 2.2229999999999E+33 - cos(q2 + q3 + q4) * cos(q2 + q3) * cos(q5) * cos(q6) * sin(q4) * 9.465000000000146E+32 - cos(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * cos(q5) * cos(q6) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * cos(q2 + q3) * cos(q4) * cos(q5) * cos(q6) * 9.465000000000146E+32 + cos(q2 + q3 + q4) * cos(q2) * cos(q3) * cos(q5) * cos(q6) * 3.9224999999999E+33 - sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q5) * cos(q6) * sin(q4) * 9.465000000000146E+32 - cos(q2 + q3 + q4) * cos(q5) * cos(q6) * sin(q2) * sin(q3) * 3.9224999999999E+33 + sin(q2 + q3 + q4) * cos(q2 + q3) * cos(q5) * cos(q6) * sin(q4) * sin(q5) * 2.2229999999999E+33 + sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * cos(q5) * cos(q6) * sin(q5) * 2.2229999999999E+33 + cos(q2 + q3 + q4) * cos(q2 + q3) * pow(cos(q5), 2.0) * sin(q4) * sin(q5) * sin(q6) * 2.2229999999999E+33 + cos(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * pow(cos(q5), 2.0) * sin(q5) * sin(q6) * 2.2229999999999E+33) * 1.0E-34) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0));
    E[1][0] = ((cos(q2 + q3 + q4) * cos(q1) * cos(q5) * sin(q6) + sin(q2 + q3 + q4) * cos(q1) * pow(cos(q5), 2.0) * cos(q6) + sin(q2 + q3 + q4) * cos(q1) * cos(q6) * pow(sin(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * sin(q1) * sin(q5) * sin(q6) + pow(sin(q2 + q3 + q4), 2.0) * sin(q1) * sin(q5) * sin(q6)) * 1.0) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0));
    E[1][1] = ((cos(q2 + q3 + q4) * cos(q5) * sin(q1) * sin(q6) + sin(q2 + q3 + q4) * pow(cos(q5), 2.0) * cos(q6) * sin(q1) - pow(cos(q2 + q3 + q4), 2.0) * cos(q1) * sin(q5) * sin(q6) * 1.0 + sin(q2 + q3 + q4) * cos(q6) * sin(q1) * pow(sin(q5), 2.0) - pow(sin(q2 + q3 + q4), 2.0) * cos(q1) * sin(q5) * sin(q6) * 1.0) * 1.0) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0));
    E[1][2] = ((sin(q2 + q3 + q4) * cos(q5) * sin(q6) * -1.0 + cos(q2 + q3 + q4) * pow(cos(q5), 2.0) * cos(q6) + cos(q2 + q3 + q4) * cos(q6) * pow(sin(q5), 2.0)) * 1.0) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0));
    E[1][3] = ((sin(q2 + q3 + q4) * cos(q5) * sin(q6) * 8.899999999999864E+32 - cos(q2 + q3 + q4) * pow(cos(q5), 2.0) * cos(q6) * 8.899999999999864E+32 - cos(q2 + q3 + q4) * cos(q6) * pow(sin(q5), 2.0) * 8.899999999999864E+32 + pow(cos(q2 + q3 + q4), 2.0) * sin(q5) * sin(q6) * 1.091499999999996E+33 + pow(sin(q2 + q3 + q4), 2.0) * sin(q5) * sin(q6) * 1.091499999999996E+33 - cos(q2 + q3 + q4) * cos(q2) * cos(q5) * sin(q6) * 4.250000000000114E+33 - sin(q2 + q3 + q4) * cos(q5) * sin(q2) * sin(q6) * 4.250000000000114E+33 + cos(q2 + q3 + q4) * sin(q2 + q3) * pow(cos(q5), 2.0) * cos(q6) * 3.9224999999999E+33 + cos(q2 + q3 + q4) * sin(q2 + q3) * cos(q6) * pow(sin(q5), 2.0) * 3.9224999999999E+33 + cos(q2 + q3 + q4) * pow(cos(q5), 2.0) * cos(q6) * sin(q2) * 4.250000000000114E+33 - sin(q2 + q3 + q4) * cos(q2) * pow(cos(q5), 2.0) * cos(q6) * 4.250000000000114E+33 + cos(q2 + q3 + q4) * cos(q6) * sin(q2) * pow(sin(q5), 2.0) * 4.250000000000114E+33 - sin(q2 + q3 + q4) * cos(q2) * cos(q6) * pow(sin(q5), 2.0) * 4.250000000000114E+33 + pow(sin(q2 + q3 + q4), 2.0) * cos(q5) * sin(q5) * sin(q6) * 2.2229999999999E+33 - cos(q2 + q3 + q4) * sin(q2 + q3 + q4) * cos(q6) * pow(sin(q5), 3.0) * 2.2229999999999E+33 - sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q5) * sin(q6) * 3.9224999999999E+33 + cos(q2 + q3 + q4) * cos(q2 + q3) * cos(q4) * cos(q6) * pow(sin(q5), 2.0) * 9.465000000000146E+32 - cos(q2 + q3 + q4) * sin(q2 + q3) * pow(cos(q5), 2.0) * cos(q6) * sin(q4) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * cos(q2 + q3) * pow(cos(q5), 2.0) * cos(q6) * sin(q4) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * pow(cos(q5), 2.0) * cos(q6) * 9.465000000000146E+32 + cos(q2 + q3 + q4) * cos(q2 + q3) * cos(q6) * sin(q4) * pow(sin(q5), 3.0) * 2.2229999999999E+33 + cos(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * cos(q6) * pow(sin(q5), 3.0) * 2.2229999999999E+33 - cos(q2 + q3 + q4) * sin(q2 + q3) * cos(q6) * sin(q4) * pow(sin(q5), 2.0) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * cos(q2 + q3) * cos(q6) * sin(q4) * pow(sin(q5), 2.0) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * cos(q6) * pow(sin(q5), 2.0) * 9.465000000000146E+32 - sin(q2 + q3 + q4) * cos(q2) * cos(q3) * pow(cos(q5), 2.0) * cos(q6) * 3.9224999999999E+33 - sin(q2 + q3 + q4) * cos(q2) * cos(q3) * cos(q6) * pow(sin(q5), 2.0) * 3.9224999999999E+33 + sin(q2 + q3 + q4) * pow(cos(q5), 2.0) * cos(q6) * sin(q2) * sin(q3) * 3.9224999999999E+33 + sin(q2 + q3 + q4) * cos(q6) * sin(q2) * sin(q3) * pow(sin(q5), 2.0) * 3.9224999999999E+33 - cos(q2 + q3 + q4) * sin(q2 + q3 + q4) * pow(cos(q5), 2.0) * cos(q6) * sin(q5) * 2.2229999999999E+33 + cos(q2 + q3 + q4) * cos(q2 + q3) * cos(q5) * sin(q4) * sin(q6) * 9.465000000000146E+32 + cos(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * cos(q5) * sin(q6) * 9.465000000000146E+32 - sin(q2 + q3 + q4) * cos(q2 + q3) * cos(q4) * cos(q5) * sin(q6) * 9.465000000000146E+32 - cos(q2 + q3 + q4) * cos(q2) * cos(q3) * cos(q5) * sin(q6) * 3.9224999999999E+33 + sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q5) * sin(q4) * sin(q6) * 9.465000000000146E+32 + cos(q2 + q3 + q4) * cos(q2 + q3) * cos(q4) * pow(cos(q5), 2.0) * cos(q6) * 9.465000000000146E+32 + cos(q2 + q3 + q4) * cos(q5) * sin(q2) * sin(q3) * sin(q6) * 3.9224999999999E+33 - sin(q2 + q3 + q4) * cos(q2 + q3) * cos(q5) * sin(q4) * sin(q5) * sin(q6) * 2.2229999999999E+33 - sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * cos(q5) * sin(q5) * sin(q6) * 2.2229999999999E+33 + cos(q2 + q3 + q4) * cos(q2 + q3) * pow(cos(q5), 2.0) * cos(q6) * sin(q4) * sin(q5) * 2.2229999999999E+33 + cos(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * pow(cos(q5), 2.0) * cos(q6) * sin(q5) * 2.2229999999999E+33) * 1.0E-34) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(cos(q6), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q6), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q6), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) * pow(sin(q6), 2.0));
    E[2][0] = ((-cos(q2 + q3 + q4) * cos(q1) * sin(q5) + pow(cos(q2 + q3 + q4), 2.0) * cos(q5) * sin(q1) * 1.0 + pow(sin(q2 + q3 + q4), 2.0) * cos(q5) * sin(q1) * 1.0) * -1.0) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0));
    E[2][1] = ((cos(q2 + q3 + q4) * sin(q1) * sin(q5) + pow(cos(q2 + q3 + q4), 2.0) * cos(q1) * cos(q5) + pow(sin(q2 + q3 + q4), 2.0) * cos(q1) * cos(q5)) * 1.0) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(cos(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q1), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * pow(sin(q1), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q1), 2.0) * pow(sin(q5), 2.0));
    E[2][2] = (sin(q2 + q3 + q4) * sin(q5) * -1.0) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0));
    E[2][3] = ((pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * 2.2229999999999E+33 + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) * 2.2229999999999E+33 + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) * 2.2229999999999E+33 - sin(q2 + q3 + q4) * sin(q5) * 8.899999999999864E+32 + pow(cos(q2 + q3 + q4), 2.0) * cos(q5) * 1.091499999999996E+33 + pow(sin(q2 + q3 + q4), 2.0) * cos(q5) * 1.091499999999996E+33 + sin(q2 + q3 + q4) * sin(q2 + q3) * sin(q5) * 3.9224999999999E+33 + cos(q2 + q3 + q4) * cos(q2) * sin(q5) * 4.250000000000114E+33 + sin(q2 + q3 + q4) * sin(q2) * sin(q5) * 4.250000000000114E+33 + cos(q2 + q3 + q4) * cos(q2) * cos(q3) * sin(q5) * 3.9224999999999E+33 - sin(q2 + q3 + q4) * sin(q2 + q3) * sin(q4) * sin(q5) * 9.465000000000146E+32 - cos(q2 + q3 + q4) * sin(q2) * sin(q3) * sin(q5) * 3.9224999999999E+33 + sin(q2 + q3 + q4) * cos(q2 + q3) * sin(q4) * pow(sin(q5), 2.0) * 2.2229999999999E+33 + sin(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * pow(sin(q5), 2.0) * 2.2229999999999E+33 - cos(q2 + q3 + q4) * cos(q2 + q3) * sin(q4) * sin(q5) * 9.465000000000146E+32 - cos(q2 + q3 + q4) * sin(q2 + q3) * cos(q4) * sin(q5) * 9.465000000000146E+32 + sin(q2 + q3 + q4) * cos(q2 + q3) * cos(q4) * sin(q5) * 9.465000000000146E+32) * -1.0E-34) / (pow(cos(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) + pow(cos(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(cos(q5), 2.0) + pow(sin(q2 + q3 + q4), 2.0) * pow(sin(q5), 2.0));
    E[3][3] = 1.0;
}

// Function to compute the adjoint transformation inverse
void SawsResolveRateMotionControl::AdjointTransformationInverse(double E[4][4], double Ad[6][6])
{
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++)
            Ad[r][c] = 0.0;

    Ad[0][0] = (E[1][1] * E[2][2] - E[1][2] * E[2][1]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[0][1] = -(E[0][1] * E[2][2] - E[0][2] * E[2][1]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[0][2] = (E[0][1] * E[1][2] - E[0][2] * E[1][1]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[0][3] = -(E[0][1] * E[1][2] * E[1][3] - E[0][2] * E[1][1] * E[1][3] + E[0][1] * E[2][2] * E[2][3] - E[0][2] * E[2][1] * E[2][3]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[0][4] = (E[0][1] * E[1][2] * E[0][3] - E[0][2] * E[1][1] * E[0][3] - E[1][1] * E[2][2] * E[2][3] + E[1][2] * E[2][1] * E[2][3]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[0][5] = (E[0][1] * E[2][2] * E[0][3] - E[0][2] * E[2][1] * E[0][3] + E[1][1] * E[2][2] * E[1][3] - E[1][2] * E[2][1] * E[1][3]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[1][0] = -(E[1][0] * E[2][2] - E[1][2] * E[2][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[1][1] = (E[0][0] * E[2][2] - E[0][2] * E[2][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[1][2] = -(E[0][0] * E[1][2] - E[0][2] * E[1][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[1][3] = (E[0][0] * E[1][2] * E[1][3] - E[0][2] * E[1][0] * E[1][3] + E[0][0] * E[2][2] * E[2][3] - E[0][2] * E[2][0] * E[2][3]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[1][4] = -(E[0][0] * E[1][2] * E[0][3] - E[0][2] * E[1][0] * E[0][3] - E[1][0] * E[2][2] * E[2][3] + E[1][2] * E[2][0] * E[2][3]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[1][5] = -(E[0][0] * E[2][2] * E[0][3] - E[0][2] * E[2][0] * E[0][3] + E[1][0] * E[2][2] * E[1][3] - E[1][2] * E[2][0] * E[1][3]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[2][0] = (E[1][0] * E[2][1] - E[1][1] * E[2][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[2][1] = -(E[0][0] * E[2][1] - E[0][1] * E[2][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[2][2] = (E[0][0] * E[1][1] - E[0][1] * E[1][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[2][3] = -(E[0][0] * E[1][1] * E[1][3] - E[0][1] * E[1][0] * E[1][3] + E[0][0] * E[2][1] * E[2][3] - E[0][1] * E[2][0] * E[2][3]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[2][4] = (E[0][0] * E[1][1] * E[0][3] - E[0][1] * E[1][0] * E[0][3] - E[1][0] * E[2][1] * E[2][3] + E[1][1] * E[2][0] * E[2][3]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[2][5] = (E[0][0] * E[2][1] * E[0][3] - E[0][1] * E[2][0] * E[0][3] + E[1][0] * E[2][1] * E[1][3] - E[1][1] * E[2][0] * E[1][3]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[3][3] = (E[1][1] * E[2][2] - E[1][2] * E[2][1]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[3][4] = -(E[0][1] * E[2][2] - E[0][2] * E[2][1]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[3][5] = (E[0][1] * E[1][2] - E[0][2] * E[1][1]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[4][3] = -(E[1][0] * E[2][2] - E[1][2] * E[2][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[4][4] = (E[0][0] * E[2][2] - E[0][2] * E[2][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[4][5] = -(E[0][0] * E[1][2] - E[0][2] * E[1][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[5][3] = (E[1][0] * E[2][1] - E[1][1] * E[2][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[5][4] = -(E[0][0] * E[2][1] - E[0][1] * E[2][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
    Ad[5][5] = (E[0][0] * E[1][1] - E[0][1] * E[1][0]) / (E[0][0] * E[1][1] * E[2][2] - E[0][0] * E[1][2] * E[2][1] - E[0][1] * E[1][0] * E[2][2] + E[0][1] * E[1][2] * E[2][0] + E[0][2] * E[1][0] * E[2][1] - E[0][2] * E[1][1] * E[2][0]);
}

// Function to compute the Jacobian matrix
void SawsResolveRateMotionControl::Jacobian(double q1, double q2, double q3, double q4, double q5, double q6, double J[6][6])
{
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++)
            J[r][c] = 0.0;

    // Just to remove the q6 warning
    double temp = q6;
    temp += 1;

    J[0][1] = cos(q1) * -8.899999999999864E-2;
    J[0][2] = cos(q1) * (sin(q2) * 1.416666666666705E+33 - 2.966666666666621E+32) * 2.99999999997346E-34;
    J[0][3] = cos(q1) * (sin(q2 + q3) * 3.922500000000008E+33 + sin(q2) * 4.250000000000264E+33 - 8.900000000000563E+32) * 1.000000000009837E-34;
    J[0][4] = sin(q1 - q4) * -1.96124999999995E-1 - sin(q1 + q3 + q4) * 2.125000000000057E-1 + sin(-q1 + q3 + q4) * 2.125000000000057E-1 - cos(q1 + q2 + q3 + q4) * 9.907499999999914E-2 - sin(q1 + q4) * 1.96124999999995E-1 - cos(-q1 + q2 + q3 + q4) * 1.00750000000005E-2;
    J[0][5] = cos(q1) * cos(q5) * -8.899999999999864E-2 + sin(q1) * sin(q5) * 9.465000000000146E-2 - sin(q1) * sin(q4) * sin(q5) * 3.9224999999999E-1 + cos(q1) * cos(q5) * sin(q2) * 4.250000000000114E-1 + cos(q1) * cos(q2) * cos(q5) * sin(q3) * 3.9224999999999E-1 + cos(q1) * cos(q3) * cos(q5) * sin(q2) * 3.9224999999999E-1 - cos(q3) * sin(q1) * sin(q4) * sin(q5) * 4.250000000000114E-1 - cos(q4) * sin(q1) * sin(q3) * sin(q5) * 4.250000000000114E-1 + cos(q1) * cos(q2) * cos(q3) * cos(q4) * cos(q5) * 9.465000000000146E-2 - cos(q1) * cos(q2) * cos(q3) * sin(q4) * sin(q5) * 1.091499999999996E-1 - cos(q1) * cos(q2) * cos(q4) * sin(q3) * sin(q5) * 1.091499999999996E-1 - cos(q1) * cos(q2) * cos(q5) * sin(q3) * sin(q4) * 9.465000000000146E-2 - cos(q1) * cos(q3) * cos(q4) * sin(q2) * sin(q5) * 1.091499999999996E-1 - cos(q1) * cos(q3) * cos(q5) * sin(q2) * sin(q4) * 9.465000000000146E-2 - cos(q1) * cos(q4) * cos(q5) * sin(q2) * sin(q3) * 9.465000000000146E-2 - cos(q2) * cos(q3) * cos(q4) * sin(q1) * sin(q5) * 8.899999999999864E-2 + cos(q1) * sin(q2) * sin(q3) * sin(q4) * sin(q5) * 1.091499999999996E-1 + cos(q2) * sin(q1) * sin(q3) * sin(q4) * sin(q5) * 8.899999999999864E-2 + cos(q3) * sin(q1) * sin(q2) * sin(q4) * sin(q5) * 8.899999999999864E-2 + cos(q4) * sin(q1) * sin(q2) * sin(q3) * sin(q5) * 8.899999999999864E-2;
    J[1][1] = sin(q1) * -8.899999999999864E-2;
    J[1][2] = sin(q1) * (sin(q2) * 1.416666666666705E+33 - 2.966666666666621E+32) * 2.99999999997346E-34;
    J[1][3] = sin(q1) * (sin(q2 + q3) * 3.922500000000008E+33 + sin(q2) * 4.250000000000264E+33 - 8.900000000000563E+32) * 1.000000000009837E-34;
    J[1][4] = cos(q1 - q4) * 1.96124999999995E-1 + cos(q1 + q3 + q4) * 2.125000000000057E-1 + cos(-q1 + q3 + q4) * 2.125000000000057E-1 - sin(q1 + q2 + q3 + q4) * 9.907499999999914E-2 + cos(q1 + q4) * 1.96124999999995E-1 + sin(-q1 + q2 + q3 + q4) * 1.00750000000005E-2;
    J[1][5] = cos(q1) * sin(q5) * -9.465000000000146E-2 - cos(q5) * sin(q1) * 8.899999999999864E-2 + cos(q5) * sin(q1) * sin(q2) * 4.250000000000114E-1 + cos(q1) * sin(q4) * sin(q5) * 3.9224999999999E-1 + cos(q2) * cos(q5) * sin(q1) * sin(q3) * 3.9224999999999E-1 + cos(q3) * cos(q5) * sin(q1) * sin(q2) * 3.9224999999999E-1 + cos(q1) * cos(q3) * sin(q4) * sin(q5) * 4.250000000000114E-1 + cos(q1) * cos(q4) * sin(q3) * sin(q5) * 4.250000000000114E-1 + cos(q1) * cos(q2) * cos(q3) * cos(q4) * sin(q5) * 8.899999999999864E-2 + cos(q2) * cos(q3) * cos(q4) * cos(q5) * sin(q1) * 9.465000000000146E-2 - cos(q1) * cos(q2) * sin(q3) * sin(q4) * sin(q5) * 8.899999999999864E-2 - cos(q1) * cos(q3) * sin(q2) * sin(q4) * sin(q5) * 8.899999999999864E-2 - cos(q1) * cos(q4) * sin(q2) * sin(q3) * sin(q5) * 8.899999999999864E-2 - cos(q2) * cos(q3) * sin(q1) * sin(q4) * sin(q5) * 1.091499999999996E-1 - cos(q2) * cos(q4) * sin(q1) * sin(q3) * sin(q5) * 1.091499999999996E-1 - cos(q2) * cos(q5) * sin(q1) * sin(q3) * sin(q4) * 9.465000000000146E-2 - cos(q3) * cos(q4) * sin(q1) * sin(q2) * sin(q5) * 1.091499999999996E-1 - cos(q3) * cos(q5) * sin(q1) * sin(q2) * sin(q4) * 9.465000000000146E-2 - cos(q4) * cos(q5) * sin(q1) * sin(q2) * sin(q3) * 9.465000000000146E-2 + sin(q1) * sin(q2) * sin(q3) * sin(q4) * sin(q5) * 1.091499999999996E-1;
    J[2][2] = cos(q2) * 4.250000000000114E-1;
    J[2][3] = cos(q2 + q3) * 3.9224999999999E-1 + cos(q2) * 4.250000000000114E-1;
    J[2][4] = sin(q2 + q3 + q4) * 1.091499999999996E-1;
    J[2][5] = cos(q2 - q5) * 2.125000000000057E-1 + cos(q2 + q3 + q5) * 1.96124999999995E-1 + cos(q2 + q3 - q5) * 1.96124999999995E-1 - sin(q2 + q3 + q4 + q5) * 1.019000000000005E-1 + cos(q2 + q5) * 2.125000000000057E-1 + sin(q2 + q3 + q4 - q5) * 7.249999999999091E-3;
    J[3][1] = sin(q1) * -1.0;
    J[3][2] = sin(q1) * -1.0;
    J[3][3] = sin(q1) * -1.0;
    J[3][4] = sin(q1 + q2 + q3 + q4) * -5.0E-1 - sin(-q1 + q2 + q3 + q4) * 5.0E-1;
    J[3][5] = cos(q5) * sin(q1) * -1.0 + cos(q2 + q3 + q4) * cos(q1) * sin(q5);
    J[4][1] = cos(q1);
    J[4][2] = cos(q1);
    J[4][3] = cos(q1);
    J[4][4] = cos(q1 + q2 + q3 + q4) * 5.0E-1 - cos(-q1 + q2 + q3 + q4) * 5.0E-1;
    J[4][5] = cos(q1) * cos(q5) + cos(q2 + q3 + q4) * sin(q1) * sin(q5);
    J[5][0] = 1.0;
    J[5][4] = cos(q2 + q3 + q4) * -1.0;
    J[5][5] = cos(q2 + q3 + q4 + q5) * 5.0E-1 - cos(q2 + q3 + q4 - q5) * 5.0E-1;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SawsResolveRateMotionControl>("saws_resolve_rate_motion_control");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
