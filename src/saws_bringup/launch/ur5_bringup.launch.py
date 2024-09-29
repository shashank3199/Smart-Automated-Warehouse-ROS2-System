from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ur_driver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_robot_driver'),
                    'launch',
                    'ur_control.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur5',
                'robot_ip': '172.22.22.2',
                'use_fake_hardware': 'False',
                'launch_rviz': 'True',
                'robot_controller': 'scaled_joint_trajectory_controller',
                'headless_mode': 'True'
            }.items()
        )

    link_tool0_to_gripper = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_tool0_to_gripper',
        arguments=[
            '0', '0', '0.14', '0', '0', '0',
            '/tool0', '/gripper_pick',
        ],
    )

    ur_joint_interpolator_node = Node(
        package='saws_ur5_joint_interpolator',
        executable='ur5_joint_interpolator',
        name='ur5_joint_interpolator',
    )

    return LaunchDescription(
        [
            ur_driver_launch,
            link_tool0_to_gripper,
            ur_joint_interpolator_node
        ]
    )
