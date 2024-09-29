from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    com_port_arg = DeclareLaunchArgument(
        name='com_port',
        default_value='/dev/ttyUSB0',
        description='Serial port'
    )

    gripper_driver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotiq_description'),
                    'launch',
                    'robotiq_control.launch.py'
                ])
            ]),
            launch_arguments={
                'com_port': LaunchConfiguration('com_port')
            }.items(),
        )

    link_tool0_to_robotiq_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_tool0_to_robotiq_base',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            '/tool0', '/robotiq_85_base_link',
        ],
    )

    gripper_action_server = Node(
        package='saws_gripper_actionlib',
        executable='saws_gripper_action_server',
        output='screen'
    )

    return LaunchDescription(
        [
            com_port_arg,
            gripper_driver_launch,
            link_tool0_to_robotiq_base,
            gripper_action_server
        ]
    )
