from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    joystick_device_arg = DeclareLaunchArgument(
        "joystick_device",
        default_value="/dev/input/js0",
    )

    control_type_arg = DeclareLaunchArgument(
        "control_type",
        default_value="cartesian",
        description="Type of control: cartesian or joint",
        choices=["cartesian", "joint"],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joystick_device'),
        }],
    )

    ur_bringup_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('saws_bringup'),
                    'launch',
                    'ur5_bringup.launch.py'
                ])
            ])
        ),
    ])

    cartesian_teleop_node = Node(
        package='saws_teleop_joy',
        executable='cartesian_teleop',
        name='cartesian_teleop',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_type'), "' == 'cartesian'"]))
    )

    rrmc_node = Node(
        package='saws_ur5_rrmc',
        executable='saws_rrmc',
        name='saws_rrmc',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_type'), "' == 'cartesian'"]))
    )

    joint_teleop_node = Node(
        package='saws_teleop_joy',
        executable='joint_teleop',
        name='joint_teleop',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_type'), "' == 'joint'"]))
    )

    return LaunchDescription([
        joystick_device_arg,
        control_type_arg,
        joy_node,
        ur_bringup_launch,
        cartesian_teleop_node,
        rrmc_node,
        joint_teleop_node,
    ])
