from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    joystick_device_arg = DeclareLaunchArgument(
        "joystick_device",
        default_value="/dev/input/js0",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joystick_device'),
        }],
    )

    saws_gripper_control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('saws_gripper_actionlib'),
                    'launch',
                    'saws_gripper_control.launch.py'
                ])
            ])
        )

    gripper_teleop_node = Node(
        package='saws_teleop_joy',
        executable='gripper_teleop',
        name='gripper_teleop',
    )

    return LaunchDescription([
        joystick_device_arg,
        joy_node,
        saws_gripper_control_launch,
        gripper_teleop_node,
    ])
