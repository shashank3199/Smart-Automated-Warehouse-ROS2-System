from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    saws_pickup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('saws_ur5_pickup'),
                    'launch',
                    'saws_pickup.launch.py'
                ])
            ])
        )

    saws_control_loop_node = Node(
        package='saws_ur5_control_loop',
        executable='saws_ur5_control_loop',
        name='saws_ur5_control_loop',
    )

    return LaunchDescription(
        [
            saws_pickup_launch,
            saws_control_loop_node
        ]
    )
