from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    joy_joint_launch  = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('saws_teleop_joy'),
                    'launch',
                    'saws_teleop_joy.launch.py'
                ])
            ]),
            launch_arguments={'control_type': 'joint'}.items()
        )

    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        output='screen'
    )

    aruco_node = Node(
        package="aruco_ros",
        executable="single",
        name="aruco",
        parameters=[{"marker_id": 1,
                    "marker_size": 0.1,
                    "marker_frame": "marker",
                    "camera_frame": "camera_link",
                    "image_is_rectified": True}],
        remappings=[("/image", "/color/image_raw"), ("/camera_info", "/color/camera_info")]
    )

    calibration_node = Node(
        package='saws_handeye_calibration',
        executable='calibration',
        name='calibration',
        output='screen'
    )

    return LaunchDescription([
        joy_joint_launch,
        camera_node,
        aruco_node,
        calibration_node
    ])

if __name__ == '__main__':
    generate_launch_description()
