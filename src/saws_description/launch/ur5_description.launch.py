from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    world_name = 'warehouse'
    pkg_dir = get_package_share_directory('saws_description')
    realsense_pkg_dir = get_package_share_directory('realsense2_description')

    gz_sim_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_dir, 'worlds'), ':',
            os.path.join(pkg_dir, 'models'), ':',
            os.path.join(realsense_pkg_dir, 'meshes')
        ]
    )

    world_file_path = os.path.join(
        pkg_dir, 'worlds', f'{world_name}.world'
    )

    start_ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': f'-v 4 -r {world_file_path}'
        }.items()
    )

    # ignition_launch = PathJoinSubstitution([pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    # pkg_turtlebot4_ignition_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    # robot_spawn_launch = PathJoinSubstitution([pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])

    # start_ign_gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([ignition_launch]),
    #     launch_arguments=[
    #         ('world', world_name)
    #     ]
    # )

    # robot_spawn = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([robot_spawn_launch]),
    #     launch_arguments=[
    #         ('namespace', 'tb4'),
    #         ('rviz', 'false'),
    #         ('x', '0'),
    #         ('y', '0'),
    #         ('z', '0'),
    #         ('yaw', '0')]
    # )

    xacro_file_path = os.path.join(pkg_dir, 'urdf', 'saws.urdf.xacro')
    urdf_file_path = os.path.join(pkg_dir, 'urdf', 'saws.urdf')

    os.system(f'xacro {xacro_file_path} > {urdf_file_path}')

    spawn_urdf_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', f'{urdf_file_path}'],
        output='screen'
    )

    pkg_share = FindPackageShare("saws_description")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_share, "urdf", "saws.urdf.xacro"]),
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(robot_description_content, value_type=str),
        }],
    )

    joint_command_position = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_positions",
        arguments=["position_controller", "-c", "/controller_manager"],
    )

    # pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    # tb4_xacro_file = os.path.join(pkg_turtlebot4_description, 'urdf', 'standard', 'turtlebot4.urdf.xacro')
    # tb4_urdf_file = os.path.join(pkg_dir, 'urdf', 'tb4.urdf')
    # os.system(f'xacro {tb4_xacro_file} > {tb4_urdf_file}')

    # tb4_joint_command_position = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="joint_positions",
    #     arguments=["position_controller", "-c", "/tb4/controller_manager"],
    # )

    # spawn_tb4_node = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=['-file', f'{tb4_urdf_file}'],
    #     output='screen'
    # )

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': 'false'},
    #         {'robot_description': Command([
    #             'xacro', ' ', xacro_file, ' ',
    #             'gazebo:=ignition', ' ',
    #             'namespace:=tb4'])},
    #     ],
    #     remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static')
    #     ]
    # )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': 'false'}],
    #     remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static')
    #     ]
    # )


    return LaunchDescription(
        [
            gz_sim_resource_path,
            start_ign_gazebo,
            # robot_spawn,
            spawn_urdf_node,
            robot_state_publisher_node,
            joint_command_position,
            # tb4_joint_command_position
            # spawn_tb4_node,
        ]
    )
