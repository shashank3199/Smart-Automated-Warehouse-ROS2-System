from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments for SLAM, map, and localization with default values and descriptions
    declare_slam_arg = DeclareLaunchArgument('slam', default_value='false', description='Enable SLAM?')
    declare_map_arg = DeclareLaunchArgument('map', default_value='wyman160.yaml', description='Map name if using SLAM')
    declare_localization_arg = DeclareLaunchArgument('localization', default_value='false', description='Enable localization?')

    # Get the paths for the necessary launch files from their respective packages
    joy_teleop_launch_file = os.path.join(get_package_share_directory('turtlebot4_bringup'), 'launch', 'joy_teleop.launch.py')
    slam_launch_file = os.path.join(get_package_share_directory('turtlebot4_navigation'), 'launch', 'slam.launch.py')
    localization_launch_file = os.path.join(get_package_share_directory('turtlebot4_navigation'), 'launch', 'localization.launch.py')

    # Include the joy_teleop launch file unless localization is enabled
    include_joy_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_teleop_launch_file),
        launch_arguments={'namespace': '/'}.items(),
        condition=UnlessCondition(LaunchConfiguration('localization'))
    )

    # Function to check if SLAM or localization is enabled and if a map name is provided
    def check_slam_localization_and_map(context):
        slam = LaunchConfiguration('slam').perform(context)
        localization = LaunchConfiguration('localization').perform(context)
        map_name = LaunchConfiguration('map').perform(context)
        if (slam == 'true' or localization == 'true') and not map_name:
            raise RuntimeError("Either SLAM or localization is enabled but no map name was provided.")

    # Include the SLAM launch file if SLAM is enabled
    include_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # Expressions to get the map name without extension and with the full path
    map_name_without_extension = PythonExpression(["'", LaunchConfiguration('map'), "'.replace('.yaml', '')"])
    map_directory = os.path.join(get_package_share_directory('saws_turtlebot4_teleop_setup'), 'maps')
    map_path = PythonExpression(["'", map_directory, "/", map_name_without_extension, "'"])
    map_path_with_extension = PythonExpression(["'", map_directory, "/", LaunchConfiguration('map'), "'"])

    # Include the localization launch file if localization is enabled
    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
        launch_arguments={'map': map_path_with_extension}.items(),
        condition=IfCondition(LaunchConfiguration('localization'))
    )

    # Node for the joystick listener with the map name as a parameter
    joystick_listener = Node(
        package='saws_turtlebot4_teleop_setup',
        executable='joystick_listener',
        name='joystick_listener',
        output='screen',
        parameters=[{'map_name': map_name_without_extension}]
    )

    # Return the launch description with all the actions and nodes
    return LaunchDescription([
        declare_slam_arg,
        declare_map_arg,
        declare_localization_arg,
        include_joy_teleop,
        OpaqueFunction(function=check_slam_localization_and_map),
        include_slam,
        include_localization,
        joystick_listener
    ])
