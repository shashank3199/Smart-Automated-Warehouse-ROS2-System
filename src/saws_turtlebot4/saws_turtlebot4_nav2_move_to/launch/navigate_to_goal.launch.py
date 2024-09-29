from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        # Receiving initial pose and goal pose for navigation of turtlebot4
        DeclareLaunchArgument('initial_x', default_value='0.0', description='Initial X position'),
        DeclareLaunchArgument('initial_y', default_value='0.0', description='Initial Y position'),
        DeclareLaunchArgument('initial_w', default_value='1.0', description='Initial W orientation'),
        DeclareLaunchArgument('goal_x', default_value='1.0', description='Goal X position'),
        DeclareLaunchArgument('goal_y', default_value='1.0', description='Goal Y position'),
        DeclareLaunchArgument('goal_w', default_value='1.0', description='Goal W orientation'),

        # Calling the navigation of turtlebot4
        Node(
            package='saws_turtlebot4_nav2_move_to',
            executable='navigate_to_goal',
            name='navigate_to_goal_node',
            output='screen',
            parameters=[{
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_w': LaunchConfiguration('initial_w'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_w': LaunchConfiguration('goal_w'),
            }]
        )
    ])
