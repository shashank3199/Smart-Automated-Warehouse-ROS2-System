from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get the launch file directories
    turtlebot4_navigation_launch_dir = get_package_share_directory('turtlebot4_navigation')
    saws_turtlebot4_teleop_setup_launch_dir = get_package_share_directory('saws_turtlebot4_teleop_setup')
    saws_turtlebot4_nav2_move_to_launch_dir = get_package_share_directory('saws_turtlebot4_nav2_move_to')
    
    # Path to the positions.yaml file
    positions_file = os.path.join(saws_turtlebot4_teleop_setup_launch_dir, 'config', 'positions.yaml')
    
    # Path to the wyman160.yaml file
    map_file = os.path.join(saws_turtlebot4_teleop_setup_launch_dir, 'maps', 'wyman160.yaml')

    # Read the positions.yaml file
    positions = []
    if os.path.exists(positions_file):
        with open(positions_file, 'r') as file:
            positions_data = yaml.safe_load(file)
            for position in positions_data['positions']:
                pos = {
                    'x': position['x'],
                    'y': position['y'],
                    'w': position['w']
                }
                positions.append(pos)

    # Include the nav2.launch.py file from turtlebot4_navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot4_navigation_launch_dir, 'launch', 'nav2.launch.py'))
    )

    # Include the saws_turtlebot4_teleop_setup.launch.py file with localization:=true and map argument
    teleop_setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(saws_turtlebot4_teleop_setup_launch_dir, 'launch', 'saws_turtlebot4_teleop_setup.launch.py')),
        launch_arguments={'localization': 'true', 'map': map_file}.items()
    )

    # Sequence of actions to launch navigation to each position with delays
    actions = [nav2_launch, teleop_setup_launch]

    for idx in range(len(positions) - 1):
        # The move to executable needs an inital and goal pose similar to initializing turtlebot4 in assignments within the map
        initial_pose = positions[idx]
        goal_pose = positions[idx + 1]

        # Calling the executable to move the turtlebot4
        move_to_goal_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(saws_turtlebot4_nav2_move_to_launch_dir, 'launch', 'navigate_to_goal.launch.py')),
            launch_arguments={
                'initial_x': str(initial_pose['x']),
                'initial_y': str(initial_pose['y']),
                'initial_w': str(initial_pose['w']),
                'goal_x': str(goal_pose['x']),
                'goal_y': str(goal_pose['y']),
                'goal_w': str(goal_pose['w'])
            }.items()
        )
        
        actions.append(LogInfo(msg=f"Navigating to goal {idx + 1}: {goal_pose}"))
        actions.append(move_to_goal_launch)

        # Adding a delay to wait for the turtlebot4 to get to the current goal position
        if idx < len(positions) - 2:
            actions.append(TimerAction(period=2.0, actions=[LogInfo(msg="Waiting for 2 seconds before next goal")]))

    return LaunchDescription(actions)
