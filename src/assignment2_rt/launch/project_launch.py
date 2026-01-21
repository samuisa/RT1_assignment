import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Locate the current package share directory
    pkg_assignment2 = FindPackageShare('assignment2_rt')
    
    # Launch Simulation (Gazebo + Robot + Bridge)
    # This includes the external launch file that spawns the robot
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_assignment2, 
                'launch',
                'spawn_robot.launch.py'
            ])
        ])
    )

    # Controller Node (User Input)
    # REMAPPING: The controller thinks it is publishing to /cmd_vel, 
    # but we redirect it to /cmd_vel_input so the Safety Node can intercept it.
    controller_node = Node(
        package='assignment2_rt',
        executable='controller_node', 
        name='controller',
        output='screen',
        prefix='xterm -e', # Opens a new terminal window for input
        remappings=[
            ('/cmd_vel', '/cmd_vel_input') 
        ]
    )

    # Safety Node (Obstacle Avoidance Logic)
    # This node subscribes to /cmd_vel_input (from controller) and /scan,
    # then publishes safe commands to /cmd_vel (to the robot).
    safety_node = Node(
        package='assignment2_rt',
        executable='safety_node',
        name='safety_node',
        output='screen',
        prefix='xterm -e'
        # No remapping needed here as the C++ code is already set to:
        # Sub: /cmd_vel_input
        # Pub: /cmd_vel
    )

    # Ensure Gazebo resources path is set correctly
    ensure_env_var = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.environ.get('GZ_SIM_RESOURCE_PATH', '') 
    )

    return LaunchDescription([
        ensure_env_var,
        spawn_robot_launch,
        safety_node,
        controller_node,
    ])