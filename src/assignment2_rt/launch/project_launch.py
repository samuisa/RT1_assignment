import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # MODIFICA: Ora cerchiamo i file di launch all'interno di questo stesso pacchetto
    pkg_assignment2 = FindPackageShare('assignment2_rt')
    
    # Avvio Simulazione (Gazebo + Robot + Bridge)
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_assignment2, # Usa il pacchetto corrente
                'launch',
                'spawn_robot.launch.py'
            ])
        ])
    )

    # Nodo Controller (Input Utente)
    # REMAPPING: Il controller crede di scrivere su /cmd_vel, ma noi lo
    # deviamo su /cmd_vel_input affinché il Safety Node possa intercettarlo.
    controller_node = Node(
        package='assignment2_rt',
        executable='controller_node', 
        name='controller',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', '/cmd_vel_input') 
        ]
    )

    # Nodo Safety (Logica Ostacoli)
    # Questo nodo ascolta /cmd_vel_input (dal controller) e pubblica su /cmd_vel (al robot)
    safety_node = Node(
        package='assignment2_rt',
        executable='safety_node',
        name='safety_node',
        output='screen'
        # Non serve remapping qui perché nel codice C++ abbiamo già impostato:
        # Sub: /cmd_vel_input
        # Pub: /cmd_vel
    )

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