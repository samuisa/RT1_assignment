import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Percorsi ai pacchetti
    pkg_bme_gazebo_sensors = FindPackageShare('bme_gazebo_sensors')
    
    # 2. Inclusione del Launch File della Simulazione (bme_gazebo_sensors)
    # Questo avvia Gazebo, spawna il robot, Rviz e i bridge
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_bme_gazebo_sensors,
                'launch',
                'spawn_robot.launch.py'
            ])
        ])
    )

    # 3. Nodo Controller (Input Utente)
    # Usa xterm per aprire una nuova finestra terminale dedicata all'input
    controller_node = Node(
        package='assignment2_rt',
        executable='controller_node', 
        name='controller',
        output='screen',
        prefix='xterm -e'  # Rimuovi questa riga se non hai installato xterm
    )

    # 4. Nodo Safety (Logica e Ostacoli)
    safety_node = Node(
        package='assignment2_rt',
        executable='safety_node',
        name='safety_node',
        output='screen'
    )

    # 5. Fix per la variabile d'ambiente (per evitare KeyError nel file incluso)
    ensure_env_var = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.environ.get('GZ_SIM_RESOURCE_PATH', '') 
    )

    return LaunchDescription([
        ensure_env_var,      # Imposta la var d'ambiente per sicurezza
        spawn_robot_launch,  # Avvia la simulazione fornita
        safety_node,         # Avvia il tuo nodo di sicurezza
        controller_node,     # Avvia il tuo nodo di controllo
    ])