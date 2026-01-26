from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            namespace='turtlesim1',
            parameters=[{
                'background_r': 150     
            }],

            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='turtlesim_controller',
            executable='turtlesim_service',
            name='service',
            namespace='turtlesim1',
            arguments=['--ros-args', '--log-level', 'warn']
        ),

        Node(
            package='turtlesim_controller',
            executable='turtlesim_controller3',
            name='control',
            namespace='turtlesim1'
        ),
    ])
