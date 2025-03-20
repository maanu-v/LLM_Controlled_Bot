import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Motor control node (runs normally)
        Node(
            package='bot_movement',
            executable='motor_control',
            name='crobot_motor_node',
            output='screen'
        ),

        # Keyboard control node (runs in an interactive terminal)
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'bot_movement', 'keyboard'],
            shell=True
        ),
    ])
