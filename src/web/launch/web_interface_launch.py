from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='web',
            executable='web_bridge',
            name='web_bridge',
            output='screen',
            parameters=[
                {'port': 8765}
            ],
        ),
        Node(
            package='web',
            executable='web_server',
            name='web_server',
            output='screen',
            parameters=[
                {'port': 8080}
            ],
        ),
        Node(
            package='web',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
        ),
    ])
