from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the RPLIDAR launch file
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py'  # Ensure this is the correct file name
    )

    return LaunchDescription([
        # Include the RPLIDAR launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        ),

        # For motor intialisation
        Node(
            package='bot_movement',
            executable='motor_control',
            name='crobot_motor_node',
            output='screen'
        ),

        # For publishing RPM values
        Node(
            package='bot_movement',
            executable='odom_motor_pub',
            name='crobot_odom',
            output='screen'
        ),

        # For publishing the odom topic
        Node(
            package='bot_movement',
            executable='odom_pub',
            name='odom_publisher',
            output='screen'
        ),
    ])
