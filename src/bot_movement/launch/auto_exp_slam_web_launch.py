from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to included launch files
    bot_movement_launch = os.path.join(
        get_package_share_directory('bot_movement'),
        'launch',
        'motor_odom_lidar_launch.py'
    )

    web_interface_launch = os.path.join(
        get_package_share_directory('web'),
        'launch',
        'web_interface_launch.py'
    )

    nav2_bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    realsense2_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # SLAM parameters file path
    slam_params_path = '/opt/ros/foxy/share/slam_toolbox/config/mapper_params_online_async.yaml'

    return LaunchDescription([
        # Launch motor control, odometry, and lidar
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bot_movement_launch)
        ),

        # Launch web interface
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_interface_launch)
        ),

        # Launch Nav2 bringup with SLAM enabled
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                'slam': 'True',
                'use_sim_time': 'False',
                'map': 'none',
                'slam_params_file': slam_params_path
            }.items()
        ),

        # Launch frontier exploration node
        Node(
            package='frontier_exp',
            executable='explorer_node',
            name='frontier_explorer',
            output='screen'
        ),

        # Launch RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense2_launch)
        ),
    ])

