from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Launch arguments
    num_viewpoints_arg = DeclareLaunchArgument(
        'num_viewpoints',
        default_value='3',
        description='Number of viewpoints to generate and visit'
    )
    
    image_dir_arg = DeclareLaunchArgument(
        'images_directory',
        default_value='/home/jetson/Documents/mark/images',
        description='Directory to store captured images'
    )
    
    metadata_dir_arg = DeclareLaunchArgument(
        'metadata_directory',
        default_value='/home/jetson/Documents/mark/metadata',
        description='Directory to store metadata'
    )

    # Define the nodes
    viewpoint_planner_node = Node(
        package='image_pkg',
        executable='viewpoint_planner',
        name='viewpoint_planner',
        parameters=[{
            'num_viewpoints': LaunchConfiguration('num_viewpoints'),
            'images_directory': LaunchConfiguration('images_directory'),
            'metadata_directory': LaunchConfiguration('metadata_directory')
        }],
        output='screen'
    )
    
    image_capture_node = Node(
        package='image_pkg',
        executable='image_capture_node',
        name='image_capture_node',
        parameters=[{
            'images_directory': LaunchConfiguration('images_directory'),
            'metadata_directory': LaunchConfiguration('metadata_directory')
        }],
        output='screen'
    )
    
    object_detection_node = Node(
        package='image_pkg',
        executable='object_detection_node',
        name='object_detection_node',
        parameters=[{
            'images_directory': LaunchConfiguration('images_directory'),
            'metadata_directory': LaunchConfiguration('metadata_directory')
        }],
        output='screen'
    )
    
    # Log some info for the user
    startup_info = LogInfo(msg=[
        '\n\n',
        '=== Launching Camera Capture and Object Detection System ===\n',
        'Image directory: ', LaunchConfiguration('images_directory'), '\n',
        'Metadata directory: ', LaunchConfiguration('metadata_directory'), '\n',
        '======================================================\n\n'
    ])
    
    return LaunchDescription([
        # Launch arguments
        num_viewpoints_arg,
        image_dir_arg,
        metadata_dir_arg,
        
        # Info display
        startup_info,
        
        # Launch nodes in proper order
        viewpoint_planner_node,
        image_capture_node,
        object_detection_node
    ])
