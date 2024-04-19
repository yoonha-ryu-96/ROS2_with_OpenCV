import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
        get_package_share_directory('opencv_pkg'),
        'param',
        'camera_val.yaml')
        )
    
    param_dir2 = LaunchConfiguration(
        'param_dir2',
        default=os.path.join(
        get_package_share_directory('opencv_pkg'),
        'param',
        'rgb_val.yaml')
        )
    
    param_dir3 = LaunchConfiguration(
        'param_dir3',
        default=os.path.join(
        get_package_share_directory('opencv_pkg'),
        'param',
        'capture.yaml')
        )
    
    param_dir4 = LaunchConfiguration(
        'param_dir4',
        default=os.path.join(
        get_package_share_directory('opencv_pkg'),
        'param',
        'record.yaml')
        )


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'param_dir',
                default_value=param_dir
            ),

            DeclareLaunchArgument(
                'param_dir2',
                default_value=param_dir2
            ),

            DeclareLaunchArgument(
                'param_dir3',
                default_value=param_dir3
            ),

            DeclareLaunchArgument(
                'param_dir4',
                default_value=param_dir4
            ),

            Node(
                package='opencv_pkg',
                executable='img_pub',
                name='img_with_filters',
                parameters=[param_dir],
                output='screen'),

            Node(
                package='opencv_pkg',
                executable='rgb_sub',
                name='img_with_color',
                parameters=[param_dir2],
                output='screen'),

            Node(
                package='opencv_pkg',
                executable='capture_serv',
                name='capture',
                parameters=[param_dir3],
                output='screen'),

            Node(
                package='opencv_pkg',
                executable='video_server',
                name='record',
                parameters=[param_dir4],
                output='screen'),
        ]
    )