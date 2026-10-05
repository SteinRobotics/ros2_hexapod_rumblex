"""Launch the nira lidar using its robot-specific configuration."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='nira', choices=['nira']),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rumblex_lidar_2d'), 'config', LaunchConfiguration('robot'), 'lidar.yaml'])),
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', name='ydlidar_ros2_driver_node',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
