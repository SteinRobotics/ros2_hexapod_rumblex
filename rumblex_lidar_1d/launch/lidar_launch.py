"""Launch the nox lidar using its robot-specific configuration."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='nox', choices=['nox']),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rumblex_lidar_1d'), 'config', LaunchConfiguration('robot'), 'lidar.yaml'])),
        Node(package='rumblex_lidar_1d', executable='node_lidar', name='node_lidar',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
