"""Launch the nox head-sweep processing using its robot-specific configuration."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='nox', choices=['nox']),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rumblex_perception'), 'config', LaunchConfiguration('robot'), 'perception.yaml'])),
        Node(package='rumblex_perception', executable='node_head_scan', name='node_head_scan',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
