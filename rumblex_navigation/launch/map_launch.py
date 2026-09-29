"""Publish a static occupancy map and activate its lifecycle node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=PathJoinSubstitution([
                FindPackageShare('rumblex_navigation'), 'maps', 'house_contour.yaml']),
            description='Occupancy map YAML file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
