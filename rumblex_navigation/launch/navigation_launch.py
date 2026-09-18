"""Launch head-sweep scan and reactive navigation for a RumbleX hexapod."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('rumblex_navigation')
    robot = LaunchConfiguration('robot')
    config = PathJoinSubstitution([pkg_nav, 'config', robot, 'navigation.yaml'])
    map_file = os.path.join(pkg_nav, 'maps', 'simple_room.yaml')

    enable_map_arg = DeclareLaunchArgument(
        'enable_map', default_value='false',
        description='Start map_server to publish the static occupancy grid')

    head_scan = Node(
        package='rumblex_navigation',
        executable='node_head_scan',
        name='node_head_scan',
        output='screen',
        parameters=[config],
    )

    navigation = Node(
        package='rumblex_navigation',
        executable='node_navigation',
        name='node_navigation',
        output='screen',
        parameters=[config],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_map')),
    )

    # Lifecycle manager auto-activates the map_server node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_map')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='nox',
            description='Robot configuration profile (for example: nox or nira)'),
        enable_map_arg,
        head_scan,
        navigation,
        map_server,
        lifecycle_manager,
    ])
