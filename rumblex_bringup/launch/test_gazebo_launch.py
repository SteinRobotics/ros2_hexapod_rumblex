"""Run the test stack with the Gazebo mesh robot and physical map walls.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import yaml


def _joint_bridge(context):
    robot = LaunchConfiguration('robot').perform(context)
    config = os.path.join(
        get_package_share_directory('rumblex_gazebo'), 'config', robot, 'joint_controllers.yaml')
    with open(config) as stream:
        joint_names = yaml.safe_load(stream)['forward_position_controller']['ros__parameters']['joints']
    return [Node(
        package='rumblex_gazebo', executable='joint_state_bridge.py',
        name='joint_state_bridge', output='screen',
        parameters=[{'joint_names': joint_names}],
    )]


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='nox', choices=['nox', 'nira']),
        GroupAction(actions=[
            SetParameter(name='use_sim_time', value=True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('rumblex_gazebo'), 'launch', 'simulation_mesh.launch.py'])),
                launch_arguments={
                    'robot': robot, 'spawn_x': '-2.26', 'spawn_y': '-2.13',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('rumblex_bringup'), 'launch', 'test_launch.py'])),
                launch_arguments={
                    'robot': robot,
                    'use_sim_time': 'true',
                    'enable_display': 'false',
                    'movement_joint_states_topic': 'target_joint_states',
                    'publish_gazebo_map': 'true',
                    'start_x': '-2.26',
                    'start_y': '-2.13',
                }.items(),
            ),
            OpaqueFunction(function=_joint_bridge),
            Node(
                package='ros_gz_sim', executable='create', name='spawn_map_walls',
                arguments=['-topic', 'gazebo_map_description', '-name', 'map_walls',
                           '-world', 'hexapod_world'], output='screen',
            ),
        ]),
    ])
