"""Launch the STL-based RumbleX model in Gazebo Harmonic."""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def _launch_robot(context):
    robot = LaunchConfiguration('robot').perform(context)
    if not re.fullmatch(r'[a-z][a-z0-9_]*', robot):
        raise RuntimeError(f'Invalid robot profile: {robot!r}')

    description_share = get_package_share_directory('rumblex_description')
    gazebo_share = get_package_share_directory('rumblex_gazebo')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', f'{robot}_mesh.urdf.xacro'),
        mappings={'use_sim': 'true'},
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', os.path.join(gazebo_share, 'worlds', 'empty.sdf')]}.items(),
    )
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', f'{robot}_mesh', '-z', '0.07'],
                 output='screen')
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], output='screen')
    position = Node(package='controller_manager', executable='spawner',
                    arguments=['forward_position_controller'], output='screen')
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    return [
        gazebo, rsp, spawn, clock_bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[position])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='nox',
            description='Robot model profile (for example: nox or nira)'),
        OpaqueFunction(function=_launch_robot),
    ])
