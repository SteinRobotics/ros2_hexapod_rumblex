"""Launch the STL-based RumbleX model in Gazebo Harmonic."""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def _after_success(actions, stage):
    def on_exit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=f'{stage} failed (exit {event.returncode})'))]
        return actions
    return on_exit


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
        launch_arguments={'gz_args': [
            '-r ' if LaunchConfiguration('gui').perform(context) == 'true' else '-r -s ',
            os.path.join(gazebo_share, 'worlds', 'empty.sdf'),
        ]}.items(),
    )
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', f'{robot}_mesh',
                            '-x', LaunchConfiguration('spawn_x'),
                            '-y', LaunchConfiguration('spawn_y'),
                            '-Y', LaunchConfiguration('spawn_yaw'), '-z', '0.07'],
                 output='screen')
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], output='screen')
    position = Node(package='controller_manager', executable='spawner',
                    arguments=['forward_position_controller', '--param-file',
                               os.path.join(gazebo_share, 'config', robot, 'joint_controllers.yaml')],
                    output='screen')
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=_after_success([jsb], 'Robot spawn'))),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb, on_exit=_after_success([position], 'Joint state broadcaster'))),
        RegisterEventHandler(OnProcessExit(
            target_action=position, on_exit=_after_success([], 'Position controller'))),
        gazebo, rsp, spawn, clock_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'robot', default_value='nox',
            description='Robot model profile (for example: nox or nira)'),
        DeclareLaunchArgument('spawn_x', default_value='-2.26'),
        DeclareLaunchArgument('spawn_y', default_value='-2.13'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=_launch_robot),
    ])
