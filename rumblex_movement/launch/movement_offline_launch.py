from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    config_dir = [get_package_share_directory('rumblex_movement'), 'config', robot]
    anatomy = PathJoinSubstitution([get_package_share_directory('rumblex_description'), 'config', robot, 'anatomy.yaml'])
    servo_description = PathJoinSubstitution(config_dir + ['servo_description.yaml'])

    node = Node(
        package='rumblex_movement',
        name='node_movement',
        executable='node_movement',
        output='screen',
        parameters=[anatomy, servo_description, {'servo.offline': True}],
        remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))],
    )
    return LaunchDescription([
        DeclareLaunchArgument('joint_states_topic', default_value='joint_states'),
        DeclareLaunchArgument(
            'robot', default_value='nox',
            choices=['nox', 'nira'],
            description='Robot whose configuration to load from config/<robot>/'),
        node,
    ])
