from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    config = PathJoinSubstitution([
        get_package_share_directory('rumblex_brain'), 'config', robot, 'parameter.yaml'
    ])

    node = Node(
        package='rumblex_brain',
        name='node_brain',
        executable='node_brain',
        output='screen',
        parameters=[config],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='nox',
            description='Robot configuration profile (for example: nox or nira)'),
        node,
    ])
