"""Compatibility entry point for robot-specific sensor selection."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = {
        'robot': 'nox', 'enable_driver': 'true', 'enable_head_scan': 'false',
        'enable_camera': 'false', 'camera_device': '', 'camera_params_file': '', 'camera_info_url': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rumblex_perception'), 'launch', 'sensors_launch.py'])),
            launch_arguments={name: LaunchConfiguration(name) for name in arguments}.items(),
        ),
    ])
