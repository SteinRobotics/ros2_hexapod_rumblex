"""Host webcam capture for offline Nira development; no physical robot controls."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _camera(context):
    parameters = [LaunchConfiguration('camera_params_file')]
    parameters.append({'camera_info_url': LaunchConfiguration('camera_info_url')})
    device = LaunchConfiguration('camera_device').perform(context)
    if device:
        parameters.append({'video_device': device})
    return [Node(package='rumblex_camera', executable='desktop_camera.py',
                 name='node_desktop_camera', output='screen', parameters=parameters)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value=''),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('camera_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rumblex_camera'), 'config', 'nira', 'camera_desktop.yaml'])),
        OpaqueFunction(function=_camera),
    ])
