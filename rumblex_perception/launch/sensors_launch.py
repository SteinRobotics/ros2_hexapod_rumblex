"""Select sensor drivers and optional processing from the robot profile."""
from pathlib import Path
import re
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_sensors(context):
    robot = LaunchConfiguration('robot').perform(context)
    if not re.fullmatch(r'[a-z][a-z0-9_]*', robot):
        raise RuntimeError(f'Invalid robot profile: {robot!r}')
    profile = Path(get_package_share_directory('rumblex_perception')) / 'config' / robot / 'sensors.yaml'
    if not profile.is_file():
        raise RuntimeError(f'No sensor profile for robot {robot!r}: {profile}')
    config = yaml.safe_load(profile.read_text())
    driver = Path(get_package_share_directory(config['lidar_package'])) / 'launch' / 'lidar_launch.py'
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(driver)),
        launch_arguments={'robot': robot}.items(),
        condition=IfCondition(LaunchConfiguration('enable_driver')))]
    if config['head_scan']:
        perception = Path(get_package_share_directory('rumblex_perception')) / 'launch' / 'perception_launch.py'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(perception)),
            launch_arguments={'robot': robot}.items(),
            condition=IfCondition(LaunchConfiguration('enable_head_scan'))))
    # Resolve the optional camera package only when requested, so existing sensor
    # launches also work before the camera dependencies have been installed.
    if config.get('camera_package') and IfCondition(LaunchConfiguration('enable_camera')).evaluate(context):
        camera = Path(get_package_share_directory(config['camera_package'])) / 'launch' / 'camera_launch.py'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(camera)),
            launch_arguments={
                'robot': robot,
                'video_device': LaunchConfiguration('camera_device'),
                'params_file': LaunchConfiguration('camera_params_file'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='nox'),
        DeclareLaunchArgument('enable_driver', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='false',
                              description='Enable the camera when the robot profile provides one'),
        DeclareLaunchArgument('camera_device', default_value=''),
        DeclareLaunchArgument('camera_params_file', default_value=''),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('enable_head_scan', default_value='false',
                              description='Enable active head sweeping when the robot requires it'),
        OpaqueFunction(function=_launch_sensors),
    ])
