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
    profile = Path(get_package_share_directory('rumblex_bringup')) / 'config' / robot / 'sensors.yaml'
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
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='nox'),
        DeclareLaunchArgument('enable_driver', default_value='true'),
        DeclareLaunchArgument('enable_head_scan', default_value='false',
                              description='Enable active head sweeping when the robot requires it'),
        OpaqueFunction(function=_launch_sensors),
    ])
