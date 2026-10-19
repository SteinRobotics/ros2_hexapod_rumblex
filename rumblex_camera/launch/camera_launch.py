"""Launch a robot's USB camera without selecting an arbitrary video device."""
from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_camera(context):
    robot = LaunchConfiguration('robot').perform(context)
    if not re.fullmatch(r'[a-z][a-z0-9_]*', robot):
        raise RuntimeError(f'Invalid robot profile: {robot!r}')
    profile = Path(get_package_share_directory('rumblex_camera')) / 'config' / robot / 'camera.yaml'
    if not profile.is_file():
        raise RuntimeError(f'No camera profile for robot {robot!r}: {profile}')
    params_file = LaunchConfiguration('params_file').perform(context) or str(profile)
    try:
        config = yaml.safe_load(Path(params_file).read_text())
        parameters = config['/camera/node_camera']['ros__parameters']
        if not isinstance(parameters, dict):
            raise TypeError('ros__parameters must be a mapping')
    except (OSError, yaml.YAMLError, KeyError, TypeError) as error:
        raise RuntimeError(
            f'Invalid camera parameters in {params_file}; expected /camera/node_camera/ros__parameters'
        ) from error

    device = LaunchConfiguration('video_device').perform(context) or parameters.get('video_device', '')
    if not isinstance(device, str) or not device.strip() or not Path(device).is_absolute():
        raise RuntimeError('Set video_device to an explicit absolute camera device path '
                           '(prefer /dev/v4l/by-id/...) in the launch arguments or camera YAML')
    # usb_cam 0.8.1 mishandles relative by-id symlink targets (../../videoN).
    # Resolve the user's selected path before passing it to the driver. Missing
    # devices still reach the driver and fail only that process, not bringup.
    overrides = {'video_device': str(Path(device).resolve())}
    calibration = LaunchConfiguration('camera_info_url').perform(context)
    if calibration:
        overrides['camera_info_url'] = calibration

    # Keep the driver in its own process. Its exit must not shut down robot bringup.
    # Device existence and supported formats/controls are checked by usb_cam.
    return [Node(
        package='usb_cam', executable='usb_cam_node_exe', name='node_camera', namespace='camera',
        output='screen', parameters=[params_file, overrides],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='nira'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Camera YAML override; defaults to config/<robot>/camera.yaml'),
        DeclareLaunchArgument('video_device', default_value='',
                              description='Explicit camera device path; overrides camera YAML'),
        DeclareLaunchArgument('camera_info_url', default_value='',
                              description='Calibration URL override; empty retains camera YAML value'),
        OpaqueFunction(function=_launch_camera),
    ])
