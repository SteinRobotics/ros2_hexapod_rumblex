"""Check robot pipeline selection without starting hardware or the vendor driver."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]


def evaluate(ctx, value):
    return perform_substitutions(ctx, normalize_to_list_of_substitutions(value))


def load(package, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / package / 'launch' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(robot, driver='true', sweep='true', camera='false'):
    result = LaunchContext()
    result.launch_configurations.update(robot=robot, enable_driver=driver, enable_head_scan=sweep,
                                        enable_camera=camera, camera_device='/dev/v4l/by-id/test-meet-se',
                                        camera_params_file='', camera_info_url='')
    return result


@pytest.mark.parametrize('robot,package,count', [
    ('nox', 'rumblex_lidar_1d', 2), ('nira', 'rumblex_lidar_2d', 1)])
def test_selected_pipeline(robot, package, count):
    module = load('rumblex_perception', 'sensors_launch')
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(ROOT / name)):
        ctx = context(robot)
        actions = module._launch_sensors(ctx)
        assert len(actions) == count
        assert all(isinstance(action, IncludeLaunchDescription) for action in actions)
        actions[0].launch_description_source.get_launch_description(ctx)
        assert package in actions[0].launch_description_source.location
        assert dict(actions[0].launch_arguments)['robot'] == robot
        assert actions[0].condition.evaluate(ctx)
        assert not actions[0].condition.evaluate(context(robot, driver='false'))
        if robot == 'nox':
            actions[1].launch_description_source.get_launch_description(ctx)
            assert 'rumblex_perception' in actions[1].launch_description_source.location
            assert actions[1].condition.evaluate(ctx)
            assert not actions[1].condition.evaluate(context(robot, sweep='false'))


@pytest.mark.parametrize('robot', ['unknown', '../nox', 'Nox'])
def test_invalid_profiles_fail(robot):
    module = load('rumblex_perception', 'sensors_launch')
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(ROOT / name)):
        with pytest.raises(RuntimeError):
            module._launch_sensors(context(robot))


@pytest.mark.parametrize('robot,enabled,expected', [
    ('nira', 'true', True), ('nira', 'false', False),
    ('nox', 'true', False), ('nox', 'false', False),
])
def test_camera_selection_is_optional(robot, enabled, expected):
    module = load('rumblex_perception', 'sensors_launch')
    ctx = context(robot, driver='false', camera=enabled)
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(ROOT / name)) as lookup:
        actions = module._launch_sensors(ctx)
    for action in actions:
        action.launch_description_source.get_launch_description(ctx)
    camera_actions = [action for action in actions if 'camera_launch' in
                      evaluate(ctx, action.launch_description_source.location)]
    assert len(camera_actions) == int(expected)
    assert ('rumblex_camera' in [call.args[0] for call in lookup.call_args_list]) == expected
    if expected:
        arguments = dict(camera_actions[0].launch_arguments)
        assert evaluate(ctx, arguments['video_device']) == '/dev/v4l/by-id/test-meet-se'
        assert evaluate(ctx, arguments['robot']) == 'nira'
        assert not actions[0].condition.evaluate(ctx)  # Camera can run with lidar disabled.


@pytest.mark.parametrize('package,launch_name', [
    ('rumblex_bringup', 'target_launch'), ('rumblex_bringup', 'sensors_launch'),
])
def test_camera_arguments_forwarded_through_bringup(package, launch_name):
    description = load(package, launch_name).generate_launch_description()
    defaults = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(defaults)
    assert defaults.launch_configurations['enable_camera'] == 'false'
    values = dict(enable_camera='true', camera_device='/dev/v4l/by-id/test-meet-se',
                  camera_params_file='/tmp/camera.yaml', camera_info_url='file:///tmp/calibration.yaml')
    defaults.launch_configurations.update(values)
    includes = [action for action in description.entities if isinstance(action, IncludeLaunchDescription)]
    forwarded = [dict(action.launch_arguments) for action in includes
                 if 'enable_camera' in dict(action.launch_arguments)]
    assert len(forwarded) == 1
    for name, value in values.items():
        assert evaluate(defaults, forwarded[0][name]) == value


def test_navigation_has_no_sensor_or_head_controller():
    module = load('rumblex_navigation', 'navigation_launch')
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(ROOT / name)):
        nodes = [action for action in module.generate_launch_description().entities if isinstance(action, Node)]
        assert all('head_scan' not in str(node.node_executable) for node in nodes)
        assert all('lidar' not in str(node.node_package) for node in nodes)
    for robot in ['nox', 'nira']:
        config = yaml.safe_load((ROOT / 'rumblex_navigation' / 'config' / robot / 'navigation.yaml').read_text())
        assert set(config) == {'node_navigation'}


def test_nira_uses_vendor_driver_and_urdf_frame():
    module = load('rumblex_lidar_2d', 'lidar_launch')
    nodes = [action for action in module.generate_launch_description().entities if isinstance(action, Node)]
    assert len(nodes) == 1  # No duplicate static transform or head controller.
    assert 'ydlidar_ros2_driver' in str(nodes[0].node_package)
    config = yaml.safe_load((ROOT / 'rumblex_lidar_2d/config/nira/lidar.yaml').read_text())
    assert config['ydlidar_ros2_driver_node']['ros__parameters']['frame_id'] == 'lidar_link'
