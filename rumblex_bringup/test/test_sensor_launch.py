"""Check robot pipeline selection without starting hardware or the vendor driver."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]


def load(package, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / package / 'launch' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(robot, driver='true', sweep='true'):
    result = LaunchContext()
    result.launch_configurations.update(robot=robot, enable_driver=driver, enable_head_scan=sweep)
    return result


@pytest.mark.parametrize('robot,package,count', [
    ('nox', 'rumblex_lidar_1d', 2), ('nira', 'rumblex_lidar_2d', 1)])
def test_selected_pipeline(robot, package, count):
    module = load('rumblex_bringup', 'sensors_launch')
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
    module = load('rumblex_bringup', 'sensors_launch')
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(ROOT / name)):
        with pytest.raises(RuntimeError):
            module._launch_sensors(context(robot))


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
