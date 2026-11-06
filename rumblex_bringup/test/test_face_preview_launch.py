"""Desktop composition is opt-in by robot and cannot leak into Gazebo or hardware."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

ROOT = Path(__file__).resolve().parents[2]


def module():
    spec = importlib.util.spec_from_file_location('test_launch', ROOT / 'rumblex_bringup/launch/test_launch.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context(robot='nira', camera='true', tracking='true', sim='false'):
    ctx = LaunchContext()
    ctx.launch_configurations.update(robot=robot, enable_camera=camera, enable_face_tracking=tracking,
                                    use_sim_time=sim, enable_face_tracking_view='false',
                                    camera_device='', camera_params_file='/tmp/desktop.yaml')
    return ctx


@pytest.mark.parametrize('robot,expected', [('nira', 'True'), ('nox', 'False')])
def test_defaults_follow_robot(robot, expected):
    launch = module().generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations['robot'] = robot
    # Camera config path is resolved only when launching camera; no installed package needed here.
    for action in launch.entities:
        if isinstance(action, DeclareLaunchArgument) and action.name in ('enable_camera', 'enable_face_tracking'):
            action.execute(ctx)
    assert ctx.launch_configurations['enable_camera'] == expected
    assert ctx.launch_configurations['enable_face_tracking'] == expected


def test_one_capture_and_one_detector_and_optional_view():
    actions = module()._face_preview(context())
    assert len(actions) == 3
    assert sum(isinstance(a, IncludeLaunchDescription) for a in actions) == 2
    assert dict(actions[1].launch_arguments)['desktop_preview'] == 'true'
    assert not actions[2].condition.evaluate(context())


@pytest.mark.parametrize('ctx', [context(robot='nox'), context(camera='false'), context(sim='true')])
def test_incompatible_preview_rejected(ctx):
    with pytest.raises(RuntimeError):
        module()._face_preview(ctx)


def test_disabled_does_not_resolve_camera_or_face_packages():
    assert module()._face_preview(context(robot='nox', camera='false', tracking='false')) == []
