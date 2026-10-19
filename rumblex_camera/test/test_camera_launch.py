"""Validate camera selection and parameter forwarding without opening hardware."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def camera_launch():
    spec = importlib.util.spec_from_file_location('camera_launch', PACKAGE / 'launch/camera_launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, 'get_package_share_directory', return_value=str(PACKAGE)):
        yield module


def context(module, **overrides):
    ctx = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    ctx.launch_configurations.update(overrides)
    return ctx


def test_driver_and_default_profile(camera_launch):
    ctx = context(camera_launch, video_device='/dev/v4l/by-id/test-meet-se')
    with patch.object(camera_launch, 'Node', wraps=Node) as factory:
        nodes = camera_launch._launch_camera(ctx)
    assert len(nodes) == 1
    kwargs = factory.call_args.kwargs
    assert (kwargs['package'], kwargs['executable']) == ('usb_cam', 'usb_cam_node_exe')
    assert (kwargs['namespace'], kwargs['name']) == ('camera', 'node_camera')
    config_file, overrides = kwargs['parameters']
    assert overrides == {'video_device': '/dev/v4l/by-id/test-meet-se'}
    parameters = yaml.safe_load(Path(config_file).read_text())['/camera/node_camera']['ros__parameters']
    assert (parameters['image_width'], parameters['image_height'], parameters['framerate']) == (1280, 720, 30.0)
    assert parameters['pixel_format'] == 'mjpeg2rgb'
    assert parameters['frame_id'] == 'camera_optical_frame'
    assert parameters['camera_info_url'] == ''
    # A driver exit is not wired to shutdown of the other robot processes.
    assert 'on_exit' not in kwargs


@pytest.mark.parametrize('robot', ['nox', 'unknown', '../nira', 'Nira'])
def test_unsupported_profiles(camera_launch, robot):
    with pytest.raises(RuntimeError, match='robot profile|camera profile'):
        camera_launch._launch_camera(context(camera_launch, robot=robot, video_device='/dev/video0'))


@pytest.mark.parametrize('device', ['', '   ', 'video0'])
def test_requires_explicit_absolute_device(camera_launch, device):
    with pytest.raises(RuntimeError, match='explicit absolute'):
        camera_launch._launch_camera(context(camera_launch, video_device=device))


def test_yaml_device_and_calibration_overrides(camera_launch, tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/nira/camera.yaml').read_text())
    parameters = config['/camera/node_camera']['ros__parameters']
    parameters.update(video_device='/dev/v4l/by-id/configured', camera_info_url='file:///tmp/measured.yaml')
    params_file = tmp_path / 'camera.yaml'
    params_file.write_text(yaml.safe_dump(config))
    ctx = context(camera_launch, params_file=str(params_file))
    with patch.object(camera_launch, 'Node', wraps=Node) as factory:
        camera_launch._launch_camera(ctx)
    assert factory.call_args.kwargs['parameters'] == [
        str(params_file), {'video_device': '/dev/v4l/by-id/configured'}]

    ctx.launch_configurations.update(video_device='/dev/v4l/by-id/override',
                                     camera_info_url='file:///tmp/override.yaml')
    with patch.object(camera_launch, 'Node', wraps=Node) as factory:
        camera_launch._launch_camera(ctx)
    assert factory.call_args.kwargs['parameters'][1] == {
        'video_device': '/dev/v4l/by-id/override', 'camera_info_url': 'file:///tmp/override.yaml'}


@pytest.mark.parametrize('contents', ['', '[', 'node_camera: {}',
                                     '/camera/node_camera: {ros__parameters: null}'])
def test_bad_parameter_file_is_actionable(camera_launch, tmp_path, contents):
    params_file = tmp_path / 'camera.yaml'
    params_file.write_text(contents)
    with pytest.raises(RuntimeError, match='Invalid camera parameters'):
        camera_launch._launch_camera(context(camera_launch, params_file=str(params_file)))


def test_missing_parameter_file(camera_launch, tmp_path):
    with pytest.raises(RuntimeError, match='Invalid camera parameters'):
        camera_launch._launch_camera(context(camera_launch, params_file=str(tmp_path / 'missing.yaml')))


def test_relative_by_id_symlink_is_resolved(camera_launch, tmp_path):
    device = tmp_path / 'video3'
    device.touch()
    by_id = tmp_path / 'v4l/by-id'
    by_id.mkdir(parents=True)
    selected = by_id / 'meet-se'
    selected.symlink_to('../../video3')
    with patch.object(camera_launch, 'Node', wraps=Node) as factory:
        camera_launch._launch_camera(context(camera_launch, video_device=str(selected)))
    assert factory.call_args.kwargs['parameters'][1]['video_device'] == str(device)
