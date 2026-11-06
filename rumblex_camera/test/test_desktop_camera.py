"""Device discovery selects capture devices and preserves explicit device overrides."""
import importlib.util
from pathlib import Path
from unittest.mock import mock_open, patch
import struct

ROOT = Path(__file__).resolve().parents[1]


def load(relative):
    spec = importlib.util.spec_from_file_location('desktop_camera_test_module', ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_discovery_excludes_metadata_and_deduplicates_links():
    module = load('scripts/desktop_camera.py')
    paths = [['/dev/v4l/by-id/capture', '/dev/v4l/by-id/metadata'], [], ['/dev/video0', '/dev/video1']]
    mapping = {'/dev/v4l/by-id/capture': '/dev/video0', '/dev/v4l/by-id/metadata': '/dev/video1'}
    opened = []
    def opener(path, *args, **kwargs):
        opened.append(path)
        return mock_open()(path, *args, **kwargs)
    def ioctl(_file, _request, data):
        struct.pack_into('II', data, 84, 0x80000000, 1 if opened[-1] == '/dev/video0' else 0x00800000)
    with patch.object(module.glob, 'glob', side_effect=paths), \
            patch.object(module.Path, 'resolve', lambda self: Path(mapping.get(str(self), str(self)))), \
            patch('builtins.open', side_effect=opener), patch.object(module.fcntl, 'ioctl', side_effect=ioctl):
        assert module.capture_devices() == ['/dev/v4l/by-id/capture']
    assert opened == ['/dev/video0', '/dev/video1']


def test_inaccessible_devices_do_not_become_capture_candidates():
    module = load('scripts/desktop_camera.py')
    with patch.object(module.glob, 'glob', side_effect=[[], [], ['/dev/video0']]), \
            patch('builtins.open', side_effect=PermissionError):
        assert module.capture_devices() == []


def test_launch_device_override_does_not_replace_saved_selection_when_empty():
    from launch import LaunchContext
    module = load('launch/desktop_camera_launch.py')
    ctx = LaunchContext()
    ctx.launch_configurations.update(camera_device='', camera_params_file='/tmp/camera.yaml', camera_info_url='')
    with patch.object(module, 'Node') as node:
        module._camera(ctx)
        parameters = node.call_args.kwargs['parameters']
        assert not any(isinstance(p, dict) and 'video_device' in p for p in parameters)
        ctx.launch_configurations['camera_device'] = '/dev/video9'
        module._camera(ctx)
        assert node.call_args.kwargs['parameters'][-1] == {'video_device': '/dev/video9'}
