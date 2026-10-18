"""Check setup argument guards and service/profile wiring without changing the host."""
import os
from pathlib import Path
import subprocess

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SETUP = ROOT / 'scripts/setup_raspberry_pi.sh'
LAUNCHER = ROOT / 'rumblex_bringup/autostart/start_ros2.sh'


@pytest.mark.parametrize('arguments', [[], ['robot_nira'], ['Nira'], ['nira', 'nox']])
def test_invalid_setup_arguments_do_not_invoke_sudo(tmp_path, arguments):
    marker = tmp_path / 'sudo-called'
    sudo = tmp_path / 'sudo'
    sudo.write_text(f'#!/bin/sh\ntouch "{marker}"\nexit 99\n')
    sudo.chmod(0o755)
    result = subprocess.run(
        ['bash', str(SETUP), *arguments], capture_output=True, text=True,
        env={**os.environ, 'PATH': f'{tmp_path}:{os.environ["PATH"]}'}, check=False,
    )
    assert result.returncode == 2
    assert 'Usage:' in result.stderr
    assert not marker.exists()


def test_help():
    result = subprocess.run(['bash', str(SETUP), '--help'], capture_output=True, text=True, check=False)
    assert result.returncode == 0
    assert '<nox|nira>' in result.stdout


@pytest.mark.parametrize('robot', ['', 'robot_nira', '../nox'])
def test_launcher_rejects_missing_or_invalid_profile(robot):
    result = subprocess.run(
        ['bash', str(LAUNCHER)], capture_output=True, text=True,
        env={**os.environ, 'RUMBLEX_ROBOT': robot}, check=False,
    )
    assert result.returncode != 0
    assert 'RUMBLEX_ROBOT' in result.stderr or 'Invalid robot' in result.stderr


@pytest.mark.parametrize('robot', ['nox', 'nira'])
def test_target_imu_configuration_exists(robot):
    config = yaml.safe_load((ROOT / 'rumblex_bringup/config' / robot / 'bno055_params.yaml').read_text())
    parameters = config['bno055']['ros__parameters']
    assert parameters['connection_type'] == 'i2c'
    assert parameters['frame_id'] == 'imu_link'
