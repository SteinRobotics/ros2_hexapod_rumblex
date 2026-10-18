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


@pytest.fixture
def driver_repositories(tmp_path):
    """Redirect the production SSH URL to a local repository for offline tests."""
    upstream = tmp_path / 'upstream'
    checkout = tmp_path / 'driver'
    env = {**os.environ, 'GIT_CONFIG_GLOBAL': str(tmp_path / 'gitconfig'),
           'GIT_CONFIG_NOSYSTEM': '1'}

    def git(*args, cwd=None):
        return subprocess.run(['git', *args], cwd=cwd, env=env,
                              check=True, capture_output=True, text=True).stdout.strip()

    git('config', '--global', 'user.name', 'Setup Test')
    git('config', '--global', 'user.email', 'setup@example.invalid')
    git('config', '--global', f'url.{upstream}.insteadOf',
        'git@github.com:SteinRobotics/ydlidar_ros2_driver.git')
    git('init', '--initial-branch=master', str(upstream))
    (upstream / 'driver.txt').write_text('master\n')
    git('add', '.', cwd=upstream)
    git('commit', '-m', 'baseline', cwd=upstream)
    git('switch', '-c', 'lyrical', cwd=upstream)
    (upstream / 'driver.txt').write_text('lyrical\n')
    git('commit', '-am', 'lyrical support', cwd=upstream)

    def update():
        return subprocess.run(
            ['bash', str(ROOT / 'scripts/update_ydlidar_driver.sh'), str(checkout)],
            env=env, capture_output=True, text=True, check=False)

    return upstream, checkout, git, update


def test_driver_clone_and_fast_forward(driver_repositories):
    upstream, checkout, git, update = driver_repositories
    assert update().returncode == 0
    assert git('branch', '--show-current', cwd=checkout) == 'lyrical'
    (upstream / 'driver.txt').write_text('new lyrical\n')
    git('commit', '-am', 'fix', cwd=upstream)
    assert update().returncode == 0
    assert git('rev-parse', 'HEAD', cwd=checkout) == git('rev-parse', 'HEAD', cwd=upstream)


def test_driver_migrates_clean_master(driver_repositories):
    upstream, checkout, git, update = driver_repositories
    git('clone', '--branch', 'master', str(upstream), str(checkout))
    previous = git('rev-parse', 'master', cwd=checkout)
    assert update().returncode == 0
    assert git('branch', '--show-current', cwd=checkout) == 'lyrical'
    assert git('rev-parse', 'master', cwd=checkout) == previous
    assert git('remote', 'get-url', 'origin', cwd=checkout) == str(upstream)
    assert git('config', '--get', 'remote.origin.url', cwd=checkout).startswith('git@github.com:SteinRobotics/')


def test_driver_preserves_local_patch(driver_repositories):
    upstream, checkout, git, update = driver_repositories
    git('clone', '--branch', 'master', str(upstream), str(checkout))
    (checkout / 'driver.txt').write_text('local patch\n')
    before = git('rev-parse', 'HEAD', cwd=checkout)
    assert update().returncode != 0
    assert git('rev-parse', 'HEAD', cwd=checkout) == before
    assert (checkout / 'driver.txt').read_text() == 'local patch\n'


def test_driver_preserves_unpublished_commits(driver_repositories):
    _, checkout, git, update = driver_repositories
    assert update().returncode == 0
    (checkout / 'driver.txt').write_text('local change\n')
    git('commit', '-am', 'unpublished', cwd=checkout)
    before = git('rev-parse', 'HEAD', cwd=checkout)
    assert update().returncode != 0
    assert git('rev-parse', 'HEAD', cwd=checkout) == before
