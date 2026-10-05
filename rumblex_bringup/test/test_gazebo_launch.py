"""Check composition and command routing without starting the simulator."""

import importlib.util
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import xacro

ROOT = Path(__file__).resolve().parents[2]


def load(package, filename):
    spec = importlib.util.spec_from_file_location(filename, ROOT / package / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_composes_test_stack_without_duplicate_display():
    module = load('rumblex_bringup', 'test_gazebo_launch.py')
    group = next(a for a in module.generate_launch_description().entities if isinstance(a, GroupAction))
    includes = [a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 2
    ctx = LaunchContext()
    ctx.launch_configurations.update(robot='nox', start_x='-2.26', start_y='-2.13', start_yaw='0.0')
    args = {key: perform_substitutions(ctx, normalize_to_list_of_substitutions(value))
            for key, value in includes[1].launch_arguments}
    assert args == {'robot': 'nox', 'use_sim_time': 'true', 'enable_display': 'false',
                    'movement_joint_states_topic': 'target_joint_states', 'publish_gazebo_map': 'true',
                    'start_x': '-2.26', 'start_y': '-2.13', 'start_yaw': '0.0'}
    spawn_args = {key: perform_substitutions(ctx, normalize_to_list_of_substitutions(value))
                  for key, value in includes[0].launch_arguments}
    assert spawn_args == {'robot': 'nox', 'spawn_x': args['start_x'],
                          'spawn_y': args['start_y'], 'spawn_yaw': args['start_yaw']}


def test_gazebo_test_launch_spawns_map_without_rviz():
    module = load('rumblex_bringup', 'test_gazebo_launch.py')
    with patch.object(module, 'Node', wraps=module.Node) as node:
        module.generate_launch_description()
    nodes = [call.kwargs for call in node.call_args_list]
    assert all(n['package'] != 'rviz2' for n in nodes)
    spawn = next(n for n in nodes if n['executable'] == 'create')
    assert spawn['arguments'][:2] == ['-topic', 'gazebo_map_description']


@pytest.mark.parametrize('robot', ['nox', 'nira'])
def test_bridge_joint_order_covers_mesh_model(robot):
    module = load('rumblex_bringup', 'test_gazebo_launch.py')
    ctx = LaunchContext()
    ctx.launch_configurations['robot'] = robot
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(ROOT / name)), \
            patch.object(module, 'Node') as node, \
            patch('xacro.substitution_args._eval_find', side_effect=lambda name: str(ROOT / name)):
        module._joint_bridge(ctx)
        names = node.call_args.kwargs['parameters'][0]['joint_names']
        model = ET.fromstring(xacro.process_file(
            str(ROOT / 'rumblex_description' / 'urdf' / f'{robot}_mesh.urdf.xacro'),
            mappings={'use_sim': 'true'}).toxml())
    assert len(names) == len(set(names))
    assert set(names) == {joint.get('name') for joint in model.findall('ros2_control/joint')}


def test_offline_movement_routes_commands_to_requested_topic():
    module = load('rumblex_movement', 'movement_offline_launch.py')
    with patch.object(module, 'Node') as node:
        module.generate_launch_description()
    source, target = node.call_args.kwargs['remappings'][0]
    ctx = LaunchContext()
    ctx.launch_configurations['joint_states_topic'] = 'target_joint_states'
    assert source == 'joint_states'
    assert target.perform(ctx) == 'target_joint_states'


@pytest.mark.parametrize('robot', ['nox', 'nira'])
def test_mesh_controller_receives_its_parameters(robot):
    module = load('rumblex_gazebo', 'simulation_mesh.launch.py')
    ctx = LaunchContext()
    ctx.launch_configurations.update(robot=robot, gui='false')
    # Keep real launch actions, but capture node construction without processes.
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(ROOT / name)), \
            patch('xacro.substitution_args._eval_find', side_effect=lambda name: str(ROOT / name)), \
            patch.object(module, 'Node', wraps=module.Node) as node:
        module._launch_robot(ctx)
    controller = next(call.kwargs for call in node.call_args_list
                      if call.kwargs.get('arguments', [None])[0] == 'forward_position_controller')
    assert controller['arguments'][1:] == [
        '--param-file', str(ROOT / 'rumblex_gazebo' / 'config' / robot / 'joint_controllers.yaml')]


def test_failed_controller_stops_launch():
    from types import SimpleNamespace
    from launch.actions import EmitEvent
    module = load('rumblex_gazebo', 'simulation_mesh.launch.py')
    callback = module._after_success(['next'], 'controller')
    assert callback(SimpleNamespace(returncode=0), None) == ['next']
    assert isinstance(callback(SimpleNamespace(returncode=1), None)[0], EmitEvent)


def test_bridge_holds_complete_command_for_late_controller():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from sensor_msgs.msg import JointState
    path = ROOT / 'rumblex_gazebo/scripts/joint_state_bridge.py'
    spec = importlib.util.spec_from_file_location('joint_bridge', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    bridge = SimpleNamespace(joint_names_=['a', 'b'], command=None, pub_=Mock(), get_logger=Mock())
    bridge.publish_command = lambda: module.JointStateBridge.publish_command(bridge)
    callback = lambda msg: module.JointStateBridge.on_joint_states(bridge, msg)
    callback(JointState(name=['b', 'a'], position=[0.4, 0.2]))
    assert list(bridge.command.data) == [0.2, 0.4]
    bridge.publish_command()  # Timer repeats the pose after a late controller activation.
    assert bridge.pub_.publish.call_count == 2
    for msg in (JointState(name=['a'], position=[0.1]),
                JointState(name=['a', 'b'], position=[float('nan'), 0.1]),
                JointState(name=['a', 'b'], position=[0.1])):
        callback(msg)
    assert bridge.pub_.publish.call_count == 2
    assert list(bridge.command.data) == [0.2, 0.4]
