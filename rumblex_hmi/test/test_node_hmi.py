"""Shared ROS behavior, exercised with the real message types and a recording backend."""

from unittest.mock import Mock

import pytest
import rclpy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Range
from rumblex_interfaces.msg import JoystickRequest, ServoStatus

from rumblex_hmi.node_hmi import NodeHmi


@pytest.fixture
def node(monkeypatch):
    # Keep middleware transport out of callback tests; integration is checked separately.
    monkeypatch.setattr(NodeHmi, 'create_subscription', Mock())
    monkeypatch.setattr(NodeHmi, 'create_publisher', Mock(return_value=Mock()))
    backend = Mock()
    backend.read_power.return_value = (12.0, -0.7)
    rclpy.init()
    node = NodeHmi(backend)
    node.is_ip_address_identified = True
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_callbacks_and_page_cycle(node):
    node.callback_movement_type(String(data='WALK'))
    node.callback_lidar(Range(range=1.25))
    node.callback_gravity(Vector3(z=9.81))
    node.callback_servo_status(ServoStatus(
        servo_max_temperature='LEFT_FRONT', min_voltage=10.7, max_temperature=42.5))
    assert node.text_movement_request == 'walk'
    assert node.text_lidar == 'd:  1.25m'
    assert node.text_bno055 == 'g: 9.81m/s^2'
    assert node.get_page_data('servos')[2:5] == ['left_front', 'T: 42.5°C', 'U: 10.7V']
    node.callback_joystick_request(JoystickRequest(button_select=False))
    assert node.active_page_name == 'system'
    for page in ('servos', 'sensors', 'system'):
        node.callback_joystick_request(JoystickRequest(button_select=True))
        assert node.active_page_name == page
        assert node.backend.present.call_args.args[0].size == (128, 64)


@pytest.mark.parametrize('xyz', [
    (0.0, 0.0, 9.81),
    (0.0, 0.0, -9.81),
    (9.81, 0.0, 0.0),
    (0.0, 5.886, 7.848),
])
def test_gravity_display_is_independent_of_tilt(node, xyz):
    node.callback_gravity(Vector3(x=xyz[0], y=xyz[1], z=xyz[2]))
    assert node.get_page_data('sensors')[2] == 'g: 9.81m/s^2'


def test_gravity_subscription_uses_driver_gravity_topic(node):
    node.create_subscription.assert_any_call(
        Vector3, 'bno055/grav', node.callback_gravity, 10)


def test_refresh_publishes_voltage_and_missing_reading_is_ignored(node):
    node.timer_callback()
    assert node.text_supply_voltage_and_current == 'supply V: 12.0V I: 0.7A'
    assert node.pub_supply_voltage.publish.call_args.args[0].data == 12.0
    node.pub_supply_voltage.publish.reset_mock()
    node.backend.read_power.return_value = (None, 0.0)
    node.timer_callback()
    node.pub_supply_voltage.publish.assert_not_called()


def test_refresh_shows_host_cpu_usage_and_unavailable_reading(node):
    node.cpu_usage.sample = Mock(side_effect=[23.4, 100.0, None])
    for expected in ('CPU: 23%', 'CPU: 100%', 'CPU: --%'):
        node.timer_callback()
        assert node.get_page_data('system')[4] == expected


def test_relay_and_shutdown_request(node):
    node.callback_servo_relay(Bool(data=True))
    node.backend.set_relay.assert_called_with(True)
    assert node.text_servo_relay_status == 'RELAY ON'
    node.callback_system_shutdown(Bool(data=False))
    node.backend.shutdown_host.assert_not_called()
    node.active_page_name = 'servos'
    node.callback_system_shutdown(Bool(data=True))
    node.backend.set_relay.assert_called_with(False)
    node.backend.shutdown_host.assert_called_once()
    assert node.text_servo_relay_status == 'RELAY OFF'
    assert node.active_page_name == 'system'
    assert node.text_movement_request == '!!SHUTDOWN!!'
