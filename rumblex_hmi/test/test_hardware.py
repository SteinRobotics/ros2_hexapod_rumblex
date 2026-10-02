"""Verify hardware resource ordering without requiring a Raspberry Pi."""

import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from rumblex_hmi.hardware import HardwareBackend


@pytest.fixture
def drivers(monkeypatch):
    relay, bus, display, monitor = Mock(), Mock(), Mock(), Mock()
    monkeypatch.setitem(sys.modules, 'board', SimpleNamespace(D16=16, I2C=Mock(return_value=bus)))
    monkeypatch.setitem(sys.modules, 'digitalio', SimpleNamespace(DigitalInOut=Mock(return_value=relay)))
    monkeypatch.setitem(sys.modules, 'adafruit_ssd1306', SimpleNamespace(SSD1306_I2C=Mock(return_value=display)))
    monkeypatch.setitem(sys.modules, 'adafruit_ina228', SimpleNamespace(INA228=Mock(return_value=monitor)))
    monkeypatch.setattr('rumblex_hmi.hardware.time.sleep', lambda _: None)
    return relay, bus, display, monitor


def test_hardware_startup_and_cleanup(drivers):
    relay, bus, display, monitor = drivers
    monitor.bus_voltage, monitor.current = 11.9, 0.5
    backend = HardwareBackend()
    relay.switch_to_output.assert_called_once_with(value=False)
    display.fill.assert_called_once_with(0)
    assert relay.value is True
    assert backend.read_power() == (11.9, 0.5)
    backend.close()
    assert relay.value is False
    relay.deinit.assert_called_once()
    bus.deinit.assert_called_once()
    backend.close()  # Idempotent cleanup.
    bus.deinit.assert_called_once()


def test_power_monitor_failure_releases_gpio_and_bus(drivers, monkeypatch):
    relay, bus, _, _ = drivers
    sys.modules['adafruit_ina228'].INA228.side_effect = RuntimeError('I2C error')
    with pytest.raises(RuntimeError, match='I2C error'):
        HardwareBackend()
    assert relay.value is False
    relay.deinit.assert_called_once()
    bus.deinit.assert_called_once()
