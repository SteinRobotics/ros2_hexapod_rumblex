"""Check that bounded OLED transfers preserve every framebuffer byte."""

import sys
from types import SimpleNamespace

import pytest

from rumblex_hmi.display import create_display


@pytest.mark.parametrize('size', [1024, 512, 35])
def test_framebuffer_transfers(monkeypatch, size):
    class Device:
        def __init__(self):
            self.writes = []
            self.locked = False

        def __enter__(self):
            self.locked = True

        def __exit__(self, *args):
            self.locked = False

        def write(self, data):
            assert self.locked
            assert data[0] == 0x40
            assert 1 < len(data) <= 33
            self.writes.append(bytes(data))

    pixels = bytes(index % 256 for index in range(size))

    class Driver:
        def __init__(self, width, height, bus):
            self.i2c_device = Device()
            self.buffer = bytearray(b'\x40' + pixels)
            # The real driver writes the framebuffer during construction too.
            self.write_framebuf()

    monkeypatch.setitem(sys.modules, 'adafruit_ssd1306', SimpleNamespace(SSD1306_I2C=Driver))
    display = create_display(128, 64, object())
    assert b''.join(data[1:] for data in display.i2c_device.writes) == pixels
    display.i2c_device.writes.clear()
    display.buffer[1:] = bytes(size)
    display.write_framebuf()
    assert b''.join(data[1:] for data in display.i2c_device.writes) == bytes(size)
    assert not display.i2c_device.locked
