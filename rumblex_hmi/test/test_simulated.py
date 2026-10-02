import os

os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import pytest

from rumblex_hmi.oled import OledRenderer
from rumblex_hmi.simulated import SimulatedBackend


@pytest.fixture
def backend():
    backend = SimulatedBackend()
    yield backend
    backend.close()


def test_frame_is_scaled_without_interpolation(backend):
    frame = OledRenderer().render(['SYSTEM', 'IP: 127.0.0.1'])
    backend.present(frame)
    assert backend.screen.get_size() == (512, 256)
    for y in range(64):
        for x in range(128):
            expected = 255 if frame.getpixel((x, y)) else 0
            for dx, dy in ((0, 0), (3, 3)):
                assert backend.screen.get_at((x * 4 + dx, y * 4 + dy))[:3] == (expected,) * 3


def test_power_relay_shutdown_and_close(backend, monkeypatch):
    assert backend.read_power() == (12.0, 0.0)
    backend.power_values = lambda: (11.8, -0.6)
    assert backend.read_power() == (11.8, -0.6)
    backend.set_relay(False)
    assert not backend.relay_enabled
    # Even an explicit shutdown request cannot reach an OS command through this backend.
    import subprocess
    monkeypatch.setattr(subprocess, 'call', lambda *a, **kw: pytest.fail('host shutdown invoked'))
    backend.shutdown_host(type('Logger', (), {'info': lambda self, message: None})())
    backend.set_relay(True)
    pygame.event.post(pygame.event.Event(pygame.QUIT))
    assert not backend.process_events()
    backend.close()
    assert not backend.relay_enabled
    assert not pygame.display.get_init()
