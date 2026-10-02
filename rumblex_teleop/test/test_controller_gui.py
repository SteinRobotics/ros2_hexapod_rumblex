"""Exercise GUI input with SDL's dummy driver; source ROS before running."""

import os
import sys
from pathlib import Path

import pytest

os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
pygame = pytest.importorskip('pygame')
pytest.importorskip('rclpy')
pytest.importorskip('rumblex_interfaces.msg')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rumblex_teleop.node_teleop_simulated import ControllerGui, KEY_CONTROLS
from rumblex_teleop.simulated_controller import ControllerState


@pytest.fixture
def gui():
    controller = ControllerGui(ControllerState())
    yield controller
    pygame.quit()


@pytest.mark.parametrize('symbol', ['cross', 'circle', 'square', 'triangle'])
def test_mouse_face_buttons(gui, symbol):
    position = gui.FACE_CENTERS[symbol]
    gui.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=position), 0.0)
    # Releasing outside the button still releases the captured control.
    gui.handle_event(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=(0, 0)), 0.01)
    payload = gui.state.sample(0.1)
    button = {'cross': 'a', 'circle': 'b', 'square': 'x', 'triangle': 'y'}[symbol]
    assert payload[f'button_{button}']


def test_mouse_drag_and_focus_loss(gui):
    gui.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(380, 365)), 0.0)
    gui.handle_event(pygame.event.Event(pygame.MOUSEMOTION, pos=(380, 300)), 0.1)
    assert gui.state.axes()['left_stick_vertical'] == 1.0
    gui.handle_event(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=(0, 0)), 0.2)
    assert gui.state.axes()['left_stick_vertical'] == 0.0
    gui.handle_event(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_z), 0.3)
    assert gui.handle_event(pygame.event.Event(pygame.WINDOWFOCUSLOST), 0.4) == 'reset'
    assert all(value == 0 for value in gui.state.sample(0.5).values())
    assert gui.dragging is None
    assert gui.mouse_control is None


def test_keyboard_shortcuts_and_close(gui):
    for key in (pygame.K_w, pygame.K_j, pygame.K_UP, pygame.K_q):
        gui.handle_event(pygame.event.Event(pygame.KEYDOWN, key=key), 0.0)
    payload = gui.state.sample(2.0)
    assert payload['left_stick_vertical'] == 1.0
    assert payload['right_stick_horizontal'] == -1.0
    assert payload['dpad_vertical'] == 1
    assert payload['button_long_l1']
    assert len(set(KEY_CONTROLS.values())) == 23
    gui.draw(2.0)
    assert gui.handle_event(pygame.event.Event(pygame.QUIT), 2.1) == 'quit'
    assert all(value == 0 for value in gui.state.sample(2.2).values())
