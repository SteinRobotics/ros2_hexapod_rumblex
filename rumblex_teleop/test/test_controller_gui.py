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
from rumblex_teleop.sequence_catalog import SequenceCatalog


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
    gui.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=gui.STICKS['left']), 0.0)
    gui.handle_event(pygame.event.Event(pygame.MOUSEMOTION, pos=(gui.STICKS['left'][0], gui.STICKS['left'][1] - 65)), 0.1)
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


def test_named_sequence_click_once_and_cancellation(gui):
    calls = []
    gui.catalog = SequenceCatalog({}, ('clap', 'neutral'))
    gui.on_sequence = calls.append
    position = next(gui.sequence_buttons())[1].center
    down = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=position)
    up = pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=position)
    gui.handle_event(down, 0.0)
    assert calls == []
    gui.handle_event(up, 5.0)
    gui.handle_event(up, 5.1)
    assert calls == ['clap']
    assert all(value == 0 for value in gui.state.sample(5.2).values())
    gui.handle_event(down, 6.0)
    gui.handle_event(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=(0, 0)), 6.1)
    gui.handle_event(down, 7.0)
    gui.handle_event(pygame.event.Event(pygame.WINDOWFOCUSLOST), 7.1)
    gui.handle_event(up, 7.2)
    assert calls == ['clap']


def test_sequence_scrolling_and_rendering(gui, monkeypatch):
    gui.catalog = SequenceCatalog({'r2': 'dance_hot_dogs_for_breakfast'},
                                  tuple(f'sequence_{i}' for i in range(20)))
    monkeypatch.setattr(pygame.mouse, 'get_pos', lambda: gui.SEQUENCE_PANEL.center)
    gui.handle_event(pygame.event.Event(pygame.MOUSEWHEEL, y=-100), 0.0)
    assert gui.sequence_scroll == 20 * gui.SEQUENCE_ROW_HEIGHT - gui.SEQUENCE_PANEL.height
    assert gui.hit_sequence((gui.SEQUENCE_PANEL.centerx, gui.SEQUENCE_PANEL.bottom - 20)) == 'sequence_19'
    assert gui.hit_sequence((gui.SEQUENCE_PANEL.centerx, gui.SEQUENCE_PANEL.bottom + 1)) is None
    gui.draw(0.0)
    gui.handle_event(pygame.event.Event(pygame.MOUSEWHEEL, y=100), 0.1)
    assert gui.sequence_scroll == 0
    gui.catalog_error = 'Catalog missing'
    gui.catalog = SequenceCatalog({}, ())
    gui.draw(0.2)


def test_sequence_publisher_uses_exact_name():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rumblex_teleop.node_teleop_simulated import NodeTeleopSimulated

    publisher = Mock()
    NodeTeleopSimulated.publish_sequence(SimpleNamespace(sequence_pub=publisher), 'dance_standard')
    publisher.publish.assert_called_once()
    message = publisher.publish.call_args.args[0]
    assert message.data == 'dance_standard'
