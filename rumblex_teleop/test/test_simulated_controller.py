import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rumblex_teleop.simulated_controller import ControllerState, FACE_BUTTONS, PayloadTracker


@pytest.mark.parametrize('symbol,button', [('cross', 'a'), ('circle', 'b'), ('square', 'x'), ('triangle', 'y')])
def test_face_mapping_and_quick_click(symbol, button):
    state = ControllerState()
    state.set_control('mouse', FACE_BUTTONS[symbol], True, 0.0)
    state.set_control('mouse', FACE_BUTTONS[symbol], False, 0.01)
    assert state.sample(0.1)[f'button_{button}']
    assert not state.sample(0.2)[f'button_{button}']


def test_long_press_threshold_and_release():
    state = ControllerState()
    state.set_control('key', 'select', True, 10.0)
    assert not state.sample(11.999)['button_long_select']
    assert state.sample(12.0)['button_long_select']
    state.set_control('key', 'select', False, 12.1)
    payload = state.sample(12.2)
    assert payload['button_long_select']
    assert not payload['button_select']
    assert not state.sample(12.3)['button_long_select']


def test_button_held_by_both_sources_and_key_repeat():
    state = ControllerState()
    state.set_control('mouse', 'a', True, 0.0)
    state.set_control('key', 'a', True, 0.5)
    state.set_control('key', 'a', True, 1.5)
    state.set_control('mouse', 'a', False, 1.6)
    assert not state.sample(1.7)['button_a']
    assert state.sample(2.0)['button_long_a']


def test_simultaneous_controls_opposing_directions_and_signs():
    state = ControllerState()
    for control in ('left_up', 'left_right', 'right_down', 'right_left', 'dpad_up', 'dpad_left', 'l1'):
        state.set_control(control, control, True, 0.0)
    payload = state.sample(0.1)
    assert payload['left_stick_vertical'] == 1.0
    assert payload['left_stick_horizontal'] == 1.0
    assert payload['right_stick_vertical'] == -1.0
    assert payload['right_stick_horizontal'] == -1.0
    assert payload['dpad_vertical'] == 1
    assert payload['dpad_horizontal'] == -1
    state.set_control('opposite', 'left_down', True, 0.2)
    state.set_control('opposite_dpad', 'dpad_down', True, 0.2)
    assert state.sample(0.3)['left_stick_vertical'] == 0.0
    assert state.sample(0.3)['dpad_vertical'] == 0


def test_drag_bounds_deadzone_precedence_and_release():
    state = ControllerState()
    state.drag_stick('left', 3.0, 4.0)
    assert state.axes()['left_stick_horizontal'] == pytest.approx(0.6)
    assert state.axes()['left_stick_vertical'] == pytest.approx(0.8)
    state.set_control('key', 'left_right', True, 0.0)
    state.drag_stick('left', -0.5, 0.003)
    assert state.axes()['left_stick_horizontal'] == -0.5
    assert state.axes()['left_stick_vertical'] == 0.0
    state.release_stick('left')
    assert state.axes()['left_stick_horizontal'] == 1.0
    state.set_control('key', 'left_right', False, 0.1)
    assert state.axes()['left_stick_horizontal'] == 0.0


def test_reset_cancels_button_actions_and_neutralizes_all_axes():
    state = ControllerState()
    state.set_control('key', 'a', True, 0.0)
    state.set_control('dpad', 'dpad_right', True, 0.0)
    state.drag_stick('right', 0.2, 0.4)
    state.reset()
    assert all(value == 0 for value in state.sample(5.0).values())


def test_payload_changes_include_pulse_clear_and_neutral_release():
    state = ControllerState()
    tracker = PayloadTracker()
    assert tracker.changed(state.sample(0.0))
    assert not tracker.changed(state.sample(0.1))
    state.set_control('key', 'a', True, 0.2)
    assert not tracker.changed(state.sample(0.3))
    state.set_control('key', 'a', False, 0.4)
    assert tracker.changed(state.sample(0.5))
    assert tracker.changed(state.sample(0.6))
    assert not tracker.changed(state.sample(0.7))
    state.drag_stick('left', 0.5, 0.5)
    assert tracker.changed(state.sample(0.8))
    state.release_stick('left')
    assert tracker.changed(state.sample(0.9))
