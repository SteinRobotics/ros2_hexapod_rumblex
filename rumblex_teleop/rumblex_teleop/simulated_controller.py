"""Input state for the simulated controller, independent of ROS and Pygame."""

import math


BUTTONS = ('a', 'b', 'x', 'y', 'l1', 'l2', 'r1', 'r2', 'select', 'start', 'home')
FACE_BUTTONS = {'cross': 'a', 'circle': 'b', 'square': 'x', 'triangle': 'y'}


class ControllerState:
    LONG_PRESS_SECONDS = 2.0
    DEADZONE = 0.004

    def __init__(self):
        self.reset()

    def reset(self):
        self.sources = {}
        self.press_times = {}
        self.short_presses = set()
        self.long_releases = set()
        self.dragged_sticks = {}

    def set_control(self, source, control, pressed, now):
        was_pressed = control in self.sources.values()
        if pressed:
            self.sources[source] = control
        else:
            self.sources.pop(source, None)
        is_pressed = control in self.sources.values()
        if control not in BUTTONS:
            return
        if is_pressed and not was_pressed:
            self.press_times[control] = now
        elif was_pressed and not is_pressed:
            started = self.press_times.pop(control)
            if now - started >= self.LONG_PRESS_SECONDS:
                self.long_releases.add(control)
            else:
                self.short_presses.add(control)

    def drag_stick(self, stick, horizontal, vertical):
        length = math.hypot(horizontal, vertical)
        scale = max(1.0, length)
        self.dragged_sticks[stick] = (horizontal / scale, vertical / scale)

    def release_stick(self, stick):
        self.dragged_sticks.pop(stick, None)

    def axes(self):
        pressed = set(self.sources.values())
        result = {}
        result['dpad_horizontal'] = int('dpad_right' in pressed) - int('dpad_left' in pressed)
        result['dpad_vertical'] = int('dpad_up' in pressed) - int('dpad_down' in pressed)
        for stick in ('left', 'right'):
            horizontal = float(f'{stick}_right' in pressed) - float(f'{stick}_left' in pressed)
            vertical = float(f'{stick}_up' in pressed) - float(f'{stick}_down' in pressed)
            horizontal, vertical = self.dragged_sticks.get(stick, (horizontal, vertical))
            result[f'{stick}_stick_horizontal'] = horizontal if abs(horizontal) >= self.DEADZONE else 0.0
            result[f'{stick}_stick_vertical'] = vertical if abs(vertical) >= self.DEADZONE else 0.0
        return result

    def sample(self, now):
        result = self.axes()
        for button in BUTTONS:
            result[f'button_{button}'] = button in self.short_presses
            result[f'button_long_{button}'] = (
                button in self.long_releases or
                (button in self.press_times and now - self.press_times[button] >= self.LONG_PRESS_SECONDS)
            )
        self.short_presses.clear()
        self.long_releases.clear()
        return result


class PayloadTracker:
    """Compare message fields without including the changing timestamp."""

    def __init__(self):
        self.last_payload = None

    def changed(self, payload):
        if payload == self.last_payload:
            return False
        self.last_payload = payload.copy()
        return True
