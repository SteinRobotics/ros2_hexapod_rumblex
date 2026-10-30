"""Mouse and keyboard PS3-style controller publishing JoystickRequest."""

import os
import time

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import rclpy
import yaml
from std_msgs.msg import String
from rclpy.node import Node
from rumblex_interfaces.msg import JoystickRequest

from rumblex_teleop.sequence_catalog import SequenceCatalog, installed_catalog
from rumblex_teleop.simulated_controller import ControllerState, FACE_BUTTONS, PayloadTracker


KEY_CONTROLS = {
    pygame.K_w: 'left_up', pygame.K_a: 'left_left',
    pygame.K_s: 'left_down', pygame.K_d: 'left_right',
    pygame.K_i: 'right_up', pygame.K_j: 'right_left',
    pygame.K_k: 'right_down', pygame.K_l: 'right_right',
    pygame.K_UP: 'dpad_up', pygame.K_LEFT: 'dpad_left',
    pygame.K_DOWN: 'dpad_down', pygame.K_RIGHT: 'dpad_right',
    pygame.K_z: FACE_BUTTONS['cross'], pygame.K_c: FACE_BUTTONS['circle'],
    pygame.K_v: FACE_BUTTONS['square'], pygame.K_b: FACE_BUTTONS['triangle'],
    pygame.K_q: 'l1', pygame.K_e: 'r1', pygame.K_1: 'l2', pygame.K_3: 'r2',
    pygame.K_BACKSPACE: 'select', pygame.K_RETURN: 'start', pygame.K_SPACE: 'home',
}


class ControllerGui:
    SIZE = (1440, 780)
    SEQUENCE_PANEL = pygame.Rect(1020, 100, 400, 620)
    SEQUENCE_ROW_HEIGHT = 76
    STICK_RADIUS = 48
    STICKS = {'left': (380, 440), 'right': (620, 440)}
    FACE_CENTERS = {'triangle': (775, 240), 'circle': (875, 310),
                    'cross': (775, 380), 'square': (675, 310)}

    def __init__(self, state, catalog=None, on_sequence=None, catalog_error=None):
        self.state = state
        self.catalog = catalog or SequenceCatalog({}, ())
        self.on_sequence = on_sequence
        self.catalog_error = catalog_error
        self.sequence_scroll = 0
        self.sequence_pressed = None
        pygame.display.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode(self.SIZE)
        pygame.display.set_caption('RumbleX simulated controller')
        self.font = pygame.font.Font(None, 25)
        self.small_font = pygame.font.Font(None, 21)
        self.buttons = {
            'l2': pygame.Rect(170, 70, 125, 40), 'r2': pygame.Rect(705, 70, 125, 40),
            'l1': pygame.Rect(160, 135, 140, 40), 'r1': pygame.Rect(700, 135, 140, 40),
            'select': pygame.Rect(405, 235, 75, 30), 'start': pygame.Rect(520, 235, 75, 30),
            'home': pygame.Rect(479, 285, 42, 42),
            'dpad_up': pygame.Rect(205, 240, 40, 40),
            'dpad_down': pygame.Rect(205, 380, 40, 40),
            'dpad_left': pygame.Rect(125, 310, 40, 40),
            'dpad_right': pygame.Rect(285, 310, 40, 40),
        }
        self.mouse_control = None
        self.dragging = None

    def reset(self):
        self.state.reset()
        self.sequence_pressed = None
        self.mouse_control = None
        self.dragging = None

    def hit_control(self, position):
        for symbol, center in self.FACE_CENTERS.items():
            if pygame.Vector2(position).distance_to(center) <= 27:
                return FACE_BUTTONS[symbol]
        return next((name for name, rect in self.buttons.items() if rect.collidepoint(position)), None)

    def sequence_buttons(self):
        panel = self.SEQUENCE_PANEL
        for index, name in enumerate(self.catalog.extra_sequences):
            rect = pygame.Rect(panel.x + 10,
                               panel.y + index * self.SEQUENCE_ROW_HEIGHT - self.sequence_scroll,
                               panel.width - 20, self.SEQUENCE_ROW_HEIGHT - 10)
            yield name, rect

    def hit_sequence(self, position):
        if self.SEQUENCE_PANEL.collidepoint(position):
            return next((name for name, rect in self.sequence_buttons()
                         if rect.collidepoint(position)), None)
        return None

    def move_stick(self, position):
        cx, cy = self.STICKS[self.dragging]
        self.state.drag_stick(self.dragging, (position[0] - cx) / self.STICK_RADIUS,
                              (cy - position[1]) / self.STICK_RADIUS)

    def handle_event(self, event, now):
        """Return 'reset' or 'quit' when a neutral publication is needed."""
        if event.type == pygame.QUIT:
            self.reset()
            return 'quit'
        if event.type == pygame.WINDOWFOCUSLOST:
            self.reset()
            return 'reset'
        if event.type == pygame.MOUSEWHEEL:
            if self.SEQUENCE_PANEL.collidepoint(pygame.mouse.get_pos()):
                maximum = max(0, len(self.catalog.extra_sequences) * self.SEQUENCE_ROW_HEIGHT
                              - self.SEQUENCE_PANEL.height)
                self.sequence_scroll = max(0, min(maximum, self.sequence_scroll - event.y * 40))
                self.sequence_pressed = None
        elif event.type in (pygame.KEYDOWN, pygame.KEYUP) and event.key in KEY_CONTROLS:
            self.state.set_control(('key', event.key), KEY_CONTROLS[event.key],
                                   event.type == pygame.KEYDOWN, now)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self.sequence_pressed = self.hit_sequence(event.pos)
            if self.sequence_pressed:
                return None
            self.mouse_control = self.hit_control(event.pos)
            if self.mouse_control:
                self.state.set_control('mouse', self.mouse_control, True, now)
            else:
                for stick, center in self.STICKS.items():
                    if pygame.Vector2(event.pos).distance_to(center) <= self.STICK_RADIUS:
                        self.dragging = stick
                        self.move_stick(event.pos)
                        break
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self.move_stick(event.pos)
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self.sequence_pressed and self.hit_sequence(event.pos) == self.sequence_pressed:
                if self.on_sequence:
                    self.on_sequence(self.sequence_pressed)
            self.sequence_pressed = None
            if self.mouse_control:
                self.state.set_control('mouse', self.mouse_control, False, now)
                self.mouse_control = None
            if self.dragging:
                self.state.release_stick(self.dragging)
                self.dragging = None
        return None

    def label(self, text, center, color=(220, 225, 235), small=False):
        rendered = (self.small_font if small else self.font).render(text, True, color)
        self.screen.blit(rendered, rendered.get_rect(center=center))

    def wrapped_label(self, text, center_x, top, width, color=(190, 199, 215)):
        # Split at any character so long underscore-separated names remain exact.
        lines = []
        line = ''
        for char in text:
            if line and self.small_font.size(line + char)[0] > width:
                lines.append(line)
                line = ''
            line += char
        if line:
            lines.append(line)
        for index, line in enumerate(lines):
            self.label(line, (center_x, top + 9 + index * 18), color, small=True)

    def draw(self, now):
        self.screen.fill((22, 27, 38))
        self.label('RumbleX simulated controller', (500, 32))
        # Rounded body, two grips, and symmetric PS3-style stick placement.
        pygame.draw.ellipse(self.screen, (48, 53, 63), (80, 185, 270, 370))
        pygame.draw.ellipse(self.screen, (48, 53, 63), (650, 185, 270, 370))
        pygame.draw.rect(self.screen, (48, 53, 63), (125, 190, 770, 325), border_radius=90)
        pressed = set(self.state.sources.values())
        for name, rect in self.buttons.items():
            color = (70, 153, 179) if name in pressed else (30, 34, 43)
            pygame.draw.rect(self.screen, color, rect, border_radius=8)
            label = {'dpad_up': '^', 'dpad_down': 'v', 'dpad_left': '<', 'dpad_right': '>'}.get(name, name.upper())
            self.label(label, rect.center, small=name in ('select', 'start', 'home'))
            if name in self.catalog.captions:
                width = 145 if name.startswith('dpad_') else 320
                self.wrapped_label(self.catalog.captions[name], rect.centerx, rect.bottom + 4, width)
        face_colors = {'triangle': (91, 211, 151), 'circle': (240, 112, 121),
                       'cross': (114, 174, 245), 'square': (219, 142, 219)}
        for symbol, center in self.FACE_CENTERS.items():
            button = FACE_BUTTONS[symbol]
            pygame.draw.circle(self.screen, (70, 153, 179) if button in pressed else (28, 32, 41), center, 27)
            x, y = center
            color = face_colors[symbol]
            if symbol == 'triangle':
                pygame.draw.polygon(self.screen, color, [(x, y - 14), (x - 14, y + 12), (x + 14, y + 12)], 3)
            elif symbol == 'circle':
                pygame.draw.circle(self.screen, color, center, 14, 3)
            elif symbol == 'square':
                pygame.draw.rect(self.screen, color, (x - 12, y - 12, 24, 24), 3)
            else:
                pygame.draw.line(self.screen, color, (x - 11, y - 11), (x + 11, y + 11), 3)
                pygame.draw.line(self.screen, color, (x - 11, y + 11), (x + 11, y - 11), 3)
            if button in self.catalog.captions:
                self.wrapped_label(self.catalog.captions[button], x, y + 31, 150)
        axes = self.state.axes()
        for stick, center in self.STICKS.items():
            h, v = axes[f'{stick}_stick_horizontal'], axes[f'{stick}_stick_vertical']
            pygame.draw.circle(self.screen, (20, 24, 32), center, 55)
            knob = (round(center[0] + h * self.STICK_RADIUS), round(center[1] - v * self.STICK_RADIUS))
            pygame.draw.circle(self.screen, (91, 99, 115), knob, 26)
            pygame.draw.circle(self.screen, (139, 151, 172), knob, 26, 2)
            self.label(f'{stick.upper()}  x {h:+.2f}  y {v:+.2f}', (center[0], 520), small=True)
        long_buttons = [name.upper() for name, start in self.state.press_times.items()
                        if now - start >= self.state.LONG_PRESS_SECONDS]
        self.label('Long press: ' + (', '.join(long_buttons) or 'hold a button for 2 seconds'), (500, 570), small=True)
        for index, line in enumerate((
            'Mouse: hold buttons / drag sticks. Release sticks to center.',
            'Sticks: WASD / IJKL     D-pad: arrow keys',
            'Cross A: Z     Circle B: C     Square X: V     Triangle Y: B',
            'L1: Q     R1: E     L2: 1     R2: 3',
            'Select: Backspace     Start: Enter     Home: Space',
        )):
            self.label(line, (500, 620 + index * 28), small=True)
        self.label('Additional sequences', (1220, 55))
        self.label('Click to run / scroll for more', (1220, 80), small=True)
        pygame.draw.rect(self.screen, (30, 34, 43), self.SEQUENCE_PANEL, border_radius=10)
        previous_clip = self.screen.get_clip()
        self.screen.set_clip(self.SEQUENCE_PANEL)
        for name, rect in self.sequence_buttons():
            if not rect.colliderect(self.SEQUENCE_PANEL):
                continue
            color = (70, 153, 179) if self.sequence_pressed == name else (55, 64, 80)
            pygame.draw.rect(self.screen, color, rect, border_radius=8)
            self.wrapped_label(name, rect.centerx, rect.y + 12, rect.width - 20)
        if self.catalog_error:
            self.wrapped_label('Cannot load sequences: ' + self.catalog_error,
                               1220, 120, 370, (240, 112, 121))
        elif not self.catalog.extra_sequences:
            self.wrapped_label('All sequences already have controller buttons.', 1220, 120, 370)
        self.screen.set_clip(previous_clip)
        pygame.display.flip()


class NodeTeleopSimulated(Node):
    def __init__(self):
        super().__init__('node_teleop_simulated')
        self.state = ControllerState()
        self.tracker = PayloadTracker()
        self.pub = self.create_publisher(JoystickRequest, 'joystick_request', 10)
        self.sequence_pub = self.create_publisher(String, 'sequence_request', 10)
        self.catalog_error = None
        try:
            self.catalog = installed_catalog()
        except (OSError, ValueError, LookupError, yaml.YAMLError) as error:
            self.catalog = SequenceCatalog({}, ())
            self.catalog_error = str(error)
            self.get_logger().error(f'Cannot load sequence catalog: {error}')
        self.get_logger().info('Starting simulated controller (mouse and keyboard)')

    def publish_sequence(self, name):
        self.sequence_pub.publish(String(data=name))

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_state(self, force=False):
        payload = self.state.sample(self.now_seconds())
        changed = self.tracker.changed(payload)
        if changed or force:
            msg = JoystickRequest()
            for name, value in payload.items():
                setattr(msg, name, value)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NodeTeleopSimulated()
        gui = ControllerGui(node.state, node.catalog, node.publish_sequence, node.catalog_error)
        frame_clock = pygame.time.Clock()
        next_publish = 0.0
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            for event in pygame.event.get():
                action = gui.handle_event(event, node.now_seconds())
                if action in ('reset', 'quit'):
                    node.publish_state(force=True)
                if action == 'quit':
                    running = False
                    break
            if time.monotonic() >= next_publish:
                node.publish_state()
                next_publish = time.monotonic() + 0.1
            gui.draw(node.now_seconds())
            frame_clock.tick(60)
    except KeyboardInterrupt:
        pass
    except pygame.error as error:
        if node is not None:
            node.get_logger().error(f'Cannot open simulated controller GUI: {error}')
        raise
    finally:
        if node is not None:
            if rclpy.ok():
                node.state.reset()
                node.publish_state(force=True)
            node.destroy_node()
        pygame.quit()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
