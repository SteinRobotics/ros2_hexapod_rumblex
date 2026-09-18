#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import os
# Hide the pygame support prompt
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import threading

import rclpy
from rclpy.node import Node
from rumblex_interfaces.msg import JoystickRequest

INTERVAL = 100  # milliseconds, how often the joystick data is published

class NodeTeleop(Node):

    generic_layout = {}

    # Shanwan Twin Usb Joystick
    generic_layout['Shanwan Twin Usb Joystick'] = {
        'button': {'A': 0, 'B': 1, 'X': 3, 'Y': 4, 'L1': 6, 'R1': 7, 'L2': 8, 'R2': 9, 'SELECT': 10, 'START': 11, 'HOME': 12, 'RETURN': 13, 'LIST': 14}, 
        'axis': {'LEFT_STICK_HORIZONTAL': 0, 'LEFT_STICK_VERTICAL': 1, 'RIGHT_STICK_HORIZONTAL': 2, 'RIGHT_STICK_VERTICAL': 3}, 
        'dpad': {'HORIZONTAL': 0, 'VERTICAL': 1}
    }

    # PS3 look alike
    generic_layout['Twin USB Joystick'] = {
        'button': {'A': 2, 'B': 1, 'X': 3, 'Y': 0, 'L1': 4, 'R1': 5, 'L2': 6, 'R2': 7, 'SELECT': 8, 'START': 9}, 
        'axis': {'LEFT_STICK_HORIZONTAL': 0, 'LEFT_STICK_VERTICAL': 1, 'RIGHT_STICK_HORIZONTAL': 2, 'RIGHT_STICK_VERTICAL': 3}, 
        'dpad': {'HORIZONTAL': 0, 'VERTICAL': 1}
    }

    # origin PS3
    generic_layout['Sony PLAYSTATION(R)3 Controller'] = {
        'button': {'A': 0, 'B': 1, 'X': 3, 'Y': 2, 'L1': 4, 'R1': 5, 'L2': 6, 'R2': 7, 'SELECT': 8, 'START': 9}, 
        'axis': {'LEFT_STICK_HORIZONTAL': 0, 'LEFT_STICK_VERTICAL': 1, 'RIGHT_STICK_HORIZONTAL': 2, 'RIGHT_STICK_VERTICAL': 3}, 
        'dpad': {'HORIZONTAL': 0, 'VERTICAL': 1}
    }

    button_debounce_duration_s = 2.0
    button_press_time = {
        'A': None,
        'B': None,
        'X': None,
        'Y': None,
        'L1': None,
        'R1': None,
        'L2': None,
        'R2': None,
        'SELECT': None,
        'START': None,
        'HOME': None,
    }
    axis_deadzone = 0.004

    def __init__(self):
        super().__init__('node_teleop')
        self.get_logger().info('starting node_teleop')
        self.msg = JoystickRequest()
        self._last_payload = None
        self.start_button_press_time = None  # Track when start button was first pressed

       # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected! Please connect one.")
            exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected to Joystick: {self.joystick.get_name()}")

        if self.joystick.get_name() not in self.generic_layout:
            self.get_logger().error(f"Joystick layout for '{self.joystick.get_name()}' not found.")
            exit()

        self.layout = self.generic_layout.get(self.joystick.get_name())


        self.pub = self.create_publisher(JoystickRequest, 'joystick_request', 10)

        self.running = True
        # Start the pygame event loop in a separate thread.
        self.event_thread = threading.Thread(target=self.event_loop, daemon=True)
        self.event_thread.start()

    def event_loop(self):
        while self.running and rclpy.ok():
            events = pygame.event.get()
            for event in events:
                self.handle_event(event)
            if not events:
                # Still evaluate debouncing even when no fresh events arrive
                self.handle_event(None)
            pygame.time.wait(INTERVAL)

    def _set_button_state(self, button_name, short_press, long_press):
        if button_name == 'A':
            self.msg.button_a = short_press
            self.msg.button_long_a = long_press
        elif button_name == 'B':
            self.msg.button_b = short_press
            self.msg.button_long_b = long_press
        elif button_name == 'X':
            self.msg.button_x = short_press
            self.msg.button_long_x = long_press
        elif button_name == 'Y':
            self.msg.button_y = short_press
            self.msg.button_long_y = long_press
        elif button_name == 'L1':
            self.msg.button_l1 = short_press
            self.msg.button_long_l1 = long_press
        elif button_name == 'L2':
            self.msg.button_l2 = short_press
            self.msg.button_long_l2 = long_press
        elif button_name == 'R1':
            self.msg.button_r1 = short_press
            self.msg.button_long_r1 = long_press
        elif button_name == 'R2':
            self.msg.button_r2 = short_press
            self.msg.button_long_r2 = long_press
        elif button_name == 'SELECT':
            self.msg.button_select = short_press
            self.msg.button_long_select = long_press
        elif button_name == 'START':
            self.msg.button_start = short_press
            self.msg.button_long_start = long_press
        elif button_name == 'HOME':
            self.msg.button_home = short_press
            self.msg.button_long_home = long_press

    def _current_payload(self):
        return (
            self.msg.button_a,
            self.msg.button_b,
            self.msg.button_x,
            self.msg.button_y,
            self.msg.button_l1,
            self.msg.button_l2,
            self.msg.button_r1,
            self.msg.button_r2,
            self.msg.button_select,
            self.msg.button_start,
            self.msg.button_home,
            self.msg.button_long_a,
            self.msg.button_long_b,
            self.msg.button_long_x,
            self.msg.button_long_y,
            self.msg.button_long_l1,
            self.msg.button_long_l2,
            self.msg.button_long_r1,
            self.msg.button_long_r2,
            self.msg.button_long_select,
            self.msg.button_long_start,
            self.msg.button_long_home,
            self.msg.dpad_horizontal,
            self.msg.dpad_vertical,
            self.msg.left_stick_horizontal,
            self.msg.left_stick_vertical,
            self.msg.right_stick_horizontal,
            self.msg.right_stick_vertical,
        )

    def debounce_button(self, button_pressed, press_time_tracker, debounce_duration_s, now):
        short_press = False
        long_press_active = False

        if button_pressed:
            if press_time_tracker is None:
                press_time_tracker = now
            elapsed_s = (now - press_time_tracker).nanoseconds / 1e9
            long_press_active = elapsed_s >= debounce_duration_s
        else:
            if press_time_tracker is not None:
                elapsed_s = (now - press_time_tracker).nanoseconds / 1e9
                long_press_active = elapsed_s >= debounce_duration_s
                short_press = not long_press_active
                press_time_tracker = None

        return short_press, long_press_active, press_time_tracker

    def debounce_buttons(self):
        pygame.event.pump()  # Process events
        any_button_activity = False
        now = self.get_clock().now()

        for button_name in self.button_press_time.keys():
            button_index = self.layout['button'].get(button_name)
            if button_index is None:
                self._set_button_state(button_name, False, False)
                self.button_press_time[button_name] = None
                continue

            button_pressed = bool(self.joystick.get_button(button_index))
            short_press, long_press, new_time = self.debounce_button(
                button_pressed, self.button_press_time[button_name], self.button_debounce_duration_s, now
            )

            self._set_button_state(button_name, short_press, long_press)
            self.button_press_time[button_name] = new_time

            if button_pressed or short_press or long_press:
                any_button_activity = True

        return any_button_activity

    def handle_event(self, event):
        axis_or_hat_event = (
            event is not None and (event.type == pygame.JOYAXISMOTION or event.type == pygame.JOYHATMOTION)
        )
        if self.debounce_buttons() or axis_or_hat_event:
            self.publish_joystick_data()

    def publish_joystick_data(self):
        # self.get_logger().info(f"publish_joystick_data")
        pygame.event.pump()  # Process events
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        hat_values = self.joystick.get_hat(0)  # Usually only one hat switch exists

        if abs(axes[0]) < self.axis_deadzone:
            self.msg.left_stick_horizontal = 0.0
        else:
            self.msg.left_stick_horizontal = axes[self.layout['axis']['LEFT_STICK_HORIZONTAL']]

        if abs(axes[1]) < self.axis_deadzone:
            self.msg.left_stick_vertical = 0.0
        else:
            self.msg.left_stick_vertical = -axes[self.layout['axis']['LEFT_STICK_VERTICAL']]  # Invert vertical axis

        if abs(axes[2]) < self.axis_deadzone:
            self.msg.right_stick_horizontal = 0.0
        else:
            self.msg.right_stick_horizontal = axes[self.layout['axis']['RIGHT_STICK_HORIZONTAL']]

        if abs(axes[3]) < self.axis_deadzone:
            self.msg.right_stick_vertical = 0.0
        else:
            self.msg.right_stick_vertical = -axes[self.layout['axis']['RIGHT_STICK_VERTICAL']]  # Invert vertical axis

        self.msg.dpad_horizontal = hat_values[self.layout['dpad']['HORIZONTAL']]
        self.msg.dpad_vertical = hat_values[self.layout['dpad']['VERTICAL']]
        payload = self._current_payload()
        if payload == self._last_payload:
            return

        self._last_payload = payload
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

    def shutdown(self):
        # Signal the event loop thread to stop and quit pygame.
        self.running = False
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = NodeTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
