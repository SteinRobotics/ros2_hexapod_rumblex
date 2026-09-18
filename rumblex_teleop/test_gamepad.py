#!/usr/bin/env python3
"""
Simple gamepad/joystick testing script without ROS.
Tests buttons, axes, and dpad (hat) input.
"""

import os
# Hide the pygame support prompt
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import pygame
import sys
import time


def main():
    # Initialize pygame and joystick
    pygame.init()
    pygame.joystick.init()

    # Check for connected joysticks
    if pygame.joystick.get_count() == 0:
        print("No joystick/gamepad detected!")
        return

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()


    try:
        while True:
            pygame.event.pump()

            # Clear screen (simple terminal clear)
            print("\033[H\033[J", end="")

            print(f"Gamepad detected: {joystick.get_name()}")

            # Display axes
            print("=== AXES ===")
            for i in range(joystick.get_numaxes()):
                axis_value = joystick.get_axis(i)
                bar = create_bar(axis_value)
                print(f"Axis {i}: {axis_value:6.3f} {bar}")

            # Display buttons
            print("\n=== BUTTONS ===")
            button_states = []
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    button_states.append(f"BTN{i}")
            if button_states:
                print(f"Pressed: {', '.join(button_states)}")
            else:
                print("Pressed: None")

            # Display dpad (hat)
            print("\n=== DPAD (HAT) ===")
            for i in range(joystick.get_numhats()):
                hat = joystick.get_hat(i)
                direction = get_dpad_direction(hat)
                print(f"Hat {i}: {hat} ({direction})")

            time.sleep(0.1)  # 10 Hz update rate

    except KeyboardInterrupt:
        print("\n\nExiting...")
    finally:
        pygame.quit()


def create_bar(value, length=20):
    """Create a visual bar representation of axis value (-1 to 1)"""
    center = length // 2
    pos = int((value + 1) * center)
    pos = max(0, min(length - 1, pos))
    
    bar = ['-'] * length
    bar[center] = '|'
    bar[pos] = '█'
    return ''.join(bar)


def get_dpad_direction(hat_tuple):
    """Convert hat tuple to readable direction"""
    x, y = hat_tuple
    
    if x == 0 and y == 0:
        return "CENTER"
    elif x == 0 and y == 1:
        return "UP"
    elif x == 0 and y == -1:
        return "DOWN"
    elif x == -1 and y == 0:
        return "LEFT"
    elif x == 1 and y == 0:
        return "RIGHT"
    elif x == -1 and y == 1:
        return "UP-LEFT"
    elif x == 1 and y == 1:
        return "UP-RIGHT"
    elif x == -1 and y == -1:
        return "DOWN-LEFT"
    elif x == 1 and y == -1:
        return "DOWN-RIGHT"
    else:
        return f"({x}, {y})"


if __name__ == "__main__":
    main()
