# new

https://index.ros.org/p/joy/
ros2 run joy joy_enumerate_devices

This is for topic_joy to topic_twist
https://github.com/ros2/teleop_twist_joy
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3'



# pyglet
https://github.com/pyglet/pyglet

pip3 install pyglet --break-system-packages

## joystick.buttons[i]
BUTTON_A = 2
BUTTON_B = 1
BUTTON_X = 3
BUTTON_Y = 0
BUTTON_L1 = 4
BUTTON_L2 = 6
BUTTON_R1 = 5
BUTTON_R2 = 7
BUTTON_SELECT = 8
BUTTON_START = 9

## joystick.hat_y
DPAD_VERTICAL = # DOWN =-1, UP = 1

## joystick.hat_x
DPAD_HORIZONTAL = # LEFT = -1, RIGHT = 1

## joystick.y
LEFT_STICK_VERTICAL = # TOP = -1, DOWN = 1, hangs on 0.004 -> means 0.0

## joystick.x
LEFT_STICK_HORIZONTAL = # LEFT = -1, RIGHT = 1, hangs on 0.004 -> means 0.0

## joystick.z
RICHT_STICK_HORIZONTAL = # LEFT = -1, RIGHT = 1, hangs on 0.004 -> means 0.0

## joystick.rz
RICHT_STICK_VERTICAL

joystick.rx, joystick.ry not working with mode green


# pyjoystick (deprecated!)
https://pypi.org/project/pyjoystick/

pip3 install --break-system-packages pyjoystick

import pyjoystick
from pyjoystick.sdl2 import Key, Joystick, run_event_loop

# key.keytype: Axis
AXIS_LEFT_STICK_LEFT_RIGHT = 0
AXIS_LEFT_STICK_UP_DOWN = 1
AXIS_RIGHT_STICK_LEFT_RIGHT = 2
AXIS_RIGHT_STICK_UP_DOWN = 3
# key.keytype: Button
BUTTON_START = 9
BUTTON_SELECT = 8
BUTTON_A = 2
BUTTON_B = 1
BUTTON_X = 3
BUTTON_Y = 0
BUTTON_LEFT_1 = 4
BUTTON_LEFT_2 = 6
BUTTON_RIGHT_1 = 5
BUTTON_RIGHT_2 = 7
# key.keytype: Hat
HAT_CENTER = 0
HAT_UP = 1
HAT_DOWN = 4
HAT_LEFT = 8
HAT_RIGHT = 2

# joystick general info
sudo apt-get install joystick

/dev/input/js0

testprogramm:
jstest /dev/input/js0
