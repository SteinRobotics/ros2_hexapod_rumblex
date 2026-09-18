# send requests

## stehe auf:
```
ros2 topic pub --once /joystick_request rumblex_interfaces/msg/JoystickRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
button_a: false
button_b: false
button_x: false
button_y: false
button_l1: false
button_l2: false
button_r1: false
button_r2: false
button_select: false
button_start: false
dpad_vertical: 1
dpad_horizontal: 0
left_stick_vertical: 0.0
left_stick_horizontal: 0.0
right_stick_horizontal: 0.0
right_stick_vertical: 0.0"
```

## cyclic movement
```
ros2 topic pub --rate 10 /joystick_request rumblex_interfaces/msg/JoystickRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
button_a: false
button_b: false
button_x: false
button_y: false
button_l1: false
button_l2: false
button_r1: false
button_r2: false
button_select: false
button_start: false
dpad_vertical: 0
dpad_horizontal: 0
left_stick_vertical: 0.4
left_stick_horizontal: 0.0
right_stick_horizontal: 0.0
right_stick_vertical: 0.0"
```
```
ros2 topic pub --once /joystick_request rumblex_interfaces/msg/JoystickRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
button_a: false
button_b: false
button_x: false
button_y: false
button_l1: false
button_l2: false
button_r1: false
button_r2: false
button_select: false
button_start: false
dpad_vertical: 0
dpad_horizontal: 0
left_stick_vertical: 0.0
left_stick_horizontal: 0.0
right_stick_horizontal: 0.0
right_stick_vertical: 0.0"
```