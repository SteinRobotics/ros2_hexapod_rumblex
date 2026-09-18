## service
```
sudo systemctl start autostart_ros2
sudo systemctl status autostart_ros2
```

## install dependencies
```
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths ~/Workspace/colcon_rumblex --ignore-src -r -y

python3 -m pip install lewansoul-lx16a --break-system-packages
python3 -m pip install lewansoul-lx16a-terminal --break-system-packages

git submodule update --init
```

## build
```
colcon build --symlink-install
colcon build --packages-up-to rumblex_servo
```

```
source install/local_setup.bash
```

## log topics
```
ros2 topic list
ros2 topic echo --csv /servos_status
ros2 topic echo --once /servos_status
```

## send requests
```
ros2 topic pub --once /request_listening std_msgs/msg/Bool data:\ true

ros2 topic pub --once /speech_recognition_online std_msgs/msg/String data:\ "lauf nach vorne"

ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'steh auf'}"

ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'leg dich hin'}"

ros2 topic pub --once /speech_recognition_online std_msgs/msg/String data:\ "teste beine"

ros2 topic pub --once /request_music std_msgs/msg/String "{data: 'musicfox_hot_dogs_for_breakfast.mp3'}"

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
button_home: false
button_long_a: false
button_long_b: false
button_long_x: false
button_long_y: false
button_long_l1: false
button_long_l2: false
button_long_r1: false
button_long_r2: false
button_long_select: false
button_long_start: false
button_long_home: false
dpad_vertical: 0
dpad_horizontal: 0
left_stick_vertical: 0.50
left_stick_horizontal: 0.0
right_stick_horizontal: 0.0
right_stick_vertical: 0.0"
---

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
---

ros2 topic pub --once /cmd_movement rumblex_interfaces/msg/MovementRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
type: 13
direction: 0
duration_s: 2.0
name: 'SEQUENCE_BODY_ROLL'
"
---

ros2 topic pub --once /cmd_movement rumblex_interfaces/msg/MovementRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
type: 4
direction: 0
duration_s: 1.0
name: ''
"

ros2 topic pub --once /cmd_movement rumblex_interfaces/msg/MovementRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
type: 20
direction: 0
duration_s: 1.0
name: 'CONTINUOUS_RUNNING'
"

ros2 topic pub --once /cmd_movement_update rumblex_interfaces/msg/ContinuousMovementUpdate "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
velocity:
  linear: {x: 0.5, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}
body_pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
head_orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
"
---

ros2 topic echo --once /movement_type_actual
---

ros2 topic pub --once /servo_status rumblex_interfaces/msg/ServoStatus "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
servo_error_code: 'LEG_LEFT_FRONT_COXA'
error_code: 0
servo_max_temperature: 'LEG_LEFT_FRONT_COXA'
max_temperature: 40.0
servo_max_voltage: 'LEG_LEFT_FRONT_COXA'
max_voltage: 12.0
servo_min_voltage: 'LEG_LEFT_FRONT_COXA'
min_voltage: 12.0
"
---



```
ros2 topic pub --once /single_servo_request rumblex_interfaces/msg/ServoAngle "
  name: LEG_RIGHT_BACK_COXA 
  angle_deg: 0.0"
```

## launch robot
```
source install/local_setup.bash
ros2 launch rumblex_bringup target_launch.py
ros2 launch rumblex_bringup test_launch.py

ros2 launch rumblex_brain brain_launch.py
ros2 launch rumblex_communication communication_launch.py
ros2 launch rumblex_movement movement_launch.py
ros2 launch rumblex_servo servo_launch.py
ros2 launch rumblex_teleop teleop_launch.py
ros2 launch rumblex_lidar lidar_launch.yaml

#ros2 launch rumblex_servo_controller servo_controller_launch.py


```

## remote computer

