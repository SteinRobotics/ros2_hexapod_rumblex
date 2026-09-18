# topics of the node servo_controller 
```

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
```

## servo direct request
### set Servo ID
```
ros2 topic pub --once /servo_direct_request rumblex_interfaces/msg/ServoDirectRequest "
name: 'LEG_LEFT_FRONT_COXA'
cmd: 13
data1: 10
data2: 0
"
```


ros2 topic pub --once /servo_request rumblex_interfaces/msg/ServoRequest "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
target_angles:
- name: 'HEAD_YAW'
  angle_deg: 0.0
- name: 'HEAD_PITCH'
  angle_deg: 0.0
- name: 'LEG_LEFT_FRONT_COXA'
  angle_deg: 0.0
- name: 'LEG_LEFT_FRONT_FEMUR'
  angle_deg: 20.0
- name: 'LEG_LEFT_FRONT_TIBIA'
  angle_deg: 20.0
- name: 'LEG_LEFT_MID_COXA'
  angle_deg: 0.0
- name: 'LEG_LEFT_MID_FEMUR'
  angle_deg: 0.0
- name: 'LEG_LEFT_MID_TIBIA'
  angle_deg: 0.0
- name: 'LEG_LEFT_BACK_COXA'
  angle_deg: 0.0
- name: 'LEG_LEFT_BACK_FEMUR'
  angle_deg: 0.0
- name: 'LEG_LEFT_BACK_TIBIA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_FRONT_COXA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_FRONT_FEMUR'
  angle_deg: 0.0
- name: 'LEG_RIGHT_FRONT_TIBIA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_MID_COXA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_MID_FEMUR'
  angle_deg: 0.0
- name: 'LEG_RIGHT_MID_TIBIA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_BACK_COXA'
  angle_deg: 0.0
- name: 'LEG_RIGHT_BACK_FEMUR'
  angle_deg: 0.0
- name: 'LEG_RIGHT_BACK_TIBIA'
  angle_deg: 0.0
time_to_reach_target_angles_ms: 3000"
```

```
ros2 topic pub --once /single_servo_request rumblex_interfaces/msg/ServoAngle "
  name: LEG_RIGHT_BACK_COXA 
  angle_deg: 0.0"
```
