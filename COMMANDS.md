# RumbleX ROS 2 topic commands

Prepare your terminal using [BUILD.md](BUILD.md#terminal-environment), then start
the required nodes using [README_LAUNCH.md](README_LAUNCH.md). These examples use
the default ROS namespace. Nox is the existing robot; Nira hardware bringup
requires its own completed [profile](ROBOT_PROFILES.md).

## Inspect topics

```bash
ros2 topic list -t
ros2 topic echo --once /servo_status
ros2 topic echo --csv /servo_status
ros2 topic echo --once --qos-durability transient_local /movement_name
ros2 topic echo --once --qos-durability transient_local /body_pose_actual
ros2 topic echo --once /joint_states
```

## Trigger named sequences

The [virtual joystick](rumblex_teleop/readme.md) publishes the exact sequence name
as `std_msgs/msg/String` on `/sequence_request`. Send the same requests from a
terminal with the brain running:

```bash
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'standup'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'laydown'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'neutral'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'look_left'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'look_right'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'leg_wave'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'clap'}"
```

Replace `data` with any exact `name` from the installed
[sequence catalog](rumblex_brain/config/sequences). The brain uses normal joystick
priority, except `standup`, which has high priority. Unknown names are logged and
ignored. Rebuild `rumblex_brain` and restart the brain and GUI after catalog changes;
see [BUILD.md](BUILD.md#incremental-builds).

## Activate IMU torso balancing

With the hardware stack running, request the built-in `level_torso` sequence:

```bash
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'level_torso'}"
```

This stands up first, then uses IMU orientation to correct torso roll/pitch for
**5 seconds**, and finally holds the resulting pose. Feet, torso position/yaw,
and head pose stay fixed during leveling. This is a timed standing activity;
it does not enable continuous balancing while walking.

For Nox, start hardware bringup if it is not already running:

```bash
ros2 launch rumblex_bringup target_launch.py robot:=nox
```

This starts the brain, movement, and BNO055 driver. Leveling also requires a TF
transform from `base_link` to the IMU message's `header.frame_id` (normally
`imu_link`). Hardware bringup does not start a robot-state publisher. If no
existing publisher supplies that transform, run the primitive model's publisher
in another sourced terminal, provided its mounting matches the physical IMU:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro "$(ros2 pkg prefix --share rumblex_description)/urdf/nox.urdf.xacro" use_sim:=false)"
```

Nox's mesh model does not include `imu_link`; it needs a separately verified
mounting TF. Hardware axis remapping must agree with the frame used by the IMU.
Check the sensor and transform before requesting leveling:

```bash
ros2 topic echo --once --qos-reliability best_effort /bno055/imu
ros2 run tf2_ros tf2_echo base_link imu_link
```

The IMU must publish valid orientation quaternions with current timestamps and
`orientation_covariance[0]` other than `-1`. Defaults allow 2 seconds to acquire
valid IMU/TF data and require readings fresher than 0.5 seconds. Missing, invalid,
or stale data aborts the sequence; check the brain log for
`LEVEL_TORSO ended sequence`.

To change the active duration, edit `kLevelDuration` in
[`level_torso.yaml`](rumblex_brain/config/sequences/level_torso.yaml), rebuild
`rumblex_brain`, and restart the brain. Configure the topic, gains, and limits
under `level_torso` in `rumblex_brain/config/<robot>/gait.yaml`, then rebuild and
restart to load the updated file. Defaults are approximately a 1-degree deadband,
8-degree maximum correction, and 2 degrees/second maximum correction rate;
physical tuning is still required. See the
[sensor gait documentation](rumblex_brain/config/sequences/README.md) for details.

## Send requests

Inject recognized speech into the running brain:

```bash
ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'steh auf'}"
ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'leg dich hin'}"
ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'lauf nach vorne'}"
```

Activate robot-name listening, or play a file from the configured music directory:

```bash
ros2 topic pub --once /request_listening std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /request_music std_msgs/msg/String "{data: 'musicfox_hot_dogs_for_breakfast.mp3'}"
```

Inject joystick input (omitted fields default to zero/false), or send velocity
commands to the brain. Linear velocity is in metres per second; angular velocity
is in radians per second:

```bash
# Stand up (D-pad up):
ros2 topic pub --once /joystick_request rumblex_interfaces/msg/JoystickRequest "{dpad_vertical: 1}"
# Cycle the gait mode:
ros2 topic pub --once /joystick_request rumblex_interfaces/msg/JoystickRequest "{button_start: true}"
# Walk with a turn:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.1}}"
# Stop:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

For continuous joystick movement, choose a walking gait with Start if necessary
(Start also cycles through pose mode). Positive vertical stick input means forward.
Run the publisher below, stop it with **Ctrl+C**, then send neutral input to stop
walking; stopping the publisher alone does not send a stop request:

```bash
ros2 topic pub --rate 10 /joystick_request rumblex_interfaces/msg/JoystickRequest \
  "{left_stick_vertical: 0.4}"
# After stopping the repeating publisher:
ros2 topic pub --once /joystick_request rumblex_interfaces/msg/JoystickRequest '{}'
```

`/cmd_movement` carries `rumblex_interfaces/msg/BodyPose`: head orientation,
torso pose, and six toe positions. Angles are in degrees and positions in metres;
toe order is right front, right middle, right back, left front, left middle,
left back. Inspect an existing target before constructing a complete custom pose:

```bash
ros2 topic echo --once /cmd_movement
```

Use offline movement separately when testing custom poses because the active brain
publishes its own targets.

For a direct hardware servo check with movement running and the brain stopped,
send an angle in degrees using a name from the selected robot's
`rumblex_movement/config/<robot>/servo_description.yaml`:

```bash
ros2 topic pub --once /single_servo_request rumblex_interfaces/msg/ServoAngle \
  "{name: 'LEG_RIGHT_BACK_COXA', angle_deg: 0.0}"
```

## Direct servo maintenance and status testing

With movement running and the brain stopped, read a servo ID (`cmd: 14`) or
change it (`cmd: 13`, new ID in `data1`). Names must exist in the selected robot's
servo description. Choose an unused ID on the bus; after changing it, update the
matching `servo.serial_ids` entry in
`rumblex_movement/config/<robot>/servo_description.yaml` before restarting movement.
The example below changes the left front coxa ID to 10; replace 10 with the
intended unused ID for your robot.

```bash
ros2 topic pub --once /servo_direct_request rumblex_interfaces/msg/ServoDirectRequest \
  "{name: 'LEG_LEFT_FRONT_COXA', cmd: 14}"
ros2 topic pub --once /servo_direct_request rumblex_interfaces/msg/ServoDirectRequest \
  "{name: 'LEG_LEFT_FRONT_COXA', cmd: 13, data1: 10, data2: 0}"
```

Read results appear in the movement node's log. The direct-request callback
currently handles only ID read/write (14/13) and LED read/write (34/33), even
though the message defines additional command constants.

For testing status consumers, publish a synthetic status with the real movement
publisher stopped. This does not command a servo or read hardware:

```bash
ros2 topic pub --once /servo_status rumblex_interfaces/msg/ServoStatus \
  "{servo_error_code: 'LEG_LEFT_FRONT_COXA', error_code: 0,
    servo_max_temperature: 'LEG_LEFT_FRONT_COXA', max_temperature: 40.0,
    servo_max_voltage: 'LEG_LEFT_FRONT_COXA', max_voltage: 12.0,
    servo_min_voltage: 'LEG_LEFT_FRONT_COXA', min_voltage: 12.0}"
```

## Gazebo joint controller

With the [mesh-only simulation](README_LAUNCH.md#mesh-model-in-gazebo), command all
20 joints in radians. The array follows
`rumblex_gazebo/config/nox/joint_controllers.yaml`. The full simulation's joint
bridge continuously publishes its own targets, so use this with the mesh-only launch.

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0]}"
```
