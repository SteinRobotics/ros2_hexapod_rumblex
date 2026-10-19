# RumbleX command reference

Commands assume the workspace is at `~/Workspace/colcon_rumblex` and uses ROS 2
Lyrical. See [README_SETUP.md](README_SETUP.md) for dependencies,
[README_LAUNCH.md](README_LAUNCH.md) for launch details, and
[ROBOT_PROFILES.md](ROBOT_PROFILES.md) for robot configuration.
Nox is the existing robot; Nira hardware bringup requires its own completed profile.

## Prepare each terminal

```bash
cd ~/Workspace/colcon_rumblex
source /opt/ros/lyrical/setup.bash
source .venv/bin/activate
source install/local_setup.bash
```

Create the venv with `--system-site-packages` as described in the setup guide.
Build Python packages with the active venv so their executables use its interpreter.

## Setup and dependencies

For a fresh Ubuntu Server 26.04 arm64 Raspberry Pi 5, run the setup script as the
robot user from a fresh login shell, before sourcing ROS or activating a venv:

```bash
cd ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex
./scripts/setup_raspberry_pi.sh nox
```

The script installs dependencies, builds the runtime, configures hardware access,
and enables the robot user's boot service. Reboot afterward to apply permissions
and start the robot. For a developer PC or manual installation, follow the
platform-specific dependency commands in [README_SETUP.md](README_SETUP.md).
Install Python dependencies inside `.venv`.

Update the Garmin LiDAR library submodule from the repository root:

```bash
git submodule update --init --recursive
```

For Nira's YDLIDAR, after installing the SDK described in [SENSORS.md](SENSORS.md),
select the maintained driver fork's `lyrical` branch (requires GitHub SSH access):

```bash
./scripts/update_ydlidar_driver.sh
```

## Build and test

Run from the workspace root with ROS sourced and the venv active:

```bash
# Complete workspace, with all dependencies installed:
python -m colcon build --symlink-install
# Or build the robot runtime and its workspace dependencies:
python -m colcon build --symlink-install --packages-up-to rumblex_bringup
# Or rebuild just movement:
python -m colcon build --symlink-install --packages-select rumblex_movement
source install/local_setup.bash

python -m colcon test --packages-select rumblex_movement --event-handlers console_direct+
colcon test-result --verbose
```

## Robot boot service

The setup script installs a **user service**. Run these as the robot user:

```bash
systemctl --user start autostart_ros2.service
systemctl --user status autostart_ros2.service
systemctl --user stop autostart_ros2.service
systemctl --user restart autostart_ros2.service
journalctl --user -u autostart_ros2.service -b -f
cat ~/.config/rumblex/robot.env
```

The profile is selected by `RUMBLEX_ROBOT` in `~/.config/rumblex/robot.env`.
Stop the service before starting a manual bringup that would use the same hardware.

## Launch robot or simulation

Choose one complete launch at a time; stop it with **Ctrl+C** before switching.

```bash
# Physical robot:
ros2 launch rumblex_bringup target_launch.py robot:=nox
# Physical robot with navigation and head-sweep sensing:
ros2 launch rumblex_bringup target_launch.py robot:=nox enable_navigation:=true
# Enable robot-name recognition when the brain starts:
ros2 launch rumblex_bringup target_launch.py robot:=nox autostart_listening:=true

# Offline movement, simulated controls, map, and RViz:
ros2 launch rumblex_bringup test_launch.py robot:=nox
# Test stack with Gazebo mesh visualization and map walls:
ros2 launch rumblex_bringup test_gazebo_launch.py robot:=nox
# Primitive-model Gazebo simulation:
ros2 launch rumblex_gazebo simulation_gazebo.launch.py robot:=nox

# Model previews with joint sliders:
ros2 launch rumblex_description display.launch.py robot:=nox
ros2 launch rumblex_description display_mesh.launch.py robot:=nira

# Inspect arguments without starting nodes:
ros2 launch rumblex_bringup test_launch.py --show-args
```

Test and simulation stacks still use the computer's audio devices. Listening needs
the robot-specific ONNX model and openWakeWord resources; online recognition needs
Google speech credentials. See [communication setup](rumblex_communication/readme.md).

## Individual components

Complete bringup already starts its components. Use separate prepared terminals
when launching components independently:

```bash
ros2 launch rumblex_brain brain_launch.py robot:=nox
ros2 launch rumblex_communication communication_launch.py robot:=nox
ros2 launch rumblex_movement movement_launch.py robot:=nox
ros2 launch rumblex_teleop teleop_launch.py
ros2 launch rumblex_hmi hmi_launch.py
ros2 launch rumblex_bringup sensors_launch.py robot:=nox
ros2 launch rumblex_navigation navigation_launch.py robot:=nox
```

Servo control belongs to `rumblex_movement`. The sensor launch selects the
robot-specific LiDAR pipeline; see [SENSORS.md](SENSORS.md) for navigation inputs.
For development without movement, HMI, or controller hardware:

```bash
ros2 launch rumblex_movement movement_offline_launch.py robot:=nox
ros2 launch rumblex_hmi hmi_simulated_launch.py
ros2 launch rumblex_teleop teleop_simulated_launch.py
```

## Inspect topics and interfaces

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo --once /servo_status
ros2 topic echo --csv /servo_status
ros2 topic echo --once --qos-durability transient_local /movement_name
ros2 topic echo --once --qos-durability transient_local /body_pose_actual
ros2 topic echo --once /joint_states
ros2 interface show rumblex_interfaces/msg/BodyPose
ros2 interface show rumblex_interfaces/msg/JoystickRequest
```

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
