# Launching RumbleX

Install dependencies and build the workspace using [README_SETUP.md](README_SETUP.md) and [BUILD.md](BUILD.md) first. These commands use ROS 2 Lyrical. Nox is the default robot profile; launch files with a `robot` argument load configuration from `config/<robot>/`. See [ROBOT_PROFILES.md](ROBOT_PROFILES.md) for profile details.

## Prepare every terminal

Follow [BUILD.md](BUILD.md#terminal-environment) to source ROS, activate the venv,
and load the workspace overlay. For missing Python dependencies or an executable
using the wrong interpreter, see [build troubleshooting](BUILD.md#python-executable-uses-the-wrong-interpreter).

Choose one complete bringup or simulation at a time and stop it with **Ctrl+C** before starting another. Complete launches already include several component nodes; start individual components separately only when needed.

## Physical robot

Run on the Raspberry Pi with Nox's hardware connected and configured:

```bash
ros2 launch rumblex_bringup target_launch.py robot:=nox
```

This starts communication, HMI, movement, teleoperation, LiDAR, and the BNO055 IMU, then starts the brain after five seconds. Enable head-sweep scanning and reactive navigation with:

```bash
ros2 launch rumblex_bringup target_launch.py robot:=nox enable_navigation:=true
```

For automatic startup, use the systemd service instructions in [README_SETUP.md](README_SETUP.md#raspberry-pi-5-headless-robot). The service must source Lyrical, the venv, and the workspace in its startup command.

## Offline test bringup

Run on a developer PC with a graphical desktop:

```bash
ros2 launch rumblex_bringup test_launch.py robot:=nox
```

This starts offline movement, the brain, simulated HMI and teleoperation, communication, and the mesh model in RViz. It also loads the house-contour map, displays its walls, simulates one-dimensional LiDAR readings, and estimates odometry from supporting feet. The joint-slider GUI is disabled because movement supplies the displayed joint states.

Communication still uses the PC's microphone and audio output. Its offline keyword recognizer selects `nox.onnx` or `nira.onnx` using the `robot` launch argument at a 0.5 score threshold (see [communication setup](rumblex_communication/readme.md)); online recognition also requires Google speech credentials.

Disable the simulated teleoperation GUI, or choose a different occupancy-map YAML file:

```bash
ros2 launch rumblex_bringup test_launch.py robot:=nox enable_simulated_teleop:=false
ros2 launch rumblex_bringup test_launch.py robot:=nox map:=/absolute/path/to/map.yaml
```

Other test arguments are `wall_height` (default `1.0` metres), `simulate_lidar` (default `true`), and `publish_test_map_tf` (default `true`). Set `simulate_lidar:=false` when supplying real LiDAR readings and `publish_test_map_tf:=false` when supplying odometry/localization externally.

Both `target_launch.py` and `test_launch.py` accept `autostart_listening` (default
`false`). Set it to `true` to activate speech-to-text (STT) when the brain starts:

```bash
ros2 launch rumblex_bringup target_launch.py robot:=nox autostart_listening:=true
ros2 launch rumblex_bringup test_launch.py robot:=nox autostart_listening:=true
```

This forwards the brain's `autostart_listening` parameter and starts offline
robot-name recognition; recognizing the robot's name then starts online command
recognition. The microphone, robot-specific ONNX wake-word model, openWakeWord
feature models, and Google speech credentials described above are required. The same option is available in `brain_launch.py` when
launching the brain separately.

## Individual components

Run the required component in its own prepared terminal:

```bash
ros2 launch rumblex_brain brain_launch.py robot:=nox
ros2 launch rumblex_communication communication_launch.py robot:=nox
ros2 launch rumblex_movement movement_launch.py robot:=nox
ros2 launch rumblex_teleop teleop_launch.py
ros2 launch rumblex_hmi hmi_launch.py
ros2 launch rumblex_lidar_1d lidar_launch.py
ros2 launch rumblex_navigation navigation_launch.py robot:=nox
```

The regular movement, teleoperation, HMI, and LiDAR launches access hardware. For development, these alternatives provide offline movement and simulated HMI/teleoperation:

```bash
ros2 launch rumblex_movement movement_offline_launch.py robot:=nox
ros2 launch rumblex_hmi hmi_simulated_launch.py
ros2 launch rumblex_teleop teleop_simulated_launch.py
```

Navigation consumes `scan` and odometry. For Nox, also launch `ros2 launch rumblex_perception_lidar perception_launch.py robot:=nox` to assemble head sweeps; movement and a LiDAR source must be running. See [SENSORS.md](SENSORS.md) for robot-specific pipelines. Include the bundled simple-room map server for simulation with:

```bash
ros2 launch rumblex_navigation navigation_launch.py robot:=nox enable_map:=true
```

That map server uses simulated time and expects `/clock`. To publish a map separately with wall-clock time, use:

```bash
ros2 launch rumblex_navigation map_launch.py use_sim_time:=false
# Or select another map:
ros2 launch rumblex_navigation map_launch.py map:=/absolute/path/to/map.yaml use_sim_time:=false
```

Inspect a launch file's available arguments without starting its nodes:

```bash
ros2 launch rumblex_bringup test_launch.py --show-args
```

## Simulation and visualization

Use the [developer PC setup](README_SETUP.md#developer-pc-visualization-and-simulation) for RViz and Gazebo dependencies.

### Preview the robot model in RViz

Display the primitive model with interactive joint sliders:

```bash
ros2 launch rumblex_description display.launch.py robot:=nox
```

Display the CAD/STL model with its separate launch file:

```bash
ros2 launch rumblex_description display_mesh.launch.py robot:=nox
```

Nira's primitive and mesh models are also available for preview:

```bash
ros2 launch rumblex_description display.launch.py robot:=nira
ros2 launch rumblex_description display_mesh.launch.py robot:=nira
```

Mesh assets live in `rumblex_description/meshes/nox/` and `rumblex_description/meshes/nira/`. Model availability does not imply that a robot's hardware configuration is ready for bringup.

The mesh display supports `joint_state_publisher_gui:=false` when another node supplies joint states, `joint_states_topic:=<topic>` to select those states, and `fixed_frame:=<frame>` to select RViz's fixed frame.

The RViz launchers use Qt's `xcb` backend and disable Qt high-DPI scaling for the
RViz process. This works around [RViz's scaled-display rendering bug](https://github.com/ros2/rviz/issues/1052),
which can produce flickering, a black viewport, or a small rectangle in its upper-left
corner after resizing. Desktop scaling and other GUI processes keep their settings.
RViz icons may appear smaller; enlarge the window or its side panels as needed.

### Full Gazebo simulation

For the test bringup with the mesh robot, simulated controls, and house-contour walls:

```bash
ros2 launch rumblex_bringup test_gazebo_launch.py robot:=nox
```

This uses Gazebo as the viewer and does not start RViz. The ROS occupancy map is
extruded into static Gazebo walls with collision geometry, preserving the map
origin, resolution, doorways, and `wall_height`. Select another map with
`map:=/absolute/path/to/map.yaml`; restart the launch when changing maps.
Movement targets feed the Gazebo position controller, and Gazebo publishes the
actual joint feedback. Controllers receive their robot-specific parameter file
explicitly; failed controller startup stops the mesh launch.

The test stack still estimates odometry from supporting feet and simulates the
single-beam LiDAR from the map; these are not Gazebo ground-truth sensors.
For a headless run:

```bash
ros2 launch rumblex_bringup test_gazebo_launch.py gui:=false \
  enable_simulated_teleop:=false enable_simulated_hmi:=false
```

Launch the primitive-model simulation through the installed Lyrical `ros_gz_sim` integration:

```bash
ros2 launch rumblex_gazebo simulation_gazebo.launch.py robot:=nox
```

This starts Gazebo, the robot model, ros2_control controllers, clock and LiDAR bridges, offline movement, the joint-command bridge, brain, and communication. Speech still uses the PC's audio devices and requires the communication dependencies and resources described above.

Select the simple-room world and enable navigation:

```bash
ros2 launch rumblex_gazebo simulation_gazebo.launch.py robot:=nox \
  world:="$(ros2 pkg prefix rumblex_gazebo)/share/rumblex_gazebo/worlds/simple_room.sdf" \
  enable_navigation:=true
```

The default world is `empty.sdf`; `world` accepts an absolute SDF path. Enabling navigation also starts the bundled simple-room map server.

### Mesh model in Gazebo

```bash
ros2 launch rumblex_gazebo simulation_mesh.launch.py robot:=nox
```

This spawns the STL model in the empty world, loads the 20-joint controller configuration, and starts the joint-state broadcaster, position controller, and clock bridge. It does not start brain, movement, communication, or navigation. Nox's mesh model omits head visuals but retains invisible head yaw and pitch joints for controller compatibility.

For direct joint commands and topic inspection, see [COMMANDS.md](COMMANDS.md),
including the [Gazebo joint controller](COMMANDS.md#gazebo-joint-controller).

## Nira desktop face-tracking preview

With the new camera, face-perception, interfaces, brain, and bringup packages built
and the overlay sourced:

```bash
ros2 launch rumblex_bringup test_launch.py robot:=nira
```

This uses the host's webcam, runs local YuNet face detection, and moves the **offline**
Nira head in RViz. An annotated image window opens alongside RViz. No servo hardware
is opened by movement. The desktop camera stays fixed, so the brain treats face
bearings as absolute virtual pointing directions; a stationary off-centre face
settles at a bounded head angle. This preview does not validate physical head control.

The file form `ros2 launch test_launch.py robot:=nira` works from
`rumblex_bringup/launch`. With multiple cameras, the first usable persistent device
path is selected and logged; select the desired camera explicitly when needed:

```bash
ros2 launch rumblex_bringup test_launch.py robot:=nira \
  camera_device:=/dev/v4l/by-id/YOUR_CAMERA-video-index0

# No GUI processes from the test stack (speech/audio behavior is unchanged):
ros2 launch rumblex_bringup test_launch.py robot:=nira enable_display:=false \
  enable_simulated_teleop:=false enable_simulated_hmi:=false enable_face_tracking_view:=false

# Preserve the previous Nira offline workflow without camera tracking:
ros2 launch rumblex_bringup test_launch.py robot:=nira enable_camera:=false enable_face_tracking:=false
```

Dependencies: OpenCV >=4.8, `cv_bridge`, Python OpenCV, and the standard ROS message
packages. The desktop detector and image view use compressed image topics to reduce transport
losses; raw images are published when subscribed. The small detector model is
bundled with its license; no startup download
is needed. Desktop capture uses OpenCV/V4L2 and does not require `usb_cam`; physical
camera launch still uses `usb_cam`. Camera access may require a desktop-session ACL
or `video` group membership. In a sandbox that hides `/dev/video*`, run the launch
from the host terminal. Missing/busy cameras leave the rest of the offline stack
running and the head waits for observations.

Optional `camera_params_file` uses the `node_desktop_camera` YAML selector and
controls resolution/rate. `camera_info_url:=file:///absolute/calibration.yaml`
loads calibration for this camera and matching resolution. Without it, the preview
uses an approximate 65° horizontal field of view, reported in status; this is not
physical calibration. `use_sim_time:=true` is rejected for live desktop capture.
Gazebo and Nox keep camera tracking disabled by default.

Manual movement/velocity commands, sequences, and cancellation disable preview.
Once idle, explicitly re-enable it with:

```bash
ros2 service call /brain/set_face_tracking std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /brain/face_tracking_status
ros2 topic echo /perception/face_status
```

A false service request disables tracking. Lost/stale face observations stop head
updates after 300 ms; the last virtual pose is held. Camera disconnection requires
restarting capture/launch. This increment does not enable physical face tracking,
identify people, or add walking while tracking.
