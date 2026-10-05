# Launching RumbleX

Install dependencies and build the workspace using [README_SETUP.md](README_SETUP.md) first. These commands use ROS 2 Lyrical. Nox is the default robot profile; launch files with a `robot` argument load configuration from `config/<robot>/`. See [ROBOT_PROFILES.md](ROBOT_PROFILES.md) for profile details.

## Prepare every terminal

Run this in each terminal used for launching nodes or sending commands:

```bash
cd ~/Workspace/colcon_rumblex
source /opt/ros/lyrical/setup.bash
source .venv/bin/activate
source install/local_setup.bash
```

The venv must have been created with `--system-site-packages` so ROS modules remain available. Check the Python environment and communication dependencies:

```bash
which python
python -c 'import rclpy, vosk, speech_recognition, sounddevice, gtts; print("Python dependencies OK")'
```

Build Python packages with the active venv using `python -m colcon build`. Their generated executables record the build interpreter, so activating `.venv` afterward does not repair an executable built with `/usr/bin/python3`. If communication reports missing dependencies such as `vosk`, rebuild it:

```bash
python -m colcon build --symlink-install --packages-select rumblex_communication
source install/local_setup.bash
head -n 1 install/rumblex_communication/lib/rumblex_communication/node_communication
```

The first line should point to the workspace's `.venv/bin/python`. If the import check itself fails, install the Python dependencies as described in [README_SETUP.md](README_SETUP.md).

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

Communication still uses the PC's microphone and audio output. Its offline recognizer expects the `vosk-model-small-en-us-0.15` model under the communication package's `models/` directory; online recognition also requires Google speech credentials.

Disable the simulated teleoperation GUI, or choose a different occupancy-map YAML file:

```bash
ros2 launch rumblex_bringup test_launch.py robot:=nox enable_simulated_teleop:=false
ros2 launch rumblex_bringup test_launch.py robot:=nox map:=/absolute/path/to/map.yaml
```

Other test arguments are `wall_height` (default `2.0` metres), `simulate_lidar` (default `true`), and `publish_test_map_tf` (default `true`). Set `simulate_lidar:=false` when supplying real LiDAR readings and `publish_test_map_tf:=false` when supplying odometry/localization externally.

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

Navigation consumes `scan` and odometry. For Nox, also launch `ros2 launch rumblex_perception perception_launch.py robot:=nox` to assemble head sweeps; movement and a LiDAR source must be running. See [SENSORS.md](SENSORS.md) for robot-specific pipelines. Include the bundled simple-room map server for simulation with:

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

Command all 20 joints directly in radians:

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0]}"
```

The array follows the joint order in `rumblex_gazebo/config/nox/joint_controllers.yaml`. Use direct controller commands with the mesh-only launch; the full simulation's joint-command bridge continuously publishes its own targets.

## Interact with running nodes

Monitor available topics and servo status:

```bash
ros2 topic list
ros2 topic echo /servos_status
```

Inject a recognized speech command for the brain:

```bash
ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'steh auf'}"
```

Inspect the body-pose interface before sending a custom movement command:

```bash
ros2 interface show rumblex_interfaces/msg/BodyPose
ros2 topic echo /cmd_movement --once
```

Publish a complete `BodyPose` YAML payload to `/cmd_movement` using `ros2 topic pub --once`. Angles are in degrees, positions in metres, and the six toe targets are ordered right front, right middle, right back, left front, left middle, left back. The active brain also publishes movement targets, so use offline movement separately when testing custom poses.

Inject a joystick request, for example to cycle the brain's gait mode:

```bash
ros2 topic pub --once /joystick_request rumblex_interfaces/msg/JoystickRequest "{button_start: true}"
```

Use `ros2 interface show rumblex_interfaces/msg/JoystickRequest` to inspect the other buttons and stick fields.
