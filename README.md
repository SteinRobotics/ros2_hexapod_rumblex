
# RumbleX Hexapod Platform (Maker Project)

**⚠️ WARNING: This repository is under heavy development! Breaking changes, incomplete features, and experimental code are expected. Use at your own risk. Contributions, feedback, and ideas are welcome.**

RumbleX is an open-source, modular hexapod robot platform for makers, tinkerers, and robotics enthusiasts. The project is built around ROS2 and aims to be a flexible playground for learning, hacking, and experimenting with robotics and human-machine interaction.

The first robot built on the platform is **Nox**, which uses the existing hardware and configuration. A
second robot, **Nira**, is planned. Shared ROS packages therefore use the `rumblex_` prefix, while
robot-specific parameters and models are selected by the lowercase `robot` launch argument. Nox is the
default (`robot:=nox`). See [ROBOT_PROFILES.md](ROBOT_PROFILES.md) for the profile layout and the files
needed to add Nira.

## Features & Goals
- **Maker Focus**: Designed for hands-on experimentation, learning, and creative robotics projects.
- **ROS2 Native**: Modern robotics workflows and easy integration with other ROS2 packages.
- **Modular Architecture**: Movement, communication, HMI, teleoperation, servo control, and more
- **Speech Recognition & Synthesis**: Online/offline STT and TTS with caching.
- **Flexible Gait & Kinematics**: Advanced gait controller and kinematics for smooth, stable walking and body pose control.
- **Teleoperation**: Joystick and remote control support.
- **Extensive Launch & Config**: Launch files and configuration options for different hardware and scenarios.
- **Diagnostics & Services**: Systemd integration and ROS2 topics for monitoring and diagnostics.


## Project Structure
- `rumblex_brain/`         — High-level behavior, action planning, and coordination
- `rumblex_movement/`      — Gait, kinematics, and movement primitives
- `rumblex_communication/` — Speech recognition, TTS, chatbot, and audio I/O
- `rumblex_hmi/`           — Human-machine interface (OLED, relay control)
- `rumblex_teleop/`        — Teleoperation (joystick, remote)
- `rumblex_lidar/`         — LIDAR sensor integration
- `rumblex_navigation/`    — 1D-lidar head-sweep navigation with obstacle avoidance
- `rumblex_interfaces/`    — Custom ROS2 message and service definitions
- `rumblex_bringup/`       — Launch and bringup scripts
- `rumblex_doc/`           — Documentation, diagrams, and hardware info
- `rumblex_utils/`         — Shared utilities, math helpers, and tests
- `rumblex_description/`   — URDF/XACRO robot model for visualization and simulation
- `rumblex_gazebo/`        — Gazebo Harmonic simulation (gz-sim 8.x)

## Quick Start (for Makers)
1. **Install Dependencies**
   ```bash
   PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths ~/Workspace/colcon_rumblex --ignore-src -r -y
   git submodule update --init --recursive
   ```
   If you want the simplest setup, keep using the full-workspace install above. If you want to split machines, the current package layout already allows two practical ROS setups:

   **Robot / headless target**
   - Intended for the Raspberry Pi or onboard computer.
   - Keeps runtime, sensor, audio, and hardware nodes.
   - Skips GUI and Gazebo packages such as `rviz2`, `joint_state_publisher_gui`, `ros_gz_sim`, and controller GUI tooling.
   ```bash
   PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -r -y --ignore-src \
     --from-paths \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_interfaces \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_utils \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_movement \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_brain \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_communication \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_hmi \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_lidar \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_navigation \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_teleop \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_bringup

   colcon build --symlink-install --packages-up-to rumblex_bringup
   ```

   **Remote PC / simulation and GUI tools**
   - Intended for Gazebo, RViz, URDF inspection, and desktop debugging.
   - Keeps shared logic packages plus the visualization/simulation packages.
   - Can skip robot-only hardware packages such as `rumblex_hmi` and usually `rumblex_bringup`.
   ```bash
   PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -r -y --ignore-src \
     --from-paths \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_interfaces \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_utils \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_movement \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_brain \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_communication \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_navigation \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_description \
       ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_gazebo

   colcon build --symlink-install \
     --packages-up-to rumblex_gazebo rumblex_description rumblex_navigation
   ```

   Notes:
   - The split is already possible because the GUI-heavy dependencies are isolated mainly in `rumblex_description` and `rumblex_gazebo`, while `rumblex_bringup` stays on the robot/runtime side.
   - Do not run `rosdep install --from-paths ...` over the entire workspace on the robot if you want a lean headless install, because that will pull the Gazebo and RViz dependencies too.
   - Robot-specific configuration is selected independently through the `robot` launch argument.
2. **Build the Workspace**
   ```bash
   colcon build --symlink-install
   source install/local_setup.bash
   ```
3. **Launch the Robot**
   ```bash
   ros2 launch rumblex_bringup target_launch.py robot:=nox
   # with navigation enabled
   ros2 launch rumblex_bringup target_launch.py robot:=nox enable_navigation:=true
   # or for testing
   ros2 launch rumblex_bringup test_launch.py robot:=nox
   ```
4. **Launch Individual Components**
   ```bash
   ros2 launch rumblex_brain brain_launch.py
   ros2 launch rumblex_communication communication_launch.py
   ros2 launch rumblex_movement movement_launch.py
   ros2 launch rumblex_teleop teleop_launch.py
   ros2 launch rumblex_lidar lidar_launch.py
   ros2 launch rumblex_navigation navigation_launch.py
   # with map server
   ros2 launch rumblex_navigation navigation_launch.py enable_map:=true
   ```
5. **Interact & Hack**
   - Send movement commands:
     ```bash
     ros2 topic pub --once /cmd_movement rumblex_interfaces/msg/MovementRequest "..."
     ```
   - Monitor topics:
     ```bash
     ros2 topic list
     ros2 topic echo /servos_status
     ```
   - Speech commands:
     ```bash
     ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'steh auf'}"
     ```
   - Joystick/teleop:
     ```bash
     ros2 topic pub --once /joystick_request rumblex_interfaces/msg/JoystickRequest "..."
     ```

## Simulation & Visualization

### Preview the Robot Model in RViz
Visualize the URDF model with interactive joint sliders — no Gazebo or hardware needed:
```bash
source install/setup.bash
ros2 launch rumblex_description display.launch.py
```

To preview the CAD/STL-based model instead, use its separate launch file:
```bash
ros2 launch rumblex_description display_mesh.launch.py
```
The original primitive model remains available through `display.launch.py`.

Nox's STL assets are stored in `rumblex_description/meshes/nox/`. Each robot's
mesh Xacro sets `mesh_directory`, which is also used by the shared leg macro.
The future Nira mesh model should point to `rumblex_description/meshes/nira/`
and will be selected with `robot:=nira` once its model and assets are added.

### Gazebo Simulation
Run the full hexapod simulation in Gazebo Harmonic:
```bash
# Install simulation dependencies (once)
sudo apt-get install ros-jazzy-gz-ros2-control ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster ros-jazzy-forward-command-controller \
  ros-jazzy-joint-state-publisher-gui

# Navigation dependencies (optional)
sudo apt-get install ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager

# Build and launch
colcon build --symlink-install --packages-select rumblex_description rumblex_gazebo
source install/setup.bash
ros2 launch rumblex_gazebo simulation_gazebo.launch.py robot:=nox

# Launch with the simple room world and navigation
ros2 launch rumblex_gazebo simulation_gazebo.launch.py \
  world:=$(ros2 pkg prefix rumblex_gazebo)/share/rumblex_gazebo/worlds/simple_room.sdf \
  enable_navigation:=true
```

### Mesh Model in Gazebo
The mesh model has its own launch file and does not change the existing primitive-model simulation:
```bash
colcon build --symlink-install --packages-select rumblex_description rumblex_gazebo
source install/setup.bash
ros2 launch rumblex_gazebo simulation_mesh.launch.py
```
This starts Gazebo Harmonic, spawns the STL-based robot, loads the existing 20-joint
controller configuration, and starts the joint-state broadcaster. The mesh model omits
head visuals but retains invisible head yaw and pitch joints for controller compatibility.

Command joints in the simulation (all 20 joints, values in radians):
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0]}"
```


## Raspberry Pi 5 Pin Layout
The Raspberry Pi 5 inside Nox hosts most of the human-machine interface hardware that lives in `rumblex_hmi/`. The table follows the standard 40-pin header (odd numbers on the left when the USB ports face you). Pins with descriptions are currently wired up; empty cells are free for experiments.

| Pin   | Signal        | Usage                  | Pin    | Signal        | Usage               |
| ----- | ------------- | ---------------------- | ------ | ------------- | ------------------- |
| **1** | 3V3           | supply for I2C modules | 2      | 5V            | —                   |
| **3** | GPIO2 (SDA1)  | I2C bus                | **4**  | 5V            | Servo Power Relay   |
| **5** | GPIO3 (SCL1)  | I2C bus                | **6**  | GND           | GND for I2C modules |
| 7     | GPIO4         | —                      | 8      | GPIO14 (TXD)  | —                   |
| 9     | GND           | —                      | 10     | GPIO15 (RXD)  | —                   |
| 11    | GPIO17        | —                      | 12     | GPIO18        | —                   |
| 13    | GPIO27        | —                      | **14** | GND           | Servo Power Relay   |
| 15    | GPIO22        | —                      | 16     | GPIO23        |                     |
| 17    | 3V3           | —                      | 18     | GPIO24        | —                   |
| 19    | GPIO10 (MOSI) | —                      | 20     | GND           | —                   |
| 21    | GPIO9 (MISO)  | —                      | 22     | GPIO25        | —                   |
| 23    | GPIO11 (SCLK) | —                      | 24     | GPIO8 (CE0)   | —                   |
| 25    | GND           | —                      | 26     | GPIO7 (CE1)   | —                   |
| 27    | GPIO0 (ID_SD) | —                      | 28     | GPIO1 (ID_SC) | —                   |
| 29    | GPIO5         | —                      | 30     | GND           | —                   |
| 31    | GPIO6         | —                      | 32     | GPIO12        | —                   |
| 33    | GPIO13        | —                      | **34** | GND           | BNO055 I2C switch   |
| 35    | GPIO19        | —                      | **36** | GPIO16        | Servo Power Relay   |
| 37    | GPIO26        | —                      | 38     | GPIO20        | —                   |
| 39    | GND           | —                      | 40     | GPIO21        | —                   |


### I2C Device Addresses
| Device               | Address |
| -------------------- | ------- |
| BNO055 IMU           | 0x29    |
| SSD1306 OLED Display | 0x3C    |
| INA228 Power Monitor | 0x40    |
| Garmin Lidar Lite    | 0x62    |



## Systemd Service (Optional)
To auto-start ROS2 on boot:
```bash
sudo systemctl start autostart_ros2
sudo systemctl status autostart_ros2
```


## Documentation
- See `rumblex_doc/` for hardware pinouts, protocol docs, and setup guides.
- Diagrams and images are provided for wiring and architecture overview.


## Music
https://www.musicfox.com/info/kostenlose-gemafreie-musik/

## Contributing
This is a maker project—experimentation, hacking, and learning are encouraged! Contributions, bug reports, and feature requests are welcome. Please open issues or pull requests for improvements.


## License
Copyright (c) 2021-2025 Christian Stein

---
