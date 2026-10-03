
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
- `rumblex_gazebo/`        — Gazebo simulation through ROS 2 integration

## Getting Started

Follow [README_SETUP.md](README_SETUP.md) to install dependencies, create the ROS 2 Python virtual environment, and build the workspace on a Raspberry Pi 5 or developer PC.

See [README_LAUNCH.md](README_LAUNCH.md) for physical robot bringup, offline tests, individual components, RViz previews, Gazebo simulation, and example ROS commands. It includes the terminal environment and venv interpreter checks needed for Python nodes.

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
