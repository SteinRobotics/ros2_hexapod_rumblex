# RumbleX setup

This guide sets up **Nox**, the current RumbleX robot, on a headless Raspberry Pi 5 and sets up a separate developer PC for RViz and Gazebo. Nira is planned; use the lowercase `robot` launch argument to select a profile when its configuration and model are available. See [README.md](README.md) for the project overview and pin layout, and [ROBOT_PROFILES.md](ROBOT_PROFILES.md) for the profile layout.

The commands below assume **Ubuntu 26.04** and **ROS 2 Lyrical** on both machines. Use the Ubuntu system Python (3.14 on Ubuntu 26.04) for ROS and the virtual environment; mixing Python versions can break ROS Python extensions. Install ROS from the [official Lyrical Ubuntu instructions](https://docs.ros.org/en/lyrical/Get-Started/Installation/Ubuntu-Install-Debs.html) before continuing. Choose `ros-lyrical-ros-base` on the Pi and `ros-lyrical-desktop` on the PC. Both machines need the same ROS distribution to share topics reliably. The C++ packages require a C++23-capable compiler and CMake 3.25 or newer; Ubuntu 26.04 supplies both.

[BUILD.md](BUILD.md) contains terminal sourcing, platform-specific build selections,
resource limits, tests, and build troubleshooting. Use its terminal environment
instructions after creating the venv below, before installing ROS dependencies.

## Raspberry Pi 5: headless robot

For Nox's 2.4 GHz WiFi restriction, see [band verification and Netplan configuration](rumblex_doc/setup_raspberry_pi/setup_ubuntu_26.04.md#wlan-auf-24-ghz-begrenzen). A current 2.4 GHz connection alone does not prove that the profile excludes 5 GHz; the setup script does not configure this restriction.

1. Flash Ubuntu Server 26.04 (64-bit), configure networking and SSH, boot the Pi, and log in over SSH. Connect the I2C devices, relay, LiDAR, controller, microphone, and speaker according to the hardware documentation in `rumblex_doc/` and the pin table in [README.md](README.md#raspberry-pi-5-pin-layout). Enable I2C in the Pi's boot configuration and reboot; verify the devices with `sudo i2cdetect -y 1` (expected addresses for Nox are `0x29`, `0x3c`, `0x40`, and `0x62`). Give the runtime user access to the relevant I2C, GPIO, and audio devices according to the OS device permissions.

2. After installing `ros-lyrical-ros-base` and the ROS apt repository as linked above, install development and native audio/I2C support:

   ```bash
   sudo apt update
   sudo apt install git cmake g++ ros-dev-tools python3-venv python3-pip python3-sounddevice \
     i2c-tools python3-smbus python3-pygame python3-pyaudio python3-pil \
     portaudio19-dev libmagicenum-dev fonts-liberation libyaml-cpp-dev ros-lyrical-std-srvs ros-lyrical-xacro espeak-ng
   sudo rosdep init  # only once per machine; skip if already initialized
   rosdep update
   ```

3. Create the workspace and fetch the LiDAR library submodule:

   ```bash
   mkdir -p ~/Workspace/colcon_rumblex/src
   cd ~/Workspace/colcon_rumblex/src
   git clone https://github.com/SteinRobotics/ros2_hexapod_rumblex.git
   cd ros2_hexapod_rumblex
   git submodule update --init --recursive
   ```

4. Create a virtual environment **with system site packages**. This makes the apt-installed ROS modules and packages such as `pygame` and `pyaudio` visible inside it. Install the additional Python packages used by the speech and HMI nodes into the venv. Do not use `sudo pip` or `--break-system-packages` for these packages.

   ```bash
   cd ~/Workspace/colcon_rumblex
   /usr/bin/python3 -m venv --system-site-packages .venv
   ```

   Activate the venv using [BUILD.md](BUILD.md#terminal-environment), then install Python dependencies:

   ```bash
   python -m pip install --upgrade pip wheel 'setuptools<80'
   python -m pip install \
     SpeechRecognition google-cloud-speech \
     gTTS \
     adafruit-blinka adafruit-circuitpython-ssd1306 adafruit-circuitpython-ina228
   ```

   `rclpy` and the other ROS Python modules come from apt, not pip. The Google speech service also needs credentials if you enable online recognition; follow the communication package's configuration for that feature.

5. Install the **runtime** ROS dependencies. If `usb_cam` has no Lyrical binary, first follow the [camera source-install fallback](rumblex_camera/README.md#dependencies-and-build) and include `src/usb_cam` in the rosdep paths below. Include `rumblex_description` because `rumblex_bringup` declares it as a dependency, but skip its RViz and joint slider dependencies on the Pi. The Python rosdep keys listed below are handled by the venv installation above; `magic_enum` and `fonts-liberation` are installed explicitly because their rosdep mappings are missing for Ubuntu 26.04.

   ```bash
   cd ~/Workspace/colcon_rumblex
   rosdep install --from-paths \
     src/ros2_hexapod_rumblex/rumblex_interfaces \
     src/ros2_hexapod_rumblex/rumblex_utils \
     src/ros2_hexapod_rumblex/rumblex_movement \
     src/ros2_hexapod_rumblex/rumblex_brain \
     src/ros2_hexapod_rumblex/rumblex_communication \
     src/ros2_hexapod_rumblex/rumblex_hmi \
     src/ros2_hexapod_rumblex/rumblex_lidar_1d \
     src/ros2_hexapod_rumblex/rumblex_lidar_2d \
     src/ros2_hexapod_rumblex/rumblex_camera \
     src/ros2_hexapod_rumblex/rumblex_perception \
     src/ros2_hexapod_rumblex/rumblex_navigation \
     src/ros2_hexapod_rumblex/rumblex_teleop \
     src/ros2_hexapod_rumblex/rumblex_description \
     src/ros2_hexapod_rumblex/rumblex_bringup \
     --ignore-src -r -y \
     --skip-keys 'ydlidar_ros2_driver rviz2 joint_state_publisher_gui magic_enum fonts-liberation python-google-cloud-speech-pip python3-sounddevice-pip python-gTTS-pip'
   ```

   Build the robot runtime using [BUILD.md](BUILD.md#initial-builds), then source the overlay.

   The Nox-only setup skips `ydlidar_ros2_driver`; Nira requires the external driver and SDK described in [SENSORS.md](SENSORS.md).

   `rosdep` installs system and ROS dependencies through apt; the explicit pip step installs the Python-only runtime dependencies into `.venv`. `libyaml-cpp-dev` and `ros-lyrical-std-srvs` supply native build dependencies; `ros-lyrical-xacro` is needed for the robot models and their tests. Do not run `rosdep` over the entire workspace on the Pi: that would also install Gazebo dependencies.

For offline robot-name recognition, follow the [openWakeWord ONNX setup](rumblex_communication/readme.md) before launching communication.

6. Check the environment, then start Nox:

   ```bash
   python -c 'import rclpy, pygame, pyaudio, sounddevice, openwakeword, speech_recognition, gtts, board, adafruit_ssd1306, adafruit_ina228; print("Python dependencies OK")'
   ros2 launch rumblex_bringup target_launch.py robot:=nox
   # Optional: start head-sweep navigation too
   ros2 launch rumblex_bringup target_launch.py robot:=nox enable_navigation:=true
   ```

   Run one launch command at a time. For startup at boot, follow the [Ubuntu 26.04 setup guide](rumblex_doc/setup_raspberry_pi/setup_ubuntu_26.04.md). The setup script installs `autostart_ros2.service` as a user service for the login user (`nox` on Nox), with its launcher, environment file, audio services, and lingering.

   ```bash
   systemctl --user status autostart_ros2.service
   systemctl --user stop autostart_ros2.service
   journalctl --user -u autostart_ros2.service -b
   ```

   These service commands need no sudo. Host shutdown and reboot have separate permissions; see [shutdown and sudo permissions](rumblex_doc/setup_raspberry_pi/setup_ubuntu_26.04.md#herunterfahren-neustart-und-sudo-rechte). `NOPASSWD` permits the listed command through `sudo` without a password.

## Developer PC: visualization and simulation

1. Install Ubuntu 26.04, the ROS apt repository, `ros-lyrical-desktop`, and `ros-dev-tools` using the [official Lyrical Ubuntu instructions](https://docs.ros.org/en/lyrical/Get-Started/Installation/Ubuntu-Install-Debs.html). Install the remaining development tools:

   ```bash
   sudo apt update
   sudo apt install git cmake g++ python3-venv python3-pip python3-sounddevice \
     portaudio19-dev python3-pyaudio python3-pygame libmagicenum-dev fonts-liberation \
     libyaml-cpp-dev ros-lyrical-std-srvs ros-lyrical-xacro espeak-ng \
     ros-lyrical-gz-ros2-control ros-lyrical-controller-manager \
     ros-lyrical-joint-state-broadcaster ros-lyrical-forward-command-controller \
     ros-lyrical-joint-state-publisher-gui \
     ros-lyrical-nav2-map-server ros-lyrical-nav2-lifecycle-manager
   sudo rosdep init  # only once per machine; skip if already initialized
   rosdep update
   ```

2. Clone the workspace and create its venv:

   ```bash
   mkdir -p ~/Workspace/colcon_rumblex/src
   cd ~/Workspace/colcon_rumblex/src
   git clone https://github.com/SteinRobotics/ros2_hexapod_rumblex.git
   cd ros2_hexapod_rumblex
   git submodule update --init --recursive
   cd ~/Workspace/colcon_rumblex
   /usr/bin/python3 -m venv --system-site-packages .venv
   ```

   Activate the venv using [BUILD.md](BUILD.md#terminal-environment), then install Python dependencies:

   ```bash
   python -m pip install --upgrade pip wheel 'setuptools<80'
   python -m pip install \
     SpeechRecognition google-cloud-speech \
     gTTS
   ```

   Also complete the [openWakeWord ONNX setup](rumblex_communication/readme.md) in this venv before building. Vosk is no longer required.

   These pip packages support the communication node used by simulation. The rosdep skip keys below cover these Python dependencies and the explicitly installed `libmagicenum-dev` and `fonts-liberation` packages. The PC setup does not need the Pi's Adafruit HMI libraries. ROS Python packages, `pygame`, and `pyaudio` remain apt-managed and are visible through `--system-site-packages`.

3. Install the PC packages' dependencies and build the simulation and shared packages:

   ```bash
   cd ~/Workspace/colcon_rumblex
   rosdep install --from-paths \
     src/ros2_hexapod_rumblex/rumblex_interfaces \
     src/ros2_hexapod_rumblex/rumblex_utils \
     src/ros2_hexapod_rumblex/rumblex_movement \
     src/ros2_hexapod_rumblex/rumblex_brain \
     src/ros2_hexapod_rumblex/rumblex_communication \
     src/ros2_hexapod_rumblex/rumblex_bringup \
     src/ros2_hexapod_rumblex/rumblex_hmi \
     src/ros2_hexapod_rumblex/rumblex_teleop \
     src/ros2_hexapod_rumblex/rumblex_lidar_1d \
     src/ros2_hexapod_rumblex/rumblex_lidar_2d \
     src/ros2_hexapod_rumblex/rumblex_camera \
     src/ros2_hexapod_rumblex/rumblex_perception \
     src/ros2_hexapod_rumblex/rumblex_navigation \
     src/ros2_hexapod_rumblex/rumblex_description \
     src/ros2_hexapod_rumblex/rumblex_gazebo \
     --ignore-src -r -y \
     --skip-keys 'ydlidar_ros2_driver magic_enum fonts-liberation python-google-cloud-speech-pip python3-sounddevice-pip python-gTTS-pip'
   ```

4. Build the developer PC selection and source the overlay using [BUILD.md](BUILD.md#initial-builds), then preview the model or run Gazebo:

   ```bash
   ros2 launch rumblex_description display.launch.py robot:=nox
   # Or preview the CAD/STL model:
   ros2 launch rumblex_description display_mesh.launch.py robot:=nox
   # Or run the primitive-model simulation:
   ros2 launch rumblex_gazebo simulation_gazebo.launch.py robot:=nox
   # Or run the mesh-based simulation:
   ros2 launch rumblex_gazebo simulation_mesh.launch.py robot:=nox
   ```

   Run one launch command at a time. [README_LAUNCH.md](README_LAUNCH.md#simulation-and-visualization) includes examples for navigation, worlds, and simulated joint commands.

## Build, terminal setup, and upgrades

See [BUILD.md](BUILD.md) for all source and colcon build commands, tests, ROS/Python
upgrade steps, and interpreter troubleshooting. Launch examples are in
[README_LAUNCH.md](README_LAUNCH.md); topic commands are in [COMMANDS.md](COMMANDS.md).
