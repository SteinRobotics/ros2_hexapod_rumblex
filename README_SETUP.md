# RumbleX setup

This guide sets up **Nox**, the current RumbleX robot, on a headless Raspberry Pi 5 and sets up a separate developer PC for RViz and Gazebo. Nira is planned; use the lowercase `robot` launch argument to select a profile when its configuration and model are available. See [README.md](README.md) for the project overview, pin layout, and example ROS topics, and [ROBOT_PROFILES.md](ROBOT_PROFILES.md) for the profile layout.

The commands below assume **Ubuntu 26.04** and **ROS 2 Lyrical** on both machines. Use the Ubuntu system Python (3.14 on Ubuntu 26.04) for ROS and the virtual environment; mixing Python versions can break ROS Python extensions. Install ROS from the [official Lyrical Ubuntu instructions](https://docs.ros.org/en/lyrical/Get-Started/Installation/Ubuntu-Install-Debs.html) before continuing. Choose `ros-lyrical-ros-base` on the Pi and `ros-lyrical-desktop` on the PC. Both machines need the same ROS distribution to share topics reliably. The C++ packages require a C++23-capable compiler and CMake 3.25 or newer; Ubuntu 26.04 supplies both.

## Raspberry Pi 5: headless robot

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
   source .venv/bin/activate
   python -m pip install --upgrade pip wheel 'setuptools<80'
   python -m pip install \
     vosk SpeechRecognition google-cloud-speech google-api-python-client \
     oauth2client gTTS sounddevice \
     adafruit-blinka adafruit-circuitpython-ssd1306 adafruit-circuitpython-ina228
   ```

   `rclpy` and the other ROS Python modules come from apt, not pip. The Google speech service also needs credentials if you enable online recognition; follow the communication package's configuration for that feature.

5. Install the **runtime** ROS dependencies. Include `rumblex_description` because `rumblex_bringup` declares it as a dependency, but skip its RViz and joint slider dependencies on the Pi. The Python rosdep keys listed below are handled by the venv installation above; `magic_enum` and `fonts-liberation` are installed explicitly because their rosdep mappings are missing for Ubuntu 26.04.

   ```bash
   source /opt/ros/lyrical/setup.bash
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
     src/ros2_hexapod_rumblex/rumblex_perception \
     src/ros2_hexapod_rumblex/rumblex_navigation \
     src/ros2_hexapod_rumblex/rumblex_teleop \
     src/ros2_hexapod_rumblex/rumblex_description \
     src/ros2_hexapod_rumblex/rumblex_bringup \
     --ignore-src -r -y \
     --skip-keys 'ydlidar_ros2_driver rviz2 joint_state_publisher_gui magic_enum fonts-liberation python3-oauth2client python-google-cloud-speech-pip python3-sounddevice-pip python-gTTS-pip'
   python -m colcon build --symlink-install --packages-up-to rumblex_bringup
   source install/local_setup.bash
   ```

   The Nox-only setup skips `ydlidar_ros2_driver`; Nira requires the external driver and SDK described in [SENSORS.md](SENSORS.md).

   `rosdep` installs system and ROS dependencies through apt; the explicit pip step installs the Python-only runtime dependencies into `.venv`. `libyaml-cpp-dev` and `ros-lyrical-std-srvs` supply native build dependencies; `ros-lyrical-xacro` is needed for the robot models and their tests. Do not run `rosdep` over the entire workspace on the Pi: that would also install Gazebo dependencies.

6. Check the environment, then start Nox:

   ```bash
   python -c 'import rclpy, pygame, pyaudio, sounddevice, vosk, speech_recognition, gtts, board, adafruit_ssd1306, adafruit_ina228; print("Python dependencies OK")'
   ros2 launch rumblex_bringup target_launch.py robot:=nox
   # Optional: start head-sweep navigation too
   ros2 launch rumblex_bringup target_launch.py robot:=nox enable_navigation:=true
   ```

   Run one launch command at a time. The optional boot service in `rumblex_bringup/autostart/autostart_ros2.service` assumes the Pi user is `rumblex`. Copy it to `/etc/systemd/system/autostart_ros2.service` and edit `User`, `Group`, home paths, and audio session settings to match the Pi. Replace its `ExecStart` with the following line, substituting the actual user's home directory if needed:

   ```ini
   ExecStart=/bin/bash -lc 'source /opt/ros/lyrical/setup.bash && source /home/rumblex/Workspace/colcon_rumblex/.venv/bin/activate && source /home/rumblex/Workspace/colcon_rumblex/install/local_setup.bash && ros2 launch rumblex_bringup target_launch.py robot:=nox'
   ```

   Then run `sudo systemctl daemon-reload`, `sudo systemctl enable --now autostart_ros2.service`, and check `sudo systemctl status autostart_ros2.service` or `journalctl -u autostart_ros2.service -b`.

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
   source .venv/bin/activate
   python -m pip install --upgrade pip wheel 'setuptools<80'
   python -m pip install \
     vosk SpeechRecognition google-cloud-speech google-api-python-client \
     oauth2client gTTS sounddevice
   ```

   These pip packages support the communication node used by simulation. The rosdep skip keys below cover these Python dependencies and the explicitly installed `libmagicenum-dev` and `fonts-liberation` packages. The PC setup does not need the Pi's Adafruit HMI libraries. ROS Python packages, `pygame`, and `pyaudio` remain apt-managed and are visible through `--system-site-packages`.

3. Install the PC packages' dependencies and build the simulation and shared packages:

   ```bash
   source /opt/ros/lyrical/setup.bash
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
     src/ros2_hexapod_rumblex/rumblex_perception \
     src/ros2_hexapod_rumblex/rumblex_navigation \
     src/ros2_hexapod_rumblex/rumblex_description \
     src/ros2_hexapod_rumblex/rumblex_gazebo \
     --ignore-src -r -y \
     --skip-keys 'ydlidar_ros2_driver magic_enum fonts-liberation python3-oauth2client python-google-cloud-speech-pip python3-sounddevice-pip python-gTTS-pip'
   python -m colcon build --symlink-install \
     --packages-up-to rumblex_gazebo rumblex_description rumblex_navigation
   source install/local_setup.bash
   ```

4. Preview the model or run Gazebo:

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

## Upgrading an existing workspace to Lyrical

After upgrading to Ubuntu 26.04 and installing ROS 2 Lyrical, open a fresh terminal and source `/opt/ros/lyrical/setup.bash`. Remove any old ROS setup commands from your shell startup files and update the robot's boot service to use Lyrical as shown above.

Archive the existing `build/`, `install/`, and `log/` directories before rebuilding. Recreate `.venv` with `/usr/bin/python3` on Ubuntu 26.04 and reinstall its Python dependencies using the steps above. Then rerun `rosdep install` and the build command for the Pi or PC. Source the new `install/local_setup.bash` after the build completes.

## Every new terminal

On **either** machine, activate the venv and source ROS and the workspace before using the built packages:

```bash
cd ~/Workspace/colcon_rumblex
source .venv/bin/activate
source /opt/ros/lyrical/setup.bash
source install/local_setup.bash
```

Check the interpreter with `which python` and `python -c 'import rclpy; print(rclpy.__file__)'`. For a package-level check, run `python -m colcon test --packages-select <package_name> --event-handlers console_direct+` followed by `colcon test-result --verbose` from the workspace root.

If a communication node reports `ModuleNotFoundError: No module named 'vosk'` even though it is installed in `.venv`, rebuild the package with the venv's Python:

```bash
python -m colcon build --symlink-install --packages-select rumblex_communication
source install/local_setup.bash
head -n 1 install/rumblex_communication/lib/rumblex_communication/node_communication
```

The executable's first line should point to `.venv/bin/python`. Activating a venv after a build does not change the interpreter recorded in existing ROS Python executables.
