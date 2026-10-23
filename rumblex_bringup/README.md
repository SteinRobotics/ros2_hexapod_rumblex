# RumbleX bringup

This package provides launch files for the physical robot, offline development,
and Gazebo simulation, plus a systemd service for starting the robot at boot.
Select the robot configuration with `robot:=nox` (the default) or `robot:=nira`.

## Autostart

The Raspberry Pi setup script, [`scripts/setup_raspberry_pi.sh`](../scripts/setup_raspberry_pi.sh),
installs `autostart_ros2.service` as a **user service** for the robot's login user.
See the [Ubuntu 26.04 setup guide](../rumblex_doc/setup_raspberry_pi/setup_ubuntu_26.04.md)
for installation prerequisites and setup instructions.

The setup script installs:

- `autostart/autostart_ros2.service` to `~/.config/systemd/user/autostart_ros2.service`.
- `autostart/start_ros2.sh` to `~/.local/bin/rumblex-start`.
- The selected profile as `RUMBLEX_ROBOT=nox` or `RUMBLEX_ROBOT=nira` in
  `~/.config/rumblex/robot.env`.

It enables the service and user lingering, so the robot starts at boot without an
interactive login. Reboot after setup to apply hardware permissions and start the
robot. The service depends on PipeWire, PipeWire PulseAudio support, and WirePlumber
for audio. Optional speech environment variables can be placed in
`~/.config/rumblex/speech.env`.

The launcher sources ROS 2 Lyrical, the workspace virtual environment, and the
installed workspace in `~/Workspace/colcon_rumblex`, then runs
`target_launch.py` with the selected robot profile. It uses headless SDL settings.
Navigation, the camera, and automatic speech listening retain their launch defaults
(`false`). The service restarts after a failure with a ten-second delay.

### Service commands

Run these commands as the robot's login user, using `--user` without `sudo`:

| Action | Command |
| --- | --- |
| See status and recent output | `systemctl --user status autostart_ros2.service` |
| Start now | `systemctl --user start autostart_ros2.service` |
| Enable startup at boot | `systemctl --user enable autostart_ros2.service` |
| Enable and start now | `systemctl --user enable --now autostart_ros2.service` |
| Stop now | `systemctl --user stop autostart_ros2.service` |
| Restart | `systemctl --user restart autostart_ros2.service` |
| Disable startup and stop now | `systemctl --user disable --now autostart_ros2.service` |
| Check whether startup is enabled | `systemctl --user is-enabled autostart_ros2.service` |

`enable` alone does not start the service immediately. `stop` leaves startup enabled,
so the robot will start again at the next boot. An explicit stop does not trigger
the failure restart policy.

Follow the service logs from the current boot with:

```bash
journalctl --user -u autostart_ros2.service -b -f
```

After editing `robot.env` or `speech.env`, restart the service. After editing the
installed service unit, run `systemctl --user daemon-reload` before restarting.
Changes to the repository's autostart files must also be copied to their installed
locations to take effect.

## Launch files

For manual launches, source ROS and the built workspace first:

```bash
source /opt/ros/lyrical/setup.bash
source ~/Workspace/colcon_rumblex/.venv/bin/activate
source ~/Workspace/colcon_rumblex/install/local_setup.bash
```

Stop the autostart service before manually launching the robot stack to avoid
duplicate nodes and competing hardware access. Run one bringup stack at a time.

| Launch file | Purpose |
| --- | --- |
| `target_launch.py` | Physical robot: communication, hardware HMI, movement, teleoperation, robot-specific lidar, and BNO055 IMU. Starts the brain after five seconds. Optionally includes navigation and a camera. |
| `sensors_launch.py` | Sensor-only compatibility entry point that forwards to `rumblex_perception`: selects the lidar driver and optional head scanning and camera from the robot profile. |
| `test_launch.py` | Offline development: communication, offline movement, simulated teleoperation and HMI, a house map with wall visualization and simulated lidar, supporting-foot odometry, robot visualization in RViz, and the brain after two seconds. |
| `test_gazebo_launch.py` | Gazebo mesh robot with physical map walls, the test stack, and a joint-state bridge. Uses simulation time, routes movement targets to `target_joint_states`, and disables the separate offline RViz display. |

### Physical robot

```bash
ros2 launch rumblex_bringup target_launch.py robot:=nox
ros2 launch rumblex_bringup target_launch.py robot:=nira enable_navigation:=true enable_camera:=true
```

| Argument | Default | Effect |
| --- | --- | --- |
| `robot` | `nox` | Select `nox` or `nira`. |
| `enable_navigation` | `false` | Start navigation and request head scanning where supported. |
| `enable_camera` | `false` | Start the camera when the robot's sensor profile provides one. |
| `camera_device` | empty | Override the camera device. |
| `camera_params_file` | empty | Override the camera parameter file. |
| `camera_info_url` | empty | Override the camera calibration URL. |
| `autostart_listening` | `false` | Activate speech-to-text listening when the brain starts. |

Empty camera overrides use the camera launch defaults. Speech listening requires
the speech dependencies and configuration described in the setup documentation.

### Sensors only

```bash
ros2 launch rumblex_bringup sensors_launch.py robot:=nox
```

This launch accepts `robot`, `enable_camera`, and the three camera overrides above.
It also accepts `enable_driver` (default `true`) and `enable_head_scan` (default
`false`). Head scanning and camera startup depend on support in the robot profile.
The BNO055 IMU is started by `target_launch.py`, rather than this sensor entry point.

### Offline development

```bash
ros2 launch rumblex_bringup test_launch.py robot:=nox
```

Useful arguments include:

| Argument | Default | Effect |
| --- | --- | --- |
| `enable_display` | `true` | Start the mesh state publisher and RViz. |
| `enable_simulated_teleop` | `true` | Start the mouse and keyboard controller GUI. |
| `enable_simulated_hmi` | `true` | Start the simulated HMI GUI. |
| `map` | `rumblex_navigation/maps/house_contour.yaml` | Select an occupancy map YAML file. |
| `simulate_lidar` | `true` | Generate offline `scan_1d` readings from map walls. |
| `publish_test_map_tf` | `true` | Publish the initial `map` to `odom` transform and supporting-foot odometry. Disable when supplying external odometry/localization. |
| `start_x`, `start_y`, `start_yaw` | `-2.26`, `-2.13`, `0.0` | Initial map pose in metres and radians. |
| `wall_height` | `1.0` | Map wall height in metres. |
| `autostart_listening` | `false` | Activate speech-to-text listening when the brain starts. |
| `use_sim_time` | `false` | Use a simulation clock. |
| `movement_joint_states_topic` | `joint_states` | Output topic for offline movement joint states. |
| `publish_gazebo_map` | `false` | Publish the map description used to spawn Gazebo walls. |

### Gazebo development

```bash
ros2 launch rumblex_bringup test_gazebo_launch.py robot:=nox
```

This entry point exposes `robot`, `start_x`, `start_y`, and `start_yaw`, with the
same defaults as above. It includes the offline test stack with its remaining
defaults and starts the Gazebo mesh simulation and map wall spawner.

To inspect the arguments exposed by any entry point, use, for example:

```bash
ros2 launch rumblex_bringup target_launch.py --show-args
```
