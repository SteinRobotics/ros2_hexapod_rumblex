# Robot sensor pipelines

`rumblex_perception/config/<robot>/sensors.yaml` selects the lidar integration package
and whether head-sweep processing is needed. Drivers publish relative topic names,
so ROS namespaces and remappings remain available. Sensor mounting transforms belong
to `rumblex_description`; neither lidar integration publishes its own mounting TF.

The bringup sensor launch forwards to `rumblex_perception/launch/sensors_launch.py`.
Gazebo uses that shared launch directly, keeping its dependencies independent of bringup.

| Robot | Driver | Processing | Navigation input |
| --- | --- | --- | --- |
| Nox | `rumblex_lidar_1d`: Garmin LIDAR-Lite v3 | `rumblex_perception`: `scan_1d` (`Range`) plus head position → `scan` | `LaserScan` |
| Nira | `rumblex_lidar_2d`: upstream YDLIDAR T-mini driver | Native planar scan; no head movement | `LaserScan` |

## Launching

```bash
# Drivers only (does not move the head).
ros2 launch rumblex_bringup sensors_launch.py robot:=nox
ros2 launch rumblex_bringup sensors_launch.py robot:=nira

# Nox driver plus active head sweep; requires the movement stack/joint feedback.
ros2 launch rumblex_bringup sensors_launch.py robot:=nox enable_head_scan:=true

# Process an existing real or simulated Nox Range source without another driver.
ros2 launch rumblex_bringup sensors_launch.py robot:=nox enable_driver:=false enable_head_scan:=true

# Navigation is separate and requires scan, odometry, and the movement stack.
ros2 launch rumblex_navigation navigation_launch.py robot:=nox
```

`target_launch.py enable_navigation:=true` composes the selected driver, optional
head scanner, and navigation. With navigation disabled, Nox's head does not sweep.
Standalone navigation no longer starts a head scanner. Nira sensor/navigation profiles
are supplied, but complete Nira bringup still requires the other hardware profiles
(including its IMU configuration).

## Nira dependency and configuration

Install/build [YDLIDAR-SDK](https://github.com/YDLIDAR/YDLidar-SDK) and
[ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble)
following the upstream instructions. The integration expects the ordinary ROS node
`ydlidar_ros2_driver_node` used on the upstream `humble` branch. Compatibility with
ROS Lyrical and the physical T-mini must be checked on the target hardware.
This repository declares the driver as an external runtime dependency and does not
vendor it. A Nox-only installation can skip the `ydlidar_ros2_driver` rosdep key.

`rumblex_lidar_2d/config/nira/lidar.yaml` uses the upstream
[Tmini settings](https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/humble/params/Tmini.yaml),
with `frame_id: lidar_link` matching the existing Nira URDF. Verify the serial port
(`/dev/ttyUSB0` by default), access permissions, and scan orientation on the robot.
Use `params_file:=/absolute/path/lidar.yaml` with either direct lidar launch to override
its configuration. No driver or hardware installation is performed by the launch.

Navigation accepts sensor-data QoS (including best-effort vendor scans) and rejects
finite measurements outside the scan's advertised range bounds. Both robots start
with conservative navigation tuning; Nira's values require hardware tuning.

## Existing head-sweep limitations

The moved Nox scanner preserves the existing algorithm and direct
`single_servo_request` control. It assumes a level, nearly stationary robot and uses
the latest range/joint readings; timestamp synchronization, accurate sweep geometry,
motion compensation, stale-scan handling, and head-control arbitration are not
implemented by this package refactor. Its assembled `base_link` scan remains an
approximation. Avoid concurrent head-command sources while sweeping. The offline
wall-ray simulator remains a Nox single-beam simulator, not a Nira planar scanner.
Gazebo keeps the Nox head processor when navigation is enabled, but its existing
`lidar_scan` bridge publishes a single-ray `LaserScan`; a `Range` adapter/source is
still needed to feed `scan_1d`. This refactor does not implement that simulator adapter.

## Migrating existing workspaces

`rumblex_lidar` was renamed to `rumblex_lidar_1d`; update direct launch commands and
rebuild/source the new packages. The Garmin submodule moved with the package;
run `git submodule update --init --recursive` after updating your checkout.
`node_head_scan` and its parameters moved from navigation to perception. Old installed
package artifacts can remain in an existing install directory; use a fresh build/install
base when validating the migration.
