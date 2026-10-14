# Nira sensor simulation

## BNO055 IMU

Nira's `imu_link` follows the CAD board origin at `(0, -0.002, 0.0185)` metres
from `base_link`. Its roll/pitch/yaw is `(pi, 0, -pi/2)`, equivalent to the
CAD's 180-degree Y flip followed by a 90-degree Z rotation. Local X points
toward robot -Y, local Y toward robot -X, and local Z downward. The mesh torso
already contains the board; the primitive model uses the CAD board envelope.
This registration describes the board model, not a measured chip location or
verified hardware axis-remap setting.

Both Nira variants include an ideal Gazebo IMU when `use_sim:=true`, and both
supplied worlds load the Imu system. Both simulation launches bridge
`/imu/data` as `sensor_msgs/Imu` with frame `imu_link`, at a nominal simulation
rate of 100 Hz. Orientation uses Gazebo's ENU world reference. This models
orientation, angular velocity and acceleration, not the BNO055's calibration,
magnetic disturbances, temperature, or fusion algorithm. See the
[Gazebo IMU reference](https://gazebosim.org/api/sensors/10/classgz_1_1sensors_1_1ImuSensor.html).

Acceleration includes gravity: a level stationary robot reports approximately
`(0, 0, -9.8)` m/s² in this upside-down sensor frame. The installed hardware
driver publishes gravity-removed acceleration on `/bno055/imu`, so simulation
data deliberately uses `/imu/data`; it is not a direct replacement for that
driver topic or the navigation node's subscription.

The saved RViz configuration includes **IMU mounting axes** attached to
`imu_link` (X red, Y green, Z blue). This displays the TF mounting frame, not
measured attitude. Inspect live measurements with:

```bash
ros2 topic echo /imu/data sensor_msgs/msg/Imu --once
```

Headless checks of both variants verified ROS delivery, the frame ID, zero
stationary angular velocity, downward local-Z acceleration and a quaternion
equivalent to `(x, y, z, w) = (0.7071, -0.7071, 0, 0)`. Regression tests also
check that the sensor pose survives Gazebo's fixed-joint reduction.

## T-mini lidar

Nira's torso-mounted YDLIDAR T-mini is a planar 2D lidar. Its `lidar_link`
origin is the CAD scan centre: `(0.093, 0, 0.0933)` metres from `base_link`,
with +X forward and +Z up. Both Nira model variants simulate one horizontal
360-degree scan with 400 samples at 10 Hz and a 0.03–12 m range, matching the
nominal settings in `rumblex_lidar_2d/config/nira/lidar.yaml`.

The mesh model includes the real housing and surrounding body geometry. Its
rear body obstructs part of the scan, leaving approximately the CAD's intended
270-degree clear sector. The primitive model has a simplified body and lidar
housing; its body visual stays below the scan plane. Its collision envelope
and inertia remain coarse estimates. Use the mesh model to assess occlusion.

Both simulation launches bridge `/lidar_scan` to ROS `sensor_msgs/LaserScan`,
with frame ID `lidar_link`. Both supplied worlds load Gazebo's Sensors system.
The sensor setup follows the [Gazebo sensor configuration](https://gazebosim.org/docs/latest/sensors/).

After building and sourcing the workspace, run either:

```bash
ros2 launch rumblex_gazebo simulation_mesh.launch.py robot:=nira
ros2 launch rumblex_gazebo simulation_gazebo.launch.py robot:=nira
```

To view the running simulation in another terminal:

```bash
rviz2 -d "$(ros2 pkg prefix --share rumblex_description)/rviz/model.rviz" --ros-args -p use_sim_time:=true
```

The saved RViz LaserScan display uses `/lidar_scan` with Best Effort reliability,
so it accepts both reliable and sensor-data publishers. The separate `/scan_1d`
Range display is for Nox. Nira does not publish that topic. RViz-only model
display launches do not generate simulated measurements; use Gazebo for those,
or `visualization_rviz.launch.py robot:=nira enable_lidar:=true` with hardware.

Validation: headless Gazebo checks in `simple_room.sdf` produced 400 horizontal
samples for both variants. The mesh scan passed through `ros_gz_bridge` with
frame `lidar_link`; at the room origin, forward returns were approximately
1.407 m, with rear body returns approximately 0.067 m. These checks held the
robot static to isolate the sensor from locomotion and controller behavior.
