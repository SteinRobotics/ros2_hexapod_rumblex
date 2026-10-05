# House ground-floor map

`house_contour.yaml` and `house_contour.pgm` describe the house ground floor from
the supplied ground-floor plan. The printed dimensions are 8.90 × 9.30 m,
with 0.14 m exterior walls. The footprint is rectangular, aligned with the
image: +x points right and +y points toward the garage at the top of the plan.
The house center is (0, 0); this is not a geographic coordinate system.

The grid uses 0.02 m cells, with a 0.50 m unknown margin on every side:
495 × 515 pixels. Black (0) is occupied, white (254) is free, and gray (205)
is unknown. Windows and exterior doors are closed portions of the contour.
Interior walls divide the upper room, WC, kitchen, hall, and lower rooms.
Interior door openings and the wide passage between the lower rooms remain
free. The entire stair enclosure, including its landing, is occupied so that
navigation cannot route across the stairs. Chimney blocks are occupied.
Garage, garden, furniture, and door swing arcs are omitted.

Interior positions are approximate traces from the scanned plan, not surveyed
measurements. Check wall positions and doorway clearance against the house
before navigating. The map retains its existing filename for launch compatibility.

To adjust the geometry, edit the metre-based rectangles in
`scripts/generate_house_map.py` and regenerate with:

```bash
python3 rumblex_navigation/scripts/generate_house_map.py
```

`rumblex_bringup/launch/test_launch.py` loads this map automatically using
`map_launch.py`, which activates the map server and publishes `/map` in the
`map` frame using wall time. Both packages already install their launch/map
directories. After rebuilding and sourcing the workspace:

```bash
ros2 launch rumblex_bringup test_launch.py
# Map server alone:
ros2 launch rumblex_navigation map_launch.py
# Override the map:
ros2 launch rumblex_bringup test_launch.py map:=/absolute/path/to/map.yaml
```

Runtime dependencies are `nav2_map_server` and `nav2_lifecycle_manager`,
already declared by `rumblex_navigation`. The map publisher does not provide
localization or a `map` to `odom` transform. To inspect the map alone in RViz,
select `map` as the fixed frame and `/map` as the Map display topic. Align
the robot's pose with this map before using it for navigation.

The offline `test_launch.py` uses `map` as the RViz fixed frame and publishes an
identity `map` to `odom` transform. Its `offline_odometry.py` node estimates
planar motion from supporting toe targets in `/body_pose_actual` during
`CONTINUOUS_MOVE` and `CONTINUOUS_RUNNING`, selected through `/movement_name`.
Standing toe heights come from the selected robot's anatomy profile; the default
`support_tolerance_m` is 0.001 m. At least two distinct feet must support in both
successive samples. Missing support or a feedback gap over 0.5 s holds the planar
pose and reanchors the next sample.

The node composes this planar estimate with the torso translation and rotation
for `/odom` and the dynamic `odom` to `base_link` transform. Stationary torso
gestures rotate or translate the body without accumulating locomotion.
`node_offline_visualization` publishes `/visualization_joint_states`, applying
full inverse torso transforms to toe targets before display-only inverse
kinematics. RViz uses these states; hardware `/joint_states` and servo commands
remain unchanged.

Commands routed through the brain continue to drive the legs. Send a zero Twist
to stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.02}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

This estimates contact from gait targets, without measured foot-contact feedback,
collision physics, or localization. The body's displayed speed follows the actual
gait stride rather than `/movement_velocity`. Settled cycle-average speed now
matches physical commands; instantaneous speed varies within each segment.
`/movement_velocity_estimated` reports stamped body-frame twist from the same
estimate. Navigation consumes `/odom` rather than creating command-based odometry.
When the test estimator is already running, launch navigation with
`use_external_odometry:=true`.
Inconsistent support trajectories and CAD versus kinematic geometry can leave
some visible foot sliding. Use Gazebo for physics-based validation.
Disable both test transforms and offline odometry with
`publish_test_map_tf:=false` when providing an actual odometry/localization
transform tree; two publishers must not own the robot's pose.

The test launch extrudes occupied cells into 2 m high walls (from ground level
to z=2 m) on `/test_map_walls`, displayed automatically in the model RViz config.
It also simulates the Nox single-beam lidar on `/scan_1d` by intersecting the
lidar frame's forward ray with these same boxes, including head yaw and pitch.
Unknown and free cells remain empty; blocked stairs are extruded like walls.
The occupancy map itself remains 2D. This is an offline geometric range simulator,
not Gazebo collision geometry or a simulation of the Nira scanning lidar.
Use `wall_height:=2.0` to change height and `simulate_lidar:=false` when a hardware
lidar provides `/scan_1d`. Range readings wait for the map and lidar TF to arrive.
The wall publisher adds runtime dependencies on `rclpy` and `visualization_msgs`.

Check the map asset with:

```bash
python3 -m unittest discover -s rumblex_navigation/test -p 'test_*.py'
```
