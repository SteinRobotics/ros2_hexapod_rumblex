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

The offline `test_launch.py` additionally publishes an identity `map` to
`base_link` transform so the map can be displayed in the default RViz
`base_link` fixed frame. This places the robot at the map origin for visualization
only. Disable it with `publish_test_map_tf:=false` when providing an actual
odometry/localization transform tree; two publishers must not own the robot's pose.

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
