# Lidar perception

Formerly `rumblex_perception`. This package owns Nox's `node_head_scan`, converting
head-swept 1D range measurements into a planar scan. Its topics, parameters, and
head commands are unchanged. It has no camera or face-inference dependency.

```bash
ros2 launch rumblex_perception_lidar perception_launch.py robot:=nox
```

Robot-wide sensor profiles and composition now live in `rumblex_bringup`:

```bash
ros2 launch rumblex_bringup sensors_launch.py robot:=nox enable_head_scan:=true
ros2 launch rumblex_bringup sensors_launch.py robot:=nira
```

Rebuild/re-source the workspace after the rename. Old direct package launch
commands must use the new package name; the bringup sensor entry point is stable.
Use a fresh build/install overlay when verifying the migration so stale installed
copies of the old package cannot conceal missing dependencies.
