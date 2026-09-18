# Robot profiles

RumbleX contains shared ROS 2 packages for multiple hexapods. The physical robots have separate profiles:

- `nox` is the existing robot and the default profile.
- `nira` is reserved for the planned second robot. Its files should be added when its hardware values are
  known; copying Nox's values would hide real hardware differences.

Launch the complete Nox stack with:

```bash
ros2 launch rumblex_bringup target_launch.py robot:=nox
```

The bringup launch forwards `robot` to component launches. Simulation, RViz, and direct component
launches accept the same argument. Each package stores only the settings it owns:

| Package | Profile files |
| --- | --- |
| `rumblex_brain` | `config/<robot>/parameter.yaml` |
| `rumblex_bringup` | `config/<robot>/bno055_params.yaml` |
| `rumblex_communication` | `config/<robot>/params.yaml` |
| `rumblex_movement` | `config/<robot>/anatomy.yaml`, `servo_description.yaml` |
| `rumblex_navigation` | `config/<robot>/navigation.yaml` |
| `rumblex_gazebo` | `config/<robot>/joint_controllers.yaml` |
| `rumblex_description` | `urdf/<robot>.urdf.xacro`, optionally `<robot>_mesh.urdf.xacro` |

To add Nira, create the corresponding `config/nira/` files and `nira.urdf.xacro`, then launch with
`robot:=nira`. Profile names must start with a lowercase letter and contain only lowercase letters,
digits, and underscores. A missing profile file fails at launch instead of silently using Nox values.
