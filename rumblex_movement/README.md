# Movement units

Kinematics uses [mp-units](https://mpusz.github.io/mp-units/2.5/) for link lengths,
squared lengths, and angles. `units.hpp` exposes `m`, `mm`, `deg`, and `rad` plus
the `Length`, `Area`, and `Angle` quantity types. For example, `units::Length length =
50.0 * units::mm` represents the same length as `0.050 * units::m`.
Extract numbers explicitly with `.numerical_value_in(unit)` at existing interfaces.

`CPosition` coordinates and `CTorsoCenterOffset` coordinates are `Length`
quantities. `COrientation` members (`roll`, `pitch`, `yaw`), `CLegAngles`
members (`torso_coxa`, `coxa_femur`, `femur_tibia`), and the torso offset `psi` are `Angle`
quantities. Existing YAML keys, ROS parameter names, servo IDs, and URDF joint
names retain their `coxa`/`femur`/`tibia` naming at the interfaces.
Torso poses and center offsets exclude the head, whose orientation is stored
separately. Internal names use `torso`; existing ROS `body_pose` fields,
`SEQUENCE_BODY_ROLL` IDs, body parameter keys, and YAML `body` keys remain compatible.
Toe positions are stored in `toe_pos_`; methods use `Toe` in their names.
Existing `footPositions` YAML keys and ROS parameter roots retain their names
for compatibility.
`CLeg` stores its joint angles in `angles_`. Arithmetic and
interpolation preserve units, and assignments reject bare numbers or quantities
of the wrong dimension. For example:

```cpp
CPosition toe;
toe.x = 50.0 * units::mm;
CLegAngles joints;
joints.torso_coxa = 1.0 * units::rad;
auto degrees = joints.torso_coxa.numerical_value_in(units::deg);
```

Quantity constructors support mixed compatible units. The existing double
constructors interpret lengths as metres and angles as degrees for configuration
and message adapters. Head commands and position tolerances take quantities.
Gait displacements, stored head angles, and length/angle parameters also retain
their units. Units are attached when loading parameters, so gait updates need no
repeated conversions. Kinematics uses typed trigonometry and `mp_units::hypot`;
length ratios remain dimensionless quantities through clamping and inverse trig.

Robot YAML parameters and position/pose interfaces continue to use metres;
movement orientations and servo targets use degrees. The ROS `joint_states`
publisher converts angles to radians. The angular system gives angles their own
dimension, preventing accidental use as lengths or plain scalars.

Building requires C++23 and CMake 3.25 or newer. CMake uses an installed mp-units
2.5 package when available, otherwise downloads version 2.5.0 with a SHA256 check.
The fallback disables optional contract checks to avoid a GSL dependency; it uses
the compiler's standard formatting support or requires fmt if unavailable.
mp-units is managed by CMake, so no additional rosdep key is needed.

For an offline build, install mp-units first and add its prefix to
`CMAKE_PREFIX_PATH`, or supply a local release checkout:

```bash
colcon build --packages-select rumblex_movement \
  --cmake-args -DFETCHCONTENT_SOURCE_DIR_MP-UNITS=/path/to/mp-units-2.5.0
colcon test --packages-select rumblex_movement --event-handlers console_direct+
colcon test-result --verbose
```
