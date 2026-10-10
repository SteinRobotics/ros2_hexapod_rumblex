# brain

- requests navigation goals
- contains system checks

## Physical quantities

Brain uses the shared mp-units types from `rumblex_utils/units.hpp` for lengths,
angles, durations, linear and angular velocities, voltages, and temperatures.
Gait `start`/`updateTimed` methods and movement request durations require typed
seconds, for example `gait.start(1.0 * units::s, direction)`. Continuous gaits
accept `brain::Velocity`, whose linear components are in m/s and angular
components are in rad/s. Angles retain a distinct dimension, so angular velocity
cannot be substituted for linear velocity.

ROS messages and YAML retain their existing numeric formats: metres, seconds,
degrees for poses, radians/second for Twist angular velocities, volts, and Celsius.
`Velocity::fromMsg`/`toMsg` handle the velocity boundary; `limitVelocity` validates
all six ROS components before producing a limited planar velocity. ROS and
steady-clock timestamps remain clock types, with durations converted at the
clock boundary.

Temperature readings and thresholds are Celsius points, created with
`units::celsius(value)` and converted for display with `units::inCelsius(value)`.
Filtering adds a fraction of the sample-minus-previous temperature difference to
the previous reading. The initial filtered temperature remains 0°C; temperature
errors use the raw sample, while voltage errors use filtered voltage.

Normalized trajectory progress, filter coefficients, joystick deflection, gait
selection thresholds remain scalars. Gait timing,
velocity limits, robot-specific configuration, and monitoring thresholds are
unchanged. See [MOVEMENT.md](MOVEMENT.md) for the motion pipeline and build setup.
