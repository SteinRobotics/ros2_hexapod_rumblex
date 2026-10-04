# Body-pose movement pipeline

Brain owns gait selection, trajectories, velocity/head/torso inputs, and gait
transitions. `MovementRequest` is now an internal C++ request with the existing
behavior IDs. The coordinator receives gait changes through a local callback.
Gait updates run at 10 Hz; the coordinator continues to run at 20 Hz.

Movement subscribes to `cmd_movement` (`rumblex_interfaces/msg/BodyPose`) and
forwards all six toe targets and the torso pose to `rumblex_geometry::CBodyModel::moveTorso`, and
the head orientation to `rumblex_geometry::CBodyModel::setHeadOrientation`. Its 10 Hz update sends
the latest target to the servos with a 100 ms duration. Without a new command it
publishes state but does not initiate motion.

## Message contract

```text
Orientation head_pose
Pose torso_pose
geometry_msgs/Vector3[6] toe_positions
```

Positions are metres and orientations are degrees. Toe indices are right front,
right middle, right back, left front, left middle, left back. Targets use the
robot coordinate frame supplied to `moveTorso`, before the torso transform.
Every message carries all six toes. Non-finite messages are rejected as a unit.
The command topic uses reliable, volatile delivery with depth 1.

`body_pose_actual` carries the same message with reliable, transient-local
 delivery and depth 1. Brain waits for its first valid sample before starting
queued movement and its completion timer. Subsequent feedback does not overwrite
the running trajectory. This is movement's kinematic estimate, initialized using
available servo angles; it is not continuous measured toe tracking. A brain
restart initializes from movement's current estimate. A movement restart during
an active gait does not perform an automatic trajectory resynchronization.

Shared geometry lives in `rumblex_utils`. Brain and movement own independent pose
models. Joint-based clap, high-five, and leg-test trajectories are converted to
toe targets with the current torso transform so the receiver reconstructs the
joint angles. Stored toe positions remain in the command frame.

## Configuration and telemetry

Both nodes load anatomy from `rumblex_description/config/<robot>/anatomy.yaml`.
Brain loads gait settings from `rumblex_brain/config/<robot>/gait.yaml`; movement
loads its servo description. Nox and Nira retain separate profiles.

The coordinator requires `max_velocity_linear`, `max_velocity_rotation`,
`body_factor_height`, `joystick_deadzone`, `min_body_height`, and `max_body_height`
from the robot profile. Values must be finite, velocity limits positive, the
height factor nonnegative, the deadzone in `[0, 1)`, and height limits bracket zero.
Movement-request locks expire in the coordinator update loop using steady wall
time, independently of paused simulation time. New timed requests replace the
previous deadline; walking and running clear it.

Brain publishes `movement_name` (`std_msgs/String`) for HMI and
`movement_velocity` (`geometry_msgs/Twist`) as commanded-velocity telemetry.
Offline odometry instead estimates motion from supporting toes in `body_pose_actual`. Velocity is zero
outside walking/running. Movement types no longer cross the ROS boundary.
`MovementRequest.msg`, `ContinuousMovementUpdate.msg`, `cmd_movement_update`, and
`movement_type_actual` have been removed. Rebuild and restart all consumers
together; external publishers must migrate to the complete BodyPose contract.

## Build

Build the interface, shared utilities, and both nodes together. C++23 and CMake
3.25 or newer are required. `rumblex_utils` locates mp-units 2.5 or fetches the
pinned 2.5.0 release. An offline build can supply
`-DFETCHCONTENT_SOURCE_DIR_MP-UNITS=/path/to/mp-units-2.5.0`.

```bash
colcon build --packages-up-to rumblex_brain rumblex_movement
colcon test --packages-select rumblex_utils rumblex_brain rumblex_movement rumblex_navigation --event-handlers console_direct+
colcon test-result --verbose
```

## Motion trajectory and handoff contract

Every gait captures the current command-frame pose on entry. `start`,
`requestStop`, and `cancelStop` do not move the model. A gait only reports
`Stopped` at a rest endpoint. The controller finishes the outgoing trajectory,
then starts the incoming gait from that endpoint. Fixed poses and gestures use
`trajectoryProgress(t) = 10t³ - 15t⁴ + 6t⁵`, with zero velocity and acceleration
at both ends. Pose requests are captured once per finite segment; changing a
request cannot move an existing segment's destination. Pose transitions have a
minimum duration of one second at the 10 Hz command rate.

Walking and running share `CStridePlanner`. Patterns specify ordered swing
leg groups, step length, lift height, head amplitude, and phase gain. A segment
captures the actual current planned toe, torso, and head poses, then generates
a complete swing/support trajectory. Swing lift is `64t³(1-t)³`, also with zero
endpoint velocity and acceleration. Torso and head targets follow the same
segment time law. No motion-command filter or output blending is used.

Speed, direction, and pattern changes are accepted at touchdown. The scheduler
retains its cycle phase when changing the number of groups. Thus an airborne
leg completes its trajectory before a new pattern starts; rapid requests cannot
repeatedly restart an entry blend. A stop completes the current swing, then
places each group at standing through finite swings while other toes retain
their current positions. Cancellation finishes any segment already in progress.
Zero velocity triggers settlement and new velocity resumes locomotion; an
explicit behavior stop cannot be undone by a stale velocity command.

There is a deliberate timing tradeoff: segments start and end at rest, and take
at least ten updates (one second). This bounds sampling resolution and produces
C2 position trajectories, but limits top speed and adds request latency. Phase
gains retain their per-update units; increasing them cannot bypass the minimum
segment duration. Running uses the same contact-preserving alternating tripod
planner with its own parameters; the previous discontinuous flight-overlap
formula is removed. It is no longer an aerial running gait.

Clap and high-five finish their current sequence/raise before returning;
leg-wave, waiting, and torso-roll finish their closed cycle. Look and watch
return to their captured origins. Diagnostic leg motions now interpolate their
joint targets rather than issuing instantaneous steps. Continuous pose finishes
its current segment and holds the endpoint on stop instead of snapping back.
Stop requests are graceful motion requests, not emergency actuator stops.

The obsolete `gait.running.velocity_filter_alpha`, `gait.running.flight_fraction`,
`gait.move_combined.velocity_filter_alpha`, and
`gait.move_combined.transition_phase_span_rad` keys remain accepted for older
configuration files but are unused. No new dependencies or required parameters
are introduced.

Continuity refers to the planned kinematic state, initialized from
`body_pose_actual`, not continuously measured foot positions. Contact, servo
tracking, and balance require simulation/hardware validation. The test suite
covers trajectory derivatives, displaced startup poses, every behavior pair,
every walking-pattern pair, direction reversal during swing, settlement,
resumption, and gesture cancellation regressions.
