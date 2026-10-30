# Body-pose movement pipeline

Brain owns gait selection, trajectories, velocity/head/torso inputs, and gait
transitions. `MovementRequest` is now an internal C++ request with the existing
sequence IDs. The coordinator receives gait changes through a local callback.
Locomotion, pose/orientation trajectories, and the coordinator run at 50 Hz.
Other gestures retain their 10 Hz cadence. Pose, yaw and torso-roll trajectories
use elapsed seconds, so finer sampling preserves their duration.

Movement subscribes to `cmd_movement` (`rumblex_interfaces/msg/BodyPose`) and
forwards all six toe targets and the torso pose to `rumblex_geometry::CBodyModel::moveTorso`, and
the head orientation to `rumblex_geometry::CBodyModel::setHeadOrientation`. Its 50 Hz update sends
the latest target with a duration based on the pose delivery interval (normally 20 ms
for locomotion/orientation and 100 ms for other gestures). Without a new command it
publishes state but does not initiate motion.

## Request completion and stopping

`IHandler::requestFinished()` means the current request no longer blocks the
scheduler. It does not imply success or stopped motion. The planner's
`requestGroupFinished()` caches the aggregate from its last handler update,
before dispatching the next request group.

Finite movement requests finish when their selected gait reaches `Stopped`.
Repeating requests finish when their dwell deadline expires, measured from gait
activation; this does not request a stop. Repeating requests reset that deadline
when resubmitted. Dwell deadlines continue to use the ROS clock.

`HOLD_DISTANCE` and `LEVEL_TORSO` are finite `ISequenceGait` implementations,
selected by the same `RequestMovementType` as other gaits. The former accepts an
optional C++ `MovementRequest::distance_m` field which is required for that gait;
the latter has no distance target. Each gait owns its sensor subscription, control
state, startup checks, duration, and stopping. Holding distance composes the
existing pose and walking gaits with head scanning disabled; leveling retains a
standing set of toe targets. There is no feedback overlay in the movement handler.
`hold_distance_control` owns range acquisition, beam validation, distance settings,
and speed calculation. `level_torso_control` owns IMU acquisition, orientation
transforms, leveling settings, and tilt calculation. Each has a separate reader
(`RangeFeedback` or `ImuFeedback`); there is no sensor-mode switch. Small stateless
parameter, timestamp, quaternion, and update-interval checks are shared privately.
Autonomous gaits report velocity through `IGait::commandedVelocity()`. On a sensor
error, the gait logs the reason, requests a stop, and sets a one-shot sequence-abort
flag. The planner consumes it immediately after dispatch or update and removes the
sequence's remaining groups. No separate failure result or notification API is used.

Cancellation clears pending/tracked movement requests and immediately marks the
request finished. The controller continues the graceful stop and manages any
subsequent gait handoff at the rest endpoint. Scheduling does not wait for rest.

`stopRequested()` reports outstanding controller stop intent, including a gait
replacement or cancellation of a finite sequence that naturally runs to its end.
It clears at rest, on a new gait activation, or when an accepted same-gait request
withdraws the intent. It does not describe autonomous gait settlement from zero
velocity. `motionStopped()` reports only that the active gait is at its planned
rest endpoint; a replacement gait may still be pending. An active continuous
gait holding an unchanged pose need not report `Stopped`. Neither query measures
hardware motion, and motion state is meaningful only after initial pose feedback.

## Message contract

```text
bool has_head_orientation
Orientation head_orientation
bool has_torso_pose
Pose torso_pose
bool has_toe_positions
geometry_msgs/Vector3[6] toe_positions
```

Positions are metres and orientations are degrees. Toe indices are right front,
right middle, right back, left front, left middle, left back. Targets use the
robot coordinate frame supplied to `moveTorso`, before the torso transform.
Presence flags make each component optional. Supplied components replace their
whole target; omitted components retain the current pose. Non-finite values in
supplied components reject the command as a unit. Empty commands are rejected.
Brain trajectory samples and feedback always supply all components.

`MovementRequest::body_pose` uses this same message. A `SINGLE_POSE` request
resolves missing components at actual gait activation, then interpolates the
head, torso, and all six toes together. The target is captured once per segment;
pending requests cannot change an in-flight endpoint. Stand-up and laydown use
the same representation for their presets. See [sequence configuration](config/sequences/README.md)
for YAML, loops, snippets, and compatibility details.

The command topic uses reliable, volatile delivery with depth 1.

`body_pose_actual` carries the same message with reliable, transient-local
 delivery and depth 1. Brain waits for its first complete valid sample before starting
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
The supporting-foot estimator publishes `/odom`, `odom → base_link`, and
`movement_velocity_estimated` (`geometry_msgs/TwistStamped`, in `base_link`). It
uses applied kinematic targets in `body_pose_actual` and receive timestamps,
assuming supporting feet do not slip. This is an estimate, not sensor-measured
robot speed. Torso changes contribute body-frame twist, including outside walking.
Invalid samples, clock discontinuities and gaps over 0.5 seconds clear twist.
Navigation consumes `/odom`; it never integrates its own commands. Its launch
starts the estimator unless `use_external_odometry:=true`. When using the test
bringup estimator, pass that option to navigation to keep one odometry publisher. Movement types no longer cross the ROS boundary.
`MovementRequest.msg`, `ContinuousMovementUpdate.msg`, `cmd_movement_update`, and
`movement_type_actual` have been removed. Rebuild and restart all consumers
together; external publishers must migrate to the BodyPose presence flags and
`head_orientation` field.

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
request cannot move an existing segment's destination. Pose transitions retain
their minimum duration of one second, sampled at 50 Hz.

Walking and running share `CStridePlanner`. Patterns specify ordered swing
leg groups, step reach, lift height, and head amplitude. A segment
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
explicit sequence stop cannot be undone by a stale velocity command.

Velocity inputs use metres/second for planar translation and radians/second for
yaw. Joystick inputs are normalized; full stick maps to `max_velocity_linear`
(default 0.10 m/s in both robot profiles) or `max_velocity_rotation` (0.40 rad/s,
about 23 degrees/s).
All velocity requests are limited by planar vector magnitude and yaw magnitude;
non-finite requests are rejected. Linear direction is preserved when limiting.
The authoritative limits also drive gait selection. Legacy
`gait.move_combined.max_*_velocity_*` aliases must match them when supplied.

Gait demand is the larger of normalized planar speed and normalized yaw speed.
The wave/ripple and ripple/tripod thresholds remain 0.3 and 0.6, with hysteresis.
A full single-axis input selects tripod directly at the next contact boundary.
Running also uses the physical velocity contract and alternating tripod support.

Each stance follows a planar rigid-body transform derived from the requested
twist and segment duration. Rotational foot speed uses the foot radius in metres.
Segment duration is derived from step reach and foot speed, bounded to 0.2–1.0 s;
slow commands shorten stride length rather than prolonging command latency.
Elapsed time drives the trajectory; sampling overshoot carries into the next
segment while retaining an explicit touchdown sample. Normal commands change at
the next contact boundary, within one second. Gait transitions can temporarily
limit foot reach before all legs enter their new schedule.

Segments start and end at rest. Requested speed matches the average over complete
settled cycles; instantaneous twist varies within each segment. Phase gains and
rotation weights remain accepted for compatibility but no longer scale physical
locomotion speed. Hardware servo speed limits and slip can reduce realized speed;
validate those with measured distance/time before claiming hardware accuracy.

Clap and high-five finish their current sequence/raise before returning;
leg-wave, waiting, and torso-roll finish their closed cycle. Look and watch
return to their captured origins. Diagnostic leg motions now interpolate their
joint targets rather than issuing instantaneous steps. Continuous pose finishes
its current segment and holds the endpoint on stop instead of snapping back.
Stop requests are graceful motion requests, not emergency actuator stops.

The obsolete `gait.running.velocity_filter_alpha`, `gait.running.flight_fraction`,
`gait.move_combined.velocity_filter_alpha`, and
`gait.move_combined.transition_phase_span_rad` keys and the walking/running
`velocity_to_phase_gain` settings have been removed. Remove these unused settings
from custom configuration files. No new dependencies or required parameters are
introduced beyond the existing brain velocity limits.

Continuity refers to the planned kinematic state, initialized from
`body_pose_actual`, not continuously measured foot positions. Contact, servo
tracking, and balance require simulation/hardware validation. The test suite
covers trajectory derivatives, displaced startup poses, every sequence pair,
every walking-pattern pair, direction reversal during swing, settlement,
resumption, and gesture cancellation regressions.

## Velocity validation

Validated on ROS 2 Lyrical with offline servo output on 2026-10-05. The full
brain → movement → supporting-foot odometry pipeline was warmed up before each
six-second measurement. Both cases used tripod (three airborne feet).

| Input | Requested speed | Estimated distance/time speed |
| --- | --- | --- |
| Teleop left stick horizontal = 1.0 | 0.1000 m/s sideways | 0.1002 m/s |
| `cmd_vel.linear.x = 0.1` | 0.1000 m/s forward | 0.1004 m/s |

Focused tests cover translation, rotation, mixed commands, hysteresis, contact
boundaries, and update jitter. Supporting-foot displacement matches settled
commands within 2% across walking and running. This validates commanded
kinematics, not hardware tracking. For hardware acceptance, measure travelled
distance over timed settled cycles at low, medium and maximum commands in both
linear directions, and measure yaw over time for rotation. Compare those results
with `/movement_velocity_estimated`; servo limits or slip require hardware
calibration rather than treating target-based odometry as a sensor measurement.

Orientation trajectories now publish at 50 Hz without shortening their requested
durations. The brain uses a fixed scheduling deadline and processes incoming
commands before planning. Servo output rounds to the nearest 0.24-degree tick
and suppresses only unchanged tick targets, replacing the previous 0.49-degree
deadband. One-tick changes remain eligible at every sample; idle targets do not
restart actions. The existing actuator speed limit remains enforced.

Joystick pose mode activates `CONTINUOUS_POSE` and submits a single movement
request containing the optional torso/head components. A neutral stick preserves the existing target, as before;
orientation targets no longer leave the previous walking gait selected.

Follow-up offline validation after raising the turning limit: a full right-stick
command requested 0.4000 rad/s and supporting-foot odometry estimated 0.4012 rad/s
(22.99 degrees/s), using tripod. Joystick pose-mode activation produced 51 samples
for a one-second head trajectory to 14 degrees, with a largest adjacent yaw step
of 0.535 degrees. All 197 package tests passed, including elapsed-time orientation
checks, one-tick servo commands, and pose-mode request grouping.
