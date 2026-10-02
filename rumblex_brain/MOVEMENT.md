# Body-pose movement pipeline

Brain owns gait selection, trajectories, velocity/head/torso inputs, and gait
transitions. `MovementRequest` is now an internal C++ request with the existing
behavior IDs. The coordinator receives gait changes through a local callback.
Gait updates run at 10 Hz; the coordinator continues to run at 20 Hz.

Movement subscribes to `cmd_movement` (`rumblex_interfaces/msg/BodyPose`) and
forwards all six toe targets and the torso pose to `CKinematics::moveTorso`, and
the head orientation to `CKinematics::setHeadOrientation`. Its 10 Hz update sends
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
