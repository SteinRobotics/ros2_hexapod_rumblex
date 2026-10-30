# Sequences and snippets

The brain loads every `.yaml` and `.yml` file directly in `config/sequences`,
ordered by filename. Each file defines a named sequence, optional voice/joystick
triggers, and actions. The former `normal_` name prefix has been removed: use
`body_roll`, `high_five`, `leg_wave`, `look_left`, `look_right`, `test_body`,
`test_legs`, and `watch`. Voice and joystick triggers are unchanged.
When updating an existing installation, remove the obsolete `normal_*.yaml` files
from its installed `config/sequences` directory before restarting the brain and
GUI; incremental installation may otherwise leave both catalogs present.

```yaml
name: example
trigger:
  voice: commandExample
  joystick: button_a
actions:
  - say: Starting
  - include: snippets/example.yaml
  - say: Finished
```

List entries run sequentially. Actions in the same entry are dispatched together;
the next entry waits for the handlers' completion rules. Music, speech, listening,
and chat retain their existing communication-status completion behavior.

## Body poses

A `body_pose` action is one movement request: `SINGLE_POSE`, duration, and the same
`BodyPose` target used by the ROS interface. The three optional components are
`head_orientation`, `torso_pose`, and `toe_positions`.

```yaml
actions:
  - body_pose:
      duration_s: 1.5
      torso_pose:
        position: {x: 0.025, y: 0.025}
        orientation: {roll: -6, pitch: 4, yaw: 4}
      head_orientation: {yaw: -10, pitch: -4}
  - body_pose:
      head_orientation: {}
```

Positions are metres and orientations are degrees. A supplied component replaces
its complete target; omitted coordinates within a supplied torso/head component
are zero. `{}` explicitly resets that component to neutral. Omitted components
retain their values from the current pose **when the gait starts**, after any
outgoing trajectory finishes. They are not resolved when the YAML is loaded.
The second action above resets the head while retaining the torso and toes.

At least one component is required. `duration_s` defaults to 1.0 seconds and must
be finite and at least one second, the existing pose trajectory minimum. Every
pose action consumes its duration, including repeated identical targets.

When provided, `toe_positions` maps all six leg names to `{x, y, z}` targets,
with all three coordinates required for every toe. Names are `right_front`,
`right_mid`, `right_back`, `left_front`, `left_mid`, and `left_back`. These are
targets before the torso transformation, matching `BodyPose.msg`. Absolute toe
positions still require all six legs; relative moves can name individual legs.
Toe motion uses the same interpolation and duration as head and torso motion.

The ROS message encodes optional components with `has_head_orientation`,
`has_torso_pose`, and `has_toe_positions`. YAML presence sets these flags automatically.
Published brain trajectory samples and `body_pose_actual` are complete snapshots
with all three flags set. Feedback consumers require complete snapshots.

## Relative pose movements and clap

`clap.yaml` now contains the choreography directly: shift the torso back 5 cm,
lift/lower the right and left rear toes by 5 cm, lift the front toes by 10 cm,
close/open them six times by 30 degrees, lower them and shift forward. Six cycles
match the former gait's actual counter behavior. Preparation/return actions take
one second each; close/open actions take half a second each (14 seconds total,
plus scheduler handoffs). The `commandClap` voice trigger and `clap` sequence name
are unchanged. The old `SEQUENCE_CLAP` movement selector is removed.

These actions use `movement: {name: SINGLE_POSE, ...}` with `pose_delta` instead
of `body_pose`. Deltas resolve when the action starts, including after a pending
outgoing movement completes, so no robot-specific stance coordinates are needed.

```yaml
- movement:
    name: SINGLE_POSE
    duration_s: 0.5
    pose_delta:
      torso_translation: {x: -0.05}
      toe_translation: {right_back: {z: 0.05}}
      coxa_rotation_deg: {right_front: -30, left_front: 30}
```

All delta fields are optional, but at least one must be supplied. Translation
vectors use metres in the existing BodyPose command frame; omitted coordinates
are zero. `toe_translation` and `coxa_rotation_deg` map leg names to offsets;
omitted legs have zero offset. Unknown or duplicate leg names are rejected.
`head_rotation` and `torso_rotation` contain additive `{roll, pitch, yaw}` offsets
in degrees. Omitted axes default to zero; these add to the starting Euler angles,
not a composed local-frame rotation. Translations and torso rotations resolve
together while keeping toe targets in the existing command frame.
Coxa rotations are degree offsets from the current joint angles, applied after
translations and torso rotations. Forward kinematics converts them to toe targets using the selected
robot's geometry. The trajectory smoothly interpolates toes to those endpoints;
the clap follows a chord rather than the former joint-space arc. Head pose and
torso orientation are retained unless their rotation offsets are supplied. `pose_delta` cannot be combined with `body_pose`
and requires a finite positive duration; unlike absolute poses, it supports
sub-second motions. Inverse deltas restore the starting pose within numerical
precision for reachable poses.

As with other YAML sequences, cancellation discards remaining actions and lets
the active pose finish. It does not run the remaining clap return actions.

## Joint targets and high-five

`high_five.yaml` raises the right-front leg to joint angles
`(-20, 50, 60)` degrees and looks forward with head pitch `-20` degrees over one
second, holds for two seconds, then restores the starting pose over one second.
The spoken announcement, `commandHighFive` voice trigger and `button_b` joystick
trigger are unchanged. Speech completion can delay the following action, as with
other grouped actions. The old `SEQUENCE_HIGH_FIVE` movement selector is removed.

`SINGLE_POSE` movements accept `joint_targets` keyed by leg name. Omitted legs
retain their current toe positions, without placeholder entries:

```yaml
joint_targets:
  right_front: {torso_coxa: -20, coxa_femur: 50, femur_tibia: 60}
```

Each supplied leg requires
all three finite angles in degrees: `torso_coxa`, `coxa_femur`, `femur_tibia`.
Targets use the selected robot's forward kinematics and current torso transform.
They can accompany a `body_pose` head/torso target, but cannot be combined with
explicit toe targets or `pose_delta`. As with clap, toes interpolate to the joint
pose endpoint rather than following the old joint-space arc.

Use `save_pose: true` on the outgoing pose action to capture the complete current
pose when it starts (after any pending motion completes). A later
`movement: {name: SINGLE_POSE, duration_s: 1.0, restore_pose: true}` returns to that
snapshot and consumes it. Restore actions cannot carry other pose targets or
save again. There is one saved-pose slot per controller; another save replaces
it, so nested save/restore pairs are not supported. A restore without a saved
pose is rejected. Saving/restoring requires a finite positive duration and uses
the ordinary absolute-pose minimum of one second.

Cancellation follows normal YAML sequence behavior: finish the active pose and
discard remaining actions, including the return. A new high-five invocation
captures its own starting pose, replacing any snapshot left by cancellation.

## Look left and right

`look_left.yaml` and `look_right.yaml` are two-action pose sequences.
The first action saves the starting pose and turns the head by 25 degrees and
torso by 20 degrees over 1.5 seconds (positive yaw for left, negative for right).
The second restores the saved pose over 1.5 seconds, without a hold. Toe positions,
torso translation and the original roll/pitch angles stay unchanged. The smooth
pose interpolation replaces the previous sinusoidal excursion.

```yaml
movement:
  name: SINGLE_POSE
  duration_s: 1.5
  save_pose: true
  pose_delta:
    head_rotation: {yaw: 25}
    torso_rotation: {yaw: 20}
```

Sequence names, spoken announcements, voice commands and shoulder-button triggers
are unchanged. Speech remains grouped with the outward motion and can delay the
return. Cancellation finishes the active pose and discards remaining actions,
including restoration, as with high-five. Angles now live in the sequence YAML;
the `gait.look.*` parameters and `SEQUENCE_LOOK` selector are removed. The watch
sequence continues to use its existing two-sided yaw gait.

## Other actions

Say `teste gang` to trigger `walk_demo.yaml` (`commandWalkDemo`). The sequence
stands up, walks forward at 0.03 m/s for three seconds, then switches to
`CONTINUOUS_POSE` with zero velocity to stop and hold the pose. Continuous gait
durations advance the sequence; include an explicit stop action after walking.

| Action | Example |
| --- | --- |
| `say` | `say: Hello` |
| `chat` | `chat: What can you see?` |
| `music` | `music: musicfox_hot_dogs_for_breakfast.mp3` or `music: STOP` |
| `listening` | `listening: true` |
| `movement` | `movement: {name: SEQUENCE_STAND_UP, duration_s: 1.5}` |
| `velocity` | `velocity: {linear: {x: 0.01}, angular: {z: 0.1}}` |
| `system` | `system: {turnOffServoRelay: true}` |

Existing mapping forms for speech/music/system requests remain available.
Velocity uses metres/second and radians/second. `movement` can carry a nested
`body_pose` with the same component fields; its duration goes on `movement`.
`body_pose` shorthand always selects `SINGLE_POSE`. Toe targets require that mode.
Continuous joystick commands retain previously supplied targets when a stick is
released while the same gait remains active.

## Sensor feedback gaits

The catalog includes these directly launchable presets. Each begins with a
1.5-second standup and ends with `CONTINUOUS_POSE` and zero velocity, stopping
gracefully while retaining the resulting pose.

| Sequence | Preset | Required sensor |
| --- | --- | --- |
| `hold_distance` | Maintain 0.6 m for 10 seconds | 1D lidar |
| `level_torso` | Level torso for 5 seconds | IMU |
| `waiting` | Waiting motion for 5 seconds | None |
| `run_demo` | Run forward at 0.03 m/s for 3 seconds | None |

These sequences have no joystick or voice triggers. The virtual controller adds
named buttons automatically from the installed catalog; they can also be launched
by publishing the exact name as `std_msgs/msg/String` on `sequence_request`.
Install updated catalog files as needed, then restart the brain and GUI.
Sensor failures retain the abort behavior described below, including stopping
generated movement and removing the invocation's remaining actions.

Two independent gaits use the existing `movement` action and `RequestMovementType`:

```yaml
name: sensor_demo
actions:
  - movement: {name: SEQUENCE_STAND_UP, duration_s: 1.5}
  - movement: {name: HOLD_DISTANCE, distance_m: 0.6, duration_s: 10.0}
  - movement: {name: LEVEL_TORSO, duration_s: 5.0}
  - say: Finished
```

- `HOLD_DISTANCE` uses **only 1D lidar** to walk forward/backward toward the target
  distance. It keeps the torso pose fixed, prepares the head to forward-neutral
  over one second, and suppresses gait head scanning. It needs no IMU data or IMU
  configuration.
- `LEVEL_TORSO` uses **only IMU orientation** to level torso roll/pitch while
  standing. The feet, torso position/yaw, and head pose are retained. It starts no
  walking gait and needs no lidar data or lidar configuration.

Both gaits require the robot to be standing at a rest endpoint. `duration_s`
defaults to 1.0 seconds, as for other movements, and must be positive and finite.
`distance_m` is required only for `HOLD_DISTANCE` and must be positive and finite;
other gaits reject it. Loops and snippets work through the usual movement parser.
These gaits own their velocity and pose targets: do not combine them with a
`velocity` action or `body_pose` target. Communication actions can share the group.
The former standalone `hold_distance`/`level_torso` action keys are removed.

The duration starts after the preceding gait settles, required preparation
finishes, and the required sensor/TF is available. The gait reports completion
through its normal stopped state after its duration and graceful stopping. The final torso pose is retained. Cancellation
clears controller state and generated velocity.

Configure each gait separately under `hold_distance.*` and `level_torso.*` in
`config/<robot>/gait.yaml`. Parameters are read when the gait starts;
restart the node after changing its parameter file. Both profiles have
`body_frame: base_link`, `freshness_s: 0.5`, and `startup_timeout_s: 2.0`.
Startup timeout applies while the gait waits for its required sensor and mounting
TF, after any head preparation. Before initial pose feedback, requests use the
same deferred-start behavior as all other movement requests. Freshness checks both
message age and steady-clock reception age, so paused simulation or replayed readings
cannot sustain control.

| Config prefix | Parameter suffix | Default |
| --- | --- | --- |
| `hold_distance` | `range_topic` | `scan_1d` |
| `hold_distance` | `distance_gain` | 0.2 (1/s) |
| `hold_distance` | `distance_deadband_m` | 0.02 |
| `hold_distance` | `max_speed_m_s` | 0.02, also capped by the robot velocity limit |
| `hold_distance` | `max_acceleration_m_s2` | 0.02 |
| `level_torso` | `imu_topic` | `bno055/imu` |
| `level_torso` | `tilt_gain` | 0.3 (1/s) |
| `level_torso` | `tilt_deadband_rad` | 0.01745 (about 1 degree) |
| `level_torso` | `max_tilt_rad` | 0.13963 (about 8 degrees), also capped by gait limits |
| `level_torso` | `max_tilt_rate_rad_s` | 0.03491 (about 2 degrees/s) |

Positive distance error commands forward motion. The lidar's +X beam must point
within 15 degrees of body +X, with valid TF at the reading timestamp. Distance is
measured from the lidar along its beam; keep a stationary target in that beam.
A `sensor_msgs/Range` publisher is required; Nira's normal 2D lidar does not
supply `scan_1d` automatically.

Positive measured tilt produces a negative torso correction. Commands integrate
bounded correction rates and stop at the absolute tilt limits. IMU orientation
is transformed into the body frame using mounting TF; acceleration is not used.
Hardware axis remapping must agree with the IMU header frame. Nira's measurement
frame is upright, with roll/pitch/yaw `(0, 0, -pi/2)`. Nox's primitive URDF includes
`imu_link`; its mesh URDF needs a verified mounting TF supplied separately.
For simulation, override `level_torso.imu_topic` with `/imu/data` at startup and
use the simulation clock consistently. These gains still require physical tuning.

Missing, stale, invalid, or misaligned required sensor data during control aborts
that sequence invocation: generated speed becomes zero, walking stops gracefully,
and remaining groups belonging to that invocation are removed. Other queued
invocations are retained. Failure of the unused sensor has no effect.

## Loops

```yaml
actions:
  - loop:
      times: 3
      actions:
        - body_pose:
            duration_s: 1.5
            torso_pose: {orientation: {roll: -8}}
            head_orientation: {yaw: -12}
        - body_pose:
            duration_s: 1.5
            torso_pose: {orientation: {roll: 8}}
            head_orientation: {yaw: 12}
  - body_pose:
      torso_pose: {}
      head_orientation: {}
```

This runs the two poses three times total, then returns to neutral once. Every
action inside a loop repeats, including speech/music if included. Keep one-time
actions outside the loop. `loop` must be the only key in its entry. `times` must
be an integer from 1 to 1000; the body must be nonempty. Nested loops and includes
are supported up to 16 nested loops and 10,000 expanded groups per catalog load.
Expansion happens before request creation, so repetitions have independent objects.
The existing scheduler handles priorities and cancellation of expanded groups.
Cancellation clears pending targets and lets the active trajectory stop gracefully.

## Snippets and validation

Snippets live in `config/sequences/snippets` and contain only an `actions` list. `include`
must be the only key in its entry. Paths resolve relative to the containing file,
including nested snippets. Repeated includes are allowed. Circular includes and
nesting beyond 64 files are rejected. Keep snippets in this subdirectory so they
are not loaded as standalone sequences; catalog loading only reads files directly
in `config/sequences`.

File, directory, and string loads are transactional. Invalid actions, duplicate
names/keys, conflicting aliases, malformed includes/loops, and expansion-limit
failures retain the previous catalog. Unknown requests are errors instead of
being silently skipped. String parsing supports loops but cannot resolve includes.

The parser also accepts a `sequences: [...]` wrapper. Legacy `behaviors: [...]`
and `Request*` action keys remain migration inputs; legacy torso/head requests
are combined into their group's movement request. Do not combine legacy pose
components or a movement selector with a `body_pose` shorthand in one entry.
Runtime code uses `CSequenceParser` and one movement request carrying `BodyPose`;
the old separate torso/head request classes have been removed.

The `head_pose` → `head_orientation` rename and new presence flags change the ROS
interface. Rebuild and restart all consumers together, including movement,
visualization, and navigation. External publishers must set presence flags;
an all-false command is empty and rejected. Old installed `config/behaviors`
files are not loaded by the new coordinator.

Legacy six-entry lists remain accepted for leg fields, in the leg order listed
above. Prefer named mappings in new YAML; `null` placeholders are only supported
in legacy `joint_targets` lists.
