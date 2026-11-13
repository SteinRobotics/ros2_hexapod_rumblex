# Sequences and snippets

For choreography and walking routes, use the offline
[Sequence Studio GUI and Python builder](../../tools/README.md):
`python3 rumblex_brain/tools/sequence_generator.py` from the repository root.
It provides editable blocks, dance/route presets, YAML export, and a nominal route
preview. Existing hand-written YAML remains supported.

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

## Reusable constants

Named sequences may define an optional `variables` mapping before `actions`.
Use C++-style `kPascalCase` names for keys and native YAML anchors (`&name`),
then reuse them with aliases (`*name`), including inside loops:

```yaml
name: example_constants
variables:
  kLiftMiddle: &kLiftMiddle 0.02  # m
  kLiftDuration: &kLiftDuration 0.3  # s
actions:
  - movement:
      name: SINGLE_POSE
      duration_s: *kLiftDuration
      pose_delta:
        toe_translation:
          right_mid: {z: *kLiftMiddle}
          left_mid: {z: *kLiftMiddle}
```

Anchors must be defined before use and are local to the YAML document; included
snippet files cannot reference the parent file's anchors. The `variables` field
is supported on named sequences and snippets, not catalog wrappers. These are
static values, with no arithmetic, automatic negation, or runtime overrides.
Define negative counterparts separately (for example, `kLowerMiddle: &kLowerMiddle -0.02`).
Add units as inline comments (`# s`, `# m`, `# deg`, or `# m/s`); mark loop
counts with `# repetitions`. Extract the main tuning values and frequently reused
settings, leaving neutral zeros and simple one-action sequences inline. Snippets
can also define anchors at first use.
Keep settings with different purposes independent even when their values match.
See `clap.yaml` for durations in seconds, translations in meters, and coxa angles
in degrees.

### Shared constants

Import shared defaults explicitly; paths are relative to the importing file:

```yaml
name: example_shared
imports: [variables/common.yaml]
variables:
  kLegLiftHeight: 0.025  # m; optional local override
  kLiftFront: &kLiftFront "${kLegLiftHeight}"  # m
actions:
  - movement:
      name: SINGLE_POSE
      duration_s: 0.5
      pose_delta:
        toe_translation: {right_front: {z: *kLiftFront}}
```

`variables/common.yaml` contains only a `variables` mapping. Store shared files
in a subdirectory so catalog discovery does not treat them as named sequences.
Shared files cannot import other files. Multiple imports must have distinct
variable names; local definitions override imported defaults. Duplicate local
keys, missing files, undefined variables, and reference cycles are errors.

A whole-value reference such as `"${kLegLiftHeight}"` preserves the referenced
YAML type, including numbers, strings, lists, and mappings. References work in
action values (including loops) and variable definitions; names and mapping keys
are not substituted. Definitions may refer to other definitions, including local
overrides of imported defaults. Unused definitions are validated too. Expansion
is limited to 64 nested levels. Embedded string substitution and arithmetic are
not supported; define signed values separately.

Snippets can declare their own `imports` and `variables`. Their scope is independent:
they do not inherit their caller's definitions or overrides. Existing native YAML
anchors still work within each file. String parsing supports local references but
cannot resolve file imports because it has no source directory.

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
be finite and at least 0.1 seconds. The runtime clamps shorter absolute `movement`
requests to 0.1 seconds; the `body_pose` shorthand rejects them. This is a sampling
minimum (five updates at nominal 50 Hz), not a joint velocity or acceleration limit. Every
pose action consumes its duration, including repeated identical targets.

When provided, `toe_positions` maps all six leg names to `{x, y, z}` targets,
with all three coordinates required for every toe. Names are `right_front`,
`right_mid`, `right_back`, `left_front`, `left_mid`, and `left_back`. These are
targets before the torso transformation, matching `BodyPose.msg`. Absolute toe
positions still require all six legs; relative moves can name individual legs.
Toe motion uses the same interpolation and duration as head and torso motion.

Set `interpolation` alongside `duration_s` in a `body_pose` shorthand action or
in a `movement` with `name: SINGLE_POSE` (not inside its nested `body_pose`).
It applies to absolute targets, relative deltas, joint-derived toe targets, and
saved-pose restoration. Each action defaults independently to `linear`.

| Value | Progress for elapsed fraction `t` | Motion |
|---|---|---|
| `linear` | `t` | Constant speed |
| `sin_squared` | `sin²(πt/2)` | Slow start and finish |
| `sin` | `1 − cos(πt/2)` | Slow start, no slowdown at finish |
| `cos` | `sin(πt/2)` | Immediate speed, slow finish |

For example: `body_pose: {duration_s: 1.5, interpolation: sin_squared, head_orientation: {yaw: 25}}`.
Unknown values and explicit interpolation on other movement types are errors.
The default replaces the previous implicit quintic easing for single poses;
continuous control and walking retain their existing trajectories.

The ROS message encodes optional components with `has_head_orientation`,
`has_torso_pose`, and `has_toe_positions`. YAML presence sets these flags automatically.
Published brain trajectory samples and `body_pose_actual` are complete snapshots
with all three flags set. Feedback consumers require complete snapshots.

## Clap and relative pose movements

`clap.yaml` starts standing on level ground and keeps both middle and both rear
feet planted for the entire sequence. Absolute torso targets first level the
body and shift it back 6.5 cm and down 1 cm with all six feet planted. Absolute
front joint targets then raise the front legs for six 30-degree close/open
cycles. Each close adds a gentle 3 mm downward body pulse; each opening returns
to the settled height. There is no forward sway, lateral sway, or torso rotation
while the front legs are raised.

The pose is saved **after** settling. Restoring it lands the front toes while
keeping the body rearward; only then does a separate action return the torso to
neutral. The original toe positions and head orientation are preserved. The
initial torso offset/orientation is intentionally replaced by neutral on completion.
The head lifts slightly during preparation, nods to 3 degrees during closing,
then adds a delayed 2-degree nod over a 0.12-second contact pause. It returns to
-4 degrees during reopening. Closings use `cos` easing for an immediate start
and soft finish; other actions use `sin_squared`. Feet and torso hold during
the short head-only relative nod, so there is no accumulated body/foot drift.

The first clap closes/opens in 0.6/1.2 seconds, four middle claps use 0.5/1 seconds,
and the final clap closes in 0.5 seconds and opens over 1.4 seconds. Closing therefore
has twice the average coxa speed of reopening (2.8 times for the final clap).
All six have the 0.12-second nod pause.
Settling/lift/landing/recentering remain 2/1.6/2/2 seconds, for 18.02 seconds plus
scheduler handoffs. The original head orientation returns during landing.
The `commandClap` voice trigger and `clap` sequence name are unchanged.

The rearward bias provides a geometric support margin for the configured standing
stance, but is not a measured center-of-mass guarantee. Actual stability depends
on payload, contact, and starting stance; validate the motion on hardware before
unrestrained use. Cancellation finishes the active action without executing the
remaining landing/recentering actions.

Other sequences can use `movement: {name: SINGLE_POSE, ...}` with `pose_delta`
instead of `body_pose`. Deltas resolve when the action starts, including after a
pending outgoing movement completes.

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
these relative coxa moves follow a chord rather than a joint-space arc. Head pose and
torso orientation are retained unless their rotation offsets are supplied. `pose_delta` cannot be combined with `body_pose`
and requires a finite positive duration; unlike absolute poses, it has no
0.1-second runtime floor. Inverse deltas restore the starting pose within numerical
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
Targets interpolate joint angles using the selected curve and duration. Each sample
uses the selected robot's forward kinematics and interpolated torso transform.
They can accompany a `body_pose` head/torso target, but cannot be combined with
explicit toe targets or `pose_delta`. Equal starting and target joint angles stay
fixed throughout the action. Repeat joint targets in torso actions to carry those
legs with the torso; omitted legs keep their toes fixed in the command frame.

Use `save_pose: true` on the outgoing pose action to capture the complete current
pose when it starts (after any pending motion completes). A later
`movement: {name: SINGLE_POSE, duration_s: 1.0, restore_pose: true}` returns to that
snapshot and consumes it. Restore actions cannot carry other pose targets or
save again. There is one saved-pose slot per controller; another save replaces
it, so nested save/restore pairs are not supported. A restore without a saved
pose is rejected. Saving/restoring requires a finite positive duration and uses
the ordinary absolute-pose minimum of 0.1 seconds.

Cancellation follows normal YAML sequence behavior: finish the active pose and
discard remaining actions, including the return. A new high-five invocation
captures its own starting pose, replacing any snapshot left by cancellation.

## Run in a square

`run_in_a_square` stands up and uses `MOVE_COMBINED` for four forward legs and
four left turns. Each side commands 0.05 m/s for 10 s (nominally 0.50 m), followed
by an in-place turn at 0.20 rad/s for 7.853981634 s (nominally 90 degrees).
Both speeds are half the current Nox/Nira configured maxima. The final turn
restores the nominal starting heading; a `CONTINUOUS_POSE` action with zero
velocity explicitly stops travel. No voice or joystick trigger is assigned.

Travel takes approximately 71.42 s plus stand-up and gait handoffs. This is timed
open-loop locomotion, not odometry-based path tracking: actual side lengths,
corner angles, and return position depend on gait transitions and foot slip.
Tune durations and speeds together; durations are not recomputed from speed.

## Metal dance

`dance_metal` is a generated in-place routine with eight named sections, from
stalking and uneven chug riffs to hammer feet, windmill circles, air rakes, and
a blast-beat finale. It uses all six feet, with at most one lifted at a time,
and restores the entrance pose on normal completion. Default motion duration
is 50.50 s at 120 BPM, excluding scheduler delays. No music or triggers are set.
Use the [recipe and rehearsal guide](../../tools/README.md) to regenerate it or
export individual sections with reduced tempo/amplitude.

## Hot Dogs for Breakfast

The named dance and `dance_standard` include the same generated snippet.
Its [Python recipe](../../tools/examples/dance_hot_dogs_for_breakfast.py) preserves
16 phrases of 1.752 seconds, subdivided into 64 dynamic 0.438-second accents.
Speech and neutral preparation happen before background music starts with the
first accent. Head leads, torso rebounds, and alternating single-foot kicks add
character; the final pose has neutral head/torso and the original toe positions.
The final action explicitly stops music. See the [authoring guide](../../tools/README.md)
for regeneration, contact reports, timing limitations, and cancellation behavior.

## Bite and stomp examples

`bite.yaml` and `stomp.yaml` are selectable suggested gestures with no voice or
joystick triggers. Start from a standing pose; they do not automatically stand up.
The existing `commandBite` and `commandStomp` voice commands remain unconnected.

`bite` is a head-led snap with all six toes planted: 0.22 s gathering back/down,
0.12 s head lead, 0.18 s forward strike, 0.28 s recoil, and 0.35 s settling.
The head lead accelerates with `sin` easing; the strike decelerates with `cos`.
Other phases use `sin_squared`. Peak entrance-relative offsets are 22 mm forward,
14 degrees of head pitch, and 3 degrees of torso pitch. All deltas sum to zero
before the final one-second saved-pose restoration. Expressive motion takes
1.15 s; total duration is 2.15 s plus scheduler handoffs. Relative saving captures
the entrance pose during the short gathering phase.
This sharper version has not been visually or physically validated on hardware.
`stomp` saves the starting pose while lifting the right-front toe 20 mm, lowers
it by 20 mm, lifts the left-front toe 20 mm, then restores the saved pose.
Each stomp action takes one second. These use relative targets rather than
robot-specific stance coordinates and are suggestions, not hardware-validated motions.

Normal completion restores the full starting pose. Cancellation finishes the
active action and discards remaining actions, including restoration. As with
high-five, the next invocation replaces any saved snapshot left by cancellation.

## Look left and right

`look_left.yaml` and `look_right.yaml` are two-action pose sequences.
The first action saves the starting pose and turns the head by 25 degrees and
torso by 20 degrees over 1.5 seconds (positive yaw for left, negative for right).
The second restores the saved pose over 1.5 seconds, without a hold. Toe positions,
torso translation and the original roll/pitch angles stay unchanged. Both actions use
`interpolation: sin_squared` for a slow start and finish.

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

All YAML music playback runs in the background, including shorthand such as
`music: musicfox_excess_party.mp3` and mapping forms. The legacy `background`
field no longer changes this behavior. Playback does not block later actions;
an explicit `minDuration` still applies. Planner cancellation leaves background
music playing; `music: STOP` (as in `standup`) stops it explicitly.
Background music uses a separate audio channel so speech and listening can run
while it plays. Starting another music track replaces the previous track.

Use `music: {song: STOP, fade_out_s: 4}` alongside movement to fade background
music over four seconds while the sequence continues. `fade_out_s` accepts
0–60 seconds; positive values require `song: STOP`. Zero retains the immediate
STOP behavior. The fade does not interrupt speech/listening or wait for audio
completion. Schedule movement for the fade interval before a final cleanup STOP.
Both brain and communication must include fade support; older communication
nodes do not subscribe to the new `request_background_music_fade` topic.


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
frame is upright, with roll/pitch/yaw `(0, 0, -pi/2)`. Both Nox URDFs include
`imu_link`. Hardware bringup starts `robot_state_publisher` with the selected
robot's mesh URDF to publish the mounting TF, including when navigation is disabled.
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

Snippets live in `config/sequences/snippets` and contain an `actions` list, with optional `imports` and `variables`. `include`
must be the only key in its entry. Paths resolve relative to the containing file,
including nested snippets. Repeated includes are allowed. Circular includes and
nesting beyond 64 files are rejected. Keep snippets in this subdirectory so they
are not loaded as standalone sequences; catalog loading only reads files directly
in `config/sequences`.

File, directory, and string loads are transactional. Invalid actions, duplicate
names/keys, conflicting aliases, malformed includes/loops, and expansion-limit
failures retain the previous catalog. Unknown requests are errors instead of
being silently skipped. String parsing supports loops but cannot resolve includes.

The parser also accepts a `sequences: [...]` wrapper. Deprecated `behaviors`
wrappers, `Request*` action keys, and movement `type` aliases are rejected.
For external sequences, replace `behaviors` with `sequences`, use the documented
short action keys (`say`, `movement`, etc.), and replace movement `type` with `name`.
Move separate torso/head requests into the movement's `body_pose.torso_pose` and
`body_pose.head_orientation`. Keep movement timing on the movement action.

The `head_pose` → `head_orientation` rename and new presence flags change the ROS
interface. Rebuild and restart all consumers together, including movement,
visualization, and navigation. External publishers must set presence flags;
an all-false command is empty and rejected. Old installed `config/behaviors`
files are not loaded by the new coordinator.

Legacy six-entry lists remain accepted for leg fields, in the leg order listed
above. Prefer named mappings in new YAML; `null` placeholders are only supported
in legacy `joint_targets` lists.

## Flying

`flying.yaml` is selectable by sequence name, with no voice or joystick trigger.
It raises both middle legs, performs three synchronized down/up wing beats
(femur elevation -10 to +25 degrees), then extends them horizontally
with joint angles `(0, 0, 90)` degrees. The torso rolls twice between
-6 and +6 degrees, carrying the extended middle legs with it while all three
middle-leg joint angles stay fixed. The roll actions also look straight ahead.
The four outer toe targets remain unchanged. Finally it restores the complete
saved starting pose. All actions use `sin_squared`; the 13 expanded actions take
16 seconds plus scheduler handoffs. Geometry comes from the selected robot's
forward kinematics. The knees remain at 90 degrees throughout the wing beats.
As with high-five, cancellation finishes the active action and skips the return.

### Epic dance

`dance_epic.yaml` performs a 30-second opening excerpt of **Iron Star March**
with fast alternating foot taps, six-foot ripples, and small head/torso accents.
At 26 seconds, foot lifts and body gestures diminish while background music
fades over four seconds. The final second restores the entrance pose, followed
by a cleanup `music: STOP`. Start standing. The 30 seconds include restoration
but exclude scheduler/audio latency. See the recipe and validation limitations
in the [authoring guide](../../tools/README.md#epic-30-second-iron-star-march-opening).
