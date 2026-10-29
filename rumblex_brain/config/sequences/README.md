# Sequences and snippets

The brain loads every `.yaml` and `.yml` file directly in `config/sequences`,
ordered by filename. Each file defines a named sequence, optional voice/joystick
triggers, and actions. Sequence names and triggers are unchanged from the former
`config/behaviors` catalog.

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

When provided, `toe_positions` contains exactly six `{x, y, z}` mappings, with all
three coordinates required for every toe. Order: right front, right middle,
right back, left front, left middle, left back. These are targets before the torso
transformation, matching `BodyPose.msg`. Individual optional toes are not supported.
Toe motion uses the same interpolation and duration as head and torso motion.

The ROS message encodes optional components with `has_head_orientation`,
`has_torso_pose`, and `has_toe_positions`. YAML presence sets these flags automatically.
Published brain trajectory samples and `body_pose_actual` are complete snapshots
with all three flags set. Feedback consumers require complete snapshots.

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
