# Behaviors and action fragments

At startup the brain loads every `.yaml` and `.yml` file directly in this directory,
ordered by filename. Add one file per behavior; no manifest or launch changes are
needed. Each file contains `name`, optional `trigger`, and `actions`:

```yaml
name: example
trigger:
  voice: commandExample
  joystick: button_a
actions:
  - RequestTalking: Starting
  - include: ../behavior_fragments/example.yaml
  - RequestTalking: Finished
```

An **action fragment** contains only an `actions` sequence:

```yaml
actions:
  - RequestMusic: STOP
    RequestTalking: Hello
  - RequestMovementType:
      name: SEQUENCE_WATCH
      duration_s: 5.0
```

Includes insert the fragment's groups at that point in the sequence. Groups run
sequentially; requests within a group are dispatched together. `include` must be
the only key in its group. Paths resolve relative to the file containing the
include, including nested fragments. Repeated includes are allowed; circular
includes and nesting beyond 64 files are rejected. Keep fragments outside this
behavior directory so they are not loaded as standalone behaviors.

File and directory loads reject missing/malformed includes, invalid behaviors,
and duplicate behavior names without replacing the previous catalog. Individual
invalid/unknown requests retain the parser's existing skip-and-log behavior.
An empty directory is an error. The file parser also accepts the older
`behaviors: [...]` wrapper. String parsing does not support file includes.

Positions are in meters, orientations in degrees, and `duration_s` in seconds.
The two hot-dogs/standard dances share the same fragment without sharing mutable
request objects.
