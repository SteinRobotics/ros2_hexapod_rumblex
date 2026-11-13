# Sequence Studio: GUI and Python authoring

`sequence_generator.py` is the main offline authoring tool. Open its desktop GUI
without starting ROS, or use the same API and CLI to generate reproducible YAML.
Requires Python 3, PyYAML (`python3-yaml`) and, for the GUI, Tk (`python3-tk`).
Both runtime dependencies are declared in `rumblex_brain/package.xml`.

```bash
# Open the editor, initially showing an editable square route:
python3 rumblex_brain/tools/sequence_generator.py
# Open a dance or an existing self-contained sequence:
python3 rumblex_brain/tools/sequence_generator.py --gui --preset dance_rock
python3 rumblex_brain/tools/sequence_generator.py --gui --input rumblex_brain/config/sequences/run_in_a_square.yaml
```

The toolbar’s **Text size** selector changes interface and YAML fonts together
(16–28; default 20). The larger default window is 1600 × 1000.

The editor has a block composer, preset library, editable timeline, block YAML
inspector, generated YAML tab, and animated top-down route preview. Use the
composer for travel, turns, arcs, timed velocity, head/torso/toe offsets, holds,
save/restore, stand-up/lay-down, stop, speech, music, and sensor gaits. Pose
component fields use roll/pitch/yaw in degrees or X/Y/Z in metres. Toe offsets
select one named leg; block YAML also supports multiple toes, absolute body
poses, and joint targets using the Python builder's format.

Select a row to edit its YAML, then **Apply block edits**. Insertions go after the
selected row, or at the end when nothing is selected. Shift-select adjacent rows
and choose **Loop** to repeat a phrase. Use the arrows to reorder, **Copy** to
duplicate, and **Delete** to remove. Loops stay compact and can be edited in the
inspector. Undo/redo covers block and sequence metadata changes. Unapplied block
edits and unsaved sequences prompt before replacement or closing.

**Export YAML** validates the complete sequence, including save/restore pairing,
and writes a runtime `.yaml` file. Drafts may temporarily contain an unmatched
save while you assemble a gesture; the validation message explains what remains.
Export uses an atomic replacement and confirms overwriting through the file
dialog. Keyboard shortcuts: Ctrl+O open, Ctrl+N new, Ctrl+S export, Ctrl+Z undo,
Ctrl+Y redo (text fields retain their native editing shortcuts).

The route tab supports play/pause, a time scrubber and 0.5–10× playback. It
integrates commanded planar velocity in the robot's body frame: +X forward, +Y
left, positive yaw counterclockwise. It displays nominal coordinates and heading,
not odometry. The six-legged marker is schematic; individual pose/leg movements
and sensor feedback are not simulated. The tool never sends commands to a robot.

## Generate walking and movement sequences

```bash
# CLI mode: YAML to stdout, timing and notes to stderr.
python3 rumblex_brain/tools/sequence_generator.py --preset square --size 0.5 --speed 0.03 --yaw-rate 0.2
python3 rumblex_brain/tools/sequence_generator.py --preset figure_eight --size 0.3 --gait MOVE_CRAWL --output /tmp/figure_eight.yaml
python3 rumblex_brain/tools/sequence_generator.py --preset circle --clockwise --size 0.4 --output /tmp/circle.yaml
python3 rumblex_brain/tools/sequence_generator.py --input /tmp/circle.yaml --name circle_rehearsal
```

Route presets are `forward`, `square`, `circle`, and `figure_eight`. `--size` is
the travel distance, square side, or circle radius in metres. `--speed` is m/s;
`--yaw-rate` is the square's in-place turn speed in rad/s. Circle angular speed
is computed as linear speed / radius. Routes include stand-up, explicit velocity
axes on every movement segment, and a final stop. Choose `MOVE_COMBINED`,
`MOVE_TRIPOD`, `MOVE_RIPPLE`, `MOVE_WAVE`, or `MOVE_CRAWL`.

Choreography presets reuse `curious_look`, `dance_rock`, and `dance_metal` recipes.
Use `--amplitude` and (for dances) `--bpm`. The GUI preset controls have the same
meaning. Choreography presets start from a standing pose.

```python
from sequence_generator import Sequence, builtin, travel, turn, arc, repeat, stop

sequence = Sequence('inspection_route')
sequence.extend([
    builtin('SEQUENCE_STAND_UP'),
    repeat(2, [travel(0.25, 0.03), turn(90, 0.2)]),
    travel(-0.1, 0.02),                      # reverse
    travel(0.1, 0.02, bearing_deg=90),       # strafe left
    arc(0.3, angle_deg=-90, speed_m_s=0.03), # forward right-hand arc
    stop(),
])
sequence.write('/tmp/inspection_route.yaml', recipe=False)
```

`move(duration_s, x=..., y=..., yaw=..., gait=...)` authors a timed velocity
segment directly. Translation velocities are m/s and yaw velocity is **rad/s**;
`turn()` and `arc()` accept **degrees**. `travel()` derives duration from signed
distance / positive speed; `turn()` derives it from angle / positive yaw speed.
Negative distance reverses travel; negative turn/arc angles turn right.

The preview integrates constant body-frame velocity analytically and samples
curves for display. Runtime velocity limits, gait transitions, scheduler delays,
contact and slipping affect actual travel. A duration does not automatically
clear persistent velocity: end walking with `stop()`. The editor flags missing
final stops. Sensor gaits require their existing sensor setup; their displacement
cannot be predicted here. These tools check structure and units, not IK,
collisions, balance or hardware suitability. Rehearse exported movements in the
robot's simulation before hardware use.

## Supported YAML and compatibility

Open self-contained named sequences containing `movement`, `velocity`, `say`,
`music`, and loops. Optional voice/joystick triggers are preserved. Native YAML
anchors are resolved on import and exported as values; comments and anchor
names are not retained. Includes, imports, `${...}` variable expressions,
legacy request aliases and other runtime actions are outside the editor subset
and are rejected rather than silently discarded. Continue using the runtime's
full YAML format for those features.

The existing `choreography.py` imports remain compatible. `Sequence`, `Stage`,
`pose`, `hold`, `beats`, and `repeat` now live in `sequence_generator.py`; existing
recipes and generated choreography files remain reproducible. The old recipe
header is retained by the Python API by default; `recipe=False` selects the
editor header with velocity units. CLI/GUI exports select that header automatically.
All tools, examples and the GUI module are installed under
`share/rumblex_brain/tools`; the installed entry point can be run with:

```bash
python3 "$(ros2 pkg prefix --share rumblex_brain)/tools/sequence_generator.py"
```

`test_sequence_generator` is registered with CTest. For desktop interaction tests,
set `RUMBLEX_TEST_GUI=1`; otherwise GUI tests skip and core/CLI tests run without
a display. Run tests under the resource wrapper documented in
[`BUILD_AND_TEST.md`](../../rumblex_doc/BUILD_AND_TEST.md):

```bash
# Inside the bounded resource scope:
python3 rumblex_brain/test/test_choreography.py
RUMBLEX_TEST_GUI=1 python3 rumblex_brain/test/test_sequence_generator.py
```

Screenshots and test scope are recorded in the
[validation note](../../rumblex_doc/validation/SEQUENCE_GENERATOR.md).

## Existing choreography recipes

Use this offline builder for long movement sequences. Simple hand-written YAML
remains supported. Python functions are reusable phrases; arithmetic produces
variations, and `repeat()` emits native YAML loops. No ROS process is needed.
Requires Python 3 and PyYAML (`python3-yaml`, also declared in package.xml).

From the repository root:

```bash
python3 rumblex_brain/tools/examples/curious_look.py
python3 rumblex_brain/tools/examples/curious_look.py --amplitude 0.7 --direction right --pause-s 1.5 --repetitions 3 --output /tmp/curious_look.yaml
# Reproduce the checked-in example:
python3 rumblex_brain/tools/examples/curious_look.py --output rumblex_brain/config/sequences/curious_look.yaml --force
```

Stdout contains YAML; stderr reports expanded action count and estimated motion
time. Output files must not exist unless `--force` is supplied. Edit the Python
recipe, then regenerate its YAML. Only the YAML is loaded by the robot. Tools
and examples are also installed under the package's share directory.

## Write a recipe

Place another recipe next to `examples/curious_look.py` and use the same import
setup. The builder supports pose choreography and locomotion; the editor subset is
described above. It does not implement every runtime sequence action.

```python
from choreography import Sequence, beats, hold, pose, repeat


def nod(angle):
    return [
        pose(1.0, save=True, pose_delta={'head_rotation': {'pitch': angle}}),
        hold(beats(1, bpm=90)),
        pose(1.2, restore=True),
    ]

sequence = Sequence('gentle_nod', trigger={'voice': 'commandGentleNod'})
sequence.extend([repeat(3, nod(5))])
sequence.write('/tmp/gentle_nod.yaml')
```

`pose()` accepts `body_pose`, `pose_delta`, or `joint_targets`, with the same
component names as sequence YAML. Joint targets may accompany absolute head and
torso targets. Units are metres, degrees, and seconds. See the sequence README
for coordinate and omitted-component semantics. `hold()` sends a zero relative
head rotation, consuming its duration without resetting the pose.

Each action defaults to `sin_squared` easing; `linear`, `sin`, and `cos` are
also available. Absolute/joint poses and save/restore actions require at least
0.1 seconds in this builder; ordinary relative actions and holds accept shorter
positive durations. Beat conversion never bypasses those limits. Motion time
excludes scheduler handoffs, so beat durations do not provide exact music sync.

Phrases are copied on composition. Save/restore must be paired on every expanded
path; nested saves and leftover saves are rejected. Export revalidates actions.
Loops allow 1–1000 repetitions, 16 levels, and at most 10,000 expanded actions.
The runtime's 10,000-group limit applies to the entire catalog, so a single
sequence near this limit leaves little room for other sequences.

## Making gestures feel alive

The curious-look example anticipates with a slight head pitch, turns the head,
then follows with the torso, tilts, pauses, and restores the captured pose.
Left/right angles and pauses differ deliberately. `--direction` selects the first
side; each repetition performs both sides. `--amplitude` scales angles only.
Default output is 24 actions and 30.76 seconds, excluding scheduler handoffs.

Start standing. All feet remain planted; the recipe contains no robot-specific
stance coordinates. The example is an authored suggestion, not a simulation- or
hardware-validated movement. The builder validates structure, not reachability,
collisions, joint limits, or balance. Inspect the motion in simulation before
hardware use. Cancellation finishes the active action and discards the rest,
including restoration. There is no new automatic cancellation recovery.

For natural variation, alter phrase amplitudes, pauses, and ordering in Python.
Actions are sequential: head-leading-body means separate actions, not overlapping
tracks. Continuous trajectory blending and synchronized music playback would
require separate runtime work.

## Checks

Run `python3 rumblex_brain/test/test_choreography.py` under the resource wrapper
in `rumblex_doc/BUILD_AND_TEST.md`. CTest registers `test_choreography`; the C++
`test_sequence_parser` also loads the generated example and checks phrase order.

## Rock-and-roll example: dance_rock

```bash
python3 rumblex_brain/tools/examples/dance_rock.py --output /tmp/dance_rock.yaml
# Slower, smaller rehearsal version:
python3 rumblex_brain/tools/examples/dance_rock.py --bpm 80 --amplitude 0.5 --output /tmp/dance_rock_rehearsal.yaml
# Regenerate the catalog version:
python3 rumblex_brain/tools/examples/dance_rock.py --output rumblex_brain/config/sequences/dance_rock.yaml --force
```

The default is 112 BPM, 117 actions (116 movements plus STOP), and 53.43 seconds
plus scheduler handoffs.
The arrangement moves through a count-in crouch, counter-looking hip twists,
alternating front-foot guitar kicks, a circular torso accent, a six-foot tom fill,
a headbanging breakdown, an encore with middle-foot side kicks, and a final bow.
All six legs participate. Front/rear kicks extend forward/backward; middle-foot
kicks extend sideways. Each foot lifts 25 mm and reaches up to 25 mm horizontally.
Torso shifts reach 12 mm; head pitch reaches 14 degrees and yaw 18 degrees.

Start from a level standing stance. This is an in-place dance with airborne kicks,
not an odometry-based walking sequence. At most one foot is lifted at a time.
Each kick shifts the torso away from that foot, lifts vertically, extends and
retracts while airborne, and lands vertically before recentering. The other five
feet keep their targets. Body and head accents provide the larger visual energy.

Every cue in the recipe specifies offsets relative to the entrance pose; omitted
axes return to their entrance values. `Stage` compiles these into ordinary relative
YAML movements. Every authored offset returns to zero before the final saved-pose
restoration. This restores commanded head, torso, and toe positions on normal
completion, not a guaranteed world position after physical slipping. Cancellation
still discards the remaining choreography and return.

Tempo accepts 60–140 BPM and amplitude accepts (0, 1]. These are authoring bounds,
not proven physical limits. The routine has parser and trajectory-bookkeeping
tests, but has not been validated for reachability, balance, or appearance in
simulation or on hardware. No voice/joystick trigger is assigned.

The complete dance starts `musicfox_excess_party.mp3` in the background with the
first movement. At the finale, music fades through the bow, hold, recenter, and
entrance-pose restore, then an explicit `music: STOP` ends playback. The fade is
calculated from the remaining movement durations: 5.21 seconds at 112 BPM, with
no extra movement time. Changing BPM adjusts the fade automatically. Named
section rehearsals remain motion-only. The fade requires the updated brain and
communication packages; audio and movement clocks are not synchronized.

## Named sections and rehearsal

List the rock routine's sections, then rehearse just one:

```bash
python3 rumblex_brain/tools/examples/dance_rock.py --list-sections
python3 rumblex_brain/tools/examples/dance_rock.py --section guitar_solo --bpm 80 --amplitude 0.5 --report --output /tmp/rock_guitar_solo.yaml
```

Available sections: `intro`, `verse`, `guitar_solo`, `body_roll`, `drum_fill`,
`breakdown`, `encore`, and `finale`. `--list-sections` prints names and exits
without generating a file. Unknown section names are errors. A rehearsal is
named `dance_rock_<section>`, with no triggers. It saves the entrance pose in a
one-second hold, performs the selected section, and restores over two seconds.
Existing save flags inside the section are removed. `--force` is still required
to overwrite files. The full dance's generated YAML and timings are unchanged.

`--report` writes section ranges, durations, action counts, total motion time,
minimum commanded support-foot count, and per-action foot phases to stderr;
stdout stays valid YAML. Lift and landing feet are excluded from the support
count for that action. Contact is inferred from commanded toe offsets relative
to a level-standing entrance, using a 1e-6 metre tolerance. Below-ground targets
and horizontal motion with either endpoint on the ground produce warnings.
Warnings do not prevent export. This is not measured contact, IK, collision,
or balance validation; scheduler delays are excluded from all timing.

## Shared Stage API

`Stage` lives in `sequence_generator.py` (also exported by `choreography.py`) and can be used by other recipes:

```python
from choreography import Stage, pose

stage = Stage(bpm=100, amplitude=0.6)
with stage.section('look'):
    stage.cue(head={'yaw': 12, 'pitch': -4})
    stage.change(head={'yaw': 0})  # retain pitch and all other offsets
    stage.cue()                  # return every axis to its entrance offset

rehearsal = stage.sequence('my_gesture', section='look')
rehearsal.write('/tmp/my_gesture_look.yaml')
print(stage.report('look'))
```

`cue()` specifies a complete offset target: omitted axes reset to zero.
`change()` replaces only supplied axes, retaining everything omitted. Values are
entrance-relative targets, not increments. Explicit zero resets an axis; an
empty mapping in `change()` does not reset a component. Amplitude is applied
once to authored targets. Inputs are validated before state changes.

Use `with stage.section(name):` for unique, nonempty, non-nested sections. Empty
sections are rejected; exceptions roll back that section's cues. Rehearsal
requires zero offsets at both boundaries, including planted feet. A section
that starts or ends displaced is rejected rather than inventing an entrance or
landing. Keep restore actions outside authored sections. When exporting a full
routine with `stage.sequence(name)`, supply its save/restore actions explicitly,
as demonstrated in `dance_rock.py`; only isolated exports add a wrapper.

Section names and boundaries are Python metadata, not new YAML fields. Contact
reports support Stage relative movements and restoration, not arbitrary joint
or absolute-pose sequences. `test_section_parser` generates all eight rehearsal
files in a temporary directory and checks them with the C++ runtime parser.

## Hot Dogs for Breakfast: music-grid choreography

```bash
python3 rumblex_brain/tools/examples/dance_hot_dogs_for_breakfast.py --report --output /tmp/hot_dogs_snippet.yaml
python3 rumblex_brain/tools/examples/dance_hot_dogs_for_breakfast.py --output rumblex_brain/config/sequences/snippets/dance_hot_dogs_for_breakfast.yaml --force
```

This recipe generates a snippet, not a standalone named sequence. Both
`dance_hot_dogs_for_breakfast` and `dance_standard` include it. Voice and joystick
bindings stay in their existing wrappers. Edit the Python recipe and regenerate.

The original 16 × 1.752 s grid is preserved, now with four 0.438 s accents per
phrase (28.032 s nominal motion). There is deliberately no tempo option: changing
motion speed would break the existing alignment with the unchanged audio.
Speech and a one-second neutral head/torso preparation finish before the track
starts. Background playback starts in the first accent's group so it does not
block later movements; explicit `music: STOP` follows the final neutral pose.

The arrangement alternates head-leading grooves, front kicks, middle-foot side
taps, a body wave, shoulder roll, rear-foot flicks, and a double-nod finale.
At most one toe lifts, by 20 mm; extending toes retract while airborne before
landing. The final head and torso are neutral; toes return to their starting
positions. Start standing on level ground. Preparation resets the head/torso,
so this dance does not preserve an arbitrary starting head/torso offset.

Timing is an open-loop schedule, not audio-clock synchronization. Audio startup,
controller sampling, and action handoffs add latency; the audio asset is not in
this repository, so alignment with the recording has not been measured. At a
20 ms test update, a 0.438 s action consumes 0.440 s (0.128 s extra over 64
accents), before handoffs. The report shows nominal motion time only. No audio
speed changes or scheduler changes are made.

Cancellation discards remaining choreography, including the final landing or
neutral return and STOP. Background music continues until explicitly stopped
or replaced, following the existing background-playback behavior. The generated
foot paths and endpoint return are tested, but visual appearance and physical
balance have not been validated on hardware.

## Metal performance: dance_metal

```bash
python3 rumblex_brain/tools/examples/dance_metal.py --list-sections
python3 rumblex_brain/tools/examples/dance_metal.py --section hammer_feet --bpm 80 --amplitude 0.5 --report --output /tmp/metal_hammers.yaml
python3 rumblex_brain/tools/examples/dance_metal.py --output rumblex_brain/config/sequences/dance_metal.yaml --force
```

The default performance has 122 actions and 50.50 seconds of motion at 120 BPM,
plus scheduler delays. It has eight independently rehearsable sections:

| Section | Character |
|---|---|
| `predator_intro` | Low crouch, left/right stare, then a forward accent |
| `palm_muted_chugs` | Uneven three-hit forward punches and slower rebounds |
| `hammer_feet` | Diagonal front/rear foot order with an airborne down/up feint |
| `windmill` | Eight-point head/shoulder circles, then reversed direction |
| `air_rakes` | Middle-foot outward and rearward sweeps while airborne |
| `stalking_breakdown` | Slow side prowls interrupted by short planted-foot lunges |
| `blast_beats` | Double-time head accents with smaller torso pulses |
| `final_crash` | Compression, final bow, held chord, and controlled release |

All six feet participate, one airborne at a time. Hammer feet land at their
original height, never below it; they are visual accents, not intended floor
impacts. Air rakes retract before landing. Torso shifts reach 16 mm forward and
14 mm down; foot lifts reach 25 mm. Head pitch reaches 16 degrees, yaw 18 degrees;
torso pitch/roll reach 6 degrees. These are offsets from the entrance pose.

Each section returns to zero offsets. Full playback saves the entrance pose
once and restores it after the final release. Cancellation retains the existing
behavior: finish the active action and discard the rest, including restoration.
Start standing. No music track or voice/joystick trigger is assigned. Tempo can
be 60–140 BPM and amplitude (0, 1]; these are authoring bounds, not verified
hardware limits. The faster perceived aggression comes from timing and contrast,
not simultaneous airborne legs. Physical balance and appearance need rehearsal.

## Epic: 30-second Iron Star March opening

`examples/dance_epic.py` generates `config/sequences/dance_epic.yaml` with the
`dance_epic` name. Start standing. The routine contains **41 fast foot taps**:
front-foot alternation, six-foot ripples, driving steps, and a faster fill.
Each tap shifts the torso, lifts one toe vertically, lands it, then recenters.
All other feet remain planted. Lift heights are at most 10 mm, torso shifts at
most 5 mm laterally, and head/torso accents stay small for the fast transitions.
Most movement phases last 0.15–0.23 seconds.

The first action saves the entrance pose and starts background music. At
**26 seconds**, `music: {song: STOP, fade_out_s: 4}` starts an asynchronous
four-second fade. At the same time, the recipe reduces head, torso, and foot
movement targets toward zero. Four diminishing taps finish by 29 seconds;
the final second restores the saved pose. An ordinary `music: STOP` at 30 seconds
is a final cleanup. Music is not restarted during the fade.

| Section | Nominal music time |
| --- | --- |
| Pickup taps | 0–1.83 s |
| Alternating front patter | 1.83–5.13 s |
| Six-foot ripple | 5.13–11.62 s |
| Driving steps | 11.62–22.48 s |
| Fast fill | 22.48–26.00 s |
| Diminishing taps and audio fade | 26.00–29.00 s |
| Restore entrance pose; audio fade finishes | 29.00–30.00 s |

Phrase anchors retain selected low-frequency transients measured in the opening
of `lovemusic_ai__Iron_Star_March.mp3` (the analyzed recording is 198.421 seconds).
The analysis used a mono waveform, a 180 Hz low-pass filter, and positive changes
in 10 ms RMS energy. Fast subdivisions between those anchors are authored, not
individually detected beats. The recipe uses a seconds-based clock, not a fixed
BPM estimate. Generation and runtime require no audio-analysis dependency.

Contact checks verify commanded toe offsets, single-foot support transitions,
and decreasing lift heights in the fade. They do not verify physical balance or
reachability. This choreography has not been rehearsed in simulation or on
hardware. The 30 seconds are commanded motion time: scheduler handoffs and audio
startup latency can extend wall time and shift alignment. The fade runs on the
audio channel's clock after the 26-second action is dispatched; it does not
synchronize the movement scheduler to audio playback.

The fade requires the updated brain and communication packages. The brain sends
seconds on `request_background_music_fade` (`std_msgs/Float32`); communication
uses [Pygame Channel.fadeout](https://www.pygame.org/docs/ref/mixer.html#pygame.mixer.Channel.fadeout)
to ramp only background audio, without interrupting speech or blocking motion.
No additional ROS message package or Python dependency is needed.

```bash
python3 rumblex_brain/tools/examples/dance_epic.py --output rumblex_brain/config/sequences/dance_epic.yaml --force
python3 rumblex_brain/tools/examples/dance_epic.py --report
```
