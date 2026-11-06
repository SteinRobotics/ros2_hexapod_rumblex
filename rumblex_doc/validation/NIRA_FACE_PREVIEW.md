# Nira desktop face-preview validation

Date: 2026-11-06. Host: amd64 laptop, ROS 2 Lyrical, OpenCV 4.10.0.
This validates offline motion, not physical servo tracking or Raspberry Pi performance.
No camera images were saved.

## Build and automated checks

Built interfaces, camera, `rumblex_perception_face`, `rumblex_perception_lidar`,
brain, and bringup in an isolated `/tmp/rumblex-nira-check` build/install overlay,
using existing installed workspace dependencies. The old perception package was
not used by the new launch paths. Checked the repository package dependency graph
for cycles and searched for stale runtime references to the old package name.

All 86 focused tests passed:

| Suite | Cases |
| --- | ---: |
| Face target association and bundled YuNet inference | 4 |
| Offline head preview: convergence, stale input, limits, override | 3 |
| Existing movement handler | 14 |
| Existing sensor-feedback movement handler | 5 |
| Camera launch | 15 |
| Desktop device discovery and override | 3 |
| Sensor composition | 12 |
| Desktop face launch composition | 7 |
| Gazebo launch regression | 9 |
| Pi setup script | 14 |

CTest selections: `test_face_tracking` in face perception;
`test_face_preview`, `test_movement_handler`, `test_feedback_movement_handler`
in brain; all package-local tests in camera and bringup. Builds used two compiler
workers; focused test rebuilds and tests used one. Each workload ran sequentially
under a shared flock and user-systemd scope, with MemoryHigh=6G, MemoryMax=8G,
MemorySwapMax=1G and a bounded timeout. Enforced memory values were checked.

The same seven affected packages (including Gazebo launch installation) were then
built and installed into the normal workspace in a bounded 4 minute 42 second
batch. All 67 detector/controller/camera/bringup cases were rerun against that
installation and passed; the 19 unchanged movement regression cases had already
passed in the isolated build. Both package-form and file-form `--show-args`
commands resolved successfully.

## Live laptop camera

Camera: Sonix USB2.0 FHD UVC webcam, selected through its persistent
`/dev/v4l/by-id/...-video-index0` path. `/dev/video1` is excluded as a non-capture
node. The camera is visible on the host, but hidden inside the tool sandbox.
Capture negotiated 640×480, with a 15 Hz publication timer; the device reports
30 fps. A direct capture timing check delivered about 17 frames/s.

The test command used the normal Nira test launch with only GUI processes disabled:

```bash
ros2 launch rumblex_bringup test_launch.py robot:=nira \
  enable_display:=false enable_simulated_teleop:=false \
  enable_simulated_hmi:=false enable_face_tracking_view:=false
```

An observer counted live image/face messages and checked `/cmd_movement` for head
limits and unchanged torso/toe targets over 20 seconds. It ran in an isolated ROS
domain and shut down its own launch afterwards. SIGINT shutdown messages in logs
are from that deliberate teardown.

Initial raw-image trial: 41 received frames, 39 face observations, 6 valid faces,
and 64 head commands. Yaw stayed between −4.98° and +5.13°; pitch stayed between
8.86° and 9.98°. Torso and toe targets stayed unchanged. A second face-present
trial produced 49 valid observations and 46 bounded head commands.

The camera itself published approximately 14 frames/s, but raw-image subscribers
received far fewer frames. Switched the desktop detector and annotated view to
JPEG-compressed ROS image topics; raw input remains the detector's hardware default
and raw camera output remains available on subscription. This avoids sending large
raw frames through the desktop control path. Fast DDS documents that large samples
can lose fragments with best-effort delivery; the observed improvement supports
using compression here, without claiming a complete diagnosis of host transport
configuration. See [Fast DDS large data guidance](https://fast-dds.docs.eprosima.com/en/v3.6.1/fastdds/use_cases/large_data/large_data.html).

After switching to compression: 267 received frames and 197 processed observations
in a 20-second trial (including launch startup), with about 13.9 camera frames/s.
No face qualified during this particular trial; observations remained invalid and
no head commands were emitted. The bundled detector and control tests cover this
loss behavior separately from the earlier face-present live trials.

## Installed default GUI launch

Ran the installed `ros2 launch rumblex_bringup test_launch.py robot:=nira` with its
normal GUI defaults for 20 seconds, then stopped the test processes. RViz, the
annotated face window, simulated teleoperation, and simulated HMI all started
without process failures. The live observer reported:

```text
camera frames received: 259
face observations:      206
valid face observations: 204
head commands:          762
yaw range:              -18.21 to +17.45 degrees
pitch range:            -0.12 to +9.98 degrees
torso/toe changes:      none
final brain state:      TRACKING: offline approximate FOV
startup process failures: none
camera publication rate: approximately 14 frames/s
```

This confirms the compressed camera path drives the installed offline brain and
movement stack while the normal GUI processes are running. It does not substitute
for an operator inspecting geometric alignment and manually exercising overrides.

## RViz flicker follow-up (2026-11-06)

Reproduced the reported flicker independently of face tracking with the Nira mesh
display on this laptop's scaled Wayland/XWayland desktop. After resizing RViz to
1900 by 1300 pixels, six successive window captures alternated between the model,
a black viewport, and changes confined to the upper-left quarter of the viewport.
This matches the Qt/Ogre pixel-size mismatch described in
[RViz issue #1052](https://github.com/ros2/rviz/issues/1052) and
[the proposed upstream correction](https://github.com/ros2/rviz/pull/1870).

The repository's three RViz launchers now set `QT_QPA_PLATFORM=xcb`,
`QT_ENABLE_HIGHDPI_SCALING=0`, `QT_AUTO_SCREEN_SCALE_FACTOR=0`,
`QT_SCALE_FACTOR=1`, and `QT_SCREEN_SCALE_FACTORS=1` for RViz only.
Other GUI processes and desktop settings are unaffected. Icons can appear smaller.

Repeated the same resize using the installed mesh launch without shell-provided
Qt overrides: all six successive captures were pixel-identical, showing the full
model viewport. RViz exited cleanly. This verifies the rendering workaround on this
host; the face tracker and motion controller were not changed or retested here.
Checks used a separate ROS domain, a shared workload lock, verified 6/8/1 GiB
memory limits, and a 60–90 second timeout. Python syntax and `git diff --check`
passed. The installed launch files resolve to source symlinks, so no rebuild was needed.

- [Before: black frame during flicker](images/rviz-scaled-flicker.png)
- [After: stable viewport following resize](images/rviz-unscaled-stable.png)

## Remaining validation

- Visual inspection of the RViz head and annotated GUI together, including operator
  manual override and re-enable interaction.
- Longer trials across lighting, face distance, multiple people, camera unplug,
  and external webcam models; calibrated host-camera geometry.
- Capture-to-command latency distribution and sustained Pi performance.
- Physical camera mounting TF, safe servo limits, measured head feedback, and
  hardware visual-servo commissioning. Hardware bringup does not enable this preview.
