# Nira face observations

C++23 ROS 2 face detector using the bundled YuNet ONNX model and OpenCV CPU
inference. Publishes `/perception/face_target` (`rumblex_interfaces/FaceTarget`)
from `/camera/image_raw` and `/camera/camera_info`. Desktop launch uses
`/camera/image_raw/compressed` instead, to reduce image transport losses. Target IDs describe temporary
box association, not identity. Three consistent detections acquire a target;
loss or ambiguous overlap invalidates it. Perception never sends servo commands.

Build with the resource controls in [BUILD.md](../BUILD.md), selecting
`rumblex_interfaces rumblex_perception_face` (and their dependencies on a fresh
workspace). ROS dependencies include `cv_bridge`, `sensor_msgs`, and OpenCV >=4.8.
Model provenance, hash, and license are in [models](models/README.md); startup
requires no network downloads.

For an already running calibrated camera:

```bash
ros2 launch rumblex_perception_face face_tracking_launch.py robot:=nira
```

The default mode requires matching measured camera intrinsics and supported
plumb-bob/rational distortion. Without them it emits invalid observations.
`desktop_preview:=true` permits an explicitly uncalibrated FOV approximation.
`publish_debug_image:=true` offers `/perception/face_debug` and its `/compressed`
variant on subscription; status is available
on `/perception/face_status`. The image input keeps only the newest frame. Desktop image timestamps represent
host capture-return time; they are not exposure timestamps.
Images older than 300 ms, non-finite data, and out-of-order timestamps are rejected.
No images or identities are stored.

For a laptop camera and offline head visualization:

```bash
ros2 launch rumblex_bringup test_launch.py robot:=nira
```

See [launch instructions](../README_LAUNCH.md) for host camera overrides and
headless options. Physical tracking remains uncommissioned: the brain integration
in this increment is explicitly an offline desktop preview.
