# Nira head camera

`rumblex_camera` launches the ROS 2 `usb_cam` driver as `/camera/node_camera` for
Nira's OBSBOT Meet SE. It runs headlessly on the Raspberry Pi 5; view images from
the developer PC. The default profile requests 1280×720 at 30 fps, using MJPEG
capture decoded to RGB (`mjpeg2rgb`). This mode is listed in the
[Meet SE specifications](https://www.obsbot.com/obsbot-meet-se-full-hd-webcam/specs).
Physical operation and available controls must be commissioned on the camera.

## Dependencies and build

Use Ubuntu 26.04 / ROS 2 Lyrical. Runtime dependencies are declared in `package.xml`:
`usb_cam`, `compressed_image_transport`, launch support, `sensor_msgs`, and `std_srvs`.
Install `v4l-utils` for device inspection; install `rqt_image_view` or RViz on the PC
and `camera_calibration` when calibrating. No GUI is started on the robot.

From the workspace root, with ROS sourced:

```bash
sudo apt install v4l-utils
rosdep install --from-paths src/ros2_hexapod_rumblex/rumblex_camera --ignore-src -r -y
```

If `ros-lyrical-usb-cam` has no apt candidate, put an upstream source checkout at
`src/usb_cam` before resolving dependencies. Version `0.8.1` is the version listed
in the [Lyrical distribution manifest](https://github.com/ros/rosdistro/blob/master/lyrical/distribution.yaml):

```bash
git clone --branch 0.8.1 --depth 1 https://github.com/ros-drivers/usb_cam.git src/usb_cam
rosdep install --from-paths src/usb_cam src/ros2_hexapod_rumblex/rumblex_camera --ignore-src -r -y
```

The Pi setup script includes an existing `src/usb_cam` checkout in its dependency
and build selection. It does not clone or update that driver automatically.
The source fallback has not been built against this host's Lyrical installation;
0.8.1 uses deprecated image_transport APIs with warnings treated as errors, so
it may need an upstream compatibility patch before it can be used on Lyrical.

Before building, check for other builds, tests, simulations, or CAD jobs. Run only
one workload, with the repository resource limits. For a host with a systemd user
manager, the following inherits the sourced ROS environment and applies those limits:

```bash
systemd-run --user --scope -p MemoryHigh=6G -p MemoryMax=8G -p MemorySwapMax=1G \
  timeout --kill-after=10s 300s env CMAKE_BUILD_PARALLEL_LEVEL=2 \
  colcon build --executor sequential --packages-up-to rumblex_camera
source install/local_setup.bash
```

Rebuild `rumblex_perception` and `rumblex_bringup` too when using integrated launch.

## Select the physical camera

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
v4l2-ctl --device=/dev/v4l/by-id/ACTUAL_MEET_SE_DEVICE --list-formats-ext
v4l2-ctl --device=/dev/v4l/by-id/ACTUAL_MEET_SE_DEVICE --list-ctrls-menus
```

Replace `ACTUAL_MEET_SE_DEVICE` in all examples with the real video-capture entry,
not a metadata entry. Prefer a stable by-id path; if the device has no unique ID,
use a verified by-path entry or a local udev alias. Never rely on `/dev/video0`
remaining the same camera after reboot. The device path is intentionally empty
in the supplied YAML; startup requires an explicit absolute path.

The robot user needs access to the video device, usually through the `video`
group. The Pi setup script already adds that group; log in again after changing
membership. For permission failures, inspect `ls -l` on the resolved device and
`id`; for a busy device, stop the other application using this camera.

## Launch and ROS interfaces

```bash
ros2 launch rumblex_camera camera_launch.py robot:=nira \
  video_device:=/dev/v4l/by-id/ACTUAL_MEET_SE_DEVICE

# Optional camera alongside Nira's other sensors:
ros2 launch rumblex_bringup sensors_launch.py robot:=nira enable_camera:=true \
  camera_device:=/dev/v4l/by-id/ACTUAL_MEET_SE_DEVICE

# Full robot bringup uses the same enable_camera and camera_device arguments:
ros2 launch rumblex_bringup target_launch.py robot:=nira enable_camera:=true \
  camera_device:=/dev/v4l/by-id/ACTUAL_MEET_SE_DEVICE
```

Run only one capture instance at a time. Integrated camera startup defaults to
disabled; Nox has no camera profile. `enable_driver` controls lidar independently
of `enable_camera`. Camera process failure does not request shutdown of other
robot nodes. There is no automatic reconnect or restart loop: after reconnecting,
restart the standalone camera launch with the same stable device path.

| Interface | Type / purpose |
| --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image`, decoded RGB |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage`, image transport plugin |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera/set_capture` | `std_srvs/srv/SetBool`, upstream capture start/stop |
| `/camera/node_camera` parameters | Upstream focus, exposure, white balance, capture configuration |

For a configuration override, copy `config/nira/camera.yaml`, keep its
`/camera/node_camera: ros__parameters:` structure, and pass `params_file:=/absolute/path/camera.yaml`
to standalone launch or `camera_params_file:=...` to bringup. Nonempty launch
device/calibration arguments override YAML; empty arguments retain YAML values.
Other capture settings are configured in YAML. Use the standard `/camera`
namespace; the YAML node selector is scoped to it.

Automatic exposure, focus, and white balance are requested by default. Only use
manual values supported by `--list-ctrls-menus`; disable the corresponding auto
mode first. Restart capture after editing YAML. Runtime parameter application
depends on the installed upstream driver version; a successful parameter set
alone does not prove the device applied it.

```bash
ros2 service call /camera/set_capture std_srvs/srv/SetBool '{data: false}'
ros2 service call /camera/set_capture std_srvs/srv/SetBool '{data: true}'
```

Upstream 0.8.1 does not set the response's `success` field, even after calling
start/stop. Check actual frame delivery when commissioning this service; its
response flag alone is not reliable in that version.

## Viewing and commissioning

On the PC, source ROS and use the same ROS domain as the robot. Run
`ros2 run rqt_image_view rqt_image_view` and select `/camera/image_raw/compressed`
for network viewing. In RViz use an **Image** display on `/camera/image_raw` and
select the compressed transport. The raw stream is available for local processing.

With a viewer subscribed, check `ros2 topic hz /camera/image_raw`, dimensions,
orientation, matching image/CameraInfo timestamps and `camera_optical_frame` IDs,
and capture stop/start behavior. Aim for approximately 30 fps and inspect CPU load
and network latency on the Pi. Confirm unplugging or starting with a nonexistent
device stops only camera capture; reconnect and restart capture. Unsupported modes,
permissions, busy devices, and missing drivers should be resolved from driver logs;
there is no silent fallback to another device or format.

Calibrate at 1280×720 using ROS `camera_calibration`, then supply the saved YAML
via `camera_info_url:=file:///absolute/path/calibration.yaml` (also available through
bringup). Until then, CameraInfo is uncalibrated (`K[0] == 0`); do not use it for
metric measurements. For calibrated use, fix focus and disable auto-framing,
digital zoom, or other cropping, then recalibrate if the image geometry changes.
V4L2 support for disabling vendor features must be checked on the actual camera.

This package does not publish a mounting TF. Measure the lens position and
orientation relative to Nira's `head_pitch_link`, then add the fixed camera and
optical-frame joints in `rumblex_description`. Optical axes are +Z forward, +X
right, +Y down. Until measured, use the Image display; spatial Camera overlays
require calibration and that transform.

## API references and later work

- [ROS usb_cam](https://github.com/ros-drivers/usb_cam): V4L2 capture and standard controls.
- [OBSBOT Linux SDK](https://www.obsbot.com/sdk): account/application required;
  verify Ubuntu ARM64 support and redistribution terms before integration.
- [OBSBOT OSC](https://www.obsbot.com/explore/obsbot-center/osc): Meet SE control
  requires OBSBOT Center running; it is not the headless capture backend.

Vendor auto-framing/HDR/zoom integration, detection, recording, microphone capture,
and simulated camera images are future work.
