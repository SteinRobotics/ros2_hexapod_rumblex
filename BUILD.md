# RumbleX build and terminal setup

Install dependencies using [README_SETUP.md](README_SETUP.md). Commands below run
from `~/Workspace/colcon_rumblex` with Ubuntu 26.04 / ROS 2 Lyrical.
For launches see [README_LAUNCH.md](README_LAUNCH.md); for topic commands see
[COMMANDS.md](COMMANDS.md).

## Terminal environment

After creating `.venv` with `--system-site-packages` as described in the setup
guide, run these commands in each new terminal, including before `rosdep`:

```bash
cd ~/Workspace/colcon_rumblex
source /opt/ros/lyrical/setup.bash
source .venv/bin/activate
```

For an existing installation, also source the workspace overlay. On a fresh
workspace, run this only after the first successful build. Repeat it after rebuilding:

```bash
source install/local_setup.bash
```

Check that Python comes from the workspace's `.venv` and can import ROS:

```bash
which python
python -c 'import rclpy; print(rclpy.__file__)'
```

## Resource limits

Before any build or test, follow [resource-limited builds and tests](rumblex_doc/BUILD_AND_TEST.md).
Select the host's memory/worker settings, define `rumblex_limited` in the same shell,
inspect existing workloads, and verify the enforced limits. Run heavy workloads
sequentially. On the approximately 30 GiB amd64 host use MemoryHigh=6G,
MemoryMax=8G, MemorySwapMax=1G, a five-minute timeout, and two compiler workers;
on Pi 5 use the guide's RAM-relative limits and one worker. The wrapper sets both
`MAKEFLAGS` and `CMAKE_BUILD_PARALLEL_LEVEL`. Do not run the commands below without
that wrapper or start another job after a timeout without checking surviving processes
and logs first.

## Initial builds

Choose the selection matching your installed dependencies. Run one command at a time.

```bash
# Raspberry Pi robot runtime and workspace dependencies:
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-up-to rumblex_bringup

# Developer PC visualization, simulation, and navigation:
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-up-to rumblex_gazebo rumblex_description rumblex_navigation

# Complete workspace, only with all package dependencies installed:
rumblex_limited python -m colcon build --executor sequential --symlink-install
```

Source the overlay as shown above after a successful build. Large initial builds
may exceed the five-minute limit; inspect the expensive operation and use focused
batches as described in the resource guide.

## Incremental builds

`--packages-select` assumes workspace dependencies have already been built.
Use `--packages-up-to` when they also need building.

```bash
# Movement only:
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-select rumblex_movement

# Brain and installed sequence catalog:
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-select rumblex_brain

# Communication, using the active venv's Python:
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-select rumblex_communication

# Camera and its dependencies, including src/usb_cam when present:
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-up-to rumblex_camera

# Camera integration with perception and bringup:
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-up-to rumblex_perception rumblex_bringup

# Movement pipeline: interfaces, utilities, brain, and movement:
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-up-to rumblex_brain rumblex_movement
```

## Offline mp-units build

C++ packages require C++23 and CMake 3.25 or newer. CMake uses installed mp-units
2.5 or fetches the pinned 2.5.0 release. For an offline build, install it and add its
prefix to `CMAKE_PREFIX_PATH`, or supply a local release checkout:

```bash
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-up-to rumblex_brain rumblex_movement \
  --cmake-args -DFETCHCONTENT_SOURCE_DIR_MP-UNITS=/path/to/mp-units-2.5.0
```

## Tests

Use one worker for tests and any prerequisite rebuilds:

```bash
export RUMBLEX_BUILD_WORKERS=1
rumblex_limited python -m colcon test --executor sequential \
  --packages-select rumblex_movement --event-handlers console_direct+ \
  --ctest-args -j1 --output-on-failure
python -m colcon test-result --verbose
```

Replace the package selection with the packages affected by your change, for
example `rumblex_utils rumblex_brain rumblex_movement rumblex_navigation`,
`rumblex_brain`, or `rumblex_hmi`. The resource guide documents focused CMake/CTest
checks and recovery after interruptions.

## Reconfigure after a ROS or Python upgrade

After upgrading the OS and ROS, remove old ROS sourcing entries from shell startup
files. Archive the old `build/`, `install/`, and `log/` directories, recreate `.venv`
with the new system Python, reinstall dependencies, and use the appropriate initial
build above. Source the new overlay after building.

If only stale CMake Python metadata remains (for example a Jazzy Python 3.12 cache
on Lyrical), reconfigure using the appropriate package selection:

```bash
rumblex_limited python -m colcon build --executor sequential --symlink-install \
  --packages-up-to rumblex_bringup --cmake-clean-cache
```

## Python executable uses the wrong interpreter

Build Python packages with the active venv. Generated ROS executables record the
build interpreter; activating the venv afterward does not update them. If communication
cannot find `openwakeword`, first verify its dependencies in that environment:

```bash
python -c 'import rclpy, openwakeword, speech_recognition, sounddevice, gtts; print("Python dependencies OK")'
head -n 1 install/rumblex_communication/lib/rumblex_communication/node_communication
```

The executable's first line should point to `.venv/bin/python`. If imports succeed
but the executable uses `/usr/bin/python3`, use the communication rebuild above,
then source the overlay. If imports fail, complete the
[communication dependency setup](rumblex_communication/readme.md#installation).

## CAD virtual environment

CAD scripts use their own environment, separate from the ROS workspace venv.
With `build123d` installed in that environment, activate it from the CAD directory.
See the [CAD guide](rumblex_description/cad/README.md) for usage:

```bash
cd ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex/rumblex_description/cad
source .venv/bin/activate
```

Follow the CAD directory's resource rules before running geometry jobs.
