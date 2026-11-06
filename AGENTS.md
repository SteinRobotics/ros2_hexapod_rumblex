# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 colcon workspace for the RumbleX hexapod platform. Nox is the existing robot;
Nira is the planned second robot. Runtime packages are organized by responsibility:

- `rumblex_movement/`, `rumblex_brain/`, and `rumblex_communication/` contain core robot behavior and C++/Python nodes.
- `rumblex_interfaces/` defines shared ROS messages; `rumblex_utils/` contains reusable C++ helpers.
- `rumblex_bringup/`, `rumblex_teleop/`, `rumblex_hmi/`, `rumblex_lidar_1d/`, `rumblex_lidar_2d/`, `rumblex_perception_lidar/`, `rumblex_perception_face/`, and `rumblex_navigation/` provide hardware integration and launchable components.
- `rumblex_description/` and `rumblex_gazebo/` contain the robot model and simulation assets.
- Package sources, headers, launch files, configuration, and package-local tests belong inside their respective package directories. Hardware and setup documentation is under `rumblex_doc/`.
- Keep robot-specific parameters in `config/<robot>/` directories and select them with the lowercase
  `robot` launch argument. Do not put Nox-specific values back into shared package-level configuration.

## Build, Test, and Development Commands

Follow [resource-limited builds and tests](rumblex_doc/BUILD_AND_TEST.md) before
running builds, tests, simulations, or CAD workloads. It documents separate amd64
and Raspberry Pi 5 budgets, user-systemd access from Codex, and recovery after
interruptions. Run heavy jobs sequentially across sessions and verify limits.

For the approximately 30 GiB amd64 host, start with MemoryHigh=6G, MemoryMax=8G,
MemorySwapMax=1G and a five-minute timeout. On Pi 5, use the guide's RAM-relative
budget and one compiler worker. Keep memory-pressure protection enabled.

Use `--executor sequential` with colcon and set **both** `MAKEFLAGS="-j2 -l2"`
and `CMAKE_BUILD_PARALLEL_LEVEL=2` (use 1 for both on Pi or recovery batches).
This colcon version otherwise adds CPU-count-based Make parallelism, overriding
the intended CMake worker limit. Run tests with one worker as well.

If the sandbox cannot connect to the systemd user bus, request tool escalation
for the bounded `systemd-run --user --scope ...` command; do not silently run
without limits. If access is rejected or limits cannot be verified, report the
blocker. Inspect surviving processes and logs before retrying interrupted jobs;
never automatically restart the full workload or raise limits after an OOM/timeout.

Use package launch files for local checks, for example `ros2 launch rumblex_movement movement_launch.py` or `ros2 launch rumblex_gazebo simulation_gazebo.launch.py`.

## Coding Style & Naming Conventions

C++ uses the repository `.clang-format` (Google-derived, four-space indentation, 110-column limit); format changed C++ files before submitting. Prefer references over raw pointers and use `std::unique_ptr` or `std::shared_ptr` when ownership requires pointers. Follow existing C++23 conventions. Python uses four spaces, `snake_case` modules/functions, and ROS node names consistent with their package. Keep ROS package and interface names lowercase with underscores.

## Testing Guidelines

C++ tests use `ament_cmake_gtest`/GMock and live in each package's `test/` directory, typically named `test_<component>.cpp`. Add or update focused tests with behavior changes; no repository-wide coverage threshold is currently documented.

## Commit & Pull Request Guidelines

Recent commits use short, imperative, lowercase summaries such as `add servo interface board` and `move cad files to rumblex_description`. Keep commits focused and use the same style. Pull requests should explain the behavior or hardware change, identify affected packages, include test/build commands and results, and attach screenshots or logs for visualization, UI, or hardware-facing changes. Call out required ROS dependencies and configuration changes explicitly.
