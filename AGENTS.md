# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 colcon workspace for the RumbleX hexapod platform. Nox is the existing robot;
Nira is the planned second robot. Runtime packages are organized by responsibility:

- `rumblex_movement/`, `rumblex_brain/`, and `rumblex_communication/` contain core robot behavior and C++/Python nodes.
- `rumblex_interfaces/` defines shared ROS messages; `rumblex_utils/` contains reusable C++ helpers.
- `rumblex_bringup/`, `rumblex_teleop/`, `rumblex_hmi/`, `rumblex_lidar/`, and `rumblex_navigation/` provide hardware integration and launchable components.
- `rumblex_description/` and `rumblex_gazebo/` contain the robot model and simulation assets.
- Package sources, headers, launch files, configuration, and package-local tests belong inside their respective package directories. Hardware and setup documentation is under `rumblex_doc/`.
- Keep robot-specific parameters in `config/<robot>/` directories and select them with the lowercase
  `robot` launch argument. Do not put Nox-specific values back into shared package-level configuration.

## Build, Test, and Development Commands

From the workspace root, install dependencies and build with:

```bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

Build one package with `colcon build --packages-select <package_name>`. Run its tests and display output with:

```bash
colcon test --packages-select <package_name> --event-handlers console_direct+
colcon test-result --verbose
```

Use package launch files for local checks, for example `ros2 launch rumblex_movement movement_launch.py` or `ros2 launch rumblex_gazebo simulation_gazebo.launch.py`.

## Coding Style & Naming Conventions

C++ uses the repository `.clang-format` (Google-derived, four-space indentation, 110-column limit); format changed C++ files before submitting. Prefer references over raw pointers and use `std::unique_ptr` or `std::shared_ptr` when ownership requires pointers. Follow existing C++23 conventions. Python uses four spaces, `snake_case` modules/functions, and ROS node names consistent with their package. Keep ROS package and interface names lowercase with underscores.

## Testing Guidelines

C++ tests use `ament_cmake_gtest`/GMock and live in each package's `test/` directory, typically named `test_<component>.cpp`. Add or update focused tests with behavior changes; no repository-wide coverage threshold is currently documented.

## Commit & Pull Request Guidelines

Recent commits use short, imperative, lowercase summaries such as `add servo interface board` and `move cad files to rumblex_description`. Keep commits focused and use the same style. Pull requests should explain the behavior or hardware change, identify affected packages, include test/build commands and results, and attach screenshots or logs for visualization, UI, or hardware-facing changes. Call out required ROS dependencies and configuration changes explicitly.
