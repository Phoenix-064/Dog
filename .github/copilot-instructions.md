# Project Guidelines

## Context
- This repository is a ROS 2 Humble (Ubuntu 22.04) workspace built with `ament_cmake` and `colcon`.
- Main first-party packages are under `src/`: `dog_interfaces`, `dog_perception`, `dog_lifecycle`, `dog_behavior`.
- `build/`, `install/`, and `log/` are generated artifacts; do not hand-edit files there.

## Architecture
- Keep package boundaries clear:
- `dog_interfaces`: shared `msg`/`srv`/`action` contracts.
- `dog_perception`: sensor processing and target outputs.
- `dog_lifecycle`: health/state/degrade logic and system mode.
- `dog_behavior`: behavior execution and action client orchestration.
- Prefer changes that preserve pub/sub and action-based decoupling between packages.

## Build And Test
- Before build/test: `source /opt/ros/humble/setup.bash`.
- Full build: `colcon build` then `source install/setup.bash`.
- Package build: `colcon build --packages-select <pkg1> <pkg2>`.
- Full tests: `colcon test && colcon test-result --all --verbose`.
- Package tests: `colcon test --packages-select <pkg> && colcon test-result --all --verbose`.
- Filtered tests: `colcon test --packages-select <pkg> --ctest-args -R <test_name_regex>`.
- After adding or renaming gtests, rebuild that package before filtered test runs.

## C++ And CMake Conventions
- Follow existing package pattern:
- Build `${PROJECT_NAME}_lib` for core logic and `${PROJECT_NAME}_node` as entrypoint executable.
- Use `ament_target_dependencies(...)` for ROS deps and keep include directories split with `BUILD_INTERFACE` and `INSTALL_INTERFACE`.
- Keep compiler warnings enabled (`-Wall -Wextra -Wpedantic`) as in package `CMakeLists.txt`.
- Follow existing ROS2 style in tests and nodes (`rclcpp`, `rclcpp_action`, explicit async wait helpers).

## Project-Specific Conventions
- String payloads on lifecycle/behavior topics use semicolon-delimited `key=value` format; parse with `parseKeyValuePayload(...)`.
- Nodes expose `*ForTest()` methods for deterministic assertions in gtests; prefer these over timing-sensitive black-box checks.
- `dog_behavior` behavior tree XML source is `src/dog_behavior/config/execute_trigger_tree.xml`; preserve node IDs and blackboard key compatibility when changing behavior flow.

## Testing Conventions
- Prefer GoogleTest with `ament_cmake_gtest`.
- For async ROS tests, use executor spin + timeout helpers (`waitUntil` style) and assert after condition satisfaction to reduce timing flakiness.
- When testing actions, mirror the existing mock action server pattern used in `src/dog_behavior/test/test_behavior_node.cpp`.

## Known Pitfalls
- If workspace path changes, stale `build/<pkg>/CMakeCache.txt` may reference old paths and break builds; clean affected `build/<pkg>` and `install/<pkg>`.
- `dog_perception` requires `vision_msgs`; install with `sudo apt install -y ros-humble-vision-msgs` when missing.
- Heartbeat reconnect logic is sensitive to timing windows; keep `reconnect_pending_timeout_ms < restart_window_ms` so retry attempts can accumulate.

## Representative Files
- Interfaces: `src/dog_interfaces/msg/Target3D.msg`, `src/dog_interfaces/action/ExecuteBehavior.action`
- Perception: `src/dog_perception/src/perception_node.cpp`, `src/dog_perception/test/test_perception_node.cpp`
- Lifecycle: `src/dog_lifecycle/src/lifecycle_node.cpp`, `src/dog_lifecycle/test/test_lifecycle_node.cpp`
- Behavior: `src/dog_behavior/src/behavior_node.cpp`, `src/dog_behavior/test/test_behavior_node.cpp`

## Reference Docs
- Keep this file minimal and executable. Link to docs instead of duplicating long explanations.
- `README.md` for quick start and system data flow.
- `docs/index.md` for documentation map.
- `docs/development-instructions.md` for development/deployment basics.
- `docs/architecture.md` and `docs/integration-architecture.md` for system-level design.
- `docs/interface-architecture.md` for interface contracts.
- `docs/behavior-nav2-action-mapping-spec.md` for behavior/navigation action mapping details.
