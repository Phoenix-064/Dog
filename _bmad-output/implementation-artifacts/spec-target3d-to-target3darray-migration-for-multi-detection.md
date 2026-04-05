---
title: 'target3d-to-target3darray-migration-for-multi-detection'
type: 'feature'
created: '2026-04-05'
status: 'in-progress'
baseline_commit: '2fa19930cc8a2ef81c49387555d1800d41634b90'
context:
  - 'project-context.md'
  - 'docs/interface-architecture.md'
  - 'docs/integration-architecture.md'
---

<frozen-after-approval reason="human-owned intent - do not modify unless human renegotiates">

## Intent

**Problem:** `Target3D` is a single-target message and current box detection path sends detections one-by-one. When multiple detections exist in one frame, the stream can experience bursty publish behavior and downstream consumers may observe partial frame results.

**Approach:** Introduce `Target3DArray` as the frame-level carrier for zero/one/many detections, migrate publishers/subscribers in perception and lifecycle to the new interface, and update tests so behavior is validated for multi-detection and no-detection cases.

## Boundaries & Constraints

**Always:** Keep ROS 2 Humble + ament_cmake conventions; keep package boundaries clear (`dog_interfaces` contracts, `dog_perception` production, `dog_lifecycle` consumption); use `rclcpp` logging only; preserve existing QoS policy unless explicitly required by tests.

**Ask First:** Any change that alters topic names, introduces new node processes, or changes lifecycle state-machine semantics beyond adapting message type.

**Never:** Do not edit `build/`, `install/`, `log/`; do not remove existing heartbeat/degrade safeguards; do not silently break backward compatibility without replacing all in-repo consumers.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| MULTI_DETECTION_FRAME | One image frame with N valid detections (N>=2) | Publish one `Target3DArray` containing exactly N ordered entries for that frame | If model parse fails, publish a valid empty/`no_box`-equivalent array contract as implemented |
| SINGLE_DETECTION_FRAME | One frame with one valid target | Publish one `Target3DArray` with one element | N/A |
| NO_DETECTION_FRAME | One frame with no valid detections | Publish one `Target3DArray` with empty list or explicit sentinel entry (consistent contract) | Log throttled reason without throwing |
| LIFECYCLE_VALID_FRAME_CHECK | Lifecycle receives `Target3DArray` | Valid-frame logic derives liveness from array payload according to migrated rule | Invalid payload ignored safely |

</frozen-after-approval>

## Code Map

- `src/dog_interfaces/msg/Target3D.msg` - Existing single-target message retained as element type.
- `src/dog_interfaces/CMakeLists.txt` - ROS interface generation list, add `Target3DArray.msg`.
- `src/dog_perception/src/box_detector_node.cpp` - Convert per-detection publish loop to one array publish per frame.
- `src/dog_perception/include/dog_perception/box_detector_node.hpp` - Publisher type migration.
- `src/dog_perception/src/perception_node.cpp` - Migrate target and digit publishers to array payload.
- `src/dog_perception/include/dog_perception/perception_node.hpp` - Publisher/cache type migration.
- `src/dog_perception/src/digit_recognizer_factory.cpp` - Adapt conversion output if needed for array wrapper.
- `src/dog_lifecycle/src/lifecycle_node.cpp` - Subscribe to `Target3DArray` and migrate valid-frame checks.
- `src/dog_lifecycle/include/dog_lifecycle/lifecycle_node.hpp` - Callback/subscription signature migration.
- `src/dog_perception/test/test_box_detector_node.cpp` - Update assertions for array payload count/content.
- `src/dog_perception/test/test_perception_node.cpp` - Update target/digit subscription and expectations.
- `src/dog_lifecycle/test/test_lifecycle_node.cpp` - Update valid-frame publisher helper and tests.

## Tasks & Acceptance

**Execution:**
- [x] `src/dog_interfaces/msg/Target3DArray.msg` -- add array message containing frame header and `Target3D[]` payload -- represent full-frame detections atomically.
- [x] `src/dog_interfaces/CMakeLists.txt` -- register `Target3DArray.msg` for code generation -- make new interface buildable.
- [x] `src/dog_perception/include/dog_perception/box_detector_node.hpp` and `src/dog_perception/src/box_detector_node.cpp` -- migrate publisher and publish path to single `Target3DArray` per frame -- eliminate burst of per-target messages.
- [x] `src/dog_perception/include/dog_perception/perception_node.hpp` and `src/dog_perception/src/perception_node.cpp` -- migrate target and digit result topics to `Target3DArray` -- keep one output message per processing cycle.
- [x] `src/dog_lifecycle/include/dog_lifecycle/lifecycle_node.hpp` and `src/dog_lifecycle/src/lifecycle_node.cpp` -- migrate valid-frame subscriber/callback logic to `Target3DArray` -- preserve heartbeat recovery semantics.
- [x] `src/dog_perception/test/test_box_detector_node.cpp` -- update tests for array structure, element count, no-box behavior -- validate multi-target correctness.
- [x] `src/dog_perception/test/test_perception_node.cpp` -- migrate subscriptions/assertions to array payload -- validate normal/extrapolated/digit outputs.
- [x] `src/dog_lifecycle/test/test_lifecycle_node.cpp` -- migrate helper publishers and valid-frame stimuli to `Target3DArray` -- keep existing lifecycle assertions valid.
- [x] workspace tests -- build and run package tests for changed modules -- verify regression-free migration.

**Acceptance Criteria:**
- Given one frame with multiple detections, when box/perception nodes publish result topics, then each topic emits one `Target3DArray` per frame and array size equals detection count.
- Given no detection, when detector callback executes, then output remains parseable and lifecycle does not crash or deadlock.
- Given lifecycle heartbeat monitoring is active, when valid frames are carried via `Target3DArray`, then reconnect recovery behavior remains equivalent to pre-migration semantics.
- Given `colcon build` and package tests for `dog_interfaces`, `dog_perception`, `dog_lifecycle`, when executed after migration, then all pass.

## Spec Change Log

## Verification

**Commands:**
- `source /opt/ros/humble/setup.bash && colcon build --packages-select dog_interfaces dog_perception dog_lifecycle` -- expected: build succeeds.
- `source /opt/ros/humble/setup.bash && colcon test --packages-select dog_perception dog_lifecycle && colcon test-result --all --verbose` -- expected: relevant tests pass without new failures.
