# dog_behavior AI 开发参考（当前代码基线）

本文档描述的是当前代码基线（截至 2026-07-04），用于 AI 与开发者进行检索、影响分析、回归设计与后续迭代。

适用范围：`src/dog_behavior` 包。

---

## 1. 当前架构总览

当前已完成从旧双节点架构到单节点行为树架构的切换：

1. 运行入口为 `BehaviorTreeNode`（可执行：`dog_behavior_bt_node`）。
2. BT 叶子节点直接对接 `ExecuteBehavior`、串口目标点导航 `NavigateWaypoint`、`PlaceBoxes`，不再使用 Nav2。
3. `test_mode:=true` 时加载导航/串口/PointLIO 测试树，抓取/放置由 `AutoSuccessAction` 自动放行。
4. `dog_behavior::utils` 提供字符串协议解析与 pose 校验公共工具。

数据主链路：

```mermaid
flowchart TD
  AA[auto_start=true] --> C[tree_active=true]
  A[/behavior/execute_trigger/] -. "手动重触发" .-> B[BehaviorTreeNode::executeTriggerCallback]
  B --> C
  C --> D[Timer Tick -> tree.tickRoot()]

  E[/aft_mapped_to_init/] --> F[BehaviorTreeNode::odomCallback]
  F --> G[/dog/global_pose/]
  F --> H[Blackboard.current_pose]

  I[/lifecycle/system_mode/] --> J[BehaviorTreeNode::systemModeCallback]
  K[/lifecycle/recovery_context/] --> L[BehaviorTreeNode::recoveryContextCallback]
  J --> M[Blackboard.system_mode]
  L --> N[Blackboard.recovery_context]

  O[ExecuteBehaviorAction] --> P[/behavior/execute Action Client]
  Q[NavigateWaypointAction] --> R[/behavior/nav_execute Action Client]
  Q --> S[/behavior/nav_exec_state Topic]
  Q --> Z[/behavior/nav_goal Topic]
  T[SetBoxesTypeAction] --> U[/target/target_3d Topic]
  V[ExecutePlaceBoxesAction] --> W[/behavior/place_boxes Action Client]
  W2[/target/digit_result Topic] --> X[PublishMathAnswerAction]
  X --> Y[/math_answer Topic]
  AA[AutoSuccessAction] --> AB[test_mode auto_success]
```

---

## 2. 关键文件索引

包目录：[src/dog_behavior](../src/dog_behavior)

头文件：

1. [src/dog_behavior/include/dog_behavior/behavior_tree_node.hpp](../src/dog_behavior/include/dog_behavior/behavior_tree_node.hpp)
2. [src/dog_behavior/include/dog_behavior/common/payload_utils.hpp](../src/dog_behavior/include/dog_behavior/common/payload_utils.hpp)
3. [src/dog_behavior/include/dog_behavior/bt_nodes/check_system_mode.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/check_system_mode.hpp)
4. [src/dog_behavior/include/dog_behavior/bt_nodes/wait_for_pose_condition.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/wait_for_pose_condition.hpp)
5. [src/dog_behavior/include/dog_behavior/bt_nodes/select_waypoint_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/select_waypoint_action.hpp)
6. [src/dog_behavior/include/dog_behavior/bt_nodes/execute_behavior_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/execute_behavior_action.hpp)
7. [src/dog_behavior/include/dog_behavior/bt_nodes/navigate_to_pose_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/navigate_to_pose_action.hpp)
8. [src/dog_behavior/include/dog_behavior/bt_nodes/set_boxes_type_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/set_boxes_type_action.hpp)
9. [src/dog_behavior/include/dog_behavior/bt_nodes/advance_place_counter_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/advance_place_counter_action.hpp)
10. [src/dog_behavior/include/dog_behavior/bt_nodes/place_rule_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/place_rule_action.hpp)
11. [src/dog_behavior/include/dog_behavior/bt_nodes/place_index_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/place_index_action.hpp)
12. [src/dog_behavior/include/dog_behavior/bt_nodes/execute_place_boxes_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/execute_place_boxes_action.hpp)
13. [src/dog_behavior/include/dog_behavior/bt_nodes/publish_math_answer_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/publish_math_answer_action.hpp)
14. [src/dog_behavior/include/dog_behavior/bt_nodes/auto_success_action.hpp](../src/dog_behavior/include/dog_behavior/bt_nodes/auto_success_action.hpp)

实现：

1. [src/dog_behavior/src/behavior_tree_node.cpp](../src/dog_behavior/src/behavior_tree_node.cpp)
2. [src/dog_behavior/src/behavior_tree_main.cpp](../src/dog_behavior/src/behavior_tree_main.cpp)
3. [src/dog_behavior/src/common/payload_utils.cpp](../src/dog_behavior/src/common/payload_utils.cpp)
4. [src/dog_behavior/src/bt_nodes/check_system_mode.cpp](../src/dog_behavior/src/bt_nodes/check_system_mode.cpp)
5. [src/dog_behavior/src/bt_nodes/wait_for_pose_condition.cpp](../src/dog_behavior/src/bt_nodes/wait_for_pose_condition.cpp)
6. [src/dog_behavior/src/bt_nodes/select_waypoint_action.cpp](../src/dog_behavior/src/bt_nodes/select_waypoint_action.cpp)
7. [src/dog_behavior/src/bt_nodes/execute_behavior_action.cpp](../src/dog_behavior/src/bt_nodes/execute_behavior_action.cpp)
8. [src/dog_behavior/src/bt_nodes/navigate_to_pose_action.cpp](../src/dog_behavior/src/bt_nodes/navigate_to_pose_action.cpp)
9. [src/dog_behavior/src/bt_nodes/set_boxes_type_action.cpp](../src/dog_behavior/src/bt_nodes/set_boxes_type_action.cpp)
10. [src/dog_behavior/src/bt_nodes/advance_place_counter_action.cpp](../src/dog_behavior/src/bt_nodes/advance_place_counter_action.cpp)
11. [src/dog_behavior/src/bt_nodes/place_rule_action.cpp](../src/dog_behavior/src/bt_nodes/place_rule_action.cpp)
12. [src/dog_behavior/src/bt_nodes/place_index_action.cpp](../src/dog_behavior/src/bt_nodes/place_index_action.cpp)
13. [src/dog_behavior/src/bt_nodes/execute_place_boxes_action.cpp](../src/dog_behavior/src/bt_nodes/execute_place_boxes_action.cpp)
14. [src/dog_behavior/src/bt_nodes/publish_math_answer_action.cpp](../src/dog_behavior/src/bt_nodes/publish_math_answer_action.cpp)
15. [src/dog_behavior/src/bt_nodes/auto_success_action.cpp](../src/dog_behavior/src/bt_nodes/auto_success_action.cpp)

入口与启动：

1. [src/dog_behavior/src/behavior_tree_main.cpp](../src/dog_behavior/src/behavior_tree_main.cpp)
2. [src/dog_behavior/launch/launch.py](../src/dog_behavior/launch/launch.py)

配置：

1. [src/dog_behavior/config/behavior_tree.xml](../src/dog_behavior/config/behavior_tree.xml)
2. [src/dog_behavior/config/behavior_tree_test.xml](../src/dog_behavior/config/behavior_tree_test.xml)
3. [src/dog_behavior/config/behavior_tree_nav_serial_test.xml](../src/dog_behavior/config/behavior_tree_nav_serial_test.xml)
4. [src/dog_behavior/config/waypoints_left.yaml](../src/dog_behavior/config/waypoints_left.yaml)
5. [src/dog_behavior/config/waypoints_right.yaml](../src/dog_behavior/config/waypoints_right.yaml)

构建与依赖：

1. [src/dog_behavior/CMakeLists.txt](../src/dog_behavior/CMakeLists.txt)
2. [src/dog_behavior/package.xml](../src/dog_behavior/package.xml)
3. [src/dog_interfaces/action/ExecuteBehavior.action](../src/dog_interfaces/action/ExecuteBehavior.action)
4. [src/dog_interfaces/action/NavigateWaypoint.action](../src/dog_interfaces/action/NavigateWaypoint.action)
5. [src/dog_interfaces/action/PlaceBoxes.action](../src/dog_interfaces/action/PlaceBoxes.action)

测试：

1. [src/dog_behavior/test/test_behavior_tree_node.cpp](../src/dog_behavior/test/test_behavior_tree_node.cpp)
2. [src/dog_behavior/test/test_payload_utils.cpp](../src/dog_behavior/test/test_payload_utils.cpp)
3. [src/dog_behavior/test/test_check_system_mode.cpp](../src/dog_behavior/test/test_check_system_mode.cpp)
4. [src/dog_behavior/test/test_wait_for_pose_condition.cpp](../src/dog_behavior/test/test_wait_for_pose_condition.cpp)
5. [src/dog_behavior/test/test_select_waypoint_action.cpp](../src/dog_behavior/test/test_select_waypoint_action.cpp)
6. [src/dog_behavior/test/test_execute_behavior_action.cpp](../src/dog_behavior/test/test_execute_behavior_action.cpp)
7. [src/dog_behavior/test/test_navigate_to_pose_action.cpp](../src/dog_behavior/test/test_navigate_to_pose_action.cpp)
8. [src/dog_behavior/test/test_set_boxes_type_action.cpp](../src/dog_behavior/test/test_set_boxes_type_action.cpp)
9. [src/dog_behavior/test/test_advance_place_counter_action.cpp](../src/dog_behavior/test/test_advance_place_counter_action.cpp)
10. [src/dog_behavior/test/test_place_rule_action.cpp](../src/dog_behavior/test/test_place_rule_action.cpp)
11. [src/dog_behavior/test/test_place_index_action.cpp](../src/dog_behavior/test/test_place_index_action.cpp)
12. [src/dog_behavior/test/test_execute_place_boxes_action.cpp](../src/dog_behavior/test/test_execute_place_boxes_action.cpp)
13. [src/dog_behavior/test/test_publish_math_answer_action.cpp](../src/dog_behavior/test/test_publish_math_answer_action.cpp)
14. [src/dog_behavior/test/test_auto_success_action.cpp](../src/dog_behavior/test/test_auto_success_action.cpp)

---

## 3. 组件职责

### 3.1 BehaviorTreeNode

实现入口：[src/dog_behavior/src/behavior_tree_node.cpp](../src/dog_behavior/src/behavior_tree_node.cpp)

职责：

1. 从 `behavior_tree.xml` 加载 BT，并注册所有内建叶子节点。
2. 订阅里程计并发布 `/dog/global_pose`，同时维护 `current_pose` 黑板数据。
3. 订阅 `/behavior/execute_trigger`，支持手动重触发；默认 `auto_start=true` 启动后自动开始 tick，无需外部触发。
4. 订阅 `/lifecycle/system_mode` 与 `/lifecycle/recovery_context`，将结果写入黑板。
5. 通过 `bt_tick_period_ms` 定时 `tickRoot()`，并在树终态（SUCCESS/FAILURE）后停止 tick。
6. 初始化并维护放置链路所需黑板键（counter、箱体计数、航点 goal 等）。
7. 支持通过 `tree_xml_file_path` 参数切换行为树；launch 在 `test_mode=true` 时切到测试树。

关键约束：

1. 只有 `tree_active_ == true` 才执行 tick；`auto_start=true`（默认）意味着启动后自动进入激活状态。
2. odom 转发前必须通过 `isFinitePose` 与 `hasValidQuaternionNorm` 校验。
3. 系统模式从 `mode=...` 协议字段提取并归一化。

### 3.2 BT 叶子节点

条件节点：

1. `CheckSystemMode`：比较 `mode` 与 `expected_mode`（归一化后匹配）。
2. `WaitForPoseCondition`：检查 `has_pose`；未收到 pose 时在 `timeout_ms` 内返回 RUNNING，超时后返回 FAILURE，避免触发早于定位输入时立即中止整棵树。

执行节点：

1. `ExecuteBehaviorAction`：调用 `/behavior/execute`（`dog_interfaces/action/ExecuteBehavior`）。
2. `NavigateWaypointAction`：调用 `/behavior/nav_execute`（`dog_interfaces/action/NavigateWaypoint`），并发布 `/behavior/nav_exec_state` 与 `/behavior/nav_goal`。
3. `SetBoxesTypeAction`：订阅 `/target/target_3d`，按“两排排序”生成 `boxes_type_list`。
4. `AdvancePlaceCounterAction`：推进 `counter`，`counter > 7` 输出 `done=true`。
5. `PlaceRuleAction`：根据 `match_type + counter` 生成 `target_type` 与 `group_indices`。
6. `PlaceIndexAction`：生成 `local_indices`、`payload`、`count_after_success`、`has_target`。
7. `ExecutePlaceBoxesAction`：调用 `/behavior/place_boxes`（`dog_interfaces/action/PlaceBoxes`），成功且 accepted 时提交对应类型计数。
8. `PublishMathAnswerAction`：订阅 `/target/digit_result`，在指定航点匹配时向 `/math_answer` 发布答案。
9. `AutoSuccessAction`：测试模式专用同步叶子，记录 `test_mode_auto_success` 日志并直接返回 SUCCESS。
10. `SelectWaypointAction`：可用航点选择节点（已注册，当前主树未启用）。

### 3.3 payload_utils

头文件：[src/dog_behavior/include/dog_behavior/common/payload_utils.hpp](../src/dog_behavior/include/dog_behavior/common/payload_utils.hpp)

实现：[src/dog_behavior/src/common/payload_utils.cpp](../src/dog_behavior/src/common/payload_utils.cpp)

函数：

1. `normalizeToken`
2. `parseKeyValuePayload`
3. `isCompletedState`
4. `isFinitePose(Pose)`
5. `isFinitePose(PoseStamped)`
6. `hasValidQuaternionNorm(Pose)`
7. `hasValidQuaternionNorm(PoseStamped)`

说明：

1. `percentDecode` 是内部实现细节，不对外暴露。
2. 所有生命周期/恢复字符串协议建议统一经该工具处理。

---

## 4. 外部接口字典

### 4.1 BehaviorTreeNode（Topic）

订阅：

1. `/aft_mapped_to_init`（可配）`nav_msgs/msg/Odometry`
2. `/behavior/execute_trigger`（可配，`auto_start=true` 时可选）`std_msgs/msg/String`
3. `/lifecycle/recovery_context`（可配）`std_msgs/msg/String`
4. `/lifecycle/system_mode`（可配）`std_msgs/msg/String`

发布：

1. `/dog/global_pose`（可配）`geometry_msgs/msg/PoseStamped`

### 4.2 BT 叶子对外通信

Action Client：

1. `ExecuteBehaviorAction`：`/behavior/execute`，类型 `dog_interfaces/action/ExecuteBehavior`
2. `NavigateWaypointAction`：`/behavior/nav_execute`，类型 `dog_interfaces/action/NavigateWaypoint`
3. `ExecutePlaceBoxesAction`：`/behavior/place_boxes`，类型 `dog_interfaces/action/PlaceBoxes`

Topic：

1. `NavigateWaypointAction` 发布 `/behavior/nav_exec_state`（`std_msgs/msg/String`）
2. `NavigateWaypointAction` 发布 `/behavior/nav_goal`（`geometry_msgs/msg/PoseStamped`），供观测与串口导航节点兼容链路使用。
3. `SetBoxesTypeAction` 订阅 `/target/target_3d`（`dog_interfaces/msg/Target3DArray`）
4. `PublishMathAnswerAction` 订阅 `/target/digit_result`（`dog_interfaces/msg/Target3DArray`），并发布 `/math_answer`（`std_msgs/msg/String`）

说明：

1. 当前运行架构中已无 Nav2 `/navigate_to_pose` 路径；目标点导航完成由 `dog_serial_bridge_nav_telemetry_node` 等待 MCU `RCArrivalMX` 回包判定。
2. 统一 launch 默认不启动 `/behavior/nav_execute`、`/behavior/execute`、`/behavior/place_boxes` 的 Action Server；正式树需要启用 `use_nav_telemetry_serial:=true`、`use_serial_bridge:=true`，或由外部节点提供同名 server。
3. `/behavior/nav_goal` 当前有两个发布源：`NavigateWaypointAction` 在发送 goal 前发布一次，`NavTelemetrySerialNode` 在接收 action goal 后也会发布一次；`/behavior/nav_exec_state` 只由 `NavigateWaypointAction` 发布。

---

## 5. 字符串协议与负载规范

统一解析入口：`dog_behavior::utils::parseKeyValuePayload`

基础格式：

1. `key=value;key=value;...`

规则：

1. 键名比较前执行 `normalizeToken`（去空白、小写化）。
2. 值支持 `%xx` 百分号解码（内部实现）。
3. 缺失键返回空字符串。

当前关键字段：

1. `mode`：系统模式字段（如 `mode=normal` / `mode=idle_spinning`）。
2. `target_state`：完成态判定（`done/completed/succeeded/success/finished`）。

放置链路 payload（由 `PlaceIndexAction` 生成）：

1. 示例：`place=0,3,count=3`
2. 语义：`place` 是组内索引集合，`count` 为该类型成功后累计计数。

---

## 6. 参数与端口默认值

### 6.1 BehaviorTreeNode 参数默认值

1. `global_pose_topic` = `/dog/global_pose`
2. `localization_topic` = `/aft_mapped_to_init`
3. `default_frame_id` = `base_link`
4. `execute_behavior_trigger_topic` = `/behavior/execute_trigger`
5. `recovery_context_topic` = `/lifecycle/recovery_context`
6. `system_mode_topic` = `/lifecycle/system_mode`
7. `match_type` = `left`（仅允许 `left|right`，非法值回退 `left`）
8. `tree_xml_file_path` = `<share>/config/behavior_tree.xml`
9. `bt_tick_period_ms` = `100`
10. `auto_start` = `true` — 启动后是否自动开始 tick，设为 false 则需手动发 `/behavior/execute_trigger`
11. `waypoints_file` = `""`

注意：当前 `dog_behavior/launch/launch.py` 没有声明或透传 `auto_start`，通过统一 launch 启动时不能直接使用 `auto_start:=false`。

### 6.2 关键 BT 端口默认值

1. `NavigateWaypointAction.action_name` = `/behavior/nav_execute`
2. `NavigateWaypointAction.state_topic` = `/behavior/nav_exec_state`
3. `NavigateWaypointAction.goal_topic` = `/behavior/nav_goal`
4. `NavigateWaypointAction.feedback_timeout_sec` = `10.0`
5. `NavigateWaypointAction.server_timeout_sec` = `1.0`
6. `ExecuteBehaviorAction.action_name` = `/behavior/execute`
7. `ExecuteBehaviorAction.feedback_timeout_sec` = `2.0`
8. `ExecutePlaceBoxesAction.action_name` = `/behavior/place_boxes`
9. `ExecutePlaceBoxesAction.feedback_timeout_sec` = `2.0`
10. `ExecutePlaceBoxesAction.has_target` = `true`
11. `WaitForPose.timeout_ms` = `5000`
12. `PublishMathAnswerAction.answer` = `42`
13. `PublishMathAnswerAction.topic_name` = `/math_answer`
14. `AutoSuccessAction.label` = `auto_success`
15. `AutoSuccessAction.result_code_text` 输出 `auto_success`

`NavigateWaypointAction` 的必填端口只有 `goal`。`action_name`、`state_topic`、`goal_topic`、`feedback_timeout_sec`、`server_timeout_sec` 均可使用默认值，便于测试树只声明目标点而不重复配置通信端口。

---

## 7. 黑板与运行状态

### 7.1 BehaviorTreeNode 黑板初始化键

1. `system_mode`、`match_type`、`recovery_context`、`behavior_name`
2. `has_current_pose`、`current_pose`
3. `counter`（初值 -1）
4. `food_box_count`、`tool_box_count`、`instrument_box_count`、`medical_box_count`
5. `boxes_type_list`、`boxes_ready`、`boxes_capture_stamp`
6. `waypoints`、`waypoint_index`
7. `WayPointGoal1..4`、`PlaceGoal1..4`
8. `ros_node`

### 7.2 节点内状态（ForTest 可见）

1. `TickCountForTest()`：累计 tick 次数。
2. `LastTickStatusForTest()`：`idle|running|success|failure`。
3. `SystemModeForTest()`：解析后的 mode。
4. `BehaviorNameForTest()`：最近触发行为名。
5. `HasLatestPoseForTest()`：是否已有有效 pose。
6. `IsTreeActiveForTest()`：当前是否处于执行状态。
7. `WaypointCountForTest()`：已加载航点数量。

---

## 8. 行为树编排（`behavior_tree.xml`）

主树 `MainTree`：

1. `CheckSystemMode(mode=normal)`
2. `WaitForPose`
3. `SetBoxesTypeAction`
4. 导航到 `WayPointGoal1 -> WayPointGoal2 -> WayPointGoal3`
5. `PublishMathAnswerAction`
6. `ExecuteBehaviorAction(behavior_name=PickUpBoxes)`
7. `PlaceAtGoal` 子树正序：`PlaceGoal1 -> PlaceGoal2 -> PlaceGoal3 -> PlaceGoal4`
8. 导航到 `WayPointGoal4`
9. `ExecuteBehaviorAction(behavior_name=PickUpBoxes)`
10. `PlaceAtGoal` 子树逆序：`PlaceGoal4 -> PlaceGoal3 -> PlaceGoal2 -> PlaceGoal1`

子树 `PlaceAtGoal`：

1. `NavigateWaypointAction(place_goal)`
2. `AdvancePlaceCounterAction`
3. `PlaceRuleAction`
4. `PlaceIndexAction`
5. `ForceSuccess(ExecutePlaceBoxesAction)`

规则补充：

1. `PlaceRuleAction` 使用 `group_a={0,1,5,4}`、`group_b={2,3,7,6}`。
2. `counter` 在 `[0,7]` 内按 `match_type` 映射目标箱类型；超出范围输出空目标。
3. `ExecutePlaceBoxesAction` 在主树中采用 fail-open（由 `ForceSuccess` 包裹）；放置 action 失败不会让整棵树失败，但只有 action 成功且 `accepted=true` 时才提交对应箱型计数。
4. waypoint YAML 中的 `yaw` 被代码按弧度转换为四元数，没有角度到弧度转换。

---

## 9. 导航/串口/PointLIO 测试树

测试入口：`ros2 launch dog_behavior launch.py test_mode:=true ...`

测试树文件：`behavior_tree_nav_serial_test.xml`。

运行语义：

1. 保留 `CheckSystemMode` 与 `WaitForPose`，仍要求 lifecycle mode 为 normal 且已有 point_lio 里程计位姿。
2. 保留所有 `NavigateWaypointAction`，继续向 `/behavior/nav_execute` 发送目标；目标点串口节点仍等待 `RCArrivalMX`。
3. 不包含 `SetBoxesTypeAction` 与 `PublishMathAnswerAction`，因此不等待 `/target/target_3d`、不触发 YOLO/视觉识别。
4. 两次抓取与每个放置步骤使用 `AutoSuccessAction`，只记录 `test_mode_auto_success` 日志并返回 SUCCESS。
5. launch 在 `test_mode=true` 时不启动 `dog_perception_node`、`dog_perception_camera_node`、`dog_serial_bridge_node`，但仍允许启动 Livox、point_lio 与 `dog_serial_bridge_nav_telemetry_node`。
6. launch 在 `test_mode=true` 时将 `dog_lifecycle_node.heartbeat_timeout_ms` 设为 `600000`，避免无视觉帧时过早降级。

推荐命令：

```bash
ros2 launch dog_behavior launch.py test_mode:=true use_livox:=true livox_model:=mid360 use_point_lio:=true use_nav_telemetry_serial:=true nav_telemetry_serial_port:=/dev/ttyUSB1
```

实机串口验证记录：

1. 已使用 CH340 `/dev/ttyUSB0` 验证 `behavior_tree_nav_serial_test.xml` 可通过真实测试程序发送航点。
2. 启动后行为树自动开始执行（`auto_start=true`），日志出现 `BT tick completed` 与 `nav_telemetry_serial_tx ... RCNAV;...`。
3. 左侧航点文件中 `waypoint_goal_1` 当前为 `x=0.0, y=1.0, yaw=1.57`，串口帧对应 `goal_x=0.000;goal_y=100.000;goal_yaw=1.570`；`goal_y` 为厘米单位。
4. 下位机测试固件返回的 `0`、`1,0,0`、`0,-1,0` 等非协议行会被记录为 `nav_telemetry_serial_rx`，并因不是 `RCArrivalMX` 产生 `nav_serial_line_unmatched`。该现象只说明测试固件未返回正式到达帧，不表示串口发送失败。

---

## 10. 测试与覆盖基线

当前 gtest 目标共 15 个：

1. `test_behavior_tree_node`：主节点参数、触发执行、模式阻断、XML 结构回归检查。
2. `test_payload_utils`：字符串协议解析、完成态判定、pose 有效性校验。
3. `test_check_system_mode`：模式匹配与不匹配路径。
4. `test_wait_for_pose_condition`：有/无 pose 条件语义。
5. `test_select_waypoint_action`：航点输出与索引轮转。
6. `test_execute_behavior_action`：`ExecuteBehavior` 异步动作成功路径。
7. `test_navigate_waypoint_action`：串口目标点导航叶子成功路径（测试源文件仍为 `test_navigate_to_pose_action.cpp`）。
8. `test_set_boxes_type_action`：箱体排序与一次缓存语义。
9. `test_advance_place_counter_action`：计数推进与 done 边界。
10. `test_place_rule_action`：left/right 规则与非法 match_type。
11. `test_place_index_action`：索引筛选、payload 生成与无目标分支。
12. `test_execute_place_boxes_action`：accepted 成功提交计数、未 accepted 保持计数、无目标跳过。
13. `test_publish_math_answer_action`：指定航点发布与不匹配失败。
14. `test_auto_success_action`：测试模式自动成功叶子输出 `auto_success`。
15. `test_test_visualization_node`：test mode 路线、目标、状态可视化输出。

---

## 11. 构建与运行参考

构建定义：[src/dog_behavior/CMakeLists.txt](../src/dog_behavior/CMakeLists.txt)

当前目标：

1. `${PROJECT_NAME}_lib`
2. `${PROJECT_NAME}_bt_node`
3. 15 个 gtest 目标（见第 10 节）

推荐命令：

1. `source /opt/ros/humble/setup.bash`
2. `colcon build --packages-select dog_interfaces dog_serial_bridge dog_behavior`
3. `colcon test --packages-select dog_serial_bridge dog_behavior`
4. `colcon test-result --all --verbose`
5. `source install/setup.bash`
6. `ros2 launch dog_behavior launch.py`
7. `ros2 launch dog_behavior launch.py test_mode:=true use_livox:=true livox_model:=mid360 use_point_lio:=true use_nav_telemetry_serial:=true nav_telemetry_serial_port:=/dev/ttyUSB1`

---

## 12. 历史重构结论

旧 `BehaviorNode` / `NavigationExecutorNode` 双节点路径已退场。当前运行主线统一为 `BehaviorTreeNode + BT 叶子`，目标点导航通过 `NavigateWaypointAction` 调用 `/behavior/nav_execute`，不再维护 Nav2 `/navigate_to_pose` 路径。
