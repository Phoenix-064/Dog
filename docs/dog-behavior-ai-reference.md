# dog_behavior AI 开发参考（当前代码基线）

本文档描述的是当前代码基线（截至 2026-04-25），用于 AI 与开发者进行检索、影响分析、回归设计与后续迭代。

适用范围：`src/dog_behavior` 包。

---

## 1. 当前架构总览

当前已完成从旧双节点架构到单节点行为树架构的切换：

1. 运行入口为 `BehaviorTreeNode`（可执行：`dog_behavior_bt_node`）。
2. BT 叶子节点直接对接 `ExecuteBehavior`、Nav2 `NavigateToPose`、`PlaceBoxes`，不再使用独立导航代理节点。
3. `dog_behavior::utils` 提供字符串协议解析与 pose 校验公共工具。

数据主链路：

```mermaid
flowchart TD
  A[/behavior/execute_trigger/] --> B[BehaviorTreeNode::executeTriggerCallback]
  B --> C[tree_active=true]
  C --> D[Timer Tick -> tree.tickRoot()]

  E[/localization/dog/] --> F[BehaviorTreeNode::odomCallback]
  F --> G[/dog/global_pose/]
  F --> H[Blackboard.current_pose]

  I[/lifecycle/system_mode/] --> J[BehaviorTreeNode::systemModeCallback]
  K[/lifecycle/recovery_context/] --> L[BehaviorTreeNode::recoveryContextCallback]
  J --> M[Blackboard.system_mode]
  L --> N[Blackboard.recovery_context]

  O[ExecuteBehaviorAction] --> P[/behavior/execute Action Client]
  Q[NavigateToPoseAction] --> R[/navigate_to_pose Action Client]
  Q --> S[/behavior/nav_exec_state Topic]
  T[SetBoxesTypeAction] --> U[/target/box_result Topic]
  V[ExecutePlaceBoxesAction] --> W[/behavior/place_boxes Action Client]
  X[PublishMathAnswerAction] --> Y[/math_answer Topic]
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

入口与启动：

1. [src/dog_behavior/src/behavior_tree_main.cpp](../src/dog_behavior/src/behavior_tree_main.cpp)
2. [src/dog_behavior/launch/launch.py](../src/dog_behavior/launch/launch.py)

配置：

1. [src/dog_behavior/config/behavior_tree.xml](../src/dog_behavior/config/behavior_tree.xml)
2. [src/dog_behavior/config/behavior_tree_test.xml](../src/dog_behavior/config/behavior_tree_test.xml)
3. [src/dog_behavior/config/waypoints_left.yaml](../src/dog_behavior/config/waypoints_left.yaml)
4. [src/dog_behavior/config/waypoints_right.yaml](../src/dog_behavior/config/waypoints_right.yaml)

构建与依赖：

1. [src/dog_behavior/CMakeLists.txt](../src/dog_behavior/CMakeLists.txt)
2. [src/dog_behavior/package.xml](../src/dog_behavior/package.xml)
3. [src/dog_interfaces/action/ExecuteBehavior.action](../src/dog_interfaces/action/ExecuteBehavior.action)
4. [src/dog_interfaces/action/PlaceBoxes.action](../src/dog_interfaces/action/PlaceBoxes.action)

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

---

## 3. 组件职责

### 3.1 BehaviorTreeNode

实现入口：[src/dog_behavior/src/behavior_tree_node.cpp](../src/dog_behavior/src/behavior_tree_node.cpp)

职责：

1. 从 `behavior_tree.xml` 加载 BT，并注册所有内建叶子节点。
2. 订阅里程计并发布 `/dog/global_pose`，同时维护 `current_pose` 黑板数据。
3. 订阅 `/behavior/execute_trigger`，收到触发后将 `tree_active_` 置为 true。
4. 订阅 `/lifecycle/system_mode` 与 `/lifecycle/recovery_context`，将结果写入黑板。
5. 通过 `bt_tick_period_ms` 定时 `tickRoot()`，并在树终态（SUCCESS/FAILURE）后停止 tick。
6. 初始化并维护放置链路所需黑板键（counter、箱体计数、航点 goal 等）。

关键约束：

1. 只有 `tree_active_ == true` 才执行 tick。
2. odom 转发前必须通过 `isFinitePose` 与 `hasValidQuaternionNorm` 校验。
3. 系统模式从 `mode=...` 协议字段提取并归一化。

### 3.2 BT 叶子节点

条件节点：

1. `CheckSystemMode`：比较 `mode` 与 `expected_mode`（归一化后匹配）。
2. `WaitForPoseCondition`：检查 `has_pose`；当前实现为条件节点语义，不返回 RUNNING。

执行节点：

1. `ExecuteBehaviorAction`：调用 `/behavior/execute`（`dog_interfaces/action/ExecuteBehavior`）。
2. `NavigateToPoseAction`：调用 `/navigate_to_pose`（`nav2_msgs/action/NavigateToPose`）并发布 `/behavior/nav_exec_state`。
3. `SetBoxesTypeAction`：订阅 `/target/box_result`，按“两排排序”生成 `boxes_type_list`。
4. `AdvancePlaceCounterAction`：推进 `counter`，`counter > 7` 输出 `done=true`。
5. `PlaceRuleAction`：根据 `match_type + counter` 生成 `target_type` 与 `group_indices`。
6. `PlaceIndexAction`：生成 `local_indices`、`payload`、`count_after_success`、`has_target`。
7. `ExecutePlaceBoxesAction`：调用 `/behavior/place_boxes`（`dog_interfaces/action/PlaceBoxes`），成功且 accepted 时提交对应类型计数。
8. `PublishMathAnswerAction`：在指定航点匹配时向 `/math_answer` 发布答案。
9. `SelectWaypointAction`：可用航点选择节点（已注册，当前主树未启用）。

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

1. `/localization/dog`（可配）`nav_msgs/msg/Odometry`
2. `/behavior/execute_trigger`（可配）`std_msgs/msg/String`
3. `/lifecycle/recovery_context`（可配）`std_msgs/msg/String`
4. `/lifecycle/system_mode`（可配）`std_msgs/msg/String`

发布：

1. `/dog/global_pose`（可配）`geometry_msgs/msg/PoseStamped`

### 4.2 BT 叶子对外通信

Action Client：

1. `ExecuteBehaviorAction`：`/behavior/execute`，类型 `dog_interfaces/action/ExecuteBehavior`
2. `NavigateToPoseAction`：`/navigate_to_pose`，类型 `nav2_msgs/action/NavigateToPose`
3. `ExecutePlaceBoxesAction`：`/behavior/place_boxes`，类型 `dog_interfaces/action/PlaceBoxes`

Topic：

1. `NavigateToPoseAction` 发布 `/behavior/nav_exec_state`（`std_msgs/msg/String`）
2. `SetBoxesTypeAction` 订阅 `/target/box_result`（`dog_interfaces/msg/Target3DArray`）
3. `PublishMathAnswerAction` 发布 `/math_answer`（`std_msgs/msg/String`）

说明：

1. 当前运行架构中已无 `/behavior/navigate_execute` 内部 action server 路径。

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
2. `localization_topic` = `/localization/dog`
3. `default_frame_id` = `base_link`
4. `execute_behavior_trigger_topic` = `/behavior/execute_trigger`
5. `recovery_context_topic` = `/lifecycle/recovery_context`
6. `system_mode_topic` = `/lifecycle/system_mode`
7. `match_type` = `left`（仅允许 `left|right`，非法值回退 `left`）
8. `tree_xml_file_path` = `<share>/config/behavior_tree.xml`
9. `bt_tick_period_ms` = `100`
10. `waypoints_file` = `""`

### 6.2 关键 BT 端口默认值

1. `NavigateToPoseAction.action_name` = `/navigate_to_pose`
2. `NavigateToPoseAction.state_topic` = `/behavior/nav_exec_state`
3. `NavigateToPoseAction.feedback_timeout_sec` = `10.0`
4. `ExecuteBehaviorAction.action_name` = `/behavior/execute`
5. `ExecuteBehaviorAction.feedback_timeout_sec` = `2.0`
6. `ExecutePlaceBoxesAction.action_name` = `/behavior/place_boxes`
7. `ExecutePlaceBoxesAction.feedback_timeout_sec` = `2.0`
8. `ExecutePlaceBoxesAction.has_target` = `true`
9. `WaitForPose.timeout_ms` = `5000`
10. `PublishMathAnswerAction.answer` = `42`
11. `PublishMathAnswerAction.topic_name` = `/math_answer`

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

1. `NavigateToPoseAction(place_goal)`
2. `AdvancePlaceCounterAction`
3. `PlaceRuleAction`
4. `PlaceIndexAction`
5. `ForceSuccess(ExecutePlaceBoxesAction)`

规则补充：

1. `PlaceRuleAction` 使用 `group_a={0,1,5,6}`、`group_b={2,3,4,7}`。
2. `counter` 在 `[0,7]` 内按 `match_type` 映射目标箱类型；超出范围输出空目标。
3. `ExecutePlaceBoxesAction` 在主树中采用 fail-open（由 `ForceSuccess` 包裹）。

---

## 9. 测试与覆盖基线

当前 gtest 目标共 13 个：

1. `test_behavior_tree_node`：主节点参数、触发执行、模式阻断、XML 结构回归检查。
2. `test_payload_utils`：字符串协议解析、完成态判定、pose 有效性校验。
3. `test_check_system_mode`：模式匹配与不匹配路径。
4. `test_wait_for_pose_condition`：有/无 pose 条件语义。
5. `test_select_waypoint_action`：航点输出与索引轮转。
6. `test_execute_behavior_action`：`ExecuteBehavior` 异步动作成功路径。
7. `test_navigate_to_pose_action`：Nav2 导航叶子成功路径。
8. `test_set_boxes_type_action`：箱体排序与一次缓存语义。
9. `test_advance_place_counter_action`：计数推进与 done 边界。
10. `test_place_rule_action`：left/right 规则与非法 match_type。
11. `test_place_index_action`：索引筛选、payload 生成与无目标分支。
12. `test_execute_place_boxes_action`：accepted 成功提交计数、未 accepted 保持计数、无目标跳过。
13. `test_publish_math_answer_action`：指定航点发布与不匹配失败。

---

## 10. 构建与运行参考

构建定义：[src/dog_behavior/CMakeLists.txt](../src/dog_behavior/CMakeLists.txt)

当前目标：

1. `${PROJECT_NAME}_lib`
2. `${PROJECT_NAME}_bt_node`
3. 13 个 gtest 目标（见第 9 节）

推荐命令：

1. `source /opt/ros/humble/setup.bash`
2. `colcon build --packages-select dog_interfaces dog_behavior`
3. `colcon test --packages-select dog_behavior`
4. `colcon test-result --test-result-base build/dog_behavior/test_results --verbose`
5. `source install/setup.bash`
6. `ros2 launch dog_behavior launch.py`

---

## 11. 与重构计划的衔接

相关计划文档：[docs/behavior_tree_refactor_plan.md](behavior_tree_refactor_plan.md)

当前结论：

1. Phase 0/1/2/3 的核心代码路径已落地到当前基线。
2. `src/dog_behavior` 运行主线已统一为 `BehaviorTreeNode + BT 叶子`。
3. 当前阶段回归策略继续以包内单元测试为主。
