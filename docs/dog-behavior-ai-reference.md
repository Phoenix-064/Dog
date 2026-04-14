# dog_behavior AI 开发查询文档

本文档面向 AI 辅助开发与代码检索，覆盖整个 `dog_behavior` 包，而不仅限于导航执行子模块。

目标：

1. 快速定位跨节点执行链路（BehaviorNode -> NavigationExecutorNode -> Nav2）。
2. 明确包内外接口（Topic、Action、参数、字符串协议、状态机）。
3. 提供可直接跳转的源码与测试锚点，支持影响分析与回归设计。

## 1. 包定位与核心文件

包路径：[src/dog_behavior](../src/dog_behavior)

核心头文件：

1. [src/dog_behavior/include/dog_behavior/behavior_node.hpp](../src/dog_behavior/include/dog_behavior/behavior_node.hpp)
2. [src/dog_behavior/include/dog_behavior/navigation_executor_node.hpp](../src/dog_behavior/include/dog_behavior/navigation_executor_node.hpp)
3. [src/dog_behavior/include/dog_behavior/behavior_tree.hpp](../src/dog_behavior/include/dog_behavior/behavior_tree.hpp)

核心实现：

1. [src/dog_behavior/src/behavior_node.cpp](../src/dog_behavior/src/behavior_node.cpp)
2. [src/dog_behavior/src/navigation_executor_node.cpp](../src/dog_behavior/src/navigation_executor_node.cpp)
3. [src/dog_behavior/src/behavior_tree.cpp](../src/dog_behavior/src/behavior_tree.cpp)

入口与运行编排：

1. [src/dog_behavior/src/main.cpp](../src/dog_behavior/src/main.cpp)
2. [src/dog_behavior/src/navigation_executor_main.cpp](../src/dog_behavior/src/navigation_executor_main.cpp)
3. [src/dog_behavior/launch/launch.py](../src/dog_behavior/launch/launch.py)

配置文件：

1. [src/dog_behavior/config/execute_trigger_tree.xml](../src/dog_behavior/config/execute_trigger_tree.xml)
2. [src/dog_behavior/config/waypoints_left.yaml](../src/dog_behavior/config/waypoints_left.yaml)
3. [src/dog_behavior/config/waypoints_right.yaml](../src/dog_behavior/config/waypoints_right.yaml)

构建与依赖：

1. [src/dog_behavior/CMakeLists.txt](../src/dog_behavior/CMakeLists.txt)
2. [src/dog_behavior/package.xml](../src/dog_behavior/package.xml)

测试：

1. [src/dog_behavior/test/test_behavior_node.cpp](../src/dog_behavior/test/test_behavior_node.cpp)
2. [src/dog_behavior/test/test_navigation_executor_node.cpp](../src/dog_behavior/test/test_navigation_executor_node.cpp)
3. [src/dog_behavior/test/test_behavior_tree.cpp](../src/dog_behavior/test/test_behavior_tree.cpp)

## 2. 包级架构与职责

`dog_behavior` 由三个核心组件组成：

1. `BehaviorNode`：
  1. 订阅里程计并发布全局位姿。
  2. 接收行为触发并驱动行为树。
  3. 作为 ExecuteBehavior 和内部导航动作客户端，编排行为执行。
  4. 处理 recovery/system_mode 约束，并维护本地执行状态。
2. `NavigationExecutorNode`：
  1. 对内提供 `/behavior/navigate_execute` Action Server。
  2. 对外作为 Nav2 `/navigate_to_pose` Action Client。
  3. 维护导航执行状态并发布到 `/behavior/nav_exec_state`。
  4. 根据 recovery/system_mode 决定是否接受或取消导航。
3. `BehaviorTree`：
  1. 从 XML 加载固定序列树。
  2. 注册三个动作叶子：`TriggerExecuteBehavior`、`TriggerNavigateGoal`、`PlaceholderBehavior`。
  3. 负责顺序执行与失败短路。

### 2.1 关键调用关系

```mermaid
flowchart TD
  A[/behavior/execute_trigger/] --> B[BehaviorNode::executeTriggerCallback]
  B --> C[BehaviorTree::execute]
  C --> D[TriggerExecuteBehavior]
  D --> E[/behavior/execute Action Client]
  C --> F[TriggerNavigateGoal]
  F --> G[/behavior/navigate_execute Action Client]
  G --> H[NavigationExecutorNode Action Server]
  H --> I[/navigate_to_pose Nav2 Action Client]
  J[/lifecycle/recovery_context/] --> B
  K[/lifecycle/system_mode/] --> B
  J --> H
  K --> H
```

## 3. BehaviorNode 参考

实现文件：[src/dog_behavior/src/behavior_node.cpp](../src/dog_behavior/src/behavior_node.cpp)

### 3.1 运行时主线

1. `odomCallback`：里程计转 `PoseStamped`，做 frame 回退与位姿合法性检查，更新 `latest_pose_` 并发布。
2. `executeTriggerCallback`：读取触发字符串，调用 `behavior_tree_.execute()`。
3. `triggerExecuteBehavior`：发送 `dog_interfaces/action/ExecuteBehavior` 目标。
4. `triggerNavigateGoal`：发送内部 `nav2_msgs/action/NavigateToPose` 目标到导航执行器。
5. `recoveryContextCallback`：维护 `recovered_completed_task_phases_`。
6. `systemModeCallback`：在 `idle_spinning/degraded` 模式下阻断新目标并取消在途目标。

### 3.2 执行前置条件

`triggerExecuteBehavior` 发送目标前必须满足：

1. 非 `idle_spinning/degraded` 模式。
2. 行为名未被恢复过滤集合阻断。
3. `execute_behavior` action server 已就绪且无在途目标。
4. `latest_pose_` 可用。

`triggerNavigateGoal` 发送内部导航目标前必须满足：

1. 非 `idle_spinning/degraded` 模式。
2. `navigate_execute` action server 已就绪。
3. 无在途导航目标。
4. 以下任一条件成立：
   - `waypoints_` 非空（将使用预设航点作为目标）。
   - `waypoints_` 为空且 `latest_pose_` 可用（向后兼容模式）。

### 3.3 航点加载与使用

1. **航点加载**：节点启动时通过 `waypoints_file` 参数加载 YAML 格式的航点文件，使用 `yaml-cpp` 解析。
2. **航点查找**：`triggerNavigateGoal` 根据 `behavior_name` 在 `waypoints_` 中查找同名航点；若未找到且航点列表非空，则按 `current_waypoint_index_` 顺序取用。
3. **目标转换**：`waypointToPoseStamped` 将航点结构体转换为 `PoseStamped`，其中 `yaw` 转换为绕 Z 轴的四元数。
4. **向后兼容**：若 `waypoints_` 为空，则回退到使用 `latest_pose_` 作为导航目标。

### 3.4 反馈看门狗

1. 行为动作看门狗：`feedbackWatchdogTimerCallback`。
2. 导航动作看门狗：`navigate_feedback_watchdog_timer_` lambda。
3. 两者都按 `feedback_timeout_sec` 判断超时，超时后：
  1. 状态置为 `timeout`。
  2. 清理本地活动句柄。
  3. 异步 cancel 对应 action goal。

### 3.5 可查询接口

1. `getExecutionState()`：返回当前状态字符串。
2. `IsTaskPhaseRecoveredForTest()`：测试辅助，查询任务阶段是否被恢复过滤。
3. `IsIdleSpinningForTest()`：测试辅助，查询当前是否在空转/退化模式。

## 4. NavigationExecutorNode 参考

实现文件：[src/dog_behavior/src/navigation_executor_node.cpp](../src/dog_behavior/src/navigation_executor_node.cpp)

### 4.1 运行时主线

1. `handleGoal`：内部导航目标接收校验。
2. `handleAccepted`：转发给 Nav2。
3. `nav2GoalResponseCallback`：处理 Nav2 接受/拒绝。
4. `nav2FeedbackCallback`：透传 feedback 并刷新反馈时间。
5. `nav2ResultCallback`：映射结果并完成内部 goal。
6. `nav2ServerWaitTimerCallback`：启动期等待 Nav2 server。
7. `feedbackWatchdogTimerCallback`：导航反馈超时处理。

### 4.2 `handleGoal` 接收条件

1. Nav2 server ready，且无 active/pending 目标。
2. 不在 `idle_spinning/degraded` 模式。
3. 未被 recovery completed 状态阻断。
4. 位姿通过有限值和四元数范数检查。

### 4.3 结果映射语义

`mapNav2ResultState` 对 `ResultCode::CANCELED` 细分为三类：

1. 由 `idle_spinning/degraded` 触发取消：状态 `idle`。
2. 由反馈超时触发取消：状态 `timeout`。
3. 其他取消：状态 `failed`。

其他分支：

1. `SUCCEEDED -> succeeded`
2. `ABORTED/默认 -> failed`

### 4.4 输入校验边界

1. `isFinitePose`：拒绝 NaN/Inf。
2. `hasValidQuaternionNorm`：
  1. 四元数范数需大于 `1e-6`。
  2. 目标范数为 1.0，容差为 0.1，即接受区间约 `[0.9, 1.1]`。

### 4.5 可查询接口

1. `getExecutionState()`：返回当前导航执行状态字符串。

## 5. BehaviorTree 参考

实现文件：[src/dog_behavior/src/behavior_tree.cpp](../src/dog_behavior/src/behavior_tree.cpp)

### 5.1 固定执行语义

1. XML 路径来自 `execute_trigger_tree.xml`。
2. 运行时向 blackboard 注入 `behavior_name`。
3. 树执行结果以 `tickRoot() == SUCCESS` 作为整体成功标准。

### 5.2 注册叶子节点

1. `TriggerExecuteBehavior`：调用外部注入的 `ActionCallback`。
2. `TriggerNavigateGoal`：调用外部注入的 `NavigateActionCallback`。
3. `PlaceholderBehavior`：调用 `PlaceholderCallback`，默认无业务副作用。

### 5.3 失败传播

1. 任一叶子失败会导致序列失败并短路后续节点。
2. XML 缺失或解析异常时，`execute()` 返回 `false`。

## 6. 外部接口字典

### 6.1 BehaviorNode

Action 客户端：

1. `execute_behavior_action_name`，默认 `/behavior/execute`，类型 `dog_interfaces/action/ExecuteBehavior`。
2. `navigate_execute_action_name`，默认 `/behavior/navigate_execute`，类型 `nav2_msgs/action/NavigateToPose`。

Topic 订阅：

1. `localization_topic`，默认 `/localization/dog`，类型 `nav_msgs/msg/Odometry`，`SensorDataQoS keep_last(20)`。
2. `execute_behavior_trigger_topic`，默认 `/behavior/execute_trigger`，类型 `std_msgs/msg/String`，Reliable keep_last(10)。
3. `recovery_context_topic`，默认 `/lifecycle/recovery_context`，类型 `std_msgs/msg/String`，Reliable + TransientLocal keep_last(1)。
4. `system_mode_topic`，默认 `/lifecycle/system_mode`，类型 `std_msgs/msg/String`，Reliable + TransientLocal keep_last(1)。

Topic 发布：

1. `global_pose_topic`，默认 `/dog/global_pose`，类型 `geometry_msgs/msg/PoseStamped`，Reliable keep_last(20)。

### 6.2 NavigationExecutorNode

Action 服务端：

1. `navigate_execute_action_name`，默认 `/behavior/navigate_execute`，类型 `nav2_msgs/action/NavigateToPose`。

Action 客户端：

1. `nav2_action_name`，默认 `/navigate_to_pose`，类型 `nav2_msgs/action/NavigateToPose`。

Topic 订阅：

1. `recovery_context_topic`，默认 `/lifecycle/recovery_context`，类型 `std_msgs/msg/String`，Reliable + TransientLocal keep_last(1)。
2. `system_mode_topic`，默认 `/lifecycle/system_mode`，类型 `std_msgs/msg/String`，Reliable + TransientLocal keep_last(1)。

Topic 发布：

1. `navigate_execution_state_topic`，默认 `/behavior/nav_exec_state`，类型 `std_msgs/msg/String`，Reliable keep_last(10)。

## 7. 字符串协议

解析函数（BehaviorNode 与 NavigationExecutorNode 各自实现，语义一致）：

1. [src/dog_behavior/src/behavior_node.cpp](../src/dog_behavior/src/behavior_node.cpp)
2. [src/dog_behavior/src/navigation_executor_node.cpp](../src/dog_behavior/src/navigation_executor_node.cpp)

格式与处理规则：

1. 负载格式：`key=value;key=value;...`
2. 键名规范化：去空白、转小写。
3. 值解码：支持 `%xx` 百分号编码。

关键键与语义：

1. `mode`
  1. 在 `recovery_context` 中可取 `cold_start`、`recovered`。
  2. 在 `system_mode` 中重点处理 `idle_spinning`、`degraded`。
2. `task_phase`
  1. BehaviorNode 使用该键维护恢复过滤集合。
3. `target_state`
  1. 完成态关键字：`done/completed/succeeded/success/finished`。

## 8. 参数清单与默认值

### 8.1 BehaviorNode 参数

1. `global_pose_topic`：`/dog/global_pose`
2. `localization_topic`：`/localization/dog`
3. `default_frame_id`：`base_link`
4. `execute_behavior_action_name`：`/behavior/execute`
5. `execute_behavior_trigger_topic`：`/behavior/execute_trigger`
6. `navigate_execute_action_name`：`/behavior/navigate_execute`
7. `recovery_context_topic`：`/lifecycle/recovery_context`
8. `system_mode_topic`：`/lifecycle/system_mode`
9. `match_type`：`left`（比赛类型，决定加载 `waypoints_left.yaml` 或 `waypoints_right.yaml`）
10. `waypoints_file`：`""`（航点 YAML 文件的绝对路径，由 launch 文件根据 `match_type` 自动填充）
11. `action_server_wait_timeout_sec`：`5.0`（`<=0` 回退 `5.0`）
12. `feedback_timeout_sec`：`2.0`（`<=0` 回退 `2.0`）

### 8.2 NavigationExecutorNode 参数

1. `navigate_execute_action_name`：`/behavior/navigate_execute`
2. `nav2_action_name`：`/navigate_to_pose`
3. `navigate_execution_state_topic`：`/behavior/nav_exec_state`
4. `recovery_context_topic`：`/lifecycle/recovery_context`
5. `system_mode_topic`：`/lifecycle/system_mode`
6. `nav2_server_wait_timeout_sec`：`10.0`（`<=0` 回退 `10.0`）
7. `navigate_feedback_timeout_sec`：`10.0`（`<=0` 回退 `10.0`）

## 9. 状态模型

### 9.1 BehaviorNode 状态

定义见：[src/dog_behavior/include/dog_behavior/behavior_node.hpp](../src/dog_behavior/include/dog_behavior/behavior_node.hpp)

状态集合：

1. `idle`
2. `waiting_server`
3. `server_unavailable`
4. `sending_goal`
5. `running`
6. `succeeded`
7. `failed`
8. `rejected`
9. `timeout`

关键迁移：

1. 启动后等待 execute action server：`waiting_server -> idle|server_unavailable`。
2. 发送执行目标：`idle -> sending_goal -> running`。
3. 终态收敛：`running -> succeeded|failed|rejected|timeout|idle`。
4. `idle` 分支主要出现在 `system_mode` 抢占取消路径。

### 9.2 NavigationExecutorNode 状态

定义见：[src/dog_behavior/include/dog_behavior/navigation_executor_node.hpp](../src/dog_behavior/include/dog_behavior/navigation_executor_node.hpp)

状态集合：

1. `idle`
2. `waiting_nav2_server`
3. `nav2_server_unavailable`
4. `forwarding_goal`
5. `running`
6. `succeeded`
7. `failed`
8. `rejected`
9. `timeout`

关键迁移：

1. 启动等待 Nav2：`waiting_nav2_server -> idle|nav2_server_unavailable`。
2. 转发目标：`idle -> forwarding_goal -> running`。
3. 终态收敛：`running -> succeeded|failed|timeout|idle`。
4. `idle` 分支在 CANCELED 且 `canceled_by_idle=true` 时成立。

## 10. 测试覆盖与契约

### 10.1 BehaviorNode 测试

文件：[src/dog_behavior/test/test_behavior_node.cpp](../src/dog_behavior/test/test_behavior_node.cpp)

当前覆盖 13 个用例：

1. `NodeInitCreatesParametersAndPublisher`
2. `ConvertsOdometryToPoseStampedAndPublishes`
3. `PosePublishLatencyJitterStaysWithinFiveMilliseconds`
4. `FallsBackToDefaultFrameWhenInputFrameEmpty`
5. `DropsPoseWhenBothInputAndDefaultFrameEmpty`
6. `SendsActionGoalAndProcessesFeedbackAndResult`
7. `HandlesRejectedActionGoal`
8. `HandlesActionServerUnavailableWithoutCrash`
9. `KeepsTimeoutStateWhenWatchdogCancelsGoal`
10. `HandlesAbortedResultAsFailed`
11. `SkipsRecoveredCompletedTaskPhase`
12. `ContinuesUnfinishedRecoveredTaskPhase`
13. `IdleSpinningModeBlocksNewGoalAndKeepsRecoveredContext`

### 10.2 NavigationExecutorNode 测试

文件：[src/dog_behavior/test/test_navigation_executor_node.cpp](../src/dog_behavior/test/test_navigation_executor_node.cpp)

当前覆盖 4 个用例：

1. `ForwardsGoalToNav2AndReturnsSucceeded`
2. `RejectsInvalidPoseGoal`
3. `CancelsGoalWhenSystemModeSwitchesToIdleSpinning`
4. `MarksTimeoutWhenNav2FeedbackIsMissing`

### 10.3 BehaviorTree 测试

文件：[src/dog_behavior/test/test_behavior_tree.cpp](../src/dog_behavior/test/test_behavior_tree.cpp)

当前覆盖 3 个用例：

1. `ExecutesActionLeafThenPlaceholderLeaf`
2. `SkipsPlaceholderLeafWhenActionLeafFails`
3. `PropagatesEmptyBehaviorNameToActionLeaf`

### 10.4 建议新增回归

1. NavigationExecutorNode：`recovery_context` 中 `mode=recovered + target_state=completed` 的阻断与 `cold_start` 解阻断。
2. NavigationExecutorNode：`nav2_server_wait_timeout_sec` 到期后 `nav2_server_unavailable` 的状态发布验证。
3. BehaviorNode：`triggerNavigateGoal` 在 `navigate_server_ready_` 为 false 时的失败路径（单测直达）。

## 11. 构建与启动关联

构建定义见：[src/dog_behavior/CMakeLists.txt](../src/dog_behavior/CMakeLists.txt)

1. 库目标：`${PROJECT_NAME}_lib`，包含
  1. `behavior_node.cpp`
  2. `behavior_tree.cpp`
  3. `navigation_executor_node.cpp`
2. 可执行目标：
  1. `${PROJECT_NAME}_node`（入口 `main.cpp`）
  2. `${PROJECT_NAME}_navigation_executor_node`（入口 `navigation_executor_main.cpp`）
3. 测试目标：
  1. `test_behavior_tree`
  2. `test_behavior_node`
  3. `test_navigation_executor_node`

启动编排见：[src/dog_behavior/launch/launch.py](../src/dog_behavior/launch/launch.py)

**启动参数**：
- `match_type`（默认 `left`，可选 `left`/`right`）：比赛类型，决定加载 `waypoints_left.yaml` 或 `waypoints_right.yaml` 航点文件。

**节点启动**：
1. 启动 `dog_behavior_node`，节点名 `dog_behavior`。
2. 启动 `dog_behavior_navigation_executor_node`，节点名 `dog_navigation_executor`。
3. 与 `dog_perception`、`dog_lifecycle` 一同纳入统一 launch。

## 12. AI 查询建议

可直接用于检索或问答的短语：

1. behavior_node triggerExecuteBehavior recovered completed task_phase filtering
2. behavior_node systemModeCallback idle_spinning degraded cancel active goals
3. navigation_executor handleGoal validation finite pose quaternion norm
4. navigation_executor mapNav2ResultState canceled idle timeout failed
5. behavior_tree TriggerExecuteBehavior TriggerNavigateGoal xml sequence
6. dog_behavior launch behavior_node navigation_executor_node executable mapping
7. behavior_node match_type waypoints_file loadWaypoints yaml-cpp navigation target selection

可用于影响分析的核心入口：

1. [src/dog_behavior/src/behavior_node.cpp](../src/dog_behavior/src/behavior_node.cpp)
2. [src/dog_behavior/src/navigation_executor_node.cpp](../src/dog_behavior/src/navigation_executor_node.cpp)
3. [src/dog_behavior/src/behavior_tree.cpp](../src/dog_behavior/src/behavior_tree.cpp)
4. [src/dog_behavior/test/test_behavior_node.cpp](../src/dog_behavior/test/test_behavior_node.cpp)
5. [src/dog_behavior/test/test_navigation_executor_node.cpp](../src/dog_behavior/test/test_navigation_executor_node.cpp)

## 13. 维护建议

1. 修改 `mode`/`task_phase`/`target_state` 协议时，同时回归两个节点中的 `parseKeyValuePayload` 路径。
2. 修改取消或超时语义时，优先回归 CANCELED 的状态分支（`idle/timeout/failed`）。
3. 修改位姿校验阈值时，补充 NaN/Inf、零范数、非单位四元数边界测试。
4. 更新测试覆盖章节时，以测试文件中实际 `TEST_F` 名称为准，避免“建议新增”与“已实现”冲突。