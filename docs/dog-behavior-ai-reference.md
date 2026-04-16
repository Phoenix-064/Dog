# dog_behavior AI 开发参考（当前代码基线）

本文档描述的是当前代码基线（截至 2026-04-15），用于 AI 与开发者进行检索、影响分析、回归设计与后续重构实施。

适用范围：`src/dog_behavior` 包。

---

## 1. 当前架构总览

当前仍是双节点协作架构：

1. `BehaviorNode` 负责行为触发与编排。
2. `NavigationExecutorNode` 负责内部导航动作代理（server->client 转发到 Nav2）。
3. `BehaviorTree` 负责从 XML 加载并触发固定叶子序列。
4. `dog_behavior::utils`（Phase 0 新增）提供字符串协议解析与 pose 校验公共工具。

数据主链路：

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

---

## 2. 关键文件索引

包目录：[src/dog_behavior](../src/dog_behavior)

头文件：

1. [src/dog_behavior/include/dog_behavior/behavior_node.hpp](../src/dog_behavior/include/dog_behavior/behavior_node.hpp)
2. [src/dog_behavior/include/dog_behavior/navigation_executor_node.hpp](../src/dog_behavior/include/dog_behavior/navigation_executor_node.hpp)
3. [src/dog_behavior/include/dog_behavior/behavior_tree.hpp](../src/dog_behavior/include/dog_behavior/behavior_tree.hpp)
4. [src/dog_behavior/include/dog_behavior/common/payload_utils.hpp](../src/dog_behavior/include/dog_behavior/common/payload_utils.hpp)

实现：

1. [src/dog_behavior/src/behavior_node.cpp](../src/dog_behavior/src/behavior_node.cpp)
2. [src/dog_behavior/src/navigation_executor_node.cpp](../src/dog_behavior/src/navigation_executor_node.cpp)
3. [src/dog_behavior/src/behavior_tree.cpp](../src/dog_behavior/src/behavior_tree.cpp)
4. [src/dog_behavior/src/common/payload_utils.cpp](../src/dog_behavior/src/common/payload_utils.cpp)

入口与启动：

1. [src/dog_behavior/src/main.cpp](../src/dog_behavior/src/main.cpp)
2. [src/dog_behavior/src/navigation_executor_main.cpp](../src/dog_behavior/src/navigation_executor_main.cpp)
3. [src/dog_behavior/launch/launch.py](../src/dog_behavior/launch/launch.py)

配置：

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
4. [src/dog_behavior/test/test_payload_utils.cpp](../src/dog_behavior/test/test_payload_utils.cpp)

---

## 3. 组件职责

### 3.1 BehaviorNode

实现入口：[src/dog_behavior/src/behavior_node.cpp](../src/dog_behavior/src/behavior_node.cpp)

职责：

1. 订阅 odom，发布 `/dog/global_pose`。
2. 订阅 `/behavior/execute_trigger`，通过 `BehaviorTree::execute` 驱动执行。
3. 作为 `/behavior/execute` action client。
4. 作为 `/behavior/navigate_execute` action client（内部导航代理入口）。
5. 处理 `/lifecycle/recovery_context` 与 `/lifecycle/system_mode` 的约束。
6. 维护本地执行状态机与反馈超时看门狗。

关键条件：

1. 执行动作发送前必须满足 server ready、无在途目标、非 idle_spinning/degraded、latest_pose 可用。
2. 导航动作发送前必须满足导航代理 ready、无在途导航目标，并满足航点或 latest_pose 可用。

### 3.2 NavigationExecutorNode

实现入口：[src/dog_behavior/src/navigation_executor_node.cpp](../src/dog_behavior/src/navigation_executor_node.cpp)

职责：

1. 提供 `/behavior/navigate_execute` action server。
2. 转发目标到 Nav2 `/navigate_to_pose` action client。
3. 发布 `/behavior/nav_exec_state`。
4. 根据 recovery/system_mode 决定是否接收、取消目标。
5. 维护导航执行状态机与反馈看门狗。

关键条件：

1. `handleGoal` 接收前需通过 server ready、非 busy、非 idle/degraded、recovery 未阻断。
2. 目标 pose 必须通过有限值和四元数范数检查。

### 3.3 BehaviorTree

实现入口：[src/dog_behavior/src/behavior_tree.cpp](../src/dog_behavior/src/behavior_tree.cpp)

职责：

1. 从 `execute_trigger_tree.xml` 加载树。
2. 注册并执行 `TriggerExecuteBehavior`、`TriggerNavigateGoal`、`PlaceholderBehavior`。
3. 以 `tickRoot() == SUCCESS` 作为整体成功标准。

说明：当前为固定串行序列，尚未进入 RUNNING 驱动型 BT 架构。

### 3.4 payload_utils（Phase 0）

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

1. 已替代 BehaviorNode 与 NavigationExecutorNode 的重复实现。
2. `percentDecode` 是内部实现细节，不对外暴露。

---

## 4. 外部接口字典

### 4.1 BehaviorNode

Action client：

1. `execute_behavior_action_name` 默认 `/behavior/execute`，类型 `dog_interfaces/action/ExecuteBehavior`
2. `navigate_execute_action_name` 默认 `/behavior/navigate_execute`，类型 `nav2_msgs/action/NavigateToPose`

订阅：

1. `/localization/dog`（可配）`nav_msgs/msg/Odometry`
2. `/behavior/execute_trigger`（可配）`std_msgs/msg/String`
3. `/lifecycle/recovery_context`（可配）`std_msgs/msg/String`
4. `/lifecycle/system_mode`（可配）`std_msgs/msg/String`

发布：

1. `/dog/global_pose`（可配）`geometry_msgs/msg/PoseStamped`

### 4.2 NavigationExecutorNode

Action server：

1. `/behavior/navigate_execute`（可配）`nav2_msgs/action/NavigateToPose`

Action client：

1. `/navigate_to_pose`（可配）`nav2_msgs/action/NavigateToPose`

订阅：

1. `/lifecycle/recovery_context`（可配）`std_msgs/msg/String`
2. `/lifecycle/system_mode`（可配）`std_msgs/msg/String`

发布：

1. `/behavior/nav_exec_state`（可配）`std_msgs/msg/String`

---

## 5. 字符串协议与解析规范

统一解析入口：`dog_behavior::utils::parseKeyValuePayload`

负载格式：

1. `key=value;key=value;...`

规则：

1. 键名比较前执行 `normalizeToken`（去空白、小写化）。
2. 值支持 `%xx` 百分号解码（内部实现）。
3. 缺失键返回空字符串。

关键键：

1. `mode`
2. `task_phase`
3. `target_state`

完成态判定：`done/completed/succeeded/success/finished`

---

## 6. 参数清单（当前默认值）

### 6.1 BehaviorNode

1. `global_pose_topic` = `/dog/global_pose`
2. `localization_topic` = `/localization/dog`
3. `default_frame_id` = `base_link`
4. `execute_behavior_action_name` = `/behavior/execute`
5. `execute_behavior_trigger_topic` = `/behavior/execute_trigger`
6. `navigate_execute_action_name` = `/behavior/navigate_execute`
7. `recovery_context_topic` = `/lifecycle/recovery_context`
8. `system_mode_topic` = `/lifecycle/system_mode`
9. `match_type` = `left`
10. `waypoints_file` = `""`
11. `action_server_wait_timeout_sec` = `5.0`
12. `feedback_timeout_sec` = `2.0`

### 6.2 NavigationExecutorNode

1. `navigate_execute_action_name` = `/behavior/navigate_execute`
2. `nav2_action_name` = `/navigate_to_pose`
3. `navigate_execution_state_topic` = `/behavior/nav_exec_state`
4. `recovery_context_topic` = `/lifecycle/recovery_context`
5. `system_mode_topic` = `/lifecycle/system_mode`
6. `nav2_server_wait_timeout_sec` = `10.0`
7. `navigate_feedback_timeout_sec` = `10.0`

---

## 7. 状态模型

### 7.1 BehaviorNode 状态

定义见 [src/dog_behavior/include/dog_behavior/behavior_node.hpp](../src/dog_behavior/include/dog_behavior/behavior_node.hpp)

1. `idle`
2. `waiting_server`
3. `server_unavailable`
4. `sending_goal`
5. `running`
6. `succeeded`
7. `failed`
8. `rejected`
9. `timeout`

### 7.2 NavigationExecutorNode 状态

定义见 [src/dog_behavior/include/dog_behavior/navigation_executor_node.hpp](../src/dog_behavior/include/dog_behavior/navigation_executor_node.hpp)

1. `idle`
2. `waiting_nav2_server`
3. `nav2_server_unavailable`
4. `forwarding_goal`
5. `running`
6. `succeeded`
7. `failed`
8. `rejected`
9. `timeout`

---

## 8. 测试与覆盖基线

当前主要测试文件：

1. [src/dog_behavior/test/test_behavior_node.cpp](../src/dog_behavior/test/test_behavior_node.cpp)
2. [src/dog_behavior/test/test_navigation_executor_node.cpp](../src/dog_behavior/test/test_navigation_executor_node.cpp)
3. [src/dog_behavior/test/test_behavior_tree.cpp](../src/dog_behavior/test/test_behavior_tree.cpp)
4. [src/dog_behavior/test/test_payload_utils.cpp](../src/dog_behavior/test/test_payload_utils.cpp)

用途分层：

1. `test_behavior_node`：行为编排与执行状态路径。
2. `test_navigation_executor_node`：导航代理目标接收、转发、取消、超时。
3. `test_behavior_tree`：树执行语义与失败传播。
4. `test_payload_utils`：协议解析与 pose 校验公共函数。

---

## 9. 构建与运行参考

构建定义：[src/dog_behavior/CMakeLists.txt](../src/dog_behavior/CMakeLists.txt)

当前目标：

1. `${PROJECT_NAME}_lib`
   - `src/behavior_node.cpp`
   - `src/behavior_tree.cpp`
   - `src/navigation_executor_node.cpp`
   - `src/common/payload_utils.cpp`
2. `${PROJECT_NAME}_node`
3. `${PROJECT_NAME}_navigation_executor_node`
4. gtest：`test_behavior_tree`、`test_behavior_node`、`test_navigation_executor_node`、`test_payload_utils`

推荐命令：

1. `source /opt/ros/humble/setup.bash`
2. `colcon build --packages-select dog_behavior`
3. `colcon test --packages-select dog_behavior`
4. `colcon test-result --all --verbose`

---

## 10. 与重构计划的衔接

相关计划文档：[docs/behavior_tree_refactor_plan.md](behavior_tree_refactor_plan.md)

当前进度：

1. Phase 0 已完成（公共工具抽取与测试补齐）。
2. 下一步是 Phase 1（BehaviorTreeNode 骨架），但当前运行架构尚未切换。

实施注意：

1. 在 Phase 1+ 修改期间，优先复用 `dog_behavior::utils`，避免重复逻辑回流。
2. 仅当新 BT 架构集成验证通过后，再执行旧节点移除与接口收敛。
