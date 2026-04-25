# dog_behavior 行为树架构重构计划

## 0. 当前状态快照（2026-04-25）

本计划文档已同步到当前代码基线，可作为下一步（Phase 4 清理）实施参考。

### 0.1 已完成项

- Phase 0 已完成：`payload_utils` 公共工具已落地，重复逻辑已从 `BehaviorNode`/`NavigationExecutorNode` 抽取。
- 新增公共文件：
  - `src/dog_behavior/include/dog_behavior/common/payload_utils.hpp`
  - `src/dog_behavior/src/common/payload_utils.cpp`
- 新增测试：`src/dog_behavior/test/test_payload_utils.cpp`
- 构建系统已接线：`src/dog_behavior/CMakeLists.txt` 已将 `payload_utils.cpp` 和 `test_payload_utils` 纳入。

### 0.2 当前基线约束

- 当前默认运行入口已切换为 `BehaviorTreeNode`，`launch.py` 已移除 `navigation_executor_node`。
- 旧代码路径（`BehaviorNode` / `NavigationExecutorNode` 相关源码与测试）暂仍保留，按 Phase 4 统一清理。
- 当前阶段验收以单元测试为主，暂不执行真机冒烟测试。

### 0.3 本轮已落地（Phase 1 骨架）

以下改动已在代码中完成并通过构建/测试：

1. 新增 `BehaviorTreeNode` 主节点骨架（含 timer tick + blackboard + 关键订阅/发布）
  - `src/dog_behavior/include/dog_behavior/behavior_tree_node.hpp`
  - `src/dog_behavior/src/behavior_tree_node.cpp`
2. 新增 BT 主节点独立入口
  - `src/dog_behavior/src/behavior_tree_main.cpp`
3. 新增最小行为树 XML（仅条件节点）
  - `src/dog_behavior/config/behavior_tree.xml`
4. 构建系统接线
  - `src/dog_behavior/CMakeLists.txt` 已纳入 `behavior_tree_node.cpp`、`${PROJECT_NAME}_bt_node`、`test_behavior_tree_node`
5. 新增单测
  - `src/dog_behavior/test/test_behavior_tree_node.cpp`

验证结果：

- `colcon build --packages-select dog_behavior` 通过
- `colcon test --packages-select dog_behavior --ctest-args -R test_behavior_tree_node` 通过
- `colcon test-result --all --verbose` 显示 `test_behavior_tree_node` 3/3 通过，`dog_behavior` 相关测试无回归

### 0.4 策略更新（用于后续阶段）

- 决策：**不实现“新旧节点切换”代码**（不新增 launch 开关，不维护双路径长期共存）。
- 执行方式：直接沿 Phase 2 -> Phase 3 -> Phase 4 推进，全量替换旧架构。
- 约束：后续每阶段都要保持包内构建与测试可回归，避免一次性大爆炸提交。

### 0.5 本轮已落地（Phase 2 首轮）

以下 Phase 2 内容已在代码中落地：

1. 新增 5 个 BT 叶子节点实现与头文件
  - `src/dog_behavior/include/dog_behavior/bt_nodes/check_system_mode.hpp`
  - `src/dog_behavior/src/bt_nodes/check_system_mode.cpp`
  - `src/dog_behavior/include/dog_behavior/bt_nodes/wait_for_pose_condition.hpp`
  - `src/dog_behavior/src/bt_nodes/wait_for_pose_condition.cpp`
  - `src/dog_behavior/include/dog_behavior/bt_nodes/select_waypoint_action.hpp`
  - `src/dog_behavior/src/bt_nodes/select_waypoint_action.cpp`
  - `src/dog_behavior/include/dog_behavior/bt_nodes/execute_behavior_action.hpp`
  - `src/dog_behavior/src/bt_nodes/execute_behavior_action.cpp`
  - `src/dog_behavior/include/dog_behavior/bt_nodes/navigate_to_pose_action.hpp`
  - `src/dog_behavior/src/bt_nodes/navigate_to_pose_action.cpp`
2. `BehaviorTreeNode` 已改为注册类节点（替代内联 lambda）
  - `CheckSystemMode` / `WaitForPose` / `SelectWaypoint` / `ExecuteBehaviorAction` / `NavigateToPoseAction`
3. 主行为树 XML 已扩展到 Phase 2 链路
  - `src/dog_behavior/config/behavior_tree.xml`
  - 顺序为：`CheckSystemMode -> WaitForPose -> SelectWaypoint -> ExecuteBehaviorAction -> NavigateToPoseAction`
4. 为避免影响 Phase 1 骨架测试，新增轻量测试 XML
  - `src/dog_behavior/config/behavior_tree_test.xml`
5. 构建系统已接线
  - `src/dog_behavior/CMakeLists.txt` 已纳入 5 个 bt_nodes 源文件
  - 已新增 5 个 gtest：
    - `test_check_system_mode`
    - `test_wait_for_pose_condition`
    - `test_select_waypoint_action`
    - `test_execute_behavior_action`
    - `test_navigate_to_pose_action`

复验结果（2026-04-16）：

- `colcon build --packages-select dog_behavior` 通过
- `colcon test --packages-select dog_behavior --ctest-args -R "test_behavior_tree_node|test_check_system_mode|test_wait_for_pose_condition|test_select_waypoint_action|test_execute_behavior_action|test_navigate_to_pose_action"` 通过
- `colcon test-result --all --verbose` 汇总无失败（114 tests, 0 errors, 0 failures）

已知说明（供 Phase 3 接手时注意）：

- `test_navigate_to_pose_action` 当前验证重点是“成功进入动作派发/运行态（RUNNING）与状态发布”，尚未在该单测中断言最终 `SUCCEEDED` 终态。
- `WaitForPoseCondition` 当前实现语义为“条件节点失败即阻断，不返回 RUNNING”；若后续希望更贴近可等待语义，可在 Phase 3 引入 Decorator/重试控制而非直接改写 Condition 语义。

### 0.6 本轮已落地（Phase 3 集成替换定稿）

以下 Phase 3 内容已在代码中落地并完成包内回归：

1. 接口定稿（`dog_interfaces`）
  - 新增 `PlaceBoxes.action`：
    - Goal: `box_type`, `payload`, `step_counter`
    - Result: `accepted`, `detail`
    - Feedback: `progress`, `state`
  - 已接线 `src/dog_interfaces/CMakeLists.txt` 的 action 生成。

2. 放置链路 BT 节点落地（`dog_behavior/bt_nodes`）
  - 新增节点：
    - `SetBoxesTypeAction`
    - `AdvancePlaceCounterAction`
    - `PlaceRuleAction`
    - `PlaceIndexAction`
    - `ExecutePlaceBoxesAction`
    - `PublishMathAnswerAction`
  - 语义对齐：
    - `counter` 前推进，`counter > 7` 进入结束语义。
    - `group_a={0,1,5,6}`，`group_b={2,3,4,7}`。
    - payload 采用 `key=value`，格式示例 `place=0,3,count=3`。
    - `*_box_count` 仅在 `ExecutePlaceBoxesAction` 收到 action 成功且 accepted 时提交。
    - 放置执行采用 fail-open（XML 中 `ForceSuccess` 包裹执行节点）。

3. 主节点与黑板接线
  - `BehaviorTreeNode` 已注入并维护：
    - `match_type`、`counter`
    - `food_box_count` / `tool_box_count` / `instrument_box_count` / `medical_box_count`
    - `boxes_type_list` / `boxes_ready` / `boxes_capture_stamp`
    - `WayPointGoal1..4`、`PlaceGoal1..4` 对应 pose
  - 已完成新增节点注册。

4. BT XML 编排定稿
  - `behavior_tree.xml` 已落地完整流程：
    - `SetBoxesTypeAction` 后依次导航到 `WayPointGoal1 -> WayPointGoal2 -> WayPointGoal3`
    - 在 `WayPointGoal3` 后执行 `PublishMathAnswerAction`
    - 第一次取箱后放置批次：`PlaceGoal1 -> PlaceGoal2 -> PlaceGoal3 -> PlaceGoal4`
    - 第二次取箱后放置批次：`PlaceGoal4 -> PlaceGoal3 -> PlaceGoal2 -> PlaceGoal1`
  - 放置子树模式：
    - `AdvancePlaceCounterAction -> PlaceRuleAction -> PlaceIndexAction -> ExecutePlaceBoxesAction`

5. 入口与启动替换
  - `src/dog_behavior/src/main.cpp` 已切换到 `BehaviorTreeNode`。
  - `src/dog_behavior/launch/launch.py` 已移除 `navigation_executor_node`。
  - 启动参数继续保留 `match_type` 与 `waypoints_file` 注入。

6. 构建与测试接线（单测）
  - `src/dog_behavior/CMakeLists.txt` 已接入新增 bt 节点源文件与测试目标。
  - 新增测试：
    - `test_set_boxes_type_action`
    - `test_advance_place_counter_action`
    - `test_place_rule_action`
    - `test_place_index_action`
    - `test_execute_place_boxes_action`
    - `test_publish_math_answer_action`
  - `test_behavior_tree_node` 已扩展 Phase 3 结构覆盖：
    - 双放置批次顺序（先正序后逆序）
    - 数学题节点位于 `WayPointGoal3` 后、第一次 `PickUpBoxes` 前

复验结果（2026-04-25）：

- `colcon build --packages-select dog_interfaces dog_behavior` 通过
- `colcon test --packages-select dog_behavior --ctest-args -R "test_behavior_tree_node|test_set_boxes_type_action|test_advance_place_counter_action|test_place_rule_action|test_place_index_action|test_execute_place_boxes_action|test_publish_math_answer_action"` 通过
- `colcon test-result --test-result-base build/dog_behavior/test_results --verbose`：`55 tests, 0 errors, 0 failures, 0 skipped`

当前结论：

- Phase 3 代码与单测收口已完成。
- 真机冒烟测试按当前阶段边界延后执行。

## 1. 问题分析

### 1.1 当前架构

```
executeTriggerCallback (topic 触发)
    └─> BehaviorTree::execute(behavior_name)     ← 同步调用 tickRoot()
            ├─> TriggerExecuteBehavior 叶子       ← callback 回到 BehaviorNode::triggerExecuteBehavior()
            ├─> TriggerNavigateGoal 叶子          ← callback 回到 BehaviorNode::triggerNavigateGoal()
            └─> PlaceholderBehavior 叶子          ← callback 空操作
```

`BehaviorNode` 是一个约 940 行的上帝类，同时承担：
- ROS2 通信基础设施（odom 订阅、pose 发布、两套 action client 完整生命周期）
- 行为决策驱动（持有 `BehaviorTree` 成员并调用 `execute()`）
- 导航逻辑（waypoint 加载、轮转选择、pose fallback）
- 系统模式/恢复状态管理（idle_spinning、recovery 过滤）

`BehaviorTree` 类只是一个薄 callback 转发层，不持有 ROS 句柄，不做异步管理，`execute()` 同步调用 `tickRoot()` 后立刻返回。

### 1.2 核心缺陷

| 缺陷 | 影响 |
|------|------|
| **BT 叶子无法返回 RUNNING** | `triggerExecuteBehavior` 发送 action goal 后即刻返回 bool，行为树无法等待 action 真正完成，无法做重试/超时分支 |
| **树不被周期性 tick** | `execute()` 只调一次 `tickRoot()`，无法实现持续运行的行为（巡逻、跟踪、多段导航） |
| **职责耦合** | BT 叶子的业务逻辑全部写在 `BehaviorNode` 里，无法独立测试、独立复用 |
| **重复代码** | `BehaviorNode` 和 `NavigationExecutorNode` 曾各自实现 `normalizeToken`、`parseKeyValuePayload`、`isFinitePose`、`hasValidQuaternionNorm`、`isCompletedState`（该问题已在 Phase 0 解决） |
| **行为树 XML 形同虚设** | 当前 XML 是固定的 Sequence(Execute, Navigate, Placeholder)，无条件分支、无 Fallback、无 Decorator，行为树的能力完全未被利用 |
| **NavigationExecutorNode 定位模糊** | 作为独立进程运行的 action server/client 代理层，与 BT 架构存在冗余——BT 本身就可以管理对 Nav2 的调用 |

---

## 2. 目标架构

```
                    ┌─────────────────────────────────┐
                    │    BehaviorTreeNode (ROS2 Node)  │
                    │  - 持有 BT::Tree                 │
                    │  - timer 驱动周期性 tickRoot()    │
                    │  - 管理 Blackboard               │
                    │  - 订阅 trigger / system_mode    │
                    └──────────┬──────────────────────┘
                               │ BT 叶子节点通过
                               │ node->getInput() / blackboard
                               │ 获取参数
              ┌────────────────┼────────────────┐
              │                │                │
     ┌────────▼───────┐ ┌─────▼──────┐ ┌───────▼────────┐
     │ ExecuteBehavior │ │ NavigateTo │ │  Condition/     │
     │   BT::Action   │ │ Pose       │ │  Decorator      │
     │                 │ │ BT::Action │ │  (各种判断)     │
     │ 持有 action     │ │ 持有 action│ │                 │
     │ client          │ │ client     │ │                 │
     │ 支持 RUNNING    │ │ 支持RUNNING│ │                 │
     └─────────────────┘ └────────────┘ └─────────────────┘
```

### 2.1 设计原则

1. **行为树节点即 ROS2 节点** —— `BehaviorTreeNode` 是一个独立的 `rclcpp::Node`，负责加载 XML、创建树、周期 tick
2. **BT 叶子节点是独立类** —— 每个 `BT::StatefulActionNode` / `BT::ConditionNode` 各自管理自己需要的 ROS2 通信
3. **BT 叶子节点可以返回 RUNNING** —— 使用 `BT::StatefulActionNode`，在 `onStart()` 发送 goal，在 `onRunning()` 检查完成状态
4. **Blackboard 作为数据总线** —— 全局 pose、system_mode、waypoints 等通过 blackboard 共享
5. **消除 NavigationExecutorNode** —— BT NavigateToPose 叶子节点直接作为 Nav2 的 action client，不再需要代理层

---

## 3. 新文件结构

```
src/dog_behavior/
├── include/dog_behavior/
│   ├── behavior_tree_node.hpp          # [新] BT 主节点（ROS2 Node）
│   ├── bt_nodes/                       # [新] BT 叶子节点目录
│   │   ├── execute_behavior_action.hpp # [新] ExecuteBehavior action 叶子
│   │   ├── navigate_to_pose_action.hpp # [新] NavigateToPose action 叶子
│   │   ├── wait_for_pose_condition.hpp # [新] 等待有效 pose 条件节点
│   │   ├── check_system_mode.hpp       # [新] 检查系统模式条件节点
│   │   └── select_waypoint_action.hpp  # [新] 航点选择 action 节点
│   ├── common/                         # [已完成] 公共工具
│   │   └── payload_utils.hpp           # [已完成] normalizeToken, parseKV, pose 校验, completed-state 判定
│   ├── pose_tracker_node.hpp           # [新] odom→pose 转发（从 BehaviorNode 中拆出，或直接在 BT 节点内部处理）
│   ├── behavior_node.hpp               # [删除] 重构完成后移除
│   ├── behavior_tree.hpp               # [删除] 重构完成后移除
│   └── navigation_executor_node.hpp    # [删除] 重构完成后移除
├── src/
│   ├── behavior_tree_node.cpp          # [新]
│   ├── bt_nodes/
│   │   ├── execute_behavior_action.cpp # [新]
│   │   ├── navigate_to_pose_action.cpp # [新]
│   │   ├── wait_for_pose_condition.cpp # [新]
│   │   ├── check_system_mode.cpp       # [新]
│   │   └── select_waypoint_action.cpp  # [新]
│   ├── common/
│   │   └── payload_utils.cpp           # [已完成]
│   ├── pose_tracker_node.cpp           # [新]（可选，见第 4 节讨论）
│   └── main.cpp                        # [改] 启动 BehaviorTreeNode
├── config/
│   ├── behavior_tree.xml               # [改] 重新设计的行为树 XML
│   ├── waypoints_left.yaml
│   └── waypoints_right.yaml
├── test/
│   ├── test_execute_behavior_action.cpp  # [新]
│   ├── test_navigate_to_pose_action.cpp  # [新]
│   ├── test_behavior_tree_node.cpp       # [新]
│   ├── test_payload_utils.cpp            # [新]
│   └── ...
└── launch/
    └── launch.py                       # [改] 移除 navigation_executor 节点
```

---

## 4. 关键设计决策

### 4.1 BT 叶子节点如何获取 ROS2 Node 句柄

BehaviorTree.CPP v3 不原生支持在 BT 节点中持有 `rclcpp::Node`。标准做法是：

**方案：通过 Blackboard 传入 `rclcpp::Node::SharedPtr`**
```cpp
blackboard->set("ros_node", shared_from_this());
// 叶子节点中：
auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
```

这是 Nav2 `bt_navigator` 采用的模式，BT 叶子在构造时从 blackboard 获取 node 指针，用它创建 action client、subscriber 等。

### 4.2 BT 叶子节点基类选择

| BT 基类 | 用途 | 我们使用的场景 |
|---------|------|--------------|
| `BT::StatefulActionNode` | 异步 action，支持 RUNNING | ExecuteBehavior、NavigateToPose |
| `BT::SyncActionNode` | 同步动作，一次执行完 | SelectWaypoint |
| `BT::ConditionNode` | 条件判断 | WaitForPose、CheckSystemMode |

### 4.3 NavigationExecutorNode 的去留

**建议：取消 NavigationExecutorNode 作为独立进程**

当前 `NavigationExecutorNode` 的作用是：
1. 作为 `/behavior/navigate_execute` action server 接收 BehaviorNode 的目标
2. 转发到 Nav2 的 `/navigate_to_pose` action
3. 回传 feedback 和 result
4. 处理 system_mode 和 recovery_context

这完全可以由一个 `NavigateToPoseBTAction`（BT::StatefulActionNode）替代：
- `onStart()` —— 创建 Nav2 action client，发送 goal
- `onRunning()` —— 检查 goal 状态，返回 SUCCESS/FAILURE/RUNNING
- `onHalted()` —— 取消 goal

system_mode 和 recovery 的控制由外层 BT 结构（Condition 守卫）处理，不需要在每个叶子里重复。

### 4.4 Pose 跟踪的处理方式

当前 `BehaviorNode` 同时做两件事：
1. 订阅 odom → 发布 global_pose（给其他节点使用）
2. 缓存 latest_pose 给 action goal 使用

**选择 A：拆为独立的 PoseTrackerNode**
- 优点：BT 节点不需要关心 pose 发布
- 缺点：多一个节点进程

**选择 B：BT 主节点中订阅并写入 Blackboard**
- 优点：简单，pose 数据直接可用
- 缺点：global_pose 发布逻辑也混在 BT 节点中

**建议：选择 B，在 `BehaviorTreeNode` 中订阅 odom，发布 global_pose，同时将 latest_pose 写入 blackboard。** 这比起拆一个独立进程更简单，而且 pose 发布是 BT 节点运行的前提条件。

### 4.5 重复工具代码

`normalizeToken`、`parseKeyValuePayload`、`isFinitePose`、`hasValidQuaternionNorm`、`isCompletedState` 在 `BehaviorNode` 和 `NavigationExecutorNode` 中曾经重复。

**当前状态：Phase 0 已完成，已提取到 `common/payload_utils.hpp/.cpp`。**

**方案：提取到 `common/payload_utils.hpp`**

```cpp
namespace dog_behavior::utils {
  std::string normalizeToken(const std::string & value);
  std::string parseKeyValuePayload(const std::string & payload, const std::string & key);
  bool isFinitePose(const geometry_msgs::msg::Pose & pose);
  bool isFinitePose(const geometry_msgs::msg::PoseStamped & pose);
  bool hasValidQuaternionNorm(const geometry_msgs::msg::Pose & pose);
  bool hasValidQuaternionNorm(const geometry_msgs::msg::PoseStamped & pose);
  bool isCompletedState(const std::string & target_state);
}
```

---

## 5. BT XML 设计示例

当前 XML：
```xml
<root main_tree_to_execute="ExecuteTriggerTree">
  <BehaviorTree ID="ExecuteTriggerTree">
    <Sequence>
      <TriggerExecuteBehavior behavior_name="{behavior_name}"/>
      <TriggerNavigateGoal behavior_name="{behavior_name}"/>
      <PlaceholderBehavior/>
    </Sequence>
  </BehaviorTree>
</root>
```

重构后示例（根据实际比赛流程调整）：
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <!-- 前置守卫：系统模式必须正常 -->
      <CheckSystemMode expected_mode="normal" mode="{system_mode}"/>
      <!-- 确保有有效 pose -->
      <WaitForPose pose="{current_pose}" timeout_ms="5000"/>
      <!-- 选择目标航点 -->
      <SelectWaypoint waypoints="{waypoints}" index="{wp_index}" 
                      target_pose="{target_pose}"/>
      <!-- 执行行为（如抓取），支持 RUNNING -->
      <ExecuteBehaviorAction behavior_name="{behavior_name}" 
                             target_pose="{target_pose}"
                             action_name="/behavior/execute"/>
      <!-- 导航到下一个点，支持 RUNNING -->
      <NavigateToPoseAction goal="{target_pose}" 
                            action_name="/navigate_to_pose"/>
    </Sequence>
  </BehaviorTree>
</root>
```

---

## 6. 分阶段实施计划

### Phase 0：基础设施（已完成）

**目标：提取公共代码，不改变运行时行为**

1. [x] 创建 `common/payload_utils.hpp/.cpp`，搬移重复工具函数
2. [x] `BehaviorNode` 和 `NavigationExecutorNode` 改为调用 `utils::` 版本
3. [x] 添加 `test_payload_utils.cpp` 单元测试
4. [x] `colcon build --packages-select dog_behavior` 与 `colcon test --packages-select dog_behavior` 通过

**验收标准：** 现有测试全部通过，无行为变更。

**实施产物：**

- `src/dog_behavior/include/dog_behavior/common/payload_utils.hpp`
- `src/dog_behavior/src/common/payload_utils.cpp`
- `src/dog_behavior/test/test_payload_utils.cpp`
- `src/dog_behavior/CMakeLists.txt`（新增库源文件与 gtest 目标）

**进入 Phase 1 前置条件（已满足）：**

- [x] 关键字符串协议解析逻辑已集中到 `dog_behavior::utils`
- [x] 两个节点不再保留重复实现
- [x] payload 工具具备独立单测覆盖

### Phase 1：BT 主节点骨架（预计工作量：中）

**目标：创建 `BehaviorTreeNode`，实现 timer-tick 循环和 blackboard 初始化**

1. [x] 创建 `BehaviorTreeNode` 类，继承 `rclcpp::Node`
   - 订阅 `/behavior/execute_trigger`、`/lifecycle/system_mode`、`/lifecycle/recovery_context`
   - 订阅 odom，发布 global_pose
   - 将 `system_mode`、`current_pose`、`waypoints` 写入 blackboard
   - 创建 wall_timer，周期 tick 行为树（如 100ms）
   - 支持 trigger 模式（收到触发消息时设置 blackboard 参数并开始 tick）
2. [x] 此阶段行为树 XML 采用简单 Condition 节点做验证（`CheckSystemMode` + `WaitForPose`）
3. [x] 新增 `test_behavior_tree_node.cpp` 并通过
4. [x] 新增 `${PROJECT_NAME}_bt_node` 可执行作为骨架入口
5. [x] 将默认运行入口从旧节点切换到 `BehaviorTreeNode`（已在 Phase 3 完成）

**当前验收结论：** 已满足“新节点能启动、订阅 odom 并发布 global_pose、周期 tick 条件树”的 Phase 1 骨架目标。

### Phase 2：BT 叶子节点实现（预计工作量：大）

**目标：实现核心 BT Action/Condition 节点**

按优先级顺序：

1. [x] **`CheckSystemMode`**（ConditionNode）—— 从 blackboard 读 system_mode，非 normal 时返回 FAILURE
2. [x] **`WaitForPose`**（ConditionNode）—— 从 blackboard 读 current_pose，无效时返回 FAILURE
3. [x] **`SelectWaypoint`**（SyncActionNode）—— 从 blackboard 读 waypoints 列表，按策略选择目标 pose 写入 blackboard
4. [x] **`ExecuteBehaviorAction`**（StatefulActionNode）
   - `onStart()` —— 从 blackboard 获取 node 指针，创建 action client，发送 ExecuteBehavior goal
   - `onRunning()` —— 检查 goal handle 状态，返回 RUNNING/SUCCESS/FAILURE
   - `onHalted()` —— 取消 goal
   - 内含 feedback watchdog 逻辑
5. [x] **`NavigateToPoseAction`**（StatefulActionNode）
   - 类似 `ExecuteBehaviorAction`，直接连接 Nav2 的 `/navigate_to_pose`
   - 取代 `NavigationExecutorNode` 的功能

6. [x] 每个叶子节点都有独立单元测试并接入 `CMakeLists.txt`

**验收标准：** 每个叶子节点有独立的 gtest，可以用 mock action server 验证行为。

**当前验收结论：**

- Phase 2 首轮目标已完成并通过包内构建与定向测试。
- 进入 Phase 3 前，不需要再补基础接线；下一步重点转向“切主入口 + launch 替换 + 集成回归 + 旧路径收敛”。

### Phase 3：集成与替换（预计工作量：中）

**目标：直接用新架构替换旧架构（不保留切换开关）**

1. [x] 设计完整的行为树 XML，覆盖当前比赛流程
2. [x] 修改 `main.cpp`，启动 `BehaviorTreeNode`
3. [x] 修改 `launch.py`：
   - 移除 `navigation_executor_node`
   - 将 `dog_behavior_node` 指向新的可执行文件
4. [x] 编写集成/结构测试：覆盖双放置顺序与数学题触发点
5. [ ] 在真机上做冒烟测试（按当前阶段边界暂不执行）

**验收标准：** 旧的 `BehaviorNode` + `NavigationExecutorNode` 两个进程被新的 `BehaviorTreeNode` 一个进程替代，且不引入新旧切换逻辑，比赛流程正常运行。

**当前验收结论：** 软件侧替换目标已达成，单测回归通过；真机冒烟验证延后到后续阶段执行。

### Phase 4：清理（预计工作量：小）

**目标：删除旧代码**

1. [x] 删除 `behavior_node.hpp/.cpp`、`behavior_tree.hpp/.cpp`、`navigation_executor_node.hpp/.cpp`、`navigation_executor_main.cpp`
2. [x] 删除对应的旧测试文件
3. [x] 更新 `CMakeLists.txt`，移除旧编译目标
4. [x] 更新 `CLAUDE.md` 架构说明
5. [x] 额外清理重复入口与旧树配置：删除 `main.cpp` 与 `execute_trigger_tree.xml`

**当前验收结论（代码与文档收口）：**

- 运行入口已统一到 `dog_behavior_bt_node`，旧可执行与旧测试目标已从构建系统移除。
- 旧架构源码、旧测试与旧 XML 已删除，`src/dog_behavior` 内无旧路径残留引用。
- 主线文档（README/CLAUDE/AGENTS/接口与集成文档）已同步到“BT 叶子直接调用 Nav2”的现状。

---

## 7. 对外接口变更一览

| 接口 | 当前 | 重构后 | 影响 |
|------|------|--------|------|
| `/behavior/execute_trigger` (topic) | BehaviorNode 订阅 | BehaviorTreeNode 订阅 | dog_lifecycle 无感知 |
| `/dog/global_pose` (topic) | BehaviorNode 发布 | BehaviorTreeNode 发布 | 无变化 |
| `/behavior/execute` (action) | BehaviorNode 作为 client | ExecuteBehaviorAction BT 叶子作为 client | 无变化（server 端不受影响） |
| `/behavior/navigate_execute` (action) | BehaviorNode→NavigationExecutorNode | **删除**，BT 叶子直接调 Nav2 | NavigationExecutorNode 被移除 |
| `/navigate_to_pose` (action) | NavigationExecutorNode 作为 client | NavigateToPoseAction BT 叶子作为 client | 无变化（Nav2 不受影响） |
| `/behavior/nav_exec_state` (topic) | NavigationExecutorNode 发布 | BehaviorTreeNode 发布（或通过 BT 状态推导） | 下游如有依赖需要适配 |
| `/lifecycle/system_mode` (topic) | BehaviorNode + NavigationExecutorNode 各自订阅 | BehaviorTreeNode 统一订阅，写入 blackboard | 简化为一处 |
| `/lifecycle/recovery_context` (topic) | BehaviorNode + NavigationExecutorNode 各自订阅 | BehaviorTreeNode 统一订阅，写入 blackboard | 简化为一处 |

---

## 8. 风险与缓解

| 风险 | 缓解措施 |
|------|----------|
| BehaviorTree.CPP v3 的 StatefulActionNode 与 rclcpp_action 的异步模型集成复杂 | 参考 Nav2 `bt_action_node.hpp` 的实现，它已经解决了这个问题 |
| 周期 tick 的频率对 action 响应时间有影响 | 默认 10Hz (100ms)，对于机器狗行为足够；可通过参数配置 |
| 直接替换过程中出现行为回归 | 每阶段落地后立即执行 `colcon build --packages-select dog_behavior` + 关键 gtest + 集成冒烟，采用小步提交策略 |
| `NavigationExecutorNode` 被移除后丢失 `/behavior/nav_exec_state` 状态发布 | 在 `BehaviorTreeNode` 中添加等效的状态发布逻辑 |

---

## 9. 依赖确认

- `behaviortree_cpp_v3` —— 已在 `package.xml` 中声明，无需新增
- `nav2_msgs` —— 已有
- `rclcpp_action` —— 已有
- 无需新增外部依赖
