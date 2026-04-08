---
title: 'dog_behavior 行为树动作触发骨架接入'
type: 'feature'
created: '2026-04-06'
status: 'done'
baseline_commit: '50663fadd6fa9e996b8d464063b9890f39d4b3fb'
context: ['_bmad-output/project-context.md', 'docs/dog-behavior-ai-reference.md', 'docs/architecture.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** `dog_behavior` 目前只有 topic -> `ExecuteBehavior` action 的直接分发链，缺少一层可扩展的行为树骨架，导致后续无法把多个行为叶子按树结构组织起来，也没有一个明确的占位节点承接未完成的具体行为实现。

**Approach:** 在 `dog_behavior` 内新增一层轻量、可测试的行为树路由骨架，将现有行为触发器接入树的首个执行叶子，并预留一个空行为叶子作为后续具体行为的插入点；保持当前 action 语义、状态机与恢复/空转逻辑不变。

## Boundaries & Constraints

**Always:** 只改动 `dog_behavior` 包及其测试；保留现有 `/behavior/execute_trigger` 到 `ExecuteBehavior` action 的行为语义；日志统一使用 `RCLCPP_*`；行为树必须是可实例化、可测试的对象，避免把树逻辑散落到回调里；保持现有恢复过滤、空转熔断、watchdog 逻辑不变。

**Ask First:** 若实现过程中发现必须引入新的外部行为树依赖，或需要变更 `dog_interfaces` 的 action 契约。

**Never:** 不修改 `build/`、`install/`、`log/`；不重写现有行为状态机；不把具体行为逻辑硬编码进行为树占位叶子；不改动其他包的通信契约。

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 正常触发 | 触发 topic 收到合法行为名，且 action server 就绪、最新位姿可用 | 行为树先走 action 触发叶子，再走占位叶子；行为名按原样送入 `ExecuteBehavior` action | 若 action 触发失败，树整体返回失败并停止后续叶子 |
| 占位叶子 | 行为树执行到预留的空行为叶子 | 当前只记录为占位路径，不执行任何具体动作 | 不抛异常，不改变既有状态机 |
| 触发失败 | action 前置条件不满足、server 不可用或 goal 被拒绝 | 树返回失败，节点保持现有失败/拒绝/不可用状态语义 | 由现有 `BehaviorNode` 统一记录日志和状态 |

## Code Map

- `src/dog_behavior/include/dog_behavior/behavior_node.hpp` -- 在节点中挂载行为树实例并保留现有 action 触发入口。
- `src/dog_behavior/src/behavior_node.cpp` -- 将 topic 回调改为先进入行为树路由，再由树调用现有 action 触发器。
- `src/dog_behavior/include/dog_behavior/behavior_tree.hpp` -- 定义行为树接口、叶子节点接口和工厂入口。
- `src/dog_behavior/src/behavior_tree.cpp` -- 实现行为树序列、action 触发叶子与占位叶子。
- `src/dog_behavior/test/test_behavior_tree.cpp` -- 验证树路由、触发失败短路、占位叶子不执行具体行为。
- `src/dog_behavior/test/test_behavior_node.cpp` -- 回归现有 topic->action 集成路径，确认行为树接入后不破坏旧语义。
- `src/dog_behavior/CMakeLists.txt` -- 注册新源码与 gtest 目标。
- `src/dog_behavior/package.xml` -- 如无新增外部依赖则保持最小变更；若新增依赖则同步声明。

## Tasks & Acceptance

**Execution:**
- [x] `src/dog_behavior/include/dog_behavior/behavior_tree.hpp` 与 `src/dog_behavior/src/behavior_tree.cpp` -- 新增轻量行为树接口与实现，包含 action 触发叶子和占位叶子 -- 为后续扩展留出树结构入口。
- [x] `src/dog_behavior/include/dog_behavior/behavior_node.hpp` 与 `src/dog_behavior/src/behavior_node.cpp` -- 将 `execute_trigger_topic` 的回调切换为行为树路由 -- 让触发器真正经过树层而不是直接调用。
- [x] `src/dog_behavior/test/test_behavior_tree.cpp` 与 `src/dog_behavior/test/test_behavior_node.cpp` -- 覆盖树成功、树失败、占位叶子和节点回归路径 -- 防止树接入破坏现有 action 语义。
- [x] `src/dog_behavior/CMakeLists.txt` 与 `src/dog_behavior/package.xml` -- 注册新增源码与测试目标，必要时补齐依赖声明 -- 保证可构建可测试。

**Acceptance Criteria:**
- Given `/behavior/execute_trigger` 收到合法行为名，when `BehaviorNode` 处理该消息，then 行为树先执行 action 触发叶子并继续到占位叶子，且现有 `ExecuteBehavior` 请求内容保持不变。
- Given action 触发前置条件不满足或 goal 被拒绝，when 行为树执行，then 占位叶子不会被执行，节点状态仍遵循现有失败/拒绝语义。
- Given 运行新的 gtest 目标，when 执行包级测试，then 新增行为树测试与现有节点测试均通过。

## Spec Change Log

## Design Notes

行为树只承担“组织触发路径”的职责，不接管 `BehaviorNode` 里的恢复过滤、空转熔断、watchdog 和 action 状态机。当前树的唯一可见行为是：把触发消息送入现有 action 触发器，然后经过一个明确的空占位叶子，为后续具体行为节点预留结构而不提前绑定业务逻辑。

## Verification

**Commands:**
- `source /opt/ros/humble/setup.bash && colcon build --packages-select dog_behavior` -- expected: `dog_behavior` 构建成功且新增测试目标被注册。
- `source /opt/ros/humble/setup.bash && colcon test --packages-select dog_behavior && colcon test-result --all --verbose` -- expected: 行为树单测与现有节点测试通过。

## Suggested Review Order

**Routing Path**

- 行为树入口先接管触发消息，再复用既有 action 触发逻辑。
	[`behavior_node.cpp:73`](../../src/dog_behavior/src/behavior_node.cpp#L73)

- Topic 回调改为树路由，保留原有失败日志语义。
	[`behavior_node.cpp:235`](../../src/dog_behavior/src/behavior_node.cpp#L235)

**Tree Core**

- 动作叶子与占位叶子按固定顺序串联，形成最小可扩展树骨架。
	[`behavior_tree.cpp:1`](../../src/dog_behavior/src/behavior_tree.cpp#L1)

- 公共接口暴露 callback 注入点，测试可观察占位叶子。
	[`behavior_tree.hpp:1`](../../src/dog_behavior/include/dog_behavior/behavior_tree.hpp#L1)

**Build And Tests**

- 新树源码和 gtest 目标注册在包级构建里，确保可编译可回归。
	[`CMakeLists.txt:15`](../../src/dog_behavior/CMakeLists.txt#L15)

- 新增树单测验证成功、失败和空行为名路径。
	[`test_behavior_tree.cpp:1`](../../src/dog_behavior/test/test_behavior_tree.cpp#L1)

- 现有节点测试仍覆盖 topic 到 action 的集成回归。
	[`test_behavior_node.cpp:493`](../../src/dog_behavior/test/test_behavior_node.cpp#L493)

</frozen-after-approval>