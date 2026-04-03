# Story 2.2: 异步动作指令 Action 发送与监听

Status: done

## Story

As a 机器狗系统,
I want 使用 Action Client 下发 3D 抓取位姿序列并监控反馈,
so that 可异步推动任务流。

## Acceptance Criteria

1. Given 运控 Action Server，When 发送目标，Then 利用异步 Action API（无阻塞）完成监听并推进下一个状态机步骤。

## Tasks / Subtasks

- [x] 在 `dog_behavior` 建立 Action Client 主链路（AC: 1）
  - [x] 在 `BehaviorNode` 中引入 `rclcpp_action::Client<dog_interfaces::action::ExecuteBehavior>`，并声明参数 `execute_behavior_action_name`（默认建议 `/behavior/execute`）。
  - [x] 在节点启动后等待 Action Server 可用（带超时与日志），不可在回调中阻塞等待。
  - [x] 复用 Story 2.1 的位姿输出能力，作为 Action 目标输入来源，不新增重复位姿转换节点。

- [x] 实现异步目标发送与反馈监听（AC: 1）
  - [x] 新增发送入口（方法或触发通道），将目标位姿封装为 `ExecuteBehavior::Goal`（`behavior_name` + `target_pose`）。
  - [x] 使用异步发送（Action 的 async 接口）挂接 goal_response / feedback / result 回调，禁止 `while + spin_some` 忙等。
  - [x] 在 `result` 回调中根据 `accepted/detail` 与执行状态推进本地状态（至少区分成功、失败、拒绝、超时）。

- [x] 对齐架构约束与容错边界（AC: 1）
  - [x] 严格遵守“回调非阻塞”与“参数仅初始化期读取”规则，不在高频回调中 `get_parameter(...)`。
  - [x] 失败路径必须可观测：server 不可用、goal 被拒绝、result 异常、反馈超时均有结构化日志。
  - [x] 保持职责边界：本故事只完成 Action 发送与监听，不引入熔断策略（Story 3.1）与持久化恢复（Epic 5）。

- [x] 测试与验收（AC: 1）
  - [x] 扩展 `dog_behavior` 单测，使用最小 Mock Action Server 验证：可接收 goal、反馈可回调、result 可驱动状态变更。
  - [x] 增加失败分支测试：server 不可用或 goal 拒绝时节点不崩溃并产生日志。
  - [x] 运行 `dog_behavior` 包定向测试并通过。

### Review Findings

- [x] [Review][Patch] `kSendingGoal` 窗口可重复发起目标，存在并发竞态 [src/dog_behavior/src/behavior_node.cpp:triggerExecuteBehavior]
- [x] [Review][Patch] 反馈超时后状态会被 `CANCELED` 结果覆盖为 `failed`，丢失 `timeout` 语义 [src/dog_behavior/src/behavior_node.cpp:feedbackWatchdogTimerCallback/resultCallback]
- [x] [Review][Patch] 持锁调用 `async_cancel_goal` 存在回调链路锁竞争风险 [src/dog_behavior/src/behavior_node.cpp:feedbackWatchdogTimerCallback]
- [x] [Review][Patch] `action_server_wait_timeout_sec` 与 `feedback_timeout_sec` 缺少参数边界校验 [src/dog_behavior/src/behavior_node.cpp:BehaviorNode]
- [x] [Review][Patch] result 异常分支日志上下文字段不足（code/detail/action） [src/dog_behavior/src/behavior_node.cpp:resultCallback]
- [x] [Review][Patch] 缺少反馈超时与 result 异常分支自动化测试 [src/dog_behavior/test/test_behavior_node.cpp]

## Dev Notes

- 本故事承接 Story 2.1，当前 `BehaviorNode` 已具备 `Odometry -> PoseStamped` 发布能力与参数化 topic 配置。
- 当前接口契约已存在：`dog_interfaces/action/ExecuteBehavior.action`，字段为 `behavior_name + target_pose`，反馈为 `progress/state`，结果为 `accepted/detail`。
- `epics.md` 中“`async_send_request`”在 Action 语义下应落地为 Action 客户端异步发送接口；重点是“纯异步、无阻塞推进状态机”。

### Technical Requirements

- 统一使用 ROS 2 Action Client 与 `dog_interfaces::action::ExecuteBehavior`，不得退化为同步 Service。
- 发送目标时保持 `PoseStamped` 坐标系一致性，延续 Story 2.1 的 frame/stamp 处理。
- 回调链路需要明确处理：goal_response、feedback、result 三阶段。

### Architecture Compliance

- 保持 `dog_behavior` 作为对外执行通信边界，不把 Action 客户端下沉到 `dog_perception`。
- 遵循命名与结构规则：`.hpp/.cpp` 分离、成员后缀 `_`、Topic/Action 名称小写下划线。
- 严禁在回调内 `sleep_for` 或阻塞等待，保持执行器可重入与系统实时性。

### Library & Framework Requirements

- ROS 2 Humble：`rclcpp`、`rclcpp_action`、`geometry_msgs`、`nav_msgs`、`dog_interfaces`。
- 若新增依赖，必须同步更新 `dog_behavior` 的 `CMakeLists.txt` 与 `package.xml`。

### File Structure Requirements

- 预期主要改动文件：
  - `src/dog_behavior/include/dog_behavior/behavior_node.hpp`
  - `src/dog_behavior/src/behavior_node.cpp`
  - `src/dog_behavior/test/test_behavior_node.cpp`
  - `src/dog_behavior/CMakeLists.txt`（如新增 action 相关依赖）
  - `src/dog_behavior/package.xml`（如新增 action 相关依赖）

### Testing Requirements

- 功能链路：Action goal 发送、feedback 接收、result 处理完整可达。
- 异常链路：server 不可用 / goal rejected / result error 可被正确处理且不中断节点主循环。
- 工程质量：`colcon test --packages-select dog_behavior` 通过，无新增 warning。

### Previous Story Intelligence (2.1)

- 已建立可复用输入：`BehaviorNode` 内部里程计订阅与 `PoseStamped` 输出链路完整。
- 已建立数据健壮性检查：空 `frame_id` 回填、非有限值过滤、四元数范数校验。
- 已有测试基线可扩展：`test_behavior_node.cpp` 覆盖发布路径与异常分支，可直接追加 Action 用例。

### Git Intelligence Summary

- 最近提交呈现“按 Story 粒度交付”的模式，`dog_behavior` 已在 `feat: 实现实时全局本体位姿发布` 中形成稳定测试与日志风格。
- 继续沿用当前风格可降低回归风险：参数化 topic、明确日志上下文、单包定向测试。

### Project Context Reference

- 遵循 `_bmad-output/project-context.md`：智能指针与零拷贝优先、日志统一 `RCLCPP_*`、不在回调做阻塞操作。
- 遵循 `_bmad-output/planning-artifacts/architecture.md`：Action Client/Server 契约、异步回调推进状态机、严格边界分层。

## References

- `_bmad-output/planning-artifacts/epics.md`（Epic 2 / Story 2.2）
- `_bmad-output/planning-artifacts/prd.md`（FR-2.3, FR-2.4, FR-2.5）
- `_bmad-output/planning-artifacts/architecture.md`（Action 决策与非阻塞规则）
- `_bmad-output/project-context.md`（代理实现约束）
- `docs/interface-architecture.md`（对外通信通道语义）
- `src/dog_interfaces/action/ExecuteBehavior.action`（Action 契约）
- `src/dog_behavior/src/behavior_node.cpp`（当前 2.1 交付基线）

## Story Completion Status

- 状态：`done`
- 完成说明：Story 2.2 任务与子任务已全部完成，AC 已满足，`dog_behavior` 定向测试通过，等待代码评审。

## Dev Agent Record

### Agent Model Used

GPT-5.3-Codex

### Debug Log References

- create-story workflow execution in VS Code Copilot session (2026-04-03)
- sprint-status 自动选取首个 backlog 故事：`2-2-异步动作指令-action-发送与监听`
- 读取并分析：`epics.md`、`prd.md`、`architecture.md`、`project-context.md`、`2-1-实时全局本体位姿发布.md`
- 代码现状核查：`dog_behavior` 当前位姿发布链路与 `dog_interfaces/ExecuteBehavior.action`
- 实现 Action 客户端与异步回调链：`goal_response` / `feedback` / `result`，并加入反馈超时 watchdog
- 新增触发入口：`triggerExecuteBehavior(...)` 方法与 `execute_behavior_trigger_topic` 订阅
- 验证命令：`colcon build --packages-select dog_behavior`、`colcon test --packages-select dog_behavior`、`colcon test-result --verbose --test-result-base build/dog_behavior`

### Completion Notes List

- 自动发现并锁定 Story 2.2，按模板生成可执行故事文档。
- 注入 Action 契约与异步回调约束，避免错误实现为阻塞式流程。
- 明确与 Story 2.1、Epic 3、Epic 5 的职责边界，防止范围蔓延。
- 提供可直接落地的测试项（成功链路 + 失败链路）。
- 已在 `BehaviorNode` 落地 Action Client 主链路，启动后通过非阻塞定时轮询等待 Action Server 并记录超时日志。
- 已复用 Story 2.1 位姿链路作为目标来源，发送 `ExecuteBehavior::Goal(behavior_name + target_pose)`，并在结果回调推进状态至成功/失败/拒绝/超时。
- 已补齐 `dog_behavior` 单测：Action 成功链路、goal 拒绝、server 不可用三类场景，测试全部通过。

### File List

- `_bmad-output/implementation-artifacts/2-2-异步动作指令-action-发送与监听.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
- `src/dog_behavior/include/dog_behavior/behavior_node.hpp`
- `src/dog_behavior/src/behavior_node.cpp`
- `src/dog_behavior/test/test_behavior_node.cpp`
- `src/dog_behavior/CMakeLists.txt`
- `src/dog_behavior/package.xml`

### Change Log

- 2026-04-03: 实现 Story 2.2 Action 异步发送与监听链路，新增对应成功/失败分支测试并完成包级构建与测试验证。

