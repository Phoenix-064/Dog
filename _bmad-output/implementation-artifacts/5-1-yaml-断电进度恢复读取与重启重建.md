# Story 5.1: YAML 断电进度恢复读取与重启重建

Status: done

## Story

As a 领队,
I want 系统在不预期掉电重启后能基于缓存找回当前赛场进度,
so that 省去由于断电必须重新发车的重创。

## Acceptance Criteria

1. Given 持久化文件存在（Story 1.5 构建），When 系统从死机拉起首帧，Then 脱离该 YAML 重构上下文节点避免执行已完成的任务（FR-3.4），全栈拉起到准备完毕耗时低于 < 2s（NFR-3）。

## Tasks / Subtasks

- [x] 构建“启动即恢复”的主链路（AC: 1）
  - [x] 在 `dog_lifecycle::LifecycleNode` 启动阶段读取 `YamlStateStore::Load()` 返回状态，并形成可发布的恢复上下文。
  - [x] 明确恢复状态最小集合：`task_phase`、`target_state`、`timestamp_ms`、`version`，禁止引入与当前契约不兼容的新必填字段。
  - [x] 若主文件损坏但备份可用，复用 `YamlStateStore` 既有修复语义，不新增平行修复实现。

- [x] 建立“避免重复执行已完成任务”的恢复裁决（AC: 1）
  - [x] 在 `dog_lifecycle` 内根据恢复状态生成“已完成任务屏蔽”信号（例如发布恢复事件/状态），供行为层跳过已完成目标。
  - [x] 与 `dog_behavior` 对齐最小输入契约（任务 ID/阶段），仅做必要映射，不重构 Epic 2 的 Action 链路。
  - [x] 明确恢复失败降级策略：无有效状态时进入“冷启动全新流程”，并输出结构化日志，不阻塞节点拉起。

- [x] 满足重启时间预算与非阻塞约束（AC: 1）
  - [x] 恢复读取与状态映射必须在节点初始化路径完成，禁止在高频回调内执行阻塞 IO。
  - [x] 记录恢复阶段关键耗时（load、parse/validate、publish/dispatch），并验证总耗时满足 < 2 秒目标。
  - [x] 保持回调链路无 `sleep_for` / 忙等；所有下游通知采用 ROS 2 异步消息机制。

- [x] 完成测试与验收（AC: 1）
  - [x] 在 `dog_lifecycle` 新增/扩展单测：有效状态恢复、损坏主文件回退备份、无文件冷启动三条路径。
  - [x] 增加恢复裁决测试：已完成任务会被跳过，未完成任务会继续执行。
  - [x] 增加时延断言测试：恢复链路在测试环境可稳定满足 < 2 秒门槛。
  - [x] 运行定向验证：`colcon test --packages-select dog_lifecycle dog_behavior`。

### Review Findings

- [x] [Review][Patch] 持久化状态会话策略落地：正常停机清理、异常中断保留 [src/dog_lifecycle/src/lifecycle_node.cpp]

- [x] [Review][Patch] 恢复上下文 KV 拼接未转义导致解析注入风险 [src/dog_lifecycle/src/lifecycle_node.cpp]
- [x] [Review][Patch] 行为层恢复屏蔽集合只增不减，后续状态无法解除 [src/dog_behavior/src/behavior_node.cpp]
- [x] [Review][Patch] 缺少“主备同时损坏时冷启动”验收测试 [src/dog_lifecycle/test/test_lifecycle_node.cpp]

## Dev Notes

- 本故事直接承接 Story 1.5 已落地的持久化底座，目标是“恢复流程闭环”，不是重新实现 YAML 存储层。
- 该故事是 Epic 5 的首条故事，优先保证“异常重启可恢复”与“不重复执行”两件事，避免引入额外复杂调度。
- 与 Story 5.2/5.3 的边界：本故事不实现丢帧外推补偿与心跳重联，仅提供恢复上下文和恢复后的最小流程衔接。

### Technical Requirements

- 必须复用既有 `YamlStateStore` / `IStateStore`（`src/dog_lifecycle/include/dog_lifecycle/state_store.hpp`、`yaml_state_store.hpp`），禁止重复造轮子。
- 恢复逻辑遵循当前版本约束（`persistence.state_version`），版本不匹配时按无效状态处理并可观测告警。
- 参数读取保持初始化期完成；恢复链路仅在启动阶段执行，不下沉到高频传感回调。
- 日志统一 `RCLCPP_INFO/WARN/ERROR`，禁止 `std::cout/printf`。

### Architecture Compliance

- 保持四包职责边界：
  - `dog_lifecycle`：状态恢复判定与恢复上下文发布；
  - `dog_behavior`：消费恢复信号并执行任务跳过策略；
  - `dog_perception`：不引入持久化写读逻辑。
- 保持 `.hpp/.cpp` 分离与 include 路径规范（带包名前缀）。
- 不改动既有熔断语义（Story 3.1）与工厂热插拔语义（Story 4.1）。

### Library & Framework Requirements

- ROS 2 Humble + `rclcpp` + `std_msgs` + 现有 `dog_interfaces` 契约。
- 持久化依赖继续使用 `yaml-cpp`（经 `yaml_cpp_vendor`），不新增数据库或外部存储依赖。
- 测试沿用 `ament_cmake_gtest` 与现有 `dog_lifecycle`/`dog_behavior` 测试结构。

### File Structure Requirements

- 预期主要改动文件（按最小改动原则）：
  - `src/dog_lifecycle/include/dog_lifecycle/lifecycle_node.hpp`
  - `src/dog_lifecycle/src/lifecycle_node.cpp`
  - `src/dog_lifecycle/config/persistence.yaml`（如需补充恢复参数）
  - `src/dog_lifecycle/test/test_lifecycle_node.cpp`
  - `src/dog_behavior/include/dog_behavior/*.hpp`（仅在需要接收恢复信号时）
  - `src/dog_behavior/src/*.cpp`（仅在需要接线恢复信号时）
  - `src/dog_lifecycle/CMakeLists.txt` / `src/dog_behavior/CMakeLists.txt`（按需）
  - `src/dog_lifecycle/package.xml` / `src/dog_behavior/package.xml`（按需）

### Testing Requirements

- 恢复有效性：给定有效状态文件，启动后能恢复任务上下文并避免重复执行已完成任务。
- 容错恢复：主文件损坏时可回退备份；主备都损坏时冷启动并输出明确错误。
- 性能门槛：恢复链路启动耗时 < 2 秒（NFR-3）。
- 回归保护：不得破坏现有熔断与持久化测试（`test_lifecycle_node`、`test_yaml_state_store`）。

### Reinvention Prevention

- 严禁新增第二套“状态文件格式/存取类”；所有读写统一走 `IStateStore` 抽象。
- 严禁在 `dog_behavior` 或 `dog_perception` 直接读写 YAML 文件。
- 严禁把恢复流程做成阻塞等待外部反馈的同步链路。

### Project Context Reference

- 遵循 `_bmad-output/project-context.md`：零拷贝与智能指针优先、依赖声明同步、日志规范统一。
- 遵循 `_bmad-output/planning-artifacts/architecture.md`：Lifecycle 守护职责、参数初始化读取、异步非阻塞执行器规则。
- 承接 `_bmad-output/implementation-artifacts/1-5-持久化基础设施准备与-io-工具链.md`：原子写、备份回退、版本校验、清理语义。

## References

- `_bmad-output/planning-artifacts/epics.md`（Epic 5 / Story 5.1）
- `_bmad-output/planning-artifacts/prd.md`（FR-3.4, NFR-3）
- `_bmad-output/planning-artifacts/architecture.md`（Data Architecture / Lifecycle Patterns）
- `_bmad-output/project-context.md`（实现约束）
- `_bmad-output/implementation-artifacts/1-5-持久化基础设施准备与-io-工具链.md`（可复用持久化底座）
- `src/dog_lifecycle/include/dog_lifecycle/state_store.hpp`
- `src/dog_lifecycle/include/dog_lifecycle/yaml_state_store.hpp`
- `src/dog_lifecycle/src/yaml_state_store.cpp`
- `src/dog_lifecycle/src/lifecycle_node.cpp`

## Story Completion Status

- 状态：`done`
- 完成说明：代码评审问题已全部修复并通过定向单测，满足完成条件。

## Dev Agent Record

### Agent Model Used

GPT-5.3-Codex

### Debug Log References

- create-story workflow execution in VS Code Copilot session (2026-04-04)
- 自动选取首个 backlog 故事：`5-1-yaml-断电进度恢复读取与重启重建`
- 分析输入：`epics.md`、`prd.md`、`architecture.md`、`project-context.md`
- 复用情报：`1-5-持久化基础设施准备与-io-工具链.md` 与 `dog_lifecycle` 当前源码

### Completion Notes List

- 已将故事状态设置为 `ready-for-dev`。
- 已把“复用 Story 1.5 持久化底座”定义为强制约束，防止重复实现。
- 已给出恢复成功、恢复降级、恢复失败三类可执行行为与测试覆盖要求。
- 已显式限制本故事范围，避免与 5.2/5.3 产生职责混叠。
- 已在 `dog_lifecycle` 启动路径发布恢复上下文（`mode=recovered/cold_start`），并输出 `load_ms/map_ms/total_ms` 指标。
- 已在 `dog_behavior` 接入恢复上下文订阅，针对已完成 `task_phase` 执行跳过，未完成任务保持可执行。
- 已扩展单测覆盖：有效恢复、主损坏回退备份、无文件冷启动、恢复总耗时 < 2s、已完成任务跳过/未完成继续。

### File List

- `_bmad-output/implementation-artifacts/5-1-yaml-断电进度恢复读取与重启重建.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
- `src/dog_lifecycle/include/dog_lifecycle/lifecycle_node.hpp`
- `src/dog_lifecycle/src/lifecycle_node.cpp`
- `src/dog_lifecycle/test/test_lifecycle_node.cpp`
- `src/dog_behavior/include/dog_behavior/behavior_node.hpp`
- `src/dog_behavior/src/behavior_node.cpp`
- `src/dog_behavior/test/test_behavior_node.cpp`

### Change Log

- 2026-04-04: 完成 Story 5.1 实现；新增恢复上下文发布与消费链路，补齐恢复/回退/冷启动/时延与恢复裁决测试。
