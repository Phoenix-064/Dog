# Sprint Change Proposal - 2026-04-02

## 1) Issue Summary（问题摘要）

- 触发问题：Story `1-1-基础工程骨架与接口防腐层搭建` 与 planning-artifacts 架构文档在“决策/行为包”命名上存在冲突（`dog_decision` vs `dog_behavior`）。
- 新增触发问题 A：Story 1.1 的 Review 条目要求“恢复 `Detect`/`Localization` 构建入口”，但当前仓库已完成新架构收敛，`Detect`/`Localization` 属于旧架构遗留，不应作为现阶段验收约束。
- 新增触发问题 B：Story 1.1 已定义 `dog_lifecycle` 包结构，但 `architecture.md` 的目录树与验证描述出现“三包口径”，与四包基线不一致。
- 发现方式：实施文档与架构附录交叉阅读时发现同一职责包出现双命名，导致验收口径与后续实现指引不一致。
- 证据：
  - `_bmad-output/planning-artifacts/architecture_struct_append.md` 原有 `dog_decision`
  - `_bmad-output/planning-artifacts/epics.md` Additional Requirements 原有 `dog_decision`
  - `_bmad-output/implementation-artifacts/1-1-基础工程骨架与接口防腐层搭建.md` 与实际代码目录使用 `dog_behavior`
  - `_bmad-output/implementation-artifacts/1-1-基础工程骨架与接口防腐层搭建.md` 中存在 `Detect`/`Localization` 恢复条目，但当前 `src/` 仅包含 `dog_interfaces/dog_perception/dog_behavior/dog_lifecycle`
  - `_bmad-output/planning-artifacts/architecture.md` 在“完整目录树/结构对齐”处遗漏 `dog_lifecycle`

## 2) Impact Analysis（影响分析）

### Epic Impact
- Epic 1（Core Perception & Initialization）受影响：主要是 Story 1.1 的“骨架与命名基线”解释一致性。
- 其余 Epic 无范围变化，不需新增/删除 Epic。

### Story Impact
- Story 1.1：需要明确命名基线，避免后续 story 在依赖声明、目录命名、构建脚本上分叉。
- Story 1.1：需要关闭旧架构残留的 `Detect`/`Localization` 恢复项，避免错误驱动开发任务。
- 未来 stories（1.2+）：仅受“文档口径统一”的正向影响，无需改 AC。

### Artifact Conflicts
- 需要更新：
  - `_bmad-output/planning-artifacts/architecture.md`
  - `_bmad-output/planning-artifacts/architecture_struct_append.md`
  - `_bmad-output/planning-artifacts/architecture_val_append.md`
  - `_bmad-output/planning-artifacts/epics.md`
  - `_bmad-output/implementation-artifacts/1-1-基础工程骨架与接口防腐层搭建.md`（仅 Change Log）
  - `_bmad-output/implementation-artifacts/1-1-基础工程骨架与接口防腐层搭建.md`（Review Findings 与 Dev Notes）

### Technical Impact
- 本次为文档一致性修复，不涉及运行时代码逻辑变更。
- 已识别但未在本提案执行：ROS 接收者话题不一致问题（归属 code review 修复）。

## 3) Recommended Approach（推荐路径）

- 选型：**Option 1 Direct Adjustment（直接调整）**。
- 理由：
  - 问题本质是命名基线漂移，适合在文档层面快速收敛。
  - 不改变 MVP 范围与技术路线，不引入重排计划。
- 评估：
  - Effort：Low
  - Risk：Low
  - Timeline Impact：可在当前冲刺内即时消化

## 4) Detailed Change Proposals（详细改动提案）

### 4.1 已批准并落地：命名基线统一（`dog_decision` → `dog_behavior`）

**A. architecture_struct_append.md**
- OLD: `dog_decision/`、`include/dog_decision/`、`dog_decision 的 state_manager_node`
- NEW: `dog_behavior/`、`include/dog_behavior/`、`dog_behavior 的 state_manager_node`
- Rationale: 与 Story 1.1 和当前实现目录一致，消除骨架命名分叉。

**B. architecture.md**
- OLD: 同类 `dog_decision` 命名表述
- NEW: 全部统一为 `dog_behavior`
- Rationale: 保持主架构文档与附录一致。

**C. architecture_val_append.md**
- OLD: `dog_decision` 相关校验表述
- NEW: `dog_behavior` 表述
- Rationale: 校验附录同步，避免次级文档漂移。

**D. epics.md**
- OLD: Starter Template 四包含 `dog_decision`
- NEW: Starter Template 四包改为 `dog_behavior`
- Rationale: 与实际 Epic/Story 实施基线一致。

### 4.2 已批准并落地（按用户修订）：仅补充 Story 变更记录

**implementation story 文档（1-1）**
- OLD: Change Log 仅记录 1.1 初次落地
- NEW: 新增“Correct Course 命名基线统一”日志
- Rationale: 保留审计线索；`Review Findings` 中 deferred 的“ROS话题不一致”维持原状，留给 code review 处理。

### 4.3 本次追加纠偏：关闭旧架构恢复项 + 对齐四包结构

**A. implementation story 文档（1-1）Review Findings**
- OLD: `- [ ] [Review][Patch] 恢复被删除的 Detect/Localization 构建入口...`
- NEW: `- [x] [Review][Close] 关闭 Detect/Localization 恢复项：该项属于旧架构遗留，不再作为“并存约束”验收条件`
- Rationale: 与当前仓库架构基线一致，避免错误任务进入开发队列。

**B. implementation story 文档（1-1）Dev Notes**
- OLD: `现有仓库包含 Detect、livox_ros_driver2、point_lio...`
- NEW: `现有仓库并存模块为 livox_ros_driver2、point_lio 与新建 dog_* 包...`
- Rationale: 移除已退役模块表述，降低误导。

**C. architecture.md（项目结构与验证段）**
- OLD: 目录树仅列 `dog_interfaces/dog_perception/dog_behavior`，并在结构对齐写“三包分离”。
- NEW: 目录树补齐 `dog_lifecycle`；结构对齐改为“四包分离”；FR-3 改为 `dog_behavior + dog_lifecycle` 协同守护。
- Rationale: 与 Story 1.1 和当前代码目录一致，恢复架构文档内部自洽。

## 5) Implementation Handoff（实施交接）

### Scope Classification
- **Minor**（文档基线一致性修复）

### Handoff Recipients & Responsibilities
- Development Team / 文档维护责任人：继续按 `dog_behavior` 命名执行后续 story。
- Code Review 责任人：处理并验证“ROS 接收者话题不一致”项（非本次纠偏改动）。
- SM/PO：无需重排 backlog，仅跟踪 deferred 项关闭状态。

### Success Criteria
- 所有 planning-artifacts 中决策/行为包命名一致。
- Story 1.1 文档保留可追溯纠偏记录。
- Story 1.1 不再包含旧 `Detect`/`Localization` 恢复任务。
- `architecture.md` 与 Story 1.1 在 `dog_lifecycle` 目录结构与职责描述完全一致。
- 代码评审单独关闭“话题不一致”风险项。

## 审批记录
- 模式：Incremental
- 提案 1：Approved（a）
- 提案 2：Edited（e）后已按用户要求落地
- 最终审批：yes（2026-04-02）
- 变更范围分级：Minor
- 路由去向：Development Team（按统一命名继续实施）+ Code Review（关闭 ROS 话题不一致 deferred 项）
- 交接状态：已完成

## Workflow Execution Log

- Issue addressed: Story 1.1 与 planning-artifacts 的决策包命名冲突（dog_decision vs dog_behavior）。
- Additional issues addressed: 关闭旧架构 `Detect/Localization` 恢复误导项；修复 `dog_lifecycle` 文档结构不一致。
- Scope classification: Minor。
- Artifacts modified:
  - _bmad-output/planning-artifacts/architecture.md
  - _bmad-output/planning-artifacts/architecture_struct_append.md
  - _bmad-output/planning-artifacts/architecture_val_append.md
  - _bmad-output/planning-artifacts/epics.md
  - _bmad-output/implementation-artifacts/1-1-基础工程骨架与接口防腐层搭建.md
  - _bmad-output/planning-artifacts/sprint-change-proposal-2026-04-02.md
- Routed to: Development Team、Code Review。
- Completion message: Correct Course workflow complete, Phoenix!
