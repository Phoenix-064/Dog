---
title: 'Digit 多结果输出与 Target3DArray 贯通改造'
type: 'feature'
created: '2026-04-05'
status: 'done'
baseline_commit: 'a47b74becdf2dd315f4d1c09e9eb2e2865952f16'
context: ['_bmad-output/project-context.md', 'docs/interface-architecture.md', 'docs/integration-architecture.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 当前 `dog_perception` 数字识别链路仅支持单结果 `DigitRecognitionResult`，`perception_node` 也仅发布单个数字目标，无法表达同一帧中的多候选数字及其位置信息，影响下游决策可靠性与可解释性。

**Approach:** 将识别器接口统一升级为返回多结果集合 `DigitRecognitionResultArrary`，集合内每个元素都携带数字类别、置信度和位置；同步改造 `digit_recognizer_factory` 的转换函数使其直接输出 `Target3DArray`，并更新 `perception_node` 与相关测试以消费和验证整组推理结果。

## Boundaries & Constraints

**Always:** 遵守现有 ROS2 包边界；只改动 `dog_perception` 与必要的接口头文件；保持日志使用 `RCLCPP_*`；保持已有低置信度/强反光抑制语义；输出数组元素必须包含位置与置信度。

**Ask First:** 若发现需要变更 `dog_interfaces` 消息定义（例如新增字段）或改变下游节点订阅契约。

**Never:** 不在 `build/`、`install/`、`log/` 下手改；不引入与本需求无关的重构；不删除已有检测器实现。

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 多候选正常输出 | 检测器在同一帧检测到多个数字且置信度高于阈值 | `DigitRecognitionResultArrary` 返回多个元素；`toDigitTarget3D*` 产出 `Target3DArray.targets` 多元素，每个元素含 `target_id`、`position`、`confidence` | N/A |
| 无有效数字 | 无候选或所有候选低于阈值/被抑制 | 结果数组为空或仅包含约定无特征语义（按现有行为约束），发布端行为可预测且不崩溃 | 记录调试日志，不抛异常 |
| 单候选兼容路径 | 仅一个高置信候选 | 数组长度为 1，行为与旧流程语义一致但数据结构升级为数组 | N/A |

</frozen-after-approval>

## Code Map

- `src/dog_perception/include/dog_perception/digit_recognizer.hpp` -- 定义识别结果结构与识别器接口返回类型。
- `src/dog_perception/src/detectors/heuristic_digit_recognizer.cpp` -- 产生启发式数字候选结果。
- `src/dog_perception/src/detectors/mean_intensity_digit_recognizer.cpp` -- 产生基于均值亮度的候选结果。
- `src/dog_perception/src/detectors/opencv_dnn_yolo_digit_recognizer.cpp` -- 产生 YOLO 候选结果。
- `src/dog_perception/src/digit_recognizer_factory.cpp` -- 识别结果转换为 `Target3DArray`。
- `src/dog_perception/src/perception_node.cpp` -- 消费识别器输出并发布语义目标数组。
- `src/dog_perception/test/test_perception_node.cpp` -- 端到端节点行为验证。
- `src/dog_perception/test/test_digit_recognizer_factory.cpp` -- 工厂与转换逻辑验证。

## Tasks & Acceptance

**Execution:**
- [x] `src/dog_perception/include/dog_perception/digit_recognizer.hpp` -- 将单结果返回接口改为 `DigitRecognitionResultArrary`，并在结构中补齐每个数字的位置字段 -- 统一上游输出契约。
- [x] `src/dog_perception/src/detectors/*.cpp` -- 三个数字检测器改为输出“高置信度候选集合”而非单一 Top-1 -- 支持一帧多结果。
- [x] `src/dog_perception/src/digit_recognizer_factory.cpp` -- 将 `toDigitTarget3D` 改为直接返回 `Target3DArray`（批量转换结果+位置+置信度） -- 降低调用侧拼装复杂度。
- [x] `src/dog_perception/src/perception_node.cpp` -- 改造数字识别消费逻辑，处理并发布一组推理结果 -- 打通主流程。
- [x] `src/dog_perception/test/test_perception_node.cpp` 与 `src/dog_perception/test/test_digit_recognizer_factory.cpp` -- 更新并新增多结果场景测试，覆盖边界行为 -- 防回归。

**Acceptance Criteria:**
- Given 检测器返回多个高置信候选, when `perception_node` 处理数字识别, then 发布的 `Target3DArray` 包含对应数量的目标且每个目标有正确 `target_id`、`position`、`confidence`。
- Given 检测器返回空集合或低置信集合, when 执行发布路径, then 系统稳定运行且输出符合既有无特征语义约定。
- Given 任一数字检测器, when 调用 `infer`, then 返回类型为 `DigitRecognitionResultArrary` 且可表达多候选。

## Spec Change Log

## Design Notes

关键设计取舍：
1) 识别器层输出二维信息（类别+置信度）与几何信息（像素/图像位置）同构封装，避免在 `perception_node` 内重复做后处理拼接。
2) 转换函数上收敛：由 `digit_recognizer_factory` 统一构造 `Target3DArray`，保持位置与置信度映射的一致性。
3) 优先保持旧语义可兼容：单结果场景退化为长度为 1 的数组，而不是引入新的特例分支。

## Verification

**Commands:**
- `source /opt/ros/humble/setup.bash && colcon build --packages-select dog_perception` -- expected: `dog_perception` 构建成功。
- `source /opt/ros/humble/setup.bash && colcon test --packages-select dog_perception && colcon test-result --all --verbose` -- expected: 相关测试通过，无新增失败。

## Suggested Review Order

**Runtime Flow**

- 从节点入口看多结果消费与发布路径，先把行为链路看完整。
	[`perception_node.cpp:830`](../../src/dog_perception/src/perception_node.cpp#L830)

- 检查批量转换调用点与日志字段，确认 no_feature 语义一致。
	[`perception_node.cpp:880`](../../src/dog_perception/src/perception_node.cpp#L880)

**Factory Mapping**

- 关注数组到 `Target3DArray` 的映射规则和空结果兜底策略。
	[`digit_recognizer_factory.cpp:122`](../../src/dog_perception/src/digit_recognizer_factory.cpp#L122)

**Detector Output Contract**

- 审核 YOLO 输出解析为多候选的主逻辑和置信度筛选。
	[`opencv_dnn_yolo_digit_recognizer.cpp:205`](../../src/dog_perception/src/detectors/opencv_dnn_yolo_digit_recognizer.cpp#L205)

- 查看接口别名与位置字段定义，确认上游契约统一。
	[`digit_recognizer.hpp:44`](../../src/dog_perception/include/dog_perception/digit_recognizer.hpp#L44)

**Validation**

- 先看节点级多结果集成测试，验证端到端数组消费。
	[`test_perception_node.cpp:215`](../../src/dog_perception/test/test_perception_node.cpp#L215)

- 再看工厂转换测试，确认多结果映射和空输入兜底。
	[`test_digit_recognizer_factory.cpp:160`](../../src/dog_perception/test/test_digit_recognizer_factory.cpp#L160)
