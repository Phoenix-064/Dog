# Behavior 对外 Action 与 Nav2 导航映射规范

## 1. 文档目标

本文档定义 dog_behavior 对外标准接口与内部 Nav2 导航接口之间的稳定映射规则，用于：

1. 统一外部调用入口，避免上层直接耦合 Nav2 细节。
2. 明确字段、状态、错误语义的双向转换规则。
3. 为实现改造、回归测试与联调验收提供可执行标准。

## 2. 范围与边界

范围包含：

1. 对外 Action：/behavior/execute，类型 dog_interfaces/action/ExecuteBehavior。
2. 行为树导航 Action：/navigate_to_pose，类型 nav2_msgs/action/NavigateToPose。
3. 导航执行状态 Topic：/behavior/nav_exec_state，类型 std_msgs/msg/String。

范围不包含：

1. Nav2 内部控制器、规划器参数调优。
2. 底层运控速度接口协议定义。

## 3. 设计原则

1. 对外单一入口：上层系统只依赖 ExecuteBehavior。
2. 行为树内聚适配：BehaviorTreeNode 与 BT 叶子负责语义路由与字段映射。
3. 语义稳定优先：外部结果语义稳定，不透出 Nav2 实现细节。
4. 错误可观测：任何失败路径都应给出可检索 detail 文本。

## 4. 接口总览

| 层级 | 接口名 | 类型 | 角色 |
| --- | --- | --- | --- |
| 外部调用层 | /behavior/execute | dog_interfaces/action/ExecuteBehavior | 对外标准行为入口 |
| 行为树到 Nav2 层 | /navigate_to_pose | nav2_msgs/action/NavigateToPose | 导航执行请求 |
| 状态观测层 | /behavior/nav_exec_state | std_msgs/msg/String | 导航执行状态观测 |

## 5. Goal 字段映射

### 5.1 ExecuteBehavior Goal 到 NavigateToPose Goal

| 源字段 | 目标字段 | 规则 |
| --- | --- | --- |
| behavior_name | 路由标签 | 用于判断是否进入导航通道，不进入 Nav2 目标字段 |
| target_pose | pose | 完整拷贝为 NavigateToPose.Goal.pose |

### 5.2 路由规则

1. 行为树或路由策略判定为导航语义后，发送到 /navigate_to_pose。
2. 非导航语义继续走 ExecuteBehavior 原有执行链路。
3. 路由判定结果必须记录日志，含 behavior_name 与目标 action 名。

## 6. Feedback 映射

### 6.1 NavigateToPose Feedback 到 ExecuteBehavior Feedback

| 导航反馈来源 | 对外反馈字段 | 映射规则 |
| --- | --- | --- |
| 当前导航状态 | state | 统一输出 running 或子状态文本，如 running_navigation |
| distance_remaining | progress | 按基线距离归一化，progress = clamp(1 - d/d0, 0, 1) |
| 无法计算进度 | progress | 回退为上次有效进度，若首次则为 0.0 |

### 6.2 进度计算要求

1. 首次收到反馈时记录 d0。
2. d0 小于最小阈值时，直接将 progress 置为 1.0。
3. progress 必须单调不下降。

## 7. Result 映射

### 7.1 ResultCode 到 ExecuteBehavior.Result

| Nav2 ResultCode | accepted | detail |
| --- | --- | --- |
| SUCCEEDED | true | nav_succeeded |
| CANCELED 且因 idle_spinning 或 degraded | false | nav_canceled_by_mode |
| CANCELED 且因反馈超时 | false | nav_canceled_by_timeout |
| CANCELED 其他原因 | false | nav_canceled |
| ABORTED | false | nav_aborted |
| 其他未知码 | false | nav_unknown_result_code |

### 7.2 统一失败语义

1. 对外 accepted 仅在导航成功时为 true。
2. 任意失败或取消路径均为 accepted=false，detail 必填。

## 8. 状态机对齐

### 8.1 内部状态到对外状态文本

| 内部执行状态 | 对外状态建议 |
| --- | --- |
| waiting_server 或 waiting_nav2_server | waiting_server |
| sending_goal | sending_goal |
| running | running |
| succeeded | succeeded |
| timeout | timeout |
| rejected | rejected |
| failed | failed |
| idle | idle |

### 8.2 状态约束

1. 任何终态后必须回到 idle 或等待下一次触发。
2. timeout 必须伴随下游 cancel 行为。
3. mode 门控触发取消时，最终状态应可区分为 canceled_by_mode。

## 9. 输入校验与拒绝条件

导航请求在发送前必须满足：

1. 下游 action server 就绪。
2. 当前无在途目标。
3. target_pose 为有限值。
4. target_pose 四元数范数合法。
5. 非 idle_spinning 与 degraded 模式。
6. 未被 recovery_context completed 状态阻断。

拒绝时对外返回策略：

1. accepted=false。
2. detail 使用稳定枚举值，如 rejected_invalid_pose、rejected_unavailable、rejected_by_mode。

## 10. 参数与可配置项

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| execute_behavior_action_name | /behavior/execute | 对外标准 Action 名 |
| nav2_action_name | /navigate_to_pose | 下游 Nav2 Action 名 |
| action_server_wait_timeout_sec | 5.0 | 对外 Action server 等待超时 |
| nav2_server_wait_timeout_sec | 10.0 | Nav2 Action server 等待超时 |
| feedback_timeout_sec | 2.0 | ExecuteBehavior 反馈超时 |
| navigate_feedback_timeout_sec | 10.0 | 导航反馈超时 |

## 11. 测试与验收标准

### 11.1 必测用例

1. 导航成功映射：对外 accepted=true，detail=nav_succeeded。
2. 导航拒绝映射：非法 pose 被拒绝并返回 rejected_invalid_pose。
3. 反馈超时映射：触发 timeout 与 cancel，detail=nav_canceled_by_timeout。
4. 模式门控映射：idle_spinning 或 degraded 触发取消，detail=nav_canceled_by_mode。
5. 服务不可用映射：等待超时后返回 rejected_unavailable 或 server_unavailable 语义。

### 11.2 验收门槛

1. 新增映射逻辑具备单元测试覆盖。
2. 所有 detail 值为固定枚举，不允许自由文本漂移。
3. 联调日志可追踪一条请求在行为树与 Nav2 间的完整链路。

## 12. 迁移计划

1. 第一步：补齐映射常量与状态对照表，不改外部接口。
2. 第二步：实现 feedback 进度归一化与 detail 标准化。
3. 第三步：补充单测并回归行为树导航叶子测试。
4. 第四步：在集成环境灰度发布，观察错误码分布与超时率。

## 13. 向后兼容性

1. 对外 Action 名与消息定义保持不变。
2. 新增 detail 枚举值不影响旧调用方字段解析。
3. 行为树内部继续使用 NavigateToPose，不向上暴露 Nav2 细节。
