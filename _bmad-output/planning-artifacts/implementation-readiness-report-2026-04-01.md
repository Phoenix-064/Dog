---
stepsCompleted:
  - 1
---

# Implementation Readiness Assessment Report

**Date:** 2026-04-01
**Project:** Dog


## PRD Documents Files Found

**Whole Documents:**
- prd.md (29K, 2026-04-01)

**Sharded Documents:**
None

## Architecture Documents Files Found

**Whole Documents:**
- architecture.md (33K, 2026-04-01)
- architecture_struct_append.md (5.3K, 2026-04-01)
- architecture_val_append.md (4.7K, 2026-04-01)

**Sharded Documents:**
None

## Epics & Stories Documents Files Found

**Whole Documents:**
- epics.md (17K, 2026-04-01)

**Sharded Documents:**
None

## UX Design Documents Files Found

**Whole Documents:**
None

**Sharded Documents:**
None

## PRD Analysis

### Functional Requirements

FR-1.1: [视觉节点] 能够接收并解析外部摄像头的原始图像流。
FR-1.2: [点云节点] 能够接收 Livox 激光雷达数据并输出高频里程计与环境点云。
FR-1.3: [数学解算器] 能够在给定目标 2D 像素坐标与点云深度的前提下，计算出目标在世界坐标系及机器狗本体机器坐标系（`base_link`）下的精确 3D 位姿。
FR-1.4: [视觉节点] 能够识别并读取场地角落的数字任务题，并完成基于数字规则的内部逻辑指令分发。
FR-1.5: [状态管理器] 能够基于时间线融合或对齐图像与点云数据，并在单方面数据短暂缺失时依据历史运动先验执行外推计算（Extrapolation）。
FR-2.1: [系统守护者] 能够在系统冷启动或首帧传感器数据接收前，验证并完成视觉全局地图坐标与运控里程计基准坐标的初始对齐。
FR-2.2: [规划层] 能够通过指定的 ROS 2 话题，向运控层持续发布机器狗本体的全局实时位姿。
FR-2.3: [规划层] 能够通过指定的 ROS 2 话题，向运控层发布视野内目标箱子的 3D 位姿矩阵指令。
FR-2.4: [运控层-API 消费者] 能够通过预定义的反馈接口（如基于压力传感器），向视觉系统返回“已到达”、“抓取成功”或“抓空异常”状态。
FR-2.5: [规划层] 能够异步监听并接收运控底层反馈，并根据反馈触发状态转移或生成补救微调的新位姿系。
FR-3.1: [系统守护者 - 健康心跳] 能够在规定时间内（如持续 2 秒无帧接收）对摄像机或雷达执行健康自检报警，并尝试挂起并重新激活该死锁的数据订阅节点。
FR-3.2: [系统守护者] 能够在检测到单次抓取节点超时流产，或多次捕捉到“抓空”反馈时，触发逻辑层面的重试熔断器。
FR-3.3: [系统守护者] 能够在触发最终熔断后，放弃该抓手目标的抓取并强制产生优雅降级指令（如直接运送其他已满载箱子以避免被罚时或总分过低）。
FR-3.4: [状态管理器] 能够在系统意外掉电或被急停重置重启后，从本地非易失持久文件或内存日志中拉取已完成的任务节点与已分配搬运进度（箱子映射状态表），恢复流程并继续运行。
FR-3.5: [系统守护者] 能够在机器狗执行长程运管（无需目标重新解算）或处于待命物理急停态时，控制视觉节点转为极低负荷状态的本地位置坐标持续空转发布（Idle Spinning）。
FR-4.1: [算法同学] 能够在不依赖外部硬件推理卡（纯 CPU）的环境下，稳定编译并运行 MVP 大部分全链路功能包流。
FR-4.2: [算法同学] 能够依靠底层的工厂构建机制（Factory Builder），通过隔离的动态链接或重启单一 ROS 2 Node 的形式，热替换核心视觉目标检测与 PnP 推理算法的业务实现体，免除整车网络重新编调的灾难风险。
FR-4.3: [统筹或调参运维] 能够专门通过一套抽象加载在全节点外部的 YAML 文件，一次性调配和热覆盖所有的机械外参矩阵与关键功能超时熔断阈值（决不允许在 `cpp` 层写死参数）。

Total FRs: 18

### Non-Functional Requirements

NFR1: [Performance] 在单台 NUC 未挂载加速器时，核心推理全流程时延必须控制在 < 50ms（即 >= 20 FPS）。
NFR2: [Performance] 视觉与点云节点持续高压运行 30 分钟，合并内存占用不超过系统 60%（约 8GB），严防内存泄漏或 OOM。
NFR3: [Reliability] 熔断软重启单个节点从拉起到输出首帧有效数据耗时 < 2 秒。
NFR4: [Reliability] 视觉验证矛盾时，系统必须在 1 秒内果断抛出异常并降级开环运控。
NFR5: [Reliability] 系统栈支持连续 2 小时以上不强制重启运行。
NFR6: [Environmental] 内部 IPC 免受外部 Wi-Fi 阻塞，当外部 Wi-Fi 断开或丢包 100% 时，核心节点间本地回环时延波动不超过 5ms。
NFR7: [Environmental] 视觉模型在强遮挡、逆光、激光笔干扰下，伪阳性率应低于 2%。
NFR8: [Development] 基于 CMake 和 colcon 的编译过程必须实现 Zero Warnings 零警告。
NFR9: [Development] 替换视觉检测类的修改面积不超过 2 个文件，不修改主节点核心收发线程。

Total NFRs: 9

### Additional Requirements

- 竞赛安全规约：必须满足物理急停发生时，上层计算模块要有合适的挂起或恢复策略。
- 算力孤岛要求：必须配置 `ROS_LOCALHOST_ONLY=1` 或隔离 `ROS_DOMAIN_ID`。
- 不引入 Docker 容器化方案，要求裸机环境 `colcon build`。

### PRD Completeness Assessment

PRD 的结构非常完整，各需求配有具体的场景和边界定义，同时对于异常状态熔断、恢复策略刻画清晰，能够为接下来的工程架构与史诗映射提供清晰依据。

## Epic Coverage Validation

### Coverage Matrix

| FR Number | PRD Requirement | Epic Coverage | Status |
| --------- | --------------- | -------------- | --------- |
| FR-1.1 | 接收并解析外部摄像头的原始图像流。| Epic 1 | ✓ Covered |
| FR-1.2 | 接收 Livox 激光雷达数据并输出环境点云。| Epic 1 | ✓ Covered |
| FR-1.3 | 计算出目标在世界及本体坐标系精确 3D 位姿。| Epic 1 | ✓ Covered |
| FR-1.4 | 识别数字任务题并分发内部逻辑指令。| Epic 1 | ✓ Covered |
| FR-1.5 | 依据历史运动先验执行外推计算(Extrapolation)。| **NOT FOUND** | ❌ MISSING |
| FR-2.1 | 验证完成全局地图坐标与运控里程计的初始对齐。| **NOT FOUND** | ❌ MISSING |
| FR-2.2 | 向运控持续发布机器狗本体的全局实时位姿。| Epic 1 | ⚠️ Partial |
| FR-2.3 | 向运控发布视野内目标箱子的 3D 位姿矩阵指令。| Epic 1 | ✓ Covered |
| FR-2.4 | 返回“已到达”、“抓取成功”或“抓空异常”状态。| Epic 4 | ✓ Covered |
| FR-2.5 | 监听接收底层反馈并触发状态转移新位姿。| Epic 4 | ✓ Covered |
| FR-3.1 | 2秒无帧接收心跳健康自检并挂起激活死锁节点。| **NOT FOUND** | ❌ MISSING |
| FR-3.2 | 检测抓空反馈时，触发逻辑层面的重试熔断器。| Epic 4 | ✓ Covered |
| FR-3.3 | 熔断后强制放弃目标抓取并产生优雅降级指令。| Epic 4 | ⚠️ Partial |
| FR-3.4 | 从本地非易失持久文件或内存日志拉取进度恢复。| **NOT FOUND** | ❌ MISSING |
| FR-3.5 | 待命物理急停态转为本地位置坐标持续空转发布。| **NOT FOUND** | ❌ MISSING |
| FR-4.1 | 纯 CPU 环境编译并运行 MVP 链路。| **NOT FOUND** | ❌ MISSING |
| FR-4.2 | 工厂动态链接热替换业务体免除重新编调风险。| **NOT FOUND** | ❌ MISSING |
| FR-4.3 | 全节点外部的 YAML 文件覆盖所有矩阵与阈值。| **NOT FOUND** | ❌ MISSING |

### Missing Requirements

#### Critical Missing FRs

**FR-1.5: 历史先验外推 (Extrapolation)**
- Impact: 缺乏点云与图像单方短暂缺失时的前推补偿机制，会导致系统瞬间死锁。
- Recommendation: 应在 Epic 1 或独立的追踪/解算 Epic 中包含相关 Story。

**FR-2.1: 坐标初始对齐**
- Impact: 若系统启动时视通全局地图与里程计未对齐，发布的框体 3D 坐标对运控毫无意义。
- Recommendation: 需加入 Epic 1 初始系统状态的故事里。

**FR-3.1: 节点心跳守护与死锁重拉起**
- Impact: 高压赛场出现相机掉线时，系统将直接陷入僵死，违反底线原则。
- Recommendation: 建一个新的 Epic 处理系统级守护与生命周期管理，或者补充进 Epic 4。

**FR-3.4: 掉电持久化拉取与重启恢复**
- Impact: 比赛中如遇断电，不仅需要当前持久化的方案存储（Epic的附加需求提到了），更需要在启动生命周期中拉取它并重构任务状态机。
- Recommendation: 应作为一个专门的 Story 在 Epic 2 或独立 Epic 下体现。

### Coverage Statistics

- Total PRD FRs: 18
- FRs covered in epics: 10 (8 fully, 2 partially)
- Coverage percentage: ~55.5%

## UX Alignment Assessment

### UX Document Status

Not Found (未发现专有 UX 设计文档)

### Alignment Issues

根据系统类型分析：
本系统为 **物联网/嵌入式系统（IoT/Embedded - 边缘 ROS 2 节点化应用）**，核心场景在于机器视觉、点云处理与运动控制节点间的数据流转与自动决策。

**PRD 中对 UI/UX 的暗示分析：**
1. **赛场监控旅程**：提及领队在场外监控，但明确说明**“外部监控功能并非现阶段紧急需求”**，且为了单机性能，严禁发布稠密图像/点云供外部监控。
2. **终端交互模式**：系统预期在裸机上通过 `colcon build` 终端直接启停。无需传统图形用户界面的开发。开发与调试依赖标准的 ROS 2 工具侧（如 Rviz）。

### Warnings

系统中未包含专门的 UX 文档属于**正常且合理**的架构决策。没有由于缺乏 UX 规范而导致的实施受阻风险。因此不需要增加额外警告。

## Epic Quality Review

根据敏捷开发和系统规范最佳实践，对 `epics.md` 中的架构、独立性和用户价值设定进行审查：

### 🔴 关键违规项 (Critical Violations)

1. **用户角色/画像滥用 (Technical Personas in Stories)**
   - **问题说明**：大量 User Story 使用了“作为 行为规划模块 (As a 行为规划模块)” 或 “作为 ROS 2 开发者” 等纯技术组件作为 Persona，违背了“交付用户/业务价值”的核心理念。
   - **具体示例**：Story 1.2 (As a 行为规划模块)、Story 1.3 (As a 规则引擎模块)、Story 3.1 (As a 行为规划开发者)。
   - **修复建议**：应调整为真实世界的业务角色或赛场系统，例如：“作为赛场监控系统”、“作为机器狗本体”、“作为外包运控系统”。

### 🟠 主要问题 (Major Issues)

1. **缺乏对基础设施（持久化介质）准备的明确节点**
   - **问题说明**：在 Epic 2 (Story 2.2) 和 Epic 4 (Story 4.1) 中使用了持久性归档与状态恢复。但没有任何一个早期的 Story 负责建立这个基础持久化文件或 YAML 的 I/O 服务模块。这导致 2.2 和 4.1 在实现时会产生冲突或隐式的跨史诗依赖。
   - **修复建议**：在 Epic 1 或单独的基础设施 Epic 中引入统一读写日志及系统健康状态的 Story。

2. **史诗边界过大与缺失需求 (Incomplete FR Mapping / Epic Sizing)**
   - **问题说明**：如前述覆盖率验证所示，PRD 中包含多个关于网络降级、心跳重拉起（FR-3.1, FR-3.5）的功能。这些功能在目前的史诗列表中没有任何体现。这意味着现有的 Epic 无法产生一个符合 PRD 安全底线的最终产品。
   - **修复建议**：增加 Epic 5（赛场极端异常守护与网络降级），承载剩余的 FR。

### 🟡 次要问题 (Minor Concerns)

1. **验收标准(AC) 的可量化程度**
   - **问题说明**：Story 1.4 中的 AC “满足时延阈值（超时帧可被丢弃）” 缺乏具体的数值定义。尽管 PRD 中写了 < 50ms，但 Story 级别未将具体测试阈值实例化，导致 QA 无法独立根据该 Story 验收。
   - **修复建议**：在包含性能要求的 Story 验收标准中，硬编码回填 PRD 规定的延迟与内存百分比数值。

2. **Starter Template 合规性（合规）**
   - **检查结果**：Epic 1 Story 1 正确执行了 PRD 和架构中要求的 "基于 Starter Template 的初始工程搭建"（创建 `dog_interfaces` 等四个包），符合项目启动的基线最佳实践。

### 质量评估总结

史诗文件目前在宏观逻辑上是合理的（符合感知->规划->执行闭环的顺序），无严重的提前引用（Forward Dependencies）。但在**敏捷规范（Persona 设定）**和**完整度（缺失三分之一的核心可靠性 FR）**上存在重大缺陷，必须在实施前进行修正。

## 总结与改进建议 (Summary and Recommendations)

### 整体实施就绪状态 (Overall Readiness Status)

**NEEDS WORK (需要改进)** ⚠️

### 需要立即采取行动的关键问题 (Critical Issues Requiring Immediate Action)

1. **极低的功能覆盖率与缺失核心模块**：包含掉电恢复、节点重拉起、网络降级外推等 8 项确保“系统生命与不败底线”的 FR （占总 FR 40%以上）完全没有在当前的 Epic/Story 划分中体现。若以此推入开发环节，最终的比赛系统将极其脆弱。
2. **缺乏基础设施规划故事 (Missing Infra Stories)**：如 Epic 2 和 Epic 4 大量依赖于持久化文件进行恢复与重试判断，但缺少明确编排文件操作或持久化组件创建的前期 Story。
3. **错误的用户故事角色 (Improper Personas)**：将内部软件模块（如“作为规则引擎”）作为用户价值的体验主体，这破坏了敏捷追踪的基础，应当更换为赛场视角/系统操作人员视角。

### 下一步建议的操作 (Recommended Next Steps)

1. **增补 Epic 5 (系统生命周期与网络容错守护)**：将 PRD 中要求的 `ApproximateTime` 异步容错、持久性重启日志、和无帧空转（Idle Spinning）等 FR 集中建立技术实施故事。
2. **重构当前 User Stories 的 AC**：清查诸如“小于规定时延”类含糊不清的验收标准，直接在 Story AC 中填写 PRD 中约定的 `< 50ms` 与 `< 2s` 等物理边界限值。
3. **修复 Persona 设计**：执行一个替换动作，将所有以架构模块名作为头衔的 User Story 主体替换为 `作为机器狗系统`、`作为领队` 或 `作为视觉底层节点运维`。

### 最终说明 (Final Note)

本次准备程度评估在 PRD FR 覆盖、UX 分析和史诗质量 3 个大类中识别出了十余个不匹配项和敏捷反模式违规。建议在进入大规模实施编码之前，解决严重遗漏的 PRD 可靠性需求覆盖问题，以免背离“不败之地”的核心产品愿景。您可以选择修正计划产物或在充分知情风险的情况下继续推进。
