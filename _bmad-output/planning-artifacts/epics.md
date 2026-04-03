---
stepsCompleted:
  - step-01-validate-prerequisites
  - step-02-design-epics
  - step-03-create-stories
  - step-04-final-validation
inputDocuments:
  - /home/phoenix/source/Dog/_bmad-output/planning-artifacts/prd.md
  - /home/phoenix/source/Dog/_bmad-output/planning-artifacts/architecture.md
  - /home/phoenix/source/Dog/_bmad-output/planning-artifacts/implementation-readiness-report-2026-04-01.md
---

# Dog - Epic Breakdown

## Overview

This document provides the complete epic and story breakdown for Dog, decomposing the requirements from the PRD, UX Design if it exists, and Architecture requirements into implementable stories. This version has been fully refactored based on the Implementation Readiness Report (2026-04-01) feedback.

## Requirements Inventory

### Functional Requirements

FR-1.1: [视觉节点] 能够接收并解析外部摄像头的原始图像流。
FR-1.2: [点云节点] 能够接收 Livox 激光雷达数据并输出高频里程计与环境点云。
FR-1.3: [数学解算器] 能够在给定目标 2D 像素坐标与点云深度的前提下，计算出目标精确 3D 位姿。
FR-1.4: [视觉节点] 能够识别数字题并完成内部逻辑指令分发。
FR-1.5: [状态管理器] 依据历史运动先验执行外推计算(Extrapolation)。
FR-2.1: [系统守护者] 验证完成全局地图坐标与运控里程计的初始对齐。
FR-2.2: [规划层] 向运控持续发布本体全局实时位姿。
FR-2.3: [规划层] 向运控发布视野内目标箱子的 3D 位姿矩阵指令。
FR-2.4: [运控层-API 消费者] 返回“已到达”、“抓取成功”或“抓空异常”状态反馈。
FR-2.5: [规划层] 异步监听接收底层反馈并触发状态转移新位姿。
FR-3.1: [守护者] 2秒无帧接收心跳自检报警并挂起激活死锁节点。
FR-3.2: [守护者] 检测抓空反馈时，触发逻辑层面的重试熔断器。
FR-3.3: [守护者] 熔断后强制放弃目标抓取并产生优雅降级指令。
FR-3.4: [状态管理器] 从本地持久文件拉取进度恢复流程，继续运行。
FR-3.5: [守护者] 急停态下控制转为极低负荷持续空转状态发布位置。
FR-4.1: [算法同学] 纯 CPU 环境编译并运行 MVP 链路流程。
FR-4.2: [算法同学] 依靠工厂动态链接热替换业务体免除重新编调风险。
FR-4.3: [统筹运维] 全节点外部的 YAML 文件热覆盖所有的机械外参矩阵与关键阈值。

### NonFunctional Requirements

NFR-1: [Performance] 核心推理全流程控制在 < 50ms（即 >= 20 FPS）。
NFR-2: [Performance] 高压运行 30 分钟，内存占用不超过系统 60%（约 8GB上限），严防内存泄漏或 OOM。
NFR-3: [Reliability] 软重启单个节点从拉起到输出首帧有效数据耗时 < 2 秒。
NFR-4: [Reliability] 视觉验证矛盾时，系统必须在 1 秒内果断抛出异常降级交回闭环运控。
NFR-5: [Reliability] 整个系统栈支持连续 2 小时以上不强制重启运行。
NFR-6: [Environmental] 完全局域单机模式配置，IPC 时延波动不得超过 5ms，免疫 Wi-Fi 影响。
NFR-7: [Environmental] 在强光或激光笔干扰下，伪阳性错认率低于 2%。
NFR-8: [Development] CMake 编译开启 -Wall -Wextra 必须实现 Zero Warnings。
NFR-9: [Development] 替换一种视觉检测算法类的平均代码修改面不超过 2 个文件。

### Additional Requirements

- Starter Template Requirements: 基于分离式的 ROS 2 工作空间骨架树，建立四个单一职责业务包(`dog_interfaces`, `dog_perception`, `dog_behavior`, `dog_lifecycle`)。
- 持久化方案：使用 `yaml-cpp` 向临时文件序列化任务状态进行掉电记忆。
- Action API 契约：行为序列指令使用 ROS 2 Action Client/Server。
- 并发防错及内存锁定约束：全面应用环形缓冲区防跳变；强制采用零拷贝机制；绝不允许使用 `std::this_thread::sleep_for()`。
- 竞赛安全策略：应对物理 E-Stop 具有对应的状态挂起或空转策略。
- Pimpl 惯用封装：隐藏第三方依赖。

### FR Coverage Map

FR-1.1: Epic 1 - 接收并解析图像流
FR-1.2: Epic 1 - 接收点云里程计
FR-1.3: Epic 1 - 计算 3D 位姿
FR-1.4: Epic 1 - 识别场地数字任务题
FR-2.1: Epic 1 - 验证对齐全局与运控坐标
FR-4.1: Epic 1 - 纯 CPU 环境编译运行
FR-4.3: Epic 1 - 外部 YAML 外参调配
FR-2.2: Epic 2 - 发布本体全局位姿
FR-2.3: Epic 2 - 发布目标 3D 位姿指令
FR-2.4: Epic 2 - 接收底层反馈
FR-2.5: Epic 2 - 触发状态转移
FR-3.2: Epic 3 - 逻辑层面重试熔断器
FR-3.3: Epic 3 - 优雅降级指令
FR-4.2: Epic 4 - 动态替换算法实现
FR-1.5: Epic 5 - 先验外推计算
FR-3.1: Epic 5 - 心跳自检报警及拉起死锁节点
FR-3.4: Epic 5 - 持久化缓存读写与断电恢复
FR-3.5: Epic 5 - 急停空转位置发布

## Epic List

### Epic 1: 赛场全景感知与基础对齐 (Core Perception & Initialization)
机器狗开机后能够看清赛场，理解 3D 空间，并建立所需的基础设施服务（包括持久化前置 IO）。
**FRs covered:** FR-1.1, FR-1.2, FR-1.3, FR-1.4, FR-2.1, FR-4.1, FR-4.3

### Epic 2: 目标调度与运控执行通信 (Task Scheduling & Execution API)
视觉规划层准确下发移动抓取指令，实时获取动作状态演进。
**FRs covered:** FR-2.2, FR-2.3, FR-2.4, FR-2.5

### Epic 3: 异常熔断与任务降级 (Circuit Breakers & Mission Degradation)
抓取连续失败时，能抛掉次优解并果断降级。
**FRs covered:** FR-3.2, FR-3.3

### Epic 4: 赛场敏捷调试与热重载架构 (Agile Debugging & Hot-Swap Architecture)
无损快速替换识别模型配置。
**FRs covered:** FR-4.2

### Epic 5: 系统生命周期与网络容错守护 (Lifecycle & Network Resilience)
面对网络断连、帧率丢失、掉电重启以及物理急停时，负责心跳守护、断电拉起、无帧外推及急停避险。
**FRs covered:** FR-1.5, FR-3.1, FR-3.4, FR-3.5

---

## Epic 1: 赛场全景感知与基础对齐 (Core Perception & Initialization)

### Story 1.1: 基础工程骨架与接口防腐层搭建
As a 视觉底层节点运维,
I want 一次性初始化四大包结构与通信介质,
So that 团队后续能基于脱离业务耦合的类型系统进行开发。

**Acceptance Criteria:**
- Given 纯黑的 ROS 2 骨架树，When 执行 colcon build，Then 生成包含 .msg/.srv/.action 的 `dog_interfaces`，且实现 0 Warnings 编译保障（NFR-8）。

### Story 1.2: 全局状态初始化与外参挂载对齐
As a 机器狗系统,
I want 在开机时从 YAML热加载硬件坐标树并发布静态 `tf2`,
So that 发生形变时无需重编 CPP 即可矫正地图。

**Acceptance Criteria:**
- Given `camera_extrinsics.yaml`，When 启动节点，Then 强制 init 期拉取，并通过 `tf2_ros` 广播正确的 TF（FR-2.1, FR-4.3）。

### Story 1.3: 相机雷达数据接入与零拷贝目标 3D 解算
As a 机器狗系统,
I want 通过同进程 IPC 混图与雷达点云进行 3D 坐标解算,
So that 在资源受限时安全运行，不 OOM。

**Acceptance Criteria:**
- Given 图像和点云流，When 对齐后解算，Then 在 < 50ms 内完成准确 PnP，合并占用内存不超过 60%(8GB)（NFR-1, NFR-2）。
- And 应用 `std::circular_buffer` 和 `unique_ptr` 零拷贝。

### Story 1.4: 场地数字内容识别与语义下发
As a 机器狗系统,
I want 能够识别截取场地的数字标识,
So that 可以依此规划路径偏好。

**Acceptance Criteria:**
- Given 感知图像中存在特定数字区，When 独立识别流执行，Then 发布带数字类型的消息流，耗时 < 50ms。
- And 在强光干扰下，识别框的伪阳错报率 < 2%（NFR-7）。

### Story 1.5: 持久化基础设施准备与 IO 工具链
As a 视觉底层节点运维,
I want 预建立基于 `yaml-cpp` 的文件流操作服务模块（供其它 Epic 调用）,
So that 异常状态和任务追踪可以有安全的介质存取。

**Acceptance Criteria:**
- Given 存储介质，When 任意节点调用持久化接口，Then 文件能够安全执行同步小包快速刷写（Flush）。

## Epic 2: 目标调度与运控执行通信 (Task Scheduling & Execution API)

### Story 2.1: 实时全局本体位姿发布
As a 外包运控系统,
I want 实时获取狗的地图级位姿以作逆运动学解算,
So that 可以指引机器狗正确运动至抓取区。

**Acceptance Criteria:**
- Given 感知里程计，When 狗体移动，Then 输出 `PoseStamped` 给运控，网络延时波动 < 5ms（NFR-6）。

### Story 2.2: 异步动作指令 Action 发送与监听
As a 机器狗系统,
I want 使用 Action Client 下发 3D 抓取位姿序列并监控反馈,
So that 可异步推动任务流。

**Acceptance Criteria:**
- Given 运控 Action Server，When 发送目标，Then 利用 `async_send_request` 无堵塞完成监听操作推进下一个状态机。

## Epic 3: 异常熔断与任务降级 (Circuit Breakers & Mission Degradation)

### Story 3.1: 异常重试拦截与优雅降级熔断网关
As a 领队,
I want 机器狗在屡次抓空时果断舍去死锁目标直接降级,
So that 保障不因为卡点吃罚分（FR-3.2, 3.3）。

**Acceptance Criteria:**
- Given 连续收到 2 次抓空，When 达到 YAML 定义阈值，Then 请求被熔断并抛退，系统必须在 < 1 秒内果断抛异常并切换至降级闭环状态（NFR-4）。

## Epic 4: 赛场敏捷调试与热重载架构 (Agile Debugging)

### Story 4.1: 感知算法动态工厂与热插拔注入
As a 视觉底层节点运维,
I want 纯工厂替换底层实现包而不用动节点主逻辑,
So that 比赛现场 1 钟头内极速调整应对环境。

**Acceptance Criteria:**
- Given 新检测器 `.cpp`，When 挂载于对应工厂子类，Then 重编译面积 < 2 个文件（NFR-9），节点生命周期架构无变更即可切换。

## Epic 5: 系统生命周期与网络容错守护 (Lifecycle & Network Resilience)

### Story 5.1: YAML 断电进度恢复读取与重启重建
As a 领队,
I want 系统在不预期掉电重启后能基于缓存找回当前赛场进度,
So that 省去由于断电必须重新发车的重创。

**Acceptance Criteria:**
- Given 持久化文件存在（Story 1.5 构建），When 系统从死机拉起首帧，Then 脱离该 YAML 重构上下文节点避免执行已完成的任务（FR-3.4），全栈拉起到准备完毕耗时低于 < 2s（NFR-3）。

### Story 5.2: 单侧丢帧先验外推补偿与急停空转
As a 机器狗系统,
I want 传感器突然失能时能以外推或空转应对,
So that 机器不会由于突兀的 0 输入彻底失控（FR-1.5, FR-3.5）。

**Acceptance Criteria:**
- Given 一方流断联 > 150ms，When 队列出现空档，Then 从 `circular_buffer` 取出先验前推输出；若接收物理急停，转为降低 CPU 计算量的纯纯空转位姿自发而不重置任务状态（确保能长考维持连续 2 小时不 OOM NFR-5）。

### Story 5.3: 节点假死心跳健康守护重联
As a 机器狗系统,
I want 一直侦听相机的出图率并在卡死时对其全命周期的剔除并重启,
So that 根治底层驱动挂机（FR-3.1）。

**Acceptance Criteria:**
- Given 死锁的传感器网络导致主节点 > 2s 无数据入站，When 心跳失败，Then 强制 Lifecycle 降为 Inactive 再回到 Active，且重连产出有效数据总耗时必须 < 2 秒（NFR-3）。
