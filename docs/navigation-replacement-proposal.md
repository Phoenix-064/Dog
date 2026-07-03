# 导航方案变更提案

> 创建日期：2026-07-03
> 状态：待评审

> 注意：本文是未落地提案，不属于当前运行契约。当前代码仍使用 `NavigateWaypointAction -> /behavior/nav_execute -> NavTelemetrySerialNode -> RCNAV -> MCU`。

## 1. 背景

### 1.1 Nav2 弃用原因

项目最初尝试使用 Nav2 作为导航方案，但存在两个根本矛盾导致被废弃：

1. **MCU 的速度解复用问题**：Nav2 的局部规划器（DWB/TEB）核心机制是向机器人底盘持续下发 `geometry_msgs/Twist` 速度指令，依赖机器人精确执行速度大小来闭环控制。但项目 MCU 只取 `Twist` 的方向分量（前进/后退/左转/右转），完全忽略速度幅值，导致 Nav2 的 MPC/代价地图等反馈优化全部失效。

2. **架构冗余**：Nav2 需要在 `dog_behavior` 和电机控制之间插入一个完整的导航栈（Global Planner + Costmap + Controller），而项目的运动控制由 MCU 黑盒完成，Nav2 无法穿透到步态/足端力控层面，形成"两层规划"的尴尬架构。

因此 Nav2 在 Phase 0-4 重构中被完全移除，当前的实际导航由 `NavTelemetrySerialNode` 通过 `RCNAV` 串口协议与 MCU 直接交互完成。

### 1.2 当前串口导航的遗留问题

**协议**：ROS 2 侧通过 `/behavior/nav_execute` Action 接收目标航点，构造 `RCNAV` 串口帧（包含 `cur_x/y/yaw` 和 `goal_x/y/yaw` 的绝对坐标），发送给 MCU。MCU 自行规划路径到达目标点后返回 `RCArrivalMX`。

**痛点**：Point-LIO 提供的定位（填充 `cur_*` 字段）存在漂移误差。当机器狗应该正对目标直行时，定位误报的侧向偏移会让 MCU 认为需要先横向修正。由于步长大于修正距离，机器狗会在目标附近反复转弯微调，造成：

- 大量时间浪费在非必要转弯上
- 到达判定不稳定

**根因**：绝对坐标匹配方案将定位误差直接注入 MCU 的路径规划，而机器狗的步行特性导致小偏差反而更难修正。

## 2. 方案：Heading-First 导航

### 2.1 核心思路

将导航解耦为"先对齐方向，再走直线"两个阶段，避免绝对坐标误差干扰。

```
传统 RCNAV:       MCU 收到 (cur, goal) → cur 有误差 → 修正 → 费力

Heading-First:    ROS 2 做对齐判断 → 先让 MCU 转向到位 → 再给 MCU 一个大致准的差 → 直行到达
```

### 2.2 算法流程

```
gotoPose(target):
  loop:
    heading_error = atan2(dy, dx) - current_yaw
    distance = sqrt(dx² + dy²)

    if distance < 5cm AND |heading_error| < 3°:
      return SUCCESS   // 到达

    if |heading_error| > 5°:
      // --- Phase 1: 原地转向 ---
      方案A (MCU 支持 goal==cur 的原地转向):
        RCNAV(cur=(x,y,yaw), goal=(x,y, target_heading_yaw))
      方案B (MCU 需要 cur≠goal):
        virtual_goal = cur + 0.10m * heading_vector
        RCNAV(cur, virtual_goal)   // 诱导转向

    else:
      // --- Phase 2: 直行至目标 ---
      RCNAV(cur, target)

    await RCArrivalMX or per_step_timeout
```

### 2.3 为什么能解决当前问题

- **Phase 1** 确保狗先转正方向 → Phase 2 的 goal 差向量几乎就是纯纵向 → MCU 不需要侧向修正
- 转向阶段即使定位有小偏差也不影响：用相对航向误差驱动，而非绝对坐标差
- 到达判定同时检查 `/aft_mapped_to_init` 里程计的实时距离，防止串口丢包死等

### 2.4 架构变更

```
变更前:
  BT(NavigateWaypointAction) → /behavior/nav_execute → NavTelemetrySerialNode → RCNAV → MCU
                                        ↑ 依赖 /dog/global_pose

变更后:
  BT(NavigateWaypointAction) → /dog_nav/goto_pose → HeadingFirstNavigator (新节点)
                                       ↑ 订阅 /dog/global_pose (Point-LIO)
                                       ↓ 调用 /behavior/nav_execute → NavTelemetrySerialNode → RCNAV → MCU
                                       发布 /dog_nav/state (状态反馈)
```

**兼容性**：`NavigateWaypointAction` BT 叶子节点代码不需要修改，只需在 launch 时将 `action_name` 参数从 `/behavior/nav_execute` 改为 `/dog_nav/goto_pose`。`NavTelemetrySerialNode` 保持不变，`HeadingFirstNavigator` 作为它的 Action Client。

## 3. 实现计划

### 3.1 新建包：`dog_navigation`

```
src/dog_navigation/
├── CMakeLists.txt
├── package.xml
├── include/dog_navigation/
│   └── heading_first_navigator.hpp
├── src/
│   ├── heading_first_navigator.cpp
│   └── navigator_main.cpp
├── config/
│   └── navigator_params.yaml
└── test/
    └── test_heading_first_navigator.cpp
```

### 3.2 接口设计

| 接口 | 类型 | 说明 |
|------|------|------|
| `/dog_nav/goto_pose` | Action (`NavigateWaypoint`) | 航点导航（替代 Nav2 `/navigate_to_pose`） |
| `/dog_nav/align_heading` | Action (`NavigateWaypoint`) | 仅原地转向（可选） |
| 订阅 `/dog/global_pose` | Topic (`PoseStamped`) | Point-LIO 位姿 |
| Client `/behavior/nav_execute` | Action | 复用现有 NSB 串口 |
| 发布 `/dog_nav/state` | Topic (`String`) | 导航状态观察 |

### 3.3 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arrival_distance_threshold_m` | 0.05 | 到达判定距离阈值（米） |
| `arrival_yaw_threshold_deg` | 3.0 | 到达判定航向阈值（度） |
| `align_yaw_threshold_deg` | 5.0 | 触发 Phase 1 转向的航向阈值 |
| `use_inplace_turn` | false | false=方案B（诱导转向），true=方案A（原地转向） |
| `induce_step_meters` | 0.10 | 方案B 诱导步长（米） |
| `per_step_timeout_ms` | 8000 | 单次 RCNAV 超时（毫秒） |
| `cycle_period_ms` | 100 | 主循环周期 |

### 3.4 预估工作量

| 步骤 | 内容 | 预估时间 |
|------|------|----------|
| 1 | 搭建 `dog_navigation` 包骨架 | 0.5h |
| 2 | 实现 `HeadingFirstNavigator` 核心逻辑 | 2h |
| 3 | 编写 gtest 单元测试 | 1h |
| 4 | 修改 launch.py / behavior_tree XML 集成 | 0.5h |
| 5 | 实机测试确认 MCU 行为方案 A vs B | 0.5h |
| 6 | 实机联调 | 1h |
| **合计** | | **5.5h** |

---

## 附录：相关问题

### A.1 MCU 原地转向行为（需实测确认）

若 `goal_x/goal_y == cur_x/cur_y` 但 `goal_yaw ≠ cur_yaw` 时 MCU 的行为决定方案 A/B 的选择：

- **方案A 可行**：MCU 原地转向到 `goal_yaw`，不产生平移
- **方案A 不可行**：MCU 忽略或直接判到达，需回退到方案B（诱导步方案）

两种方案的节点代码逻辑一致，仅 `use_inplace_turn` 参数值不同。
