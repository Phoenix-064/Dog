# 机器狗视觉与定位系统文档索引

## 阅读入口

1. [README](../README.md)：运行、构建、测试、统一 launch 和端到端数据流入口。
2. [dog_interfaces AI 开发查询文档](dog-interfaces-ai-reference.md)：共享 `msg/srv/action` 契约。
3. [dog_perception AI 开发查询文档](dog-perception-ai-reference.md)：感知节点、图像/点云同步、目标输出和 QoS。
4. [dog_behavior AI 开发查询文档](dog-behavior-ai-reference.md)：统一 launch、行为树、BT 叶子和 Action 调用链。
5. [dog_serial_bridge AI 开发查询文档](dog-serial-bridge-ai-reference.md)：抓取/放置/目标点导航 Action Server 与 MCU 串口协议。
6. [2026-07-04 定位与导航异常诊断记录](navigation-localization-issue-2026-07-04.md)：实机测试中的坐标系、Point-LIO、航点 yaw 和串口回包问题证据。

## 当前运行契约

当前主线以 `ros2 launch dog_behavior launch.py` 为唯一入口。正式闭环需要外部或本仓库内串口节点提供 `/behavior/nav_execute`、`/behavior/execute`、`/behavior/place_boxes` Action Server；`test_mode:=true` 只绕过抓取/放置串口，目标点导航仍会真实发送 `RCNAV`。

## 待评审提案

1. [导航方案变更提案](navigation-replacement-proposal.md)：Heading-First 导航替换方案，当前不是运行契约。
