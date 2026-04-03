# 架构与设计 (Architecture Design)

## 技术栈
- 操作系统：Ubuntu 22.04
- 语言：C++
- 通讯框架：ROS 2 Humble
- 依赖：Livox SDK, CMake

## 架构模式
分布式节点架构 (Publish-Subscribe) - 所有传感器采集到的点云通过 `livox_ros_driver2` 发布，由 `point_lio` 及 `Localization` 输出 Odom/TF 数据；相机数据被 `Detect` 模块处理以标识物体，各结果最终由 ROS 发布供运动模块使用。
