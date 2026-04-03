# 项目概览 (Project Overview)

## 目的
本项目为机器狗的视觉识别、定位及行为模块，基于 Ubuntu 22.04 和 ROS 2 Humble (C++) 开发，主要运行在 NUC 上，将处理后的数据传递给独立外包的运动控制系统。

## 库结构
四部分构成的 Monorepo：
- **Detect**: 视觉目标检测
- **livox_ros_driver2**: 激光雷达驱动
- **Localization**: 定位模块
- **point_lio**: 里程计与定位

## 架构概览
各模块作为 ROS 节点，通过主题进行解耦的数据采集计算和通讯发送。
