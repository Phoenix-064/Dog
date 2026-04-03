# 集成架构 (Integration Architecture)

本项目包含 4 个基于 ROS 2 Humble 的通讯节点模块：

1. **Detect** (视觉目标检测)
2. **livox_ros_driver2** (Livox 激光雷达驱动)
3. **Localization** (定位模块)
4. **point_lio** (基于点云的里程计与定位)

## 通信协议
本项目的主要部件运行在一个 ROS 2 Humble 运行时网络中，遵循发布-订阅 (Publish-Subscribe) 模型。

### 数据流向
1. **livox_ros_driver2** 作为硬件接口部分，通过网络从激光雷达采集点云数据，并以 ROS Topic (`/livox/lidar` 等) 的格式发布出去。
2. **point_lio** 或 **Localization** 订阅点云主题结合特征提取给出高精度的里程计算法与定位姿态。
3. **Detect** 会订阅相机和（或）雷达融合后的数据来进行目标分类与检测，并将结果发送出去。
4. 所有最终的定位姿态和相对目标状态，通过 ROS 2 发送到**运动控制项目模块**，进行四足机器狗的联合控制。

## 集成接口详情
- **类型**: ROS Topic 通信。
- **频次**: 典型为 10Hz - 100Hz 之间以适用实时要求。
- **协议**: 自定义 ROS Message。

