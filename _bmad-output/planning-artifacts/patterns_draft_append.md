#### 增强 1：时间同步与传感器融合模式 (Time Synchronization & Fusion Patterns)
- **强制要求机制同源**：当涉及必须聚合多个非同频 Topic（如 Image 与 Lidar，或者 Odometry）以进行物理测算或状态预估时，严禁使用非确定性的“拿到最新消息便缓存”方式来进行关联。必须使用 ROS 2 官方的 `message_filters::Subscriber` 与 `TimeSynchronizer`（或 `ApproximateTimeSynchronizer`）来将回调进行强制的时间硬对齐聚合，这是高容错感知的基石。

#### 增强 2：隐藏复杂依赖与接口极简模式 (Pimpl Idiom & Include Hiding)
- **Pimpl 惯用法 (Pointer to Implementation)**：如果一个核心 `.hpp` 类的底层逻辑极其依赖大型未解耦的第三方库（例如 PCL 或 CUDA 推理 API），强烈推荐使用 Pimpl 模式（即在 `.hpp` 内仅保留前置声明，依赖库的全量 `#include` 仅写在独立的 `.cpp` 内）。这杜绝了第三方重型依赖往整个项目拓扑树上无限蔓延。

#### 增强 3：状态机与生命周期模式 (Lifecycle Node Patterns)
- **推行 LifecycleNode (基于业务需求)**：对于那些独占极高功耗外设（如连续扫描的 Lidar 或长开相机的推理模型），不允许继承普通的 `rclcpp::Node`。所有此类硬件的代理必须继承 `rclcpp_lifecycle::LifecycleNode`。只有当收到系统的显式指令转换到 `active` 状态时，才允许订阅或发布内存；在 `inactive` 状态时，底层必须释放所有多媒体管道的指针。
