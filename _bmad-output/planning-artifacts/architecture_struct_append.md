## 项目结构与边界 (Project Structure & Boundaries)

### 完整的项目目录树结构 (Complete Project Directory Structure)

```text
dog_ws/
├── README.md
├── .gitignore
├── .github/
│   └── workflows/
│       └── ros2_cpp_ci.yml
├── docs/                                  # 架构与开发文档存放区
├── src/
│   ├── build_flags.cmake                  # 全局通用的编译 Flag 与严格的 Warnings 规约
│   │
│   ├── dog_interfaces/                    # 【核心契约边界】只包含 msg, srv, action 定义
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── msg/
│   │   │   ├── BoundingBox3D.msg          # 视觉系统输出给规划或运控的标准化 3D 框
│   │   │   └── TaskState.msg              # 任务与硬件状态流转反馈体
│   │   └── srv/
│   │       └── GraspCommand.srv           # 下发抓取指令与同步获取结果
│   │
│   ├── dog_perception/                    # 【视觉与点云基建包】负责图像与雷达点云处理解算
│   │   ├── CMakeLists.txt                 # 定义多个 add_library 和少量 Node 可执行文件
│   │   ├── package.xml
│   │   ├── config/
│   │   │   ├── camera_extrinsics.yaml     # 【约束落实】不写死在CPP中的外参查表
│   │   │   └── detection_params.yaml
│   │   ├── launch/
│   │   │   └── perception_bringup.launch.py 
│   │   ├── include/dog_perception/
│   │   │   ├── detector_factory.hpp       # 【需求落实】实现算法热插拔抽象接口隔离
│   │   │   ├── detector_base.hpp          
│   │   │   └── box_pnp_solver_node.hpp    # 纯界面声明
│   │   └── src/
│   │       ├── detectors/
│   │       │   ├── color_detector.cpp     # 具体的算法实现
│   │       │   └── neural_detector.cpp    # 未来供 OpenVINO 等挂载的实现
│   │       └── box_pnp_solver_node.cpp    # Node的具体业务逻辑实体
│   │
│   ├── dog_behavior/                      # 【规划与状态机包】负责动作序列调度与熔断
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   │   └── behavior_trees.yaml        # 定义重试熔断阈值等行为树参数
│   │   ├── include/dog_behavior/
│   │   │   └── state_manager_node.hpp     # 【需求落实】监控与空转恢复的系统守护者
│   │   └── src/
│   │       ├── state_manager_node.cpp     
│   │       └── test_mock_driver.cpp       # 提供给算法同学测试用的极简空跑假运控
│   │
│   └── 3rd_party/                         # 存放无需修改的第三方驱动库
│       ├── livox_ros_driver2/             # Livox 前端驱动
│       └── point_lio/                     # 激光里程计
```

### 架构通信与内存边界 (Architectural Boundaries)

**API 契约边界 (API Boundaries):**
- **强类型隔离层**：`dog_interfaces` 包是唯一包含自定义 `msg` 和 `srv` 的无逻辑库。无论是 `dog_perception` 还是将来的闭源运控系统对接侧，都只允许强依赖此类库包。**严禁在感知或决策包内定义独立 msg 导致循环依赖。**
- **薄客户端调度**：向运控发送数据时，无论是 `GraspCommand` 还是 `/localization/boxes`，都不带有复杂的感知上下文。一旦将 `BoundingBox3D` 通过话题发出后，该包的空间所有权和生命周期便完全剥离出感知层，防止多模块抢占。

**组件与依赖边界 (Component & Inclusion Boundaries):**
- **Pimpl 库级截断**：`dog_perception` 子目录 `src/detectors/` 中的算法内部哪怕使用了庞大的 OpenCV 或 PCL 头文件，绝不允许将这些重型依赖暴露在 `include/dog_perception/detector_base.hpp` 接口头文件内，必须按规则锁死在目标实现文件中。
- **热插拔边界**：`detector_factory` 被建立为一个动态分配点。在运行 `perception_bringup` 时，通过 launch 传入的参数（或内部重启）即可直接动态载入不同的算法 `.cpp` 编译库体，从而支撑起赛场上只需 1~2 分钟的热替换。

### 数据流集成点 (Integration Points & Data Flow)

**内部传感器流 (Internal Sensor Flow):**
1. **源头**：USB 摄像头或 `livox_ros_driver2` 产生 `sensor_msgs::msg::Image` 与 `sensor_msgs::msg::PointCloud2`。
2. **零拷贝过滤**：进入 `dog_perception`，如果同属一个物理内存池上的 IPC 节点群，通过 `unique_ptr` 投递与 `ConstSharedPtr` 接收进行特征提取。通过时间同步缓冲（`message_filters::Subscriber`）软对齐雷达位姿与图像获取目标 3D 位姿。

**异常与状态熔断流 (Lifecycle & Breaker Flow):**
1. `dog_behavior` 的 `state_manager_node` 挂载全局监听器。
2. 当下发到运控的指令返回超时，或连续两次视觉位姿验证发生跳变过大（硬件致盲/假反馈），`state_manager` 直接切换底层感知节点至“Idle Spinning（空转态）”，切断重载图像通讯，直接返回并调度备用任务路线。
