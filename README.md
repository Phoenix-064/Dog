# Dog 机器狗视觉与定位系统

面向 ROS 2 Humble 的机器狗感知-生命周期-行为协同工程。项目将相机与激光雷达输入转换为目标位姿、系统模式和行为执行信号，并与运动控制模块协作。

## 1. 文档目标

本 README 主要服务三类读者：
- 首次接手项目的开发者：快速完成依赖安装、构建和启动。
- 日常开发者：按包增量构建、测试和联调。
- 集成与运维同学：定位常见构建/测试/时序问题。

## 2. 环境与依赖

- 操作系统：Ubuntu 22.04
- 中间件：ROS 2 Humble
- 语言：C++
- 构建系统：ament_cmake + colcon

常见缺失依赖：vision_msgs

~~~bash
sudo apt update
sudo apt install -y ros-humble-vision-msgs
~~~

## 3. Quick Start

### 3.1 构建工作区

~~~bash
source /opt/ros/humble/setup.bash
cd /home/ncu/wyr/Dog
colcon build
source install/setup.bash
~~~

### 3.2 一键启动（推荐）

~~~bash
source /opt/ros/humble/setup.bash
cd /home/ncu/wyr/Dog
source install/setup.bash
ros2 launch dog_behavior launch.py
~~~

说明：
- 启动入口位于 src/dog_behavior/launch/launch.py。
- 核心节点包含 dog_perception_node、dog_lifecycle_node、dog_behavior_bt_node。
- 同时尝试启动 livox_ros_driver2 与 point_lio；若第三方包未在当前 overlay 中可发现，会自动跳过，不阻塞核心节点。

### 3.3 分终端启动（用于调试）

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 终端 1
ros2 run dog_perception dog_perception_node

# 终端 2
ros2 run dog_lifecycle dog_lifecycle_node

# 终端 3
ros2 run dog_behavior dog_behavior_bt_node
~~~

## 4. 启动参数速查

统一入口：ros2 launch dog_behavior launch.py

- use_livox（默认 true）：是否启动 livox_ros_driver2
- livox_model（默认 mid360，可选 mid360/hap）：选择 Livox 配置
- use_point_lio（默认 true）：是否启动 point_lio
- use_point_lio_rviz（默认 false）：是否让 point_lio 同时启动 RViz
- use_perception_camera（默认 false）：是否启动 dog_perception_camera_node
- match_type（默认 left，可选 left/right）：比赛类型，决定加载哪组导航坐标文件

示例：

~~~bash
# 指定 HAP 配置
ros2 launch dog_behavior launch.py livox_model:=hap

# 启用 point_lio 自带 RViz
ros2 launch dog_behavior launch.py use_point_lio_rviz:=true

# 同时启动相机节点
ros2 launch dog_behavior launch.py use_perception_camera:=true

# 仅启动核心包（关闭第三方）
ros2 launch dog_behavior launch.py use_livox:=false use_point_lio:=false

# 指定比赛类型（左侧/右侧）
ros2 launch dog_behavior launch.py match_type:=right
~~~

## 5. 运行后快速验收

~~~bash
# 核对核心包可见
ros2 pkg list | grep -E "dog_perception|dog_lifecycle|dog_behavior|dog_interfaces"

# 核对关键 topic 是否存在
ros2 topic list | grep -E "/target/target_3d|/lifecycle/system_mode|/dog/global_pose"
~~~

## 6. 项目结构与职责

~~~text
Dog/
├── src/
│   ├── dog_interfaces/   # msg/srv/action 统一契约
│   ├── dog_perception/   # 感知与目标结果输出
│   ├── dog_lifecycle/    # 健康监控、降级与系统模式
│   └── dog_behavior/     # 行为执行与 Action 编排
├── 3rd_party/
│   ├── livox_ros_driver2/
│   └── point_lio/
├── docs/
├── tools/
└── build/ install/ log/  # 本地构建产物
~~~

## 7. 端到端数据流

~~~mermaid
flowchart LR
  A[Camera / Livox] --> B[dog_perception]
  B --> C["/target/target_3d"]
  B --> D["/target/digit_result"]
  C --> E[dog_lifecycle]
  E --> F["/lifecycle/system_mode"]
  E --> G["/lifecycle/health_alarm"]
  F --> H[dog_behavior]
  H --> I["/dog/global_pose"]
  H --> J["/navigate_to_pose"]
  H --> K["/behavior/nav_exec_state"]
~~~

## 8. 开发与测试

### 8.1 按包构建

~~~bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dog_interfaces dog_perception dog_lifecycle dog_behavior
source install/setup.bash
~~~

### 8.2 按包测试

~~~bash
source /opt/ros/humble/setup.bash
colcon test --packages-select dog_perception
colcon test-result --all --verbose
~~~

### 8.3 全量测试

~~~bash
source /opt/ros/humble/setup.bash
colcon test
colcon test-result --all --verbose
~~~

注意：新增或重命名 gtest 后，先对对应包执行一次 colcon build，再做按名称过滤测试。

## 9. 常见问题与排障

1. 工作区迁移后 colcon build 报 source path mismatch
处理方法：清理受影响包的 build/包名 与 install/包名 后重建。

2. dog_perception 构建时报 vision_msgs 缺失
处理方法：sudo apt install -y ros-humble-vision-msgs

3. 执行 colcon test --ctest-args -R 用例名 提示未找到测试
处理方法：先执行 colcon build --packages-select 对应包，刷新测试注册。

4. 心跳重连长期 pending
处理建议：确保 reconnect_pending_timeout_ms 小于 restart_window_ms，避免重试计数难以累积。

## 10. 相关文档

- docs/index.md
- docs/project-overview.md
- docs/architecture.md
- docs/integration-architecture.md
- docs/interface-architecture.md
- docs/development-instructions.md

## 11. BMAD 产物目录

- _bmad-output/planning-artifacts/
- _bmad-output/implementation-artifacts/

## 12. 维护说明

- build、install、log 为生成目录，不应手工编辑。
- 3rd_party 下组件请参考各自项目文档进行独立配置。
