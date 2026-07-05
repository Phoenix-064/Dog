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
- 核心节点包含 dog_perception_node、dog_behavior_bt_node。
- 默认不启动下位机串口执行桥；实机闭环运行时使用 `use_serial_bridge:=true`。
- 默认不启动目标点导航串口节点；实机导航闭环运行时使用 `use_nav_telemetry_serial:=true`。
- 默认 launch 不提供 `/behavior/execute`、`/behavior/place_boxes`、`/behavior/nav_execute` 的 Action Server；正式行为树需要启用对应串口节点，或由外部节点提供同名 Action Server。
- 同时尝试启动 livox_ros_driver2 与 point_lio；若第三方包未在当前 overlay 中可发现，会自动跳过，不阻塞核心节点。
- 导航/串口/PointLIO 联调可使用 `test_mode:=true`，该模式加载 `behavior_tree_nav_serial_test.xml`，不启动感知节点，不等待抓取/放置串口回包，导航串口仍真实发送 `RCNAV` 并等待 `RCArrivalMX`。
- 已完成 CH340 目标点导航串口实机收发验证：行为树测试程序可通过 `/behavior/nav_execute` 发送实际航点 `RCNAV` 帧，下位机测试程序可回传串口数据；测试固件若未返回 `RCArrivalMX`，上位机会按超时结束当前导航 goal。

### 3.3 分终端启动（用于调试）

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 终端 1
ros2 run dog_perception dog_perception_node

# 终端 2
ros2 run dog_behavior dog_behavior_bt_node
~~~

## 4. 启动参数速查

统一入口：ros2 launch dog_behavior launch.py

- test_mode（默认 false）：导航/串口/PointLIO 测试模式；启用后使用 `behavior_tree_nav_serial_test.xml`，跳过 YOLO 与抓取/放置串口执行
- use_livox（默认 true）：是否启动 livox_ros_driver2
- livox_model（默认 mid360，可选 mid360/hap）：选择 Livox 配置
- use_point_lio（默认 true）：是否启动 point_lio
- use_point_lio_rviz（默认 false）：是否让 point_lio 同时启动 RViz
- use_test_visualization（默认 false）：是否在 `test_mode=true` 时启动项目自带 RViz2 测试可视化；显示 Livox/PointLIO 点云、定位轨迹、测试路线、当前导航目标和导航状态
- use_perception_camera（默认 false）：是否启动 dog_perception_camera_node
- use_serial_bridge（默认 false）：是否启动 dog_serial_bridge_node，提供抓取/放置 Action Server
- serial_port（默认 /dev/ttyUSB0）：下位机行为执行串口
- serial_baud_rate（默认 115200）：下位机行为执行串口波特率
- serial_ack_timeout_ms（默认 1500）：下位机行为执行应答超时
- use_nav_telemetry_serial（默认 false）：是否启动目标点导航串口 Action Server
- nav_telemetry_serial_port（默认 /dev/ttyUSB1）：目标点导航串口
- nav_telemetry_baud_rate（默认 115200）：目标点导航串口波特率
- nav_telemetry_ack_timeout_ms（默认 10000）：等待 `RCArrivalMX` 到达回包的超时
- nav_telemetry_continuous_send_enabled（默认 true）：收到有效导航目标后是否持续发送最新定位与目标
- nav_telemetry_continuous_send_period_ms（默认 100）：持续发送 `RCNAV` 的周期，单位毫秒
- match_type（默认 left，可选 left/right）：比赛类型，决定加载哪组导航坐标文件

示例：

~~~bash
# 指定 HAP 配置
ros2 launch dog_behavior launch.py livox_model:=hap

# 启用 point_lio 自带 RViz
ros2 launch dog_behavior launch.py use_point_lio_rviz:=true

# 启用项目 test mode RViz2 可视化（推荐用于雷达/定位/导航联调）
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch dog_behavior launch.py test_mode:=true use_test_visualization:=true use_livox:=true livox_model:=mid360 use_point_lio:=true use_nav_telemetry_serial:=true nav_telemetry_serial_port:=/dev/ttyUSB0 nav_telemetry_ack_timeout_ms:=3000

# 同时启动相机节点
ros2 launch dog_behavior launch.py use_perception_camera:=true

# 仅启动核心包（关闭第三方）
ros2 launch dog_behavior launch.py use_livox:=false use_point_lio:=false

# 实机闭环运行（启动抓取/放置串口桥与目标点导航串口节点）
ros2 launch dog_behavior launch.py use_serial_bridge:=true serial_port:=/dev/ttyUSB0 use_nav_telemetry_serial:=true nav_telemetry_serial_port:=/dev/ttyUSB1

# 导航/目标点串口/PointLIO 联调（MID360 与 point_lio 正常启动，抓取/放置自动成功，不启动 YOLO；test_mode 会自动开启 dog_behavior_bt debug 定位日志）
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch dog_behavior launch.py test_mode:=true use_livox:=true livox_model:=mid360 use_point_lio:=true use_nav_telemetry_serial:=true nav_telemetry_serial_port:=/dev/ttyUSB0 nav_telemetry_ack_timeout_ms:=3000

# 仅验证目标点导航串口与行为树测试程序（不启动 Livox/PointLIO，由测试输入提供位姿时使用）
ros2 launch dog_behavior launch.py test_mode:=true use_livox:=false use_point_lio:=false use_nav_telemetry_serial:=true nav_telemetry_serial_port:=/dev/ttyUSB0 nav_telemetry_ack_timeout_ms:=3000

# 指定比赛类型（左侧/右侧）
ros2 launch dog_behavior launch.py match_type:=right
~~~

## 5. 运行后快速验收

~~~bash
# 核对核心包可见
ros2 pkg list | grep -E "dog_perception|dog_behavior|dog_interfaces|dog_serial_bridge"

# 核对关键 topic 是否存在
ros2 topic list | grep -E "/target/target_3d|/dog/global_pose"

# 启用 use_test_visualization 后核对 RViz2 可视化 topic
ros2 topic list | grep -E "/behavior/test_visualization/route|/behavior/test_visualization/markers|/behavior/nav_goal|/behavior/nav_exec_state"

# 实机闭环时核对抓取/放置/目标点导航 Action Server
ros2 action list | grep -E "/behavior/execute|/behavior/place_boxes|/behavior/nav_execute"

# 行为树启动后自动执行，无需手动触发。如需强制重触发：
ros2 topic pub --once /behavior/execute_trigger std_msgs/msg/String "{data: start}"
~~~

test mode 联调推荐按以下顺序执行：

~~~bash
# 终端 1：启动 test mode
source /opt/ros/humble/setup.bash
cd /home/ncu/wyr/Dog
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch dog_behavior launch.py test_mode:=true use_livox:=true livox_model:=mid360 use_point_lio:=true use_nav_telemetry_serial:=true nav_telemetry_serial_port:=/dev/ttyUSB0 nav_telemetry_ack_timeout_ms:=3000

# 可选：观察导航执行状态
ros2 topic echo /behavior/nav_exec_state std_msgs/msg/String --qos-reliability reliable --qos-durability volatile

# 可选：确认 RViz2 测试可视化输出
ros2 topic echo --once /behavior/test_visualization/route
ros2 topic echo --once /behavior/test_visualization/markers
~~~

行为树启动后自动开始执行（`auto_start=true`，默认）。`/behavior/nav_exec_state` 通常会先出现 `forwarding_goal`，随后进入 `running`；如果下位机返回 `RCArrivalMX`，行为树会继续推进到下一个航点。

`use_test_visualization:=true` 仅在 `test_mode:=true` 下生效，会额外启动 `dog_test_visualization_node` 与 RViz2，加载 `config/test_mode_navigation.rviz`。其中：

- 雷达/建图：显示 `/livox/lidar`、`/cloud_registered`、`/Laser_map`
- 定位：显示 `/aft_mapped_to_init`、`/path`、`/dog/global_pose`
- 导航：显示 `/behavior/nav_goal`、`/behavior/test_visualization/route`、`/behavior/test_visualization/markers`
- 若环境不能写默认 ROS 日志目录，先设置 `export ROS_LOG_DIR=/tmp/ros_logs`

说明：`BehaviorTreeNode` 内部支持 `auto_start` 参数，但当前统一 `launch.py` 没有声明或透传该参数；如需手动触发启动，需直接运行节点并设置参数，或先扩展 launch 参数。

目标点导航串口联调时，日志中出现以下信息即可判定上位机到下位机链路已经打通：

- `nav_telemetry_serial_ready port=/dev/ttyUSB...`：串口设备已打开。
- `behavior_name=auto_start`：行为树已自动启动。
- `nav_telemetry_serial_tx ... RCNAV;...goal_x=...;goal_y=...;goal_yaw=...`：实际航点目标已经通过串口发送；`goal_x/goal_y/goal_z` 单位为厘米，`goal_yaw` 单位为角度。
- `nav_telemetry_continuous_tx ... RCNAV;...cur_x=...;cur_y=...;goal_x=...;goal_y=...`：收到有效目标后，上位机正在按周期持续发送最新定位与目标。
- `nav_telemetry_serial_rx ...`：收到下位机串口回传。若下位机当前烧录测试程序，回传内容可能不是 `RCArrivalMX`，此时出现 `nav_serial_line_unmatched` 或最终 `arrival_timeout` 不代表串口通讯失败。

## 6. 项目结构与职责

~~~text
Dog/
├── src/
│   ├── dog_interfaces/   # msg/srv/action 统一契约
│   ├── dog_perception/   # 感知与目标结果输出
│   ├── dog_behavior/     # 行为执行与 Action 编排
│   └── dog_serial_bridge/# 下位机串口动作桥与导航遥测
├── 3rd_party/
│   ├── livox_ros_driver2/
│   └── point_lio_ros2/   # ROS 包名为 point_lio
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
  C --> H[dog_behavior]
  D --> H
  H --> I["/dog/global_pose"]
  H --> J["/behavior/nav_execute"]
  H --> K["/behavior/nav_exec_state"]
  H --> O["/behavior/test_visualization/*"]
  H --> L["/behavior/execute"]
  H --> M["/behavior/place_boxes"]
  J --> N[dog_serial_bridge]
  O --> P[RViz2]
  L --> N
  M --> N
~~~

## 8. 开发与测试

### 8.1 按包构建

~~~bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dog_interfaces dog_perception dog_behavior dog_serial_bridge
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
- docs/dog-perception-ai-reference.md
- docs/dog-behavior-ai-reference.md
- docs/dog-serial-bridge-ai-reference.md
- docs/dog-interfaces-ai-reference.md
- docs/navigation-replacement-proposal.md

## 11. 维护说明

- build、install、log 为生成目录，不应手工编辑。
- 3rd_party 下组件请参考各自项目文档进行独立配置。
