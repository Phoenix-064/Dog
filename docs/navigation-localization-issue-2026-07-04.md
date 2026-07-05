# 2026-07-04 定位与导航异常诊断记录

> 状态：问题记录与排查入口
> 适用范围：`test_mode:=true use_livox:=true livox_model:=mid360 use_point_lio:=true use_nav_telemetry_serial:=true`

## 1. 现象概述

实机测试中，现场观察为机器狗主要沿 `+Y` 方向向前行走约 2 m；但上位机日志中的 Point-LIO/串口当前位置显示，结束前估计位置约为 `x=-2.826 m, y=1.918 m, z=0.170 m, yaw=-2.423 rad`。如果现场观察准确，则定位或坐标解释存在米级横向偏差。

本记录不把“Point-LIO 算法精度差”作为已确认根因。当前证据更直接指向：坐标系混用、航点 yaw 单位风险、Point-LIO 数据流后段中断、上层缺少定位质量处理、串口回包异常。

## 2. 数据证据

### 2.1 首个导航目标

行为树加载左侧航点后，首个目标来自 `waypoint_goal_1`：

```text
x=0.0
y=1.0
z=0.0
yaw=0
```

串口首帧对应目标：

```text
RCNAV;seq=1;stamp_ms=1783170428618;
cur_valid=1;cur_frame=camera_init;
cur_x=-0.441;cur_y=0.455;cur_z=0.414;cur_yaw=-2.241;
goal_valid=1;goal_frame=map;
goal_x=0.000;goal_y=100.000;goal_z=0.000;goal_yaw=0.000
```

说明：串口帧中的位置单位是厘米，`goal_y=100.000` 对应 ROS 内部 `y=1.0 m`。

### 2.2 结束前定位估计

Ctrl-C 退出时，导航串口节点发送 shutdown 到达帧：

```text
RCNAV;seq=2;stamp_ms=1783170466131;event=shutdown_arrived;
cur_valid=1;cur_frame=camera_init;
cur_x=-282.565;cur_y=191.783;cur_z=17.043;cur_yaw=-2.423;
goal_valid=1;goal_frame=camera_init;
goal_x=-282.565;goal_y=191.783;goal_z=17.043;goal_yaw=-2.423
```

换算成米约为：

```text
x=-2.826 m
y= 1.918 m
z= 0.170 m
yaw=-2.423 rad
```

该结果与现场“只沿 `+Y` 方向约 2 m”的观察不一致，尤其是 `x=-2.826 m` 横向偏移。

### 2.3 Point-LIO 数据流后段中断

日志后段持续输出：

```text
point_lio_wait_sync_empty lidar_queue=0 imu_queue=12 last_imu=1783170457.671078
```

从 `1783170457.671078` 后，`last_imu` 不再更新，`lidar_queue=0`，说明 Point-LIO 后半段没有继续获得可同步的 LiDAR/IMU 数据。该问题能解释后段定位停止更新，但不能单独解释此前已经形成的米级横向偏移。

### 2.4 串口接收内容

本次日志中串口接收仅有以下无效内容：

```text
nav_telemetry_serial_rx port=/dev/ttyUSB0 data=
nav_serial_line_unmatched line=

nav_telemetry_serial_rx port=/dev/ttyUSB0 data=�
nav_serial_line_unmatched line=�

nav_telemetry_serial_rx port=/dev/ttyUSB0 data=B�
nav_serial_line_unmatched line=B�
```

未收到有效 `RCArrivalMX`。因此 `/behavior/nav_execute` 没有形成正常到达确认闭环。

## 3. 代码证据与可能问题

### 3.1 坐标系混用风险

已确认事实：

- `dog_behavior` 订阅 `/aft_mapped_to_init`，把 Point-LIO odom 的 pose 基本原样发布为 `/dog/global_pose`。代码入口：`src/dog_behavior/src/behavior_tree_node.cpp` 的 `odomCallback()`。
- `NavTelemetrySerialNode` 订阅 `/dog/global_pose`，构造 `RCNAV` 帧时直接使用当前位置数值。代码入口：`src/dog_serial_bridge/src/nav_telemetry_serial_node.cpp` 的 `currentPoseCallback()`、`executeGoal()`、`buildFrame()`。
- 本次首帧中 `cur_frame=camera_init`，`goal_frame=map`。
- launch 默认发布 `map -> camera_init` 静态 TF，默认平移和旋转均为 0。代码入口：`src/dog_behavior/launch/launch.py` 的 `static_transform_publisher`。

可能问题：

- 当前串口发送链路没有在 `map` 和 `camera_init` 之间做 TF 数值变换。
- 只有当 `map` 与 `camera_init` 在实机启动时完全重合，且朝向完全一致时，当前近似才成立。
- 如果 LiDAR 初始化坐标系与比赛场地坐标系存在初始 yaw 或平移偏差，MCU 收到的 `cur_*` 和 `goal_*` 会处在不同坐标解释下，可能造成横向误差或错误修正。

建议排查：

- 记录启动时 `map -> camera_init` 的真实外参或取消 `map`/`camera_init` 混用。
- 在发送 `RCNAV` 前统一坐标系：要么把目标变换到 `camera_init`，要么把当前位置变换到 `map`。
- 实测静止状态下 `/aft_mapped_to_init`、`/dog/global_pose`、`/behavior/nav_goal` 的 frame 与数值关系。

### 3.2 航点 yaw 单位风险（已按角度制处理）

已确认事实：

- `waypoints_left.yaml` 中存在 `yaw: 90`、`yaw: 180`、`yaw: -90`、`yaw: -150.11` 等值。
- `dog_behavior` 现在将航点 `yaw` 作为角度读取，并在构造 ROS 四元数前转换为弧度。
- `waypointToPose()` 仍使用弧度调用 `sin()` / `cos()`，但输入来自 `yaw_deg -> rad` 显式转换。
- 串口协议文档现在规定 `cur_yaw` / `goal_yaw` 单位为角度。

可能问题：

- 左侧航点文件的 yaw 按角度填写；该语义已成为当前运行契约。
- 本次首个目标 `yaw=0`，因此它不是首段 2 m 前进偏差的直接解释；但后续航点会受到影响。
- `waypoints_right.yaml` 已从弧度风格数值转换为角度风格数值，避免左右航点文件单位不一致。

建议排查：

- 当前航点配置、launch 用户参数和 `RCNAV` 串口字段的 yaw 单位统一为角度。
- ROS `PoseStamped` 四元数、TF 内部参数和三角函数调用仍保持弧度语义，并在边界处转换。
- 已增加/更新测试覆盖 `90 deg -> 1.5708 rad` 四元数转换，以及串口 `cur_yaw/goal_yaw` 角度输出。

### 3.3 Point-LIO 与 LiDAR/IMU 同步数据问题

已确认事实：

- Point-LIO MID360 配置使用 `/livox/lidar` 和 `/livox/imu`。
- 本次日志前段正常出现 `point_lio_sync`、`point_lio_undistort_ready`。
- 后段持续出现 `point_lio_wait_sync_empty lidar_queue=0 imu_queue=12 last_imu=1783170457.671078`。

可能问题：

- LiDAR 数据流在后段中断或驱动停止发布。
- IMU 时间戳/队列未继续推进，导致 Point-LIO 无法同步新包。
- Livox 驱动或网络链路在运动过程中出现短时断流。
- 该问题会导致 `/aft_mapped_to_init` 后段不再更新，上层仍可能继续使用最后一次缓存位姿。

建议排查：

- 实机复测时同时记录 `/livox/lidar`、`/livox/imu`、`/aft_mapped_to_init` 的频率和时间戳。
- 在断流时查看 Livox 驱动日志是否仍持续发布 PointCloud2/IMU。
- 检查 MID360 供电、网线、IP 配置和网络丢包。
- 记录 rosbag，用离线方式验证 Point-LIO 输入是否完整。

### 3.4 上层未对 Point-LIO 输出做质量处理

已确认事实：

- `dog_behavior` 对 odom 只做 frame 兜底、有限值检查、四元数范数检查，然后发布 `/dog/global_pose`。
- `NavTelemetrySerialNode` 缓存 `/dog/global_pose` 时同样只做有限值和四元数范数检查。
- 当前链路没有检查位姿时间戳新鲜度、速度/位置跳变、协方差、frame 是否匹配目标、Point-LIO 是否断流。

可能问题：

- 当 Point-LIO 断流时，串口节点可能继续使用最后一次缓存位姿。
- 当 Point-LIO 出现瞬时跳变或坐标系不一致时，上层没有拒绝或降级。
- MCU 收到的 `cur_*` 可信度无法判断，定位异常会直接进入底盘导航闭环。

建议排查：

- 在 `/dog/global_pose` 到串口发送之间加入 freshness 检查，例如超过阈值不发送导航目标。
- 增加 frame 一致性检查：`cur_frame` 与 `goal_frame` 不一致时必须变换或拒绝。
- 增加跳变检测和日志，例如单位时间内位移/角速度超过阈值时报警。
- 如果 Point-LIO 提供协方差或质量指标，接入导航门控。

### 3.5 串口接收异常

已确认事实：

- 上位机已成功写出 `RCNAV`。
- 串口 RX 只收到空行和乱码，没有有效 `RCArrivalMX`。
- `waitForArrival()` 只接受完整字符串 `RCArrivalMX`，其他行会记录 `nav_serial_line_unmatched`。

可能问题：

- MCU 端未运行正式导航回包协议，仍是测试固件或其他输出格式。
- 波特率、串口线、接地、编码、换行符或半双工收发存在问题。
- 如果 MCU 未回包，上位机 Action 无法成功结束，行为树会停留在等待或超时路径。

建议排查：

- 使用串口工具单独验证 `/dev/ttyUSB0` 在 115200 下的原始 RX/TX。
- 确认 MCU 固件收到 `RCNAV` 后会回 `RCArrivalMX`，且带正确换行。
- 增加原始字节十六进制日志，避免乱码无法判断具体字节。
- 若当前确实使用测试固件，应在测试记录中明确“串口回包异常不代表定位异常”。

## 4. 推荐定位顺序

1. 先验证坐标系：确认 `map` 与 `camera_init` 是否真的重合；若不重合，先修正发送 `RCNAV` 前的坐标变换。
2. 航点 yaw 单位已统一为角度制；后续复测需确认 MCU 固件也按角度解析 `RCNAV` 的 `cur_yaw/goal_yaw`。
3. 同步记录 `/livox/lidar`、`/livox/imu`、`/aft_mapped_to_init`，定位 Point-LIO 后段断流原因。
4. 给 `/dog/global_pose -> RCNAV` 增加质量门控，避免 stale pose 或跨 frame pose 直接发给 MCU。
5. 单独验证串口协议，确认 MCU 能按约定回 `RCArrivalMX`。

## 5. 仍需补充的数据

- 本次实机的 rosbag，至少包含 `/livox/lidar`、`/livox/imu`、`/aft_mapped_to_init`、`/dog/global_pose`、`/behavior/nav_goal`。
- 机器狗真实轨迹的外部测量数据，例如地面标尺、视频标注、运动捕捉或人工记录。
- 启动时机器狗朝向与场地 `map` 坐标系的关系。
- MCU 固件版本和导航回包协议说明。
