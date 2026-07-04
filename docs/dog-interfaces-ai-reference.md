# dog_interfaces AI 开发查询文档

本文档描述 `src/dog_interfaces` 的共享 ROS 接口契约。该包不包含运行节点，接口变更会影响 `dog_perception`、`dog_lifecycle`、`dog_behavior` 与 `dog_serial_bridge`，修改后必须优先构建 `dog_interfaces`。

## 1. 包定位

包路径：[src/dog_interfaces](../src/dog_interfaces)

构建入口：

1. [src/dog_interfaces/CMakeLists.txt](../src/dog_interfaces/CMakeLists.txt)
2. [src/dog_interfaces/package.xml](../src/dog_interfaces/package.xml)

接口目录：

1. [src/dog_interfaces/msg](../src/dog_interfaces/msg)
2. [src/dog_interfaces/srv](../src/dog_interfaces/srv)
3. [src/dog_interfaces/action](../src/dog_interfaces/action)

## 2. Message 契约

### 2.1 Target3D

定义：[src/dog_interfaces/msg/Target3D.msg](../src/dog_interfaces/msg/Target3D.msg)

字段：

1. `std_msgs/Header header`
2. `string target_id`
3. `geometry_msgs/Point position`
4. `float32 confidence`

运行语义：

1. 箱体检测输出时，`target_id` 通常为 `food/tool/instrument/medical/no_box`。
2. 箱体检测输出的 `position.x/y` 是归一化图像框中心，`position.z` 是归一化面积，不是世界坐标。
3. solver 或外推输出时，`target_id` 可为 `synced_target`、`extrapolated_target` 等，此时 `position` 更接近 3D 位姿语义。
4. lifecycle 健康心跳只把 `confidence > 0` 且位置有限的目标视为有效帧；`no_box`、`no_feature`、`idle_spinning` 不算健康心跳。

### 2.2 Target3DArray

定义：[src/dog_interfaces/msg/Target3DArray.msg](../src/dog_interfaces/msg/Target3DArray.msg)

字段：

1. `std_msgs/Header header`
2. `dog_interfaces/Target3D[] targets`

主要链路：

1. `/target/target_3d`：`dog_perception` 发布，`dog_lifecycle` 作为有效帧心跳源，`dog_behavior` 的 `SetBoxesTypeAction` 用于箱型排序。
2. `/target/digit_result`：`dog_perception` 发布，`dog_behavior` 的 `PublishMathAnswerAction` 用于生成 `/math_answer`。

## 3. Service 契约

### 3.1 MathDetect

定义：[src/dog_interfaces/srv/MathDetect.srv](../src/dog_interfaces/srv/MathDetect.srv)

请求：

1. `sensor_msgs/Image image`

响应：

1. `bool success`
2. `string expression`
3. `float32 value`
4. `string message`

当前状态：主运行链路未使用该 service。数字识别链路当前通过 `/target/digit_result` 发布 `Target3DArray`，由 `PublishMathAnswerAction` 消费并发布 `/math_answer`。

## 4. Action 契约

### 4.1 ExecuteBehavior

定义：[src/dog_interfaces/action/ExecuteBehavior.action](../src/dog_interfaces/action/ExecuteBehavior.action)

Goal：

1. `string behavior_name`
2. `geometry_msgs/PoseStamped target_pose`

Result：

1. `bool accepted`
2. `string detail`

Feedback：

1. `float32 progress`
2. `string state`

当前运行语义：`dog_behavior` 发送 `/behavior/execute`，`dog_serial_bridge_node` 当前只接受 `behavior_name=PickUpBoxes`。

### 4.2 NavigateWaypoint

定义：[src/dog_interfaces/action/NavigateWaypoint.action](../src/dog_interfaces/action/NavigateWaypoint.action)

Goal：

1. `geometry_msgs/PoseStamped target_pose`

Result 与 Feedback 字段同 `ExecuteBehavior`。

当前运行语义：`dog_behavior` 发送 `/behavior/nav_execute`，`dog_serial_bridge_nav_telemetry_node` 将目标转换为 `RCNAV` 串口帧，并等待 `RCArrivalMX`。

### 4.3 PlaceBoxes

定义：[src/dog_interfaces/action/PlaceBoxes.action](../src/dog_interfaces/action/PlaceBoxes.action)

Goal：

1. `string box_type`
2. `string payload`
3. `int32 step_counter`

Result 与 Feedback 字段同 `ExecuteBehavior`。

当前运行语义：`payload` 由 `PlaceIndexAction` 生成，格式为 `place=0,3,count=3`，不是通用的分号分隔 `key=value` 协议。

## 5. 结果语义注意事项

1. ROS action code 与业务成功语义需要分开看。
2. 例如 `RCPickFail` 会让 `/behavior/execute` 的 ROS action code 返回 `SUCCEEDED`，但 Result 中 `accepted=false`、`detail=pick_fail`。
3. `dog_behavior` 的 Action 叶子通常要求 ROS action code 为 `SUCCEEDED` 且 Result `accepted=true` 才视为 BT 成功。
4. 修改任一 action 字段后，需要同步更新 action client、action server、测试和文档。
