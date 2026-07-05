# UART8 上下位机通信协议

## 1. 文档目的

本文档按当前工程源码整理 `UART8` 的实际通信行为，作为下位机与上位机联调协议说明。

当前代码里，`UART8` 同时承担两类通信：

1. 上位机 -> 下位机：ASCII 文本命令接收
2. 下位机 -> 上位机：调试数据回传和到点通知

说明：

- 本文档只描述当前代码已经实现的逻辑，不描述历史版本协议。
- 当前 `UART8` 上既有 ASCII 报文，也有二进制调试帧，二者共用同一串口。

---

## 2. 物理链路与串口参数

### 2.1 串口参数

`UART8` 当前配置如下：

- 波特率：`115200`
- 数据位：`8`
- 停止位：`1`
- 校验位：`None`
- 流控：`None`
- 工作模式：`TX/RX`

代码佐证：

- `Core/Src/usart.c`
  - `MX_UART8_Init()` 中：
    - `huart8.Init.BaudRate = 115200;`
    - `huart8.Init.WordLength = UART_WORDLENGTH_8B;`
    - `huart8.Init.StopBits = UART_STOPBITS_1;`
    - `huart8.Init.Parity = UART_PARITY_NONE;`
    - `huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;`

### 2.2 引脚

- `PE1` -> `UART8_TX`
- `PE0` -> `UART8_RX`

代码佐证：

- `Core/Src/usart.c` 的 `HAL_UART_MspInit()` 中 `UART8 GPIO Configuration`

### 2.3 接收方式

`UART8` 接收使用 `DMA + IDLE`。

代码佐证：

- `Modules/unicomm/unicomm.c`
  - `UniComm_UART8_Init()` 把 `huart8` 注册给 `BSP/usart`
- `BSP/usart/bsp_usart.c`
  - `USARTServiceInit()` 调用 `HAL_UARTEx_ReceiveToIdle_DMA(...)`
  - `HAL_UARTEx_RxEventCallback()` 中收到一段数据后调用模块回调

---

## 3. 总体协议结论

### 3.1 上位机 -> 下位机

当前接收协议是“按行解析”的 ASCII 文本协议：

- 每一条命令本质上是一行文本
- 行结束符支持：
  - `\r`
  - `\n`
  - `\r\n`
- 只有在遇到行结束符时，这一行才会被正式解析

### 3.2 下位机 -> 上位机

当前存在两种发送数据：

1. 周期性二进制调试帧
2. 到点时发送一次 ASCII 字符串 `RCArrivalMX`

注意：

- 两者共用 `UART8`
- 当前代码没有统一发送队列，也没有协议复用层
- 上位机必须按帧尾或固定字符串自行区分

---

## 4. 上位机 -> 下位机协议

## 4.1 接收缓存与分帧规则

### 4.1.1 最大单行长度

- 单行最大长度：`255` 字节
- 超过后，本行直接丢弃，直到遇到下一次 `\r` 或 `\n` 才恢复

代码佐证：

- `Modules/unicomm/unicomm.c`
  - `#define UNICOMM_RX_BUFFER_SIZE 255U`
  - `unicomm_line_buffer[UNICOMM_RX_BUFFER_SIZE + 1U]`
  - 当 `unicomm_line_length >= UNICOMM_RX_BUFFER_SIZE` 时置 `overflow`

### 4.1.2 支持跨回调拼接

当前实现不是“每次 DMA 回调只解析一次完整帧”，而是流式累计：

- 一条命令可以分多次 DMA/IDLE 回调到达
- 只要最后收到了 `\r` 或 `\n`，仍然能拼成一条完整命令
- 一次 DMA 回调里也可以包含多条命令

代码佐证：

- `Modules/unicomm/unicomm.c`
  - `UniComm_ProcessIncomingBytes()` 会逐字节累积到 `unicomm_line_buffer`
  - 遇到 `\r` 或 `\n` 后才调用 `UniComm_ProcessLine()`

### 4.1.3 空行处理

- 空行会被忽略

代码佐证：

- `Modules/unicomm/unicomm.c`
  - `if (line[0] == '\0') return;`

---

## 4.2 命令 1：抓取命令

### 4.2.1 帧格式

```text
RCPickUpBoxes\r\n
```

实际解析内容是去掉换行后的：

```text
RCPickUpBoxes
```

### 4.2.2 头帧、尾帧、结束字段

- 头帧：`RCPickUpBoxes` 本身就是完整命令
- 尾帧：无独立尾帧字段
- 结束字段：`\r`、`\n` 或 `\r\n`

### 4.2.3 触发条件

- 必须与 `RCPickUpBoxes` 完全相等
- 多一个字符、少一个字符都不会触发

### 4.2.4 下位机动作

- 收到后立即执行：`state = GRAB`

代码佐证：

- `Modules/unicomm/unicomm.c`
  - `if (strcmp(line, "RCPickUpBoxes") == 0) { state = GRAB; }`

### 4.2.5 示例

合法：

```text
RCPickUpBoxes\r\n
```

非法：

```text
RCPickUpBoxes123\r\n
```

---

## 4.3 命令 2：放置/站立命令

### 4.3.1 帧格式

当前代码只检查前缀：

```text
RCplace=...
```

常见示例：

```text
RCplace=0,3,count=3\r\n
```

### 4.3.2 头帧、尾帧、结束字段

- 头帧：`RCplace=`
- 尾帧：无独立尾帧字段
- 结束字段：`\r`、`\n` 或 `\r\n`

### 4.3.3 触发条件

- 只要一行字符串前 `8` 个字符是 `RCplace=` 即触发
- 后面的内容当前不校验

### 4.3.4 下位机动作

- 收到后立即执行：`state = STAND`

代码佐证：

- `Modules/unicomm/unicomm.c`
  - `if (strncmp(line, "RCplace=", 8) == 0) { state = STAND; }`

### 4.3.5 示例

推荐发送：

```text
RCplace=0,3,count=3\r\n
```

当前也会被接受：

```text
RCplace=test\r\n
```

---

## 4.4 命令 3：导航状态命令

## 4.4.1 帧格式

导航命令必须以：

```text
RCNAV;
```

开头，并且整行中必须包含以下 `8` 个字段：

- `cur_valid=`
- `goal_valid=`
- `cur_x=`
- `cur_y=`
- `goal_x=`
- `goal_y=`
- `cur_yaw=`
- `goal_yaw=`

当前代码允许存在额外字段，额外字段会被忽略。

### 4.4.2 头帧、尾帧、结束字段

- 头帧：`RCNAV;`
- 字段分隔符：`;`
- 尾帧：无独立尾帧字段
- 结束字段：`\r`、`\n` 或 `\r\n`

### 4.4.3 字段类型

整数：

- `cur_valid`
- `goal_valid`

浮点数：

- `cur_x`
- `cur_y`
- `goal_x`
- `goal_y`
- `cur_yaw`
- `goal_yaw`

解析规则：

- `cur_valid` / `goal_valid`
  - 使用 `strtol`
  - 任何非 `0` 值最终都记为 `1`
  - `0` 记为 `0`
- 其他字段
  - 使用 `strtof`
  - 允许小数、负数
  - 必须是有限值，`NaN` / `Inf` 不接受

### 4.4.4 成功解析后的变量更新

成功后更新：

- `nav_current_valid`
- `nav_goal_valid`
- `current_x`
- `current_y`
- `target_x`
- `target_y`
- `upstream_current_yaw`
- `upstream_goal_yaw`

### 4.4.5 失败条件

以下任一情况出现，则整条导航命令无效，不更新任何导航变量：

- 不以 `RCNAV;` 开头
- 缺少任意一个必需字段
- 任意必需字段解析失败
- 字段值后面不是 `;` 或字符串结束

### 4.4.6 字段顺序要求

当前代码对字段顺序没有强依赖，只要：

- 以 `RCNAV;` 开头
- 必需字段都出现
- 字段之间用 `;` 分隔

即可被解析。

说明：

- 如果同一个字段重复出现，当前实现会取搜索到的第一个匹配值。

代码佐证：

- `Modules/unicomm/unicomm.c`
  - `if (strncmp(line, "RCNAV;", 6) == 0)`
  - `UniComm_HandleNavFrame(...)`
  - `UniComm_ParseIntField(...)`
  - `UniComm_ParseFloatField(...)`

### 4.4.7 推荐示例

最小可用格式：

```text
RCNAV;cur_valid=1;goal_valid=1;cur_x=123.000;cur_y=42.000;goal_x=300.000;goal_y=200.000;cur_yaw=-10.500;goal_yaw=-20.000\r\n
```

带扩展字段格式，当前也可接受：

```text
RCNAV;seq=1;stamp_ms=1710000000123;cur_valid=1;cur_frame=map;cur_x=123.000;cur_y=42.000;cur_z=0.000;cur_yaw=-10.500;goal_valid=1;goal_frame=map;goal_x=300.000;goal_y=200.000;goal_z=0.000;goal_yaw=-20.000\r\n
```

### 4.4.8 导航命令对运动状态的影响

导航变量更新后，`PostureControl_task()` 周期调用 `ActionProcessNavigation()`，按以下规则控制状态机：

1. `nav_current_valid == 0` 或 `nav_goal_valid == 0`
   - `state = STAND`
2. 航向误差 `yaw_error > 2.0`
   - `state = RIGHT_TURN`
3. 航向误差 `yaw_error < -2.0`
   - `state = LEFT_TURN`
4. `|current_x - target_x| < 8.0` 且 `|current_y - target_y| < 8.0`
   - `state = STAND`
   - 尝试发送一次 `RCArrivalMX`
5. 否则
   - `state = RUN`

代码佐证：

- `APP/action/action.h`
  - `#define NAV_ARRIVAL_THRESHOLD_CM 8.0f`
  - `#define NAV_YAW_FINISH_THRESHOLD_DEG 2.0f`
- `APP/action/action.c`
  - `ActionProcessNavigation()`

---

## 5. 下位机 -> 上位机协议

## 5.1 周期性二进制调试帧

这是当前实际运行中最主要的发送数据。

### 5.1.1 发送频率

`PostureControl_task()` 每 `8 ms` 运行一次，因此理论最大发送频率为：

- `125 Hz`

但实际发送还受限于：

- `huart8.gState == HAL_UART_STATE_READY`

也就是说：

- 只有 `UART8` 当前发送空闲时才会发这一帧
- 若上一帧还没发完，本周期直接跳过

代码佐证：

- `TASK/PostureControl_task/PostureControl_task.c`
  - `if (huart8.gState == HAL_UART_STATE_READY)`
  - `HAL_UART_Transmit_DMA(&huart8, debug_frame, sizeof(debug_frame));`
  - `osDelayUntil(&LastWakeTime,8);`

### 5.1.2 帧格式

帧总长度固定：

- `8 * 4 + 4 = 36` 字节

结构如下：

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `ch0~ch7` | `8 x float` | 8 路 `float`，按 STM32 小端序直接 memcpy |
| `tail` | `4 bytes` | 固定帧尾 `00 00 80 7F` |

### 5.1.3 头帧、尾帧、结束字段

- 头帧：无
- 尾帧：固定 `0x00 0x00 0x80 0x7F`
- 结束字段：无额外结束字段，帧尾本身就是分帧标志

### 5.1.4 通道定义

当前 8 路通道顺序如下：

| 通道 | 含义 |
| --- | --- |
| `ch0` | `LeftGetMotor(0)->measure.total_angle` |
| `ch1` | `LeftGetMotor(1)->measure.total_angle` |
| `ch2` | `LeftGetMotor(2)->measure.total_angle` |
| `ch3` | `LeftGetMotor(3)->measure.total_angle` |
| `ch4` | `RightGetMotor(0)->measure.total_angle` |
| `ch5` | `RightGetMotor(1)->measure.total_angle` |
| `ch6` | `RightGetMotor(2)->measure.total_angle` |
| `ch7` | `RightGetMotor(3)->measure.total_angle` |

说明：

- 这里只发 `total_angle`
- 不发速度、电流、IMU、状态量

### 5.1.5 编码方式

当前实现：

```c
memcpy(debug_frame, debug_ch, sizeof(debug_ch));
memcpy(debug_frame + sizeof(debug_ch), frame_tail, sizeof(frame_tail));
```

因此：

- `float` 按 MCU 内存布局直接发送
- STM32F4 为小端序
- 上位机应按 `IEEE754 float little-endian` 解析

### 5.1.6 示例

若 8 路通道全为 `0.0f`，则整帧为：

```text
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 80 7F
```

若 `ch0 = 1.0f`，其余为 `0.0f`，则前 4 字节为：

```text
00 00 80 3F
```

整帧起始示例：

```text
00 00 80 3F 00 00 00 00 ...
```

### 5.1.7 代码佐证

- `TASK/PostureControl_task/PostureControl_task.c`
  - `static float debug_ch[8];`
  - `static uint8_t debug_frame[sizeof(debug_ch) + 4];`
  - `static const uint8_t frame_tail[4] = {0x00, 0x00, 0x80, 0x7F};`

---

## 5.2 到点通知帧

### 5.2.1 帧格式

下位机到点后会尝试发送：

```text
RCArrivalMX
```

长度：

- `11` 字节

### 5.2.2 头帧、尾帧、结束字段

- 头帧：`RC`
- 载荷：`Arrival`
- 尾帧：`MX`
- 结束字段：无，当前代码不会追加 `\r` / `\n` / `\0`

### 5.2.3 发送条件

必须同时满足：

1. `nav_current_valid == 1`
2. `nav_goal_valid == 1`
3. 当前状态不在 `GRAB` / `STOP` / `REALSE`
4. 航向误差满足：
   - `-2.0 deg <= yaw_error <= 2.0 deg`
5. 位置误差满足：
   - `|current_x - target_x| < 8.0`
   - `|current_y - target_y| < 8.0`

满足后：

- 状态置为 `STAND`
- 若本次到点尚未上报，则发送一次 `RCArrivalMX`

### 5.2.4 发送次数

当前逻辑是“单次到点只发送一次”：

- 首次到点：发送一次
- 保持在到点区域内：不重复发送
- 离开到点条件后：发送标志清零
- 下次重新到点：再发送一次

代码佐证：

- `APP/action/action.c`
  - `static int arrival_report_sent = 0;`
  - `if (!arrival_report_sent) { UniComm_SendArrival(); arrival_report_sent = 1; }`
  - 非到点或需转向时会把 `arrival_report_sent = 0`
- `Modules/unicomm/unicomm.c`
  - `#define UNICOMM_ARRIVAL_FRAME "RCArrivalMX"`
  - `UniComm_SendArrival()`

### 5.2.5 上位机解析建议

由于 `RCArrivalMX` 是裸 ASCII 字符串，当前没有：

- 长度字段
- CRC
- 换行结束

因此上位机若同时接收二进制调试帧，应：

1. 先按二进制帧尾 `00 00 80 7F` 拆分调试帧
2. 同时在字节流中检索 ASCII 串 `RCArrivalMX`

---

## 6. 当前实现中的发送冲突风险

这一节非常重要。

`UART8` 当前既用于：

- 周期性 DMA 发送二进制调试帧
- 同步阻塞发送 `RCArrivalMX`

且二者没有发送仲裁。

`PostureControl_task()` 的执行顺序是：

1. 若 `UART8` 空闲，先发一次 36 字节 DMA 调试帧
2. 然后执行 `PostureControl()`
3. `PostureControl()` -> `ActionProcessNavigation()`
4. 到点时调用 `UniComm_SendArrival()`

而 `UniComm_SendArrival()` 最终调用的是阻塞发送：

```c
USARTSend(..., USART_TRANSFER_BLOCKING);
```

风险：

- 如果前面刚启动了 DMA 调试发送，则此时 `UART8` 可能正忙
- `UniComm_SendArrival()` 没有检查返回值
- 因此 `RCArrivalMX` 存在被 `UART Busy` 丢掉的风险

结论：

- 从“协议定义”上看，下位机有到点通知帧
- 从“当前代码可靠性”上看，这个通知帧并不是 100% 有保证送达

如果 `RCArrivalMX` 要作为正式联调协议，建议后续至少做一项改动：

1. 把调试帧和控制协议分到不同串口
2. 或统一走一个发送队列
3. 或发送前检查并重试

代码佐证：

- `TASK/PostureControl_task/PostureControl_task.c`
  - 先 `HAL_UART_Transmit_DMA(&huart8, ...)`
  - 后 `PostureControl()`
- `APP/action/action.c`
  - 到点后 `UniComm_SendArrival()`
- `Modules/unicomm/unicomm.c`
  - `UniComm_SendBytes(...)` 使用 `USART_TRANSFER_BLOCKING`
- `BSP/usart/bsp_usart.c`
  - `USARTSend(...)` 的阻塞模式调用 `HAL_UART_Transmit(...)`

---

## 7. 当前不属于“实际运行协议”的代码

工程里还有一个 `APP/debug_comm` 模块，也会通过 `UniComm_SendBytes()` 发送 VOFA 风格数据。

但按当前源码：

- `DebugComm_Init()` 在 `main()` 里被调用
- `DebugComm_Task()` 没有被任何任务或循环调用

因此：

- `APP/debug_comm` 当前不是实际运行中的协议链路
- 它不应作为当前上位机联调协议的依据

代码佐证：

- `Core/Src/main.c`
  - 调用了 `DebugComm_Init()`
- 全工程搜索
  - 没有 `DebugComm_Task()` 的实际调用点

---

## 8. 建议给上位机的对接约束

按当前代码，建议上位机遵守以下约束：

1. 发送命令统一使用 ASCII，并以 `\r\n` 结束
2. 每条命令单独成行，不要把多条命令拼成超长一行
3. 单条命令不要超过 `255` 字节
4. `RCPickUpBoxes` 必须完全匹配
5. `RCplace=` 建议发送规范内容，虽然当前下位机只检查前缀
6. `RCNAV` 必须至少带齐 8 个必需字段
7. 若要依赖 `RCArrivalMX`，最好先关闭 `UART8` 的周期调试帧，或修改发送仲裁逻辑

---

## 9. 协议速查表

| 方向 | 类型 | 头帧 | 尾帧 | 结束字段 | 频率 | 发送/触发条件 |
| --- | --- | --- | --- | --- | --- | --- |
| 上位机 -> 下位机 | 抓取命令 | `RCPickUpBoxes` | 无 | `\r` / `\n` / `\r\n` | 按需 | 完全匹配后 `state = GRAB` |
| 上位机 -> 下位机 | 放置命令 | `RCplace=` | 无 | `\r` / `\n` / `\r\n` | 按需 | 前缀匹配后 `state = STAND` |
| 上位机 -> 下位机 | 导航命令 | `RCNAV;` | 无 | `\r` / `\n` / `\r\n` | 按需 | 8 个必需字段全部解析成功后更新导航变量 |
| 下位机 -> 上位机 | 调试帧 | 无 | `00 00 80 7F` | 无 | 最大 `125 Hz` | `UART8` 空闲时由 `PostureControl_task()` 周期发送 |
| 下位机 -> 上位机 | 到点通知 | `RC` | `MX` | 无 | 单次到点 1 次 | 航向误差在 `±2 deg` 内且位置误差在 `8 cm` 内时尝试发送 |

---

## 10. 关键源码位置

- 串口参数与引脚
  - `Core/Src/usart.c`
- `UART8` 注册与协议解析
  - `Modules/unicomm/unicomm.c`
  - `Modules/unicomm/unicomm.h`
- DMA + IDLE 接收框架
  - `BSP/usart/bsp_usart.c`
- 导航状态机与到点上报
  - `APP/action/action.c`
  - `APP/action/action.h`
- 周期性二进制调试帧
  - `TASK/PostureControl_task/PostureControl_task.c`
