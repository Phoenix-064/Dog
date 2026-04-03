## 实现模式与一致性规则 (Implementation Patterns & Consistency Rules)

### 模式类别定义 (Pattern Categories Defined)

**已识别的关键冲突点 (Critical Conflict Points Identified):**
基于 C++14/17 与 ROS 2 嵌入式特性，确定了 4 个极易发生代码分歧或导致系统崩溃的领域：内存所有权边界、命名约定规则、执行器防阻塞死锁法则、以及外部配置高频查表。

### 内存与通信协议规范 (Memory & Communication Patterns)

**零拷贝的落实闭环 (Zero-Copy Implementation):**
- **必须强制启用 IPC**：在实例化核心感知节点时，必须附带选项 `rclcpp::NodeOptions().use_intra_process_comms(true)`。
- **大图与矩阵通讯**：话题订阅回调的参数强制为 `const Type::ConstSharedPtr&` 形式。如需画框标图，严禁修改原订阅指针空间，必须使用 `cv_bridge::toCvCopy` 创建隔离图层或专属遮罩 (Mask Overlay)。

**定长数据队列要求 (Fixed-Length Queue Format):**
- 由于防止内存爆栈的 NFR 要求，所有的防抖、状态确认和历数追踪缓存池，严禁使用持续延展的 `std::vector`，强制统一要求含有覆盖与淘汰逻辑的定长队列模型（如定上限的 `std::deque` 封装作为滑动窗并严格要求先 `pop_front` 后 `push_back`）。

### 命名与结构规范 (Naming & Structure Patterns)

**包与节点命名 (Package & Node Naming):**
- ROS 2 Package 必须全小写带有下划线作为词切分（例如：`dog_perception`）。
- Node 的 C++ 实例及全局管理指针必须带有显式的 `_node` 后缀。

**接口与内部变量防冲突映射 (Interface & Variable Naming):**
- C++ 业务相关类：大驼峰（如 `BoxPnpSolver`）。类内部成员私有变量强烈限定含有后置下划线后缀 `xxxx_`（如 `camera_matrix_`）。
- ROS 2 Topic/Action/Service 通讯命名：绝对全小写单词与下划线加斜杠组合（如 `/behavior/place_box`）。

### 结构与层级规范 (Structure & Hierarchy Patterns)

**头文件与实现的强制分离 (.hpp / .cpp Separation):**
- **接口防腐层**：所有 C++ 节点、核心业务类、自定义数据结构，**必须**严格遵循声明与实现分离的原则。类的定义、成员变量的内存布局、函数签名只能存在于 `include/<package_name>/xxxx.hpp` 头文件中。
- **业务逻辑隐藏**：所有的具体业务逻辑（如点云测算、图像 PnP 求解运算、ROS 2 回调函数的具体执行流程）必须实现在 `src/xxxx.cpp` 中。
- **禁止内联地雷**：严禁在 `.hpp` 的类声明大括号内直接写包含长逻辑的内联函数，以避免编译期依赖地狱和令人头疼的重定义 (ODR) 违规，同时极大缩短后续 AI 代理迭代时的增量编译时间。

**CMakeLists 与依赖可见性法则 (CMake Linked Library Visibility):**
- **CMake 功能隔离**：凡是产生了 `.hpp/.cpp` 拆分的自定义底层业务实现类（如 PnP 计算类、运动学逆解类等），除了入口节点 (Node Executor)，必须封装为 `add_library(...)` 创建链接库，不能平铺和节点挤在同一个 `add_executable` 下，避免链接交叉与增量编译缓慢。
- **头文件路径映射**：强行规定包含其它模块接口时，必须带上全路径包名，如 `#include "dog_perception/box_pnp.hpp"`。坚决不准使用类似 `#include "box_pnp.hpp"` 这样的相对目录或被污染的 include-path。

### 进程与并发防护规范 (Process & Concurrency Patterns)

**ROS 2 异步非阻塞铁律 (Anti-Deadlock Anti-Blocking Rules):**
- **禁用休眠**：绝对禁止在任何 Node 回调（包括 Topic、Timer 以及 Service 回调函数）内部执行 `std::this_thread::sleep_for()`。
- **纯异步发送**：当发生往外下发行进抓取指令等极度耗时动作时，强制要求采用 `async_send_request` 伴随异步 Future 的完毕回调（Action），绝不允许 `while(!done){rclcpp::spin_some(node);}`造成的回调堆栈嵌套及Executor执行器死锁。

**执行器与回调安全重入法则 (Reentrancy & Callback Group Rules):**
- **禁止无锁修改容器**：在涉及状态缓存或队列堆叠的地方（如使用 `std::deque` 的代码），所在的节点存在跨类型回调（比如同时有 Topic 回调和 Timer回调）或者启用了 `MultiThreadedExecutor`，**必须**使用 `std::mutex`（或者自旋锁）对成员变量的 `push_back/pop_front` 实行严格上锁，防止条件竞争导致的段错误。
- **唯一指针发布**：在发布 (Publish) 支持 IPC 的零拷贝数据类型时，严禁复用同一块通过 `new` 或栈创建的实例对象。要利用 IPC，必须强制使用 `std::unique_ptr` 进行发布 (`pub_->publish(std::move(msg_ptr))`)，明确宣示所有权的移交，从源头扼杀引用计数污染和生命周期问题。

**高频回调参数加载法则 (Config & YAML Load Rules):**
- **配置抽象**：不许写死宏与内部配置数字常量。外参、超时阈值等只能放置于统一 `config/` 目录的相应 yaml 下。
- **强制初始化期查表**：任何 `get_parameter(...)` 的调用仅允许且强制发生在 Node初始化的首个周期或配置阶段。由于哈希锁争用消耗，严禁在类 10Hz/30Hz 的图像点云判定回调内去拉取 Parameter，极度影响 CPU 利用率。

### 工业级增强防线 (Industrial Guardrail Patterns)

**时间同步与传感器融合模式 (Time Synchronization & Fusion Patterns):**
- **强制要求机制同源**：当涉及必须聚合多个非同频 Topic（如 Image 与 Lidar，或者 Odometry）以进行物理测算或状态预估时，严禁使用非确定性的“拿到最新消息便缓存”方式来进行关联。必须使用 ROS 2 官方的 `message_filters::Subscriber` 与 `TimeSynchronizer`（或 `ApproximateTimeSynchronizer`）来将回调进行强制的时间硬对齐聚合，这是高容错感知的基石。

**隐藏复杂依赖与接口极简模式 (Pimpl Idiom & Include Hiding):**
- **Pimpl 惯用法 (Pointer to Implementation)**：如果一个核心 `.hpp` 类的底层逻辑极其依赖大型未解耦的第三方库（例如 PCL 或 CUDA 推理 API），强烈推荐使用 Pimpl 模式（即在 `.hpp` 内仅保留前置声明，依赖库的全量 `#include` 仅写在独立的 `.cpp` 内）。这杜绝了第三方重型依赖往整个项目拓扑树上无限蔓延。

**状态机与生命周期模式 (Lifecycle Node Patterns):**
- **推行 LifecycleNode (基于业务需求)**：对于那些独占极高功耗外设（如连续扫描的 Lidar 或长开相机的推理模型），不允许继承普通的 `rclcpp::Node`。所有此类硬件的代理必须继承 `rclcpp_lifecycle::LifecycleNode`。只有当收到系统的显式指令转换到 `active` 状态时，才允许订阅或发布内存；在 `inactive` 状态时，底层必须释放所有多媒体管道的指针。

### 格式与错误处理规范 (Format & Error Handling Patterns)

**日志规范与可观测性 (Logging Formats):**
- 必须使用 ROS 2 官方宏（如 `RCLCPP_INFO`, `RCLCPP_ERROR` 等），**严禁使用** `std::cout` 或 `printf`。
- 错误日志必须携带具体的上下文（如 Node 名字、Frame ID、时间戳、具体引发异常的数据或状态码），方便多节点分布式排错。

**空间与坐标系转换格式 (Coordinate & Transform Formats):**
- 所有跨节点的三维点云数据交换强制标准为 `sensor_msgs::msg::PointCloud2`，节点内部物理引擎或 PCL 处理计算完毕后，必须立刻转回标准消息对外发布。
- Frame ID 命名：统一按 URDF 或 TF 树规范使用标准坐标系名词（如 `base_link`, `camera_optical_frame`），绝对禁止在 C++ 业务代码中出现零散写死的非标准字母 Frame 名称。

### 强制执行指南 (Enforcement Guidelines)

**所有 AI 代理必须遵守 (All AI Agents MUST):**
- ✅ 强制使用 `rclcpp::Node` 或 `rclcpp_lifecycle::LifecycleNode` 继承类来面向对象地封装节点逻辑，禁止在 `main` 里面平铺堆砌松散代码。
- ✅ 节点的回调队列（History/Depth）长度必须按真实吞吐量声明，默认 QoS 为 `SensorDataQoS`（针对高频传感器流）或 `SystemDefaultsQoS`（针对可靠性要求高的服务/动作）。
- ✅ 任何有极大概率产生堆内存碎片和大量动态分配的函数（如无上限的 `new` 或 `std::vector::push_back` 扩容），禁止放入高频 (>=10Hz) 的 Topic 回调主链路中。

**模式验证 (Pattern Enforcement):**
- 命名规范和基本的 C++ 语法将通过 `ament_cpplint` 构建阶段的静态检查进行拦截验证。
- 所有违反了“高频回调读参数”或“包含 `std::this_thread::sleep_for` 死锁逻辑”的代码提案，在代码审查阶段必须被 Review 机制直接驳回。

### 模式示例 (Pattern Examples)

**推荐示范 (Good Examples):**
```cpp
// ==========================================
// include/dog_perception/box_pnp_solver_node.hpp
// ==========================================
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

class BoxPnpSolverNode : public rclcpp::Node {
public:
  BoxPnpSolverNode();
private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  std::vector<double> camera_matrix_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

// ==========================================
// src/box_pnp_solver_node.cpp
// ==========================================
#include "dog_perception/box_pnp_solver_node.hpp"
#include <cv_bridge/cv_bridge.h>

BoxPnpSolverNode::BoxPnpSolverNode() : Node("box_pnp_solver_node") {
  this->declare_parameter("camera_matrix", std::vector<double>{});
  camera_matrix_ = this->get_parameter("camera_matrix").as_double_array();
  auto qos = rclcpp::SensorDataQoS();
  sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", qos, 
    std::bind(&BoxPnpSolverNode::image_callback, this, std::placeholders::_1));
}

void BoxPnpSolverNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  auto frame = cv_bridge::toCvCopy(msg, "bgr8"); 
  // 复杂的视觉 PnP 求解逻辑...
}
```

**反面模式 (Anti-Patterns):**
```cpp
// ❌ 错误 1：在回调里执行耗时堵塞任务
void action_callback() {
  std::this_thread::sleep_for(std::chrono::seconds(1)); // 必定死锁单线程执行器
}

// ❌ 错误 2：高频执行域每次重复拉取 YAML 参数
void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  double threshold = this->get_parameter("dist_threshold").as_double(); // 极大消耗 CPU
}

// ❌ 错误 3：命名不规范，没有后置下划线且使用了不应该出现的驼峰
double cameraMatrix; 
```
