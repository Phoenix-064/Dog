---
project_name: 'Dog'
user_name: 'Phoenix'
date: '2026-03-30T11:21:29.900Z'
sections_completed: ['technology_stack', 'language_rules', 'framework_rules', 'architecture_rules', 'defensive_design', 'logging_rules']
status: 'complete'
rule_count: 10
optimized_for_llm: true
---

# Project Context for AI Agents

_This file contains critical rules and patterns that AI agents must follow when implementing code in this project. Focus on unobvious details that agents might otherwise miss._

---

## Technology Stack & Versions

- **Core Technologies**: ROS 2 (Humble), C/C++ (Modern standard without Modules), OpenCV (Latest C++ interfaces encouraged), Point Cloud Library (PCL).
- **Key Dependencies**: `mid360_driver`, `point_lio`.
- **ROS Message Packages**: `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `tf2_ros`, `vision_msgs`, and planned custom `quadruped_msgs`.

## Critical Implementation Rules

### 1. Framework & Language Boundaries
- **Modern C++ Constraints**: Always use smart pointers (`std::unique_ptr`, `std::shared_ptr`) for memory management, object lifecycle, and callbacks handling to ensure zero-copy where possible.
- **Strict No-Module Policy**: NEVER use C++20 Modules (e.g., `import` syntax). All header files must utilize `#pragma once` and traditional `#include` structures.
- **OpenCV Modernization**: Prefer the most unified and modern C++ interfaces in OpenCV (e.g., modern cv::dnn::Net APIs). Avoid outdated C-style interfaces.
- **Zero-Copy Callbacks**: Sensor data and message callbacks in ROS 2 must optimally use constant references or smart pointers to prevent expensive copy operations.

### 2. Architecture & Design Patterns
- **The Factory Pattern Boundary**: 
  - ROS 2 Node classes (inheriting from `rclcpp::Node`) **DO NOT** require the factory pattern. They should be natively instantiated (e.g. `std::make_shared`).
  - **ALL** core algorithmic and non-ROS classes (like Detection algorithms, PnP solvers) **MUST** establish an abstract interface base class and be created via Factory Pattern (returning `std::unique_ptr<Interface>`). This ensures strict decoupling and forward compatibility (e.g. pivoting smoothly to OpenVINO).
- **Algorithmic State Reserved (e.g., `math_detect`)**: Even if the current MVP processes single frames linearly without handling misidentifications, **DO NOT** implement algorithmic processing as static, stateless methods. Algorithms MUST be stateful instantiated objects to natively reserve architectural space (hooks or queues like `std::deque`) for future multi-frame smoothing, validation logic, or state-machines.

### 3. Defensive Design & Operational Decoupling
- **Interface Decoupling With Memory Safety**: Abstract base classes generated via the Factory Pattern (e.g., `IDetector`) MUST decouple from framework-specific structs like `cv::Mat` or OpenVINO tensors in their external signature. To solve this without triggering deep copies or risking raw `void*` memory leaks, interfaces should accept data as custom Data Transfer Objects (DTOs) padded with robust modern handlers (e.g., encapsulating `std::shared_ptr<uint8_t>`, ROS 2's natively pooled `sensor_msgs::msg::Image::ConstSharedPtr`, or using `cv_bridge::toCvShare`). Internal algorithms must then map (not deep copy) this data into their specific framework types utilizing managed views, adhering strictly to the `Zero-Copy Callbacks` rule while eliminating purely manual memory management risks.
- **State Management Separation**: Algorithmic objects (Model Engines) should only handle single-frame inference. The responsibility of maintaining state queues (e.g., multi-frame history to resolve false positives) must reside in a separate Wrapper or State Tracker layer, keeping the core AI algorithms purely mathematical and easily swappable.
- **Lazy Activation via ROS 2 Services**: Modules like `math_detect` are NOT activated arbitrarily at system startup. They become logically relevant only when the robot reaches a specific location. Ensure that such detection nodes are implemented explicitly as **ROS 2 Service Servers** (not persistent Pub/Sub listeners). The module must remain dormant until a service request with image frames is explicitely passed in, at which point it computes the math answer and returns it as a response. This guarantees strict single-responsibility and event-driven lazy loading, preventing complex debug scenarios during navigation.

### 4. Logging & Conventions
- **ROS 2 Logging Standard**: Absolutely NO `std::cout` or `printf` for debugging or operational outputs. Exclusively and uniformly use the `RCLCPP_INFO`, `RCLCPP_ERROR`, and related macros for comprehensive trace tracking.
- **Strict Dependency Synchronization**: Whenever a new C++ dependency (like `<glog/logging.h>`) is required by an agent in `.cpp` or `.hpp` files, that agent MUST simultaneously update both `package.xml` and `CMakeLists.txt` of the corresponding module to declare the dependency before generating or testing the code.
\n---\n\n## Usage Guidelines\n\n**For AI Agents:**\n\n- Read this file before implementing any code\n- Follow ALL rules exactly as documented\n- When in doubt, prefer the more restrictive option\n- Update this file if new patterns emerge\n\n**For Humans:**\n\n- Keep this file lean and focused on agent needs\n- Update when technology stack changes\n- Review quarterly for outdated rules\n- Remove rules that become obvious over time\n\nLast Updated: 2026-03-30T11:21:29.900Z
