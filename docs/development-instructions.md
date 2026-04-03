# 开发指令

## 先决条件 (Prerequisites)
- Ubuntu 22.04
- ROS 2 Humble
- C++ 编译器 (GCC)
- CMake
- NUC 硬件环境

## 构建说明 (Build Instructions)
本项目使用标准的 ROS2 Humble 构建系统。请使用 `colcon build` 进行编译：

```bash
mkdir -p workspace/src
cp -r src/* workspace/src/
cd workspace
colcon build
```

## 运行与部署
主要采用在 NUC 上的直接运行方式，通过 ROS 各个节点通信来输出处理后的视觉和定位数据至运动控制模块。
