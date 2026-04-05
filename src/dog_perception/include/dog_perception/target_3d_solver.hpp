#pragma once

#include <dog_interfaces/msg/target3_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>

namespace dog_perception
{

struct SyncedSensorFrame
{
  sensor_msgs::msg::Image::ConstSharedPtr image;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud;
  std::string output_frame_id;
};

class ITarget3DSolver
{
public:
  /// @brief 多态目标求解器的虚析构函数。
  virtual ~ITarget3DSolver() = default;

  /// @brief 将同步传感器帧求解为 Target3D 输出。
  /// @param frame 同步图像-点云帧。
  /// @param output 待填充的 Target3D 输出消息。
  /// @return 求解成功时返回 true。
  virtual bool solve(const SyncedSensorFrame & frame, dog_interfaces::msg::Target3D & output) = 0;
};

class Target3DSolverFactory
{
public:
  /// @brief 按配置类型创建目标求解器。
  /// @param solver_type 求解器类型键。
  /// @param logger 诊断日志器。
  /// @return 目标求解器实例。
  static std::unique_ptr<ITarget3DSolver> create(
    const std::string & solver_type,
    const rclcpp::Logger & logger);
};

}  // namespace dog_perception
