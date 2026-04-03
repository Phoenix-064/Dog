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
  virtual ~ITarget3DSolver() = default;

  virtual bool solve(const SyncedSensorFrame & frame, dog_interfaces::msg::Target3D & output) = 0;
};

class Target3DSolverFactory
{
public:
  static std::unique_ptr<ITarget3DSolver> create(
    const std::string & solver_type,
    const rclcpp::Logger & logger);
};

}  // namespace dog_perception
