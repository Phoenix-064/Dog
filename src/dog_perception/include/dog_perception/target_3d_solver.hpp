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
  /// @brief Virtual destructor for polymorphic target solvers.
  virtual ~ITarget3DSolver() = default;

  /// @brief Solve synchronized sensor frame into Target3D output.
  /// @param frame Synchronized image-pointcloud frame.
  /// @param output Target3D output message to populate.
  /// @return True when solve succeeds.
  virtual bool solve(const SyncedSensorFrame & frame, dog_interfaces::msg::Target3D & output) = 0;
};

class Target3DSolverFactory
{
public:
  /// @brief Create target solver by configured type.
  /// @param solver_type Solver type key.
  /// @param logger Logger for diagnostics.
  /// @return Target solver instance.
  static std::unique_ptr<ITarget3DSolver> create(
    const std::string & solver_type,
    const rclcpp::Logger & logger);
};

}  // namespace dog_perception
