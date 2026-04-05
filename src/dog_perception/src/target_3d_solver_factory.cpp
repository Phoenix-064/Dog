#include "dog_perception/target_3d_solver.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace dog_perception
{

namespace
{

class MinimalTarget3DSolver final : public ITarget3DSolver
{
public:
  explicit MinimalTarget3DSolver(const rclcpp::Logger & logger)
  : logger_(logger)
  {
  }

  bool solve(const SyncedSensorFrame & frame, dog_interfaces::msg::Target3D & output) override
  {
    if (!frame.image || !frame.pointcloud) {
      RCLCPP_WARN(logger_, "Skip solving: image or pointcloud is null");
      return false;
    }

    if (frame.image->width == 0U || frame.image->height == 0U || frame.pointcloud->width == 0U) {
      RCLCPP_WARN(logger_, "Skip solving: invalid input image/pointcloud dimensions");
      return false;
    }

    output.header.stamp = frame.image->header.stamp;
    output.header.frame_id = frame.output_frame_id;
    output.target_id = "synced_target";
    output.position = geometry_msgs::msg::Point();
    output.position.z = 1.0;
    output.confidence = 0.6F;
    return true;
  }

private:
  rclcpp::Logger logger_;
};

class MinimalPnpSolver final : public ITarget3DSolver
{
public:
  explicit MinimalPnpSolver(const rclcpp::Logger & logger)
  : logger_(logger)
  {
  }

  bool solve(const SyncedSensorFrame & frame, dog_interfaces::msg::Target3D & output) override
  {
    if (!frame.image || !frame.pointcloud) {
      RCLCPP_WARN(logger_, "Skip solving: image or pointcloud is null");
      return false;
    }

    if (frame.image->width == 0U || frame.image->height == 0U || frame.pointcloud->width == 0U) {
      RCLCPP_WARN(logger_, "Skip solving: invalid input image/pointcloud dimensions");
      return false;
    }

    if (
      frame.pointcloud->point_step == 0U || frame.pointcloud->height == 0U ||
      frame.pointcloud->data.empty())
    {
      RCLCPP_WARN(logger_, "Skip solving: empty pointcloud payload");
      return false;
    }

    const auto safeMul = [](size_t lhs, size_t rhs, size_t & result) -> bool {
      if (lhs == 0U || rhs == 0U) {
        result = 0U;
        return true;
      }
      if (lhs > (std::numeric_limits<size_t>::max() / rhs)) {
        return false;
      }
      result = lhs * rhs;
      return true;
    };

    size_t min_row_step = 0U;
    if (!safeMul(static_cast<size_t>(frame.pointcloud->width), frame.pointcloud->point_step, min_row_step)) {
      RCLCPP_WARN(logger_, "Skip solving: pointcloud width*point_step overflows");
      return false;
    }
    if (frame.pointcloud->row_step < min_row_step) {
      RCLCPP_WARN(logger_, "Skip solving: pointcloud row_step smaller than width*point_step");
      return false;
    }

    size_t expected_payload_size = 0U;
    if (
      !safeMul(
        static_cast<size_t>(frame.pointcloud->row_step),
        static_cast<size_t>(frame.pointcloud->height),
        expected_payload_size))
    {
      RCLCPP_WARN(logger_, "Skip solving: pointcloud row_step*height overflows");
      return false;
    }
    if (frame.pointcloud->data.size() < expected_payload_size) {
      RCLCPP_WARN(logger_, "Skip solving: pointcloud payload smaller than row_step*height");
      return false;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    size_t valid_points = 0U;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*frame.pointcloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*frame.pointcloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*frame.pointcloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        sum_x += static_cast<double>(x);
        sum_y += static_cast<double>(y);
        sum_z += static_cast<double>(z);
        ++valid_points;
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN(logger_, "Skip solving: pointcloud missing xyz fields (%s)", exception.what());
      return false;
    } catch (...) {
      RCLCPP_WARN(logger_, "Skip solving: unknown exception when iterating pointcloud xyz fields");
      return false;
    }

    if (valid_points == 0U) {
      RCLCPP_WARN(logger_, "Skip solving: no finite xyz points available");
      return false;
    }

    output.header.stamp = frame.image->header.stamp;
    output.header.frame_id = frame.output_frame_id;
    output.target_id = "synced_target";
    output.position = geometry_msgs::msg::Point();
    output.position.x = sum_x / static_cast<double>(valid_points);
    output.position.y = sum_y / static_cast<double>(valid_points);
    output.position.z = sum_z / static_cast<double>(valid_points);
    output.confidence = 0.8F;
    return true;
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace

std::unique_ptr<ITarget3DSolver> Target3DSolverFactory::create(
  const std::string & solver_type,
  const rclcpp::Logger & logger)
{
  if (solver_type == "mock_minimal") {
    return std::make_unique<MinimalTarget3DSolver>(logger);
  }

  if (solver_type.empty() || solver_type == "minimal_pnp") {
    return std::make_unique<MinimalPnpSolver>(logger);
  }

  throw std::runtime_error("Unsupported target3d solver type: " + solver_type);
}

}  // namespace dog_perception
