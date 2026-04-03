#include "dog_perception/target_3d_solver.hpp"

#include <geometry_msgs/msg/point.hpp>

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

class UnimplementedPnpSolver final : public ITarget3DSolver
{
public:
  explicit UnimplementedPnpSolver(const rclcpp::Logger & logger)
  : logger_(logger)
  {
  }

  bool solve(const SyncedSensorFrame & frame, dog_interfaces::msg::Target3D & output) override
  {
    (void)frame;
    (void)output;
    RCLCPP_ERROR(
      logger_,
      "solver_type=minimal_pnp is not implemented yet, no Target3D will be published");
    return false;
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
    return std::make_unique<UnimplementedPnpSolver>(logger);
  }

  throw std::runtime_error("Unsupported target3d solver type: " + solver_type);
}

}  // namespace dog_perception
