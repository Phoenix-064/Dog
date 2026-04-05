#include "dog_perception/target_3d_solver.hpp"

#include <opencv2/calib3d.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace dog_perception
{

namespace
{

class MinimalTarget3DSolver final : public ITarget3DSolver
{
public:
  /// @brief 构造最小化 3D 目标求解器。
  /// @param logger 日志器。
  explicit MinimalTarget3DSolver(const rclcpp::Logger & logger)
  : logger_(logger)
  {
  }

  /// @brief 基于同步帧生成简化 Target3D 输出。
  /// @param frame 同步图像-点云帧。
  /// @param output 待填充输出消息。
  /// @return 求解成功时返回 true。
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
  /// @brief 构造最小化 PnP 风格求解器。
  /// @param logger 日志器。
  explicit MinimalPnpSolver(const rclcpp::Logger & logger)
  : logger_(logger)
  {
  }

  /// @brief 基于点云统计结果求解目标 3D 位置。
  /// @param frame 同步图像-点云帧。
  /// @param output 待填充输出消息。
  /// @return 求解成功时返回 true。
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

    std::vector<cv::Point3d> valid_points;
    valid_points.reserve(std::min<size_t>(128U, frame.pointcloud->data.size() / frame.pointcloud->point_step));

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
        valid_points.emplace_back(
          static_cast<double>(x),
          static_cast<double>(y),
          static_cast<double>(z));
        if (valid_points.size() >= 128U) {
          break;
        }
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN(logger_, "Skip solving: pointcloud missing xyz fields (%s)", exception.what());
      return false;
    } catch (...) {
      RCLCPP_WARN(logger_, "Skip solving: unknown exception when iterating pointcloud xyz fields");
      return false;
    }

    if (valid_points.size() < 4U) {
      RCLCPP_WARN(logger_, "Skip solving: not enough finite xyz points for PnP");
      return false;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    for (const auto & point : valid_points) {
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
    }
    const cv::Point3d centroid(
      sum_x / static_cast<double>(valid_points.size()),
      sum_y / static_cast<double>(valid_points.size()),
      sum_z / static_cast<double>(valid_points.size()));

    std::vector<cv::Point3d> object_points;
    object_points.reserve(valid_points.size());
    for (const auto & point : valid_points) {
      object_points.emplace_back(point.x - centroid.x, point.y - centroid.y, point.z - centroid.z);
    }

    std::vector<cv::Point2d> image_points;
    image_points.reserve(object_points.size());
    const double width = static_cast<double>(std::max<uint32_t>(frame.image->width, 1U));
    const double height = static_cast<double>(std::max<uint32_t>(frame.image->height, 1U));
    const double focal = std::max(width, height);
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
      focal, 0.0, width * 0.5,
      0.0, focal, height * 0.5,
      0.0, 0.0, 1.0);
    const cv::Mat distortion = cv::Mat::zeros(4, 1, CV_64F);

    const cv::Vec3d synthetic_rvec(0.0, 0.0, 0.0);
    const cv::Vec3d synthetic_tvec(centroid.x, centroid.y, centroid.z);
    cv::projectPoints(object_points, synthetic_rvec, synthetic_tvec, camera_matrix, distortion, image_points);

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    std::vector<int> inliers;
    const bool pnp_ok = cv::solvePnPRansac(
      object_points,
      image_points,
      camera_matrix,
      distortion,
      rvec,
      tvec,
      false,
      100,
      8.0,
      0.99,
      inliers,
      cv::SOLVEPNP_EPNP);
    if (!pnp_ok || inliers.size() < 4U) {
      RCLCPP_WARN(logger_, "Skip solving: solvePnPRansac failed");
      return false;
    }

    if (!std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) || !std::isfinite(tvec[2])) {
      RCLCPP_WARN(logger_, "Skip solving: no finite xyz points available");
      return false;
    }

    output.header.stamp = frame.image->header.stamp;
    output.header.frame_id = frame.output_frame_id;
    output.target_id = "synced_target";
    output.position = geometry_msgs::msg::Point();
    output.position.x = tvec[0];
    output.position.y = tvec[1];
    output.position.z = tvec[2];
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
