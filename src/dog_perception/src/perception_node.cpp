#include "dog_perception/perception_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <functional>
#include <stdexcept>

namespace dog_perception
{

namespace
{

std::string requireNonEmptyString(
  const YAML::Node & node,
  const char * field_name,
  const std::string & yaml_path)
{
  if (!node[field_name]) {
    throw std::runtime_error("Missing field '" + std::string(field_name) + "' in " + yaml_path);
  }

  const auto value = node[field_name].as<std::string>();
  if (value.empty()) {
    throw std::runtime_error("Field '" + std::string(field_name) + "' is empty in " + yaml_path);
  }
  return value;
}

double requireFiniteScalar(
  const YAML::Node & node,
  const char * field_name,
  const std::string & yaml_path)
{
  if (!node[field_name]) {
    throw std::runtime_error("Missing field '" + std::string(field_name) + "' in " + yaml_path);
  }

  const double value = node[field_name].as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error("Field '" + std::string(field_name) + "' is not finite in " + yaml_path);
  }
  return value;
}

}  // namespace

std::string PerceptionNode::getDefaultExtrinsicsYamlPath()
{
  try {
    return ament_index_cpp::get_package_share_directory("dog_perception") +
           "/config/camera_extrinsics.yaml";
  } catch (const std::exception & exception) {
    throw std::runtime_error(
            std::string("Failed to resolve package share directory for dog_perception: ") +
            exception.what());
  }
}

PerceptionNode::CameraExtrinsics PerceptionNode::loadExtrinsicsFromYaml(
  const std::string & yaml_path) const
{
  YAML::Node node;
  try {
    node = YAML::LoadFile(yaml_path);
  } catch (const std::exception & exception) {
    throw std::runtime_error(
            "Failed to open extrinsics yaml: " + yaml_path + ", reason: " + exception.what());
  }

  CameraExtrinsics extrinsics;
  extrinsics.frame_id = requireNonEmptyString(node, "frame_id", yaml_path);
  extrinsics.child_frame_id = requireNonEmptyString(node, "child_frame_id", yaml_path);
  extrinsics.x = requireFiniteScalar(node, "x", yaml_path);
  extrinsics.y = requireFiniteScalar(node, "y", yaml_path);
  extrinsics.z = requireFiniteScalar(node, "z", yaml_path);
  extrinsics.qx = requireFiniteScalar(node, "qx", yaml_path);
  extrinsics.qy = requireFiniteScalar(node, "qy", yaml_path);
  extrinsics.qz = requireFiniteScalar(node, "qz", yaml_path);
  extrinsics.qw = requireFiniteScalar(node, "qw", yaml_path);

  if (extrinsics.frame_id == extrinsics.child_frame_id) {
    throw std::runtime_error(
            "Invalid extrinsics in " + yaml_path +
            ": frame_id and child_frame_id must be different");
  }

  const double quaternion_norm = std::sqrt(
    extrinsics.qx * extrinsics.qx +
    extrinsics.qy * extrinsics.qy +
    extrinsics.qz * extrinsics.qz +
    extrinsics.qw * extrinsics.qw);

  if (quaternion_norm < 1e-6 || std::abs(quaternion_norm - 1.0) > 1e-3) {
    throw std::runtime_error(
            "Invalid quaternion norm in " + yaml_path +
            ": expected approximately 1.0, got " + std::to_string(quaternion_norm));
  }

  return extrinsics;
}

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dog_perception", options)
{
  lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
  extrinsics_yaml_path_ =
    declare_parameter<std::string>("extrinsics_yaml_path", getDefaultExtrinsicsYamlPath());

  initializeStaticTransform();

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PerceptionNode::lidarCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "dog_perception initialized, lidar_topic=%s, extrinsics_yaml_path=%s",
    lidar_topic_.c_str(),
    extrinsics_yaml_path_.c_str());
}

void PerceptionNode::initializeStaticTransform()
{
  try {
    camera_extrinsics_ = loadExtrinsicsFromYaml(extrinsics_yaml_path_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = camera_extrinsics_.frame_id;
    transform.child_frame_id = camera_extrinsics_.child_frame_id;
    transform.transform.translation.x = camera_extrinsics_.x;
    transform.transform.translation.y = camera_extrinsics_.y;
    transform.transform.translation.z = camera_extrinsics_.z;
    transform.transform.rotation.x = camera_extrinsics_.qx;
    transform.transform.rotation.y = camera_extrinsics_.qy;
    transform.transform.rotation.z = camera_extrinsics_.qz;
    transform.transform.rotation.w = camera_extrinsics_.qw;

    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    static_tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(
      get_logger(),
      "Published static TF frame_id=%s child_frame_id=%s",
      transform.header.frame_id.c_str(),
      transform.child_frame_id.c_str());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "%s", exception.what());
    throw;
  }
}

void PerceptionNode::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  (void)msg;
  RCLCPP_DEBUG(get_logger(), "Received lidar frame");
}

}  // namespace dog_perception