#include "dog_perception/perception_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rmw/qos_profiles.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string_view>
#include <vector>

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

rmw_qos_reliability_policy_t resolveReliability(
  const std::string & reliability,
  const rclcpp::Logger & logger)
{
  if (reliability == "best_effort") {
    return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }
  if (reliability == "reliable") {
    RCLCPP_WARN(
      logger,
      "qos_reliability=reliable may be incompatible with sensor publishers using best_effort");
    return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }

  throw std::runtime_error(
          "Invalid qos_reliability value: " + reliability +
          ". Allowed: best_effort, reliable");
}

rclcpp::ReliabilityPolicy toReliabilityPolicy(const std::string & reliability)
{
  if (reliability == "best_effort") {
    return rclcpp::ReliabilityPolicy::BestEffort;
  }
  return rclcpp::ReliabilityPolicy::Reliable;
}

std::string_view reliabilityToString(const rclcpp::ReliabilityPolicy reliability)
{
  return reliability == rclcpp::ReliabilityPolicy::BestEffort ? "best_effort" : "reliable";
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
, sync_queue_size_(0)
, sync_slop_ms_(0)
, stale_frame_timeout_ms_(50)
, max_future_skew_ms_(5)
, frame_cache_size_(8)
, qos_compatible_(true)
, dropped_frame_count_(0)
, solved_frame_count_(0)
, solve_failure_count_(0)
, frame_history_(8)
, latency_samples_ms_(8)
, end_to_end_latency_samples_ms_(8)
{
  constexpr int kMaxSyncQueueSize = 1024;
  constexpr int kMaxFrameCacheSize = 4096;

  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
  pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
  target3d_topic_ = declare_parameter<std::string>("target3d_topic", "/target/target_3d");
  sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
  sync_slop_ms_ = declare_parameter<int>("sync_slop_ms", 25);
  stale_frame_timeout_ms_ = declare_parameter<int>("stale_frame_timeout_ms", 50);
  max_future_skew_ms_ = declare_parameter<int>("max_future_skew_ms", 5);
  frame_cache_size_ = declare_parameter<int>("frame_cache_size", 32);
  qos_reliability_ = declare_parameter<std::string>("qos_reliability", "best_effort");
  solver_type_ = declare_parameter<std::string>("solver_type", "minimal_pnp");
  extrinsics_yaml_path_ =
    declare_parameter<std::string>("extrinsics_yaml_path", getDefaultExtrinsicsYamlPath());

  if (sync_queue_size_ <= 0 || sync_slop_ms_ <= 0 || stale_frame_timeout_ms_ <= 0 ||
    max_future_skew_ms_ < 0 || frame_cache_size_ <= 0)
  {
    throw std::runtime_error(
            "sync_queue_size, sync_slop_ms, stale_frame_timeout_ms and frame_cache_size must be > 0, max_future_skew_ms must be >= 0");
  }

  if (sync_queue_size_ > kMaxSyncQueueSize || frame_cache_size_ > kMaxFrameCacheSize) {
    throw std::runtime_error(
            "sync_queue_size/frame_cache_size exceed safety upper bound");
  }

  frame_history_.set_capacity(static_cast<size_t>(frame_cache_size_));
  latency_samples_ms_.set_capacity(static_cast<size_t>(frame_cache_size_));
  end_to_end_latency_samples_ms_.set_capacity(static_cast<size_t>(frame_cache_size_));

  solver_ = Target3DSolverFactory::create(solver_type_, get_logger());
  target3d_pub_ = create_publisher<dog_interfaces::msg::Target3D>(target3d_topic_, rclcpp::SensorDataQoS());

  initializeStaticTransform();
  setupSynchronizedPipeline();

  RCLCPP_INFO(
    get_logger(),
    "dog_perception initialized, image_topic=%s, pointcloud_topic=%s, target3d_topic=%s, extrinsics_yaml_path=%s",
    image_topic_.c_str(),
    pointcloud_topic_.c_str(),
    target3d_topic_.c_str(),
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

bool PerceptionNode::evaluateRuntimeQosCompatibility()
{
  const auto expected = toReliabilityPolicy(qos_reliability_);

  const auto check_topic = [this, expected](const std::string & topic_name) {
      const auto publishers = get_publishers_info_by_topic(topic_name);
      if (publishers.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "No publishers discovered yet for topic=%s, defer QoS compatibility decision",
          topic_name.c_str());
        return true;
      }

      for (const auto & endpoint : publishers) {
        const auto actual = endpoint.qos_profile().reliability();
        if (actual != expected) {
          RCLCPP_ERROR(
            get_logger(),
            "QoS reliability mismatch on topic=%s, expected=%s, publisher=%s",
            topic_name.c_str(),
            reliabilityToString(expected).data(),
            reliabilityToString(actual).data());
          return false;
        }
      }

      return true;
    };

  return check_topic(image_topic_) && check_topic(pointcloud_topic_);
}

void PerceptionNode::setupSynchronizedPipeline()
{
  const auto reliability_policy = resolveReliability(qos_reliability_, get_logger());

  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
  qos.reliability = reliability_policy;

  image_subscriber_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
  pointcloud_subscriber_ =
    std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();

  image_subscriber_->subscribe(this, image_topic_, qos);
  pointcloud_subscriber_->subscribe(this, pointcloud_topic_, qos);

  synchronizer_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(static_cast<uint32_t>(sync_queue_size_)),
    *image_subscriber_,
    *pointcloud_subscriber_);

  qos_compatible_ = evaluateRuntimeQosCompatibility();

  if (!qos_compatible_) {
    RCLCPP_ERROR(
      get_logger(),
      "Synchronized pipeline is disabled because QoS compatibility check failed");
    return;
  }

  synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(
      static_cast<int64_t>(sync_slop_ms_) * 1000 * 1000));
  synchronizer_->registerCallback(std::bind(
      &PerceptionNode::synchronizedCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "Synchronized pipeline ready, queue=%d, slop_ms=%d, stale_timeout_ms=%d, qos_reliability=%s",
    sync_queue_size_,
    sync_slop_ms_,
    stale_frame_timeout_ms_,
    qos_reliability_.c_str());
}

bool PerceptionNode::shouldDropAsStale(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg) const
{
  const rclcpp::Time image_stamp(image_msg->header.stamp);
  const rclcpp::Time pointcloud_stamp(pointcloud_msg->header.stamp);
  const auto source_stamp = image_stamp > pointcloud_stamp ? image_stamp : pointcloud_stamp;
  const auto age = now() - source_stamp;
  const double age_ms = static_cast<double>(age.nanoseconds()) / 1e6;
  if (age_ms < -static_cast<double>(max_future_skew_ms_)) {
    return true;
  }
  return age_ms > static_cast<double>(stale_frame_timeout_ms_);
}

void PerceptionNode::synchronizedCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg)
{
  const auto begin = now();

  if (shouldDropAsStale(image_msg, pointcloud_msg)) {
    ++dropped_frame_count_;
    frame_history_.push_back(FrameState{begin, 0.0, false});
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Dropped stale synced frame (timeout=%d ms)",
      stale_frame_timeout_ms_);
    return;
  }

  SyncedSensorFrame frame;
  frame.image = image_msg;
  frame.pointcloud = pointcloud_msg;
  frame.output_frame_id = camera_extrinsics_.frame_id;

  auto target3d_msg = std::make_unique<dog_interfaces::msg::Target3D>();
  bool solve_success = false;
  try {
    solve_success = solver_->solve(frame, *target3d_msg);
  } catch (const std::exception & exception) {
    ++solve_failure_count_;
    frame_history_.push_back(FrameState{begin, 0.0, false});
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Solver threw exception: %s",
      exception.what());
    return;
  } catch (...) {
    ++solve_failure_count_;
    frame_history_.push_back(FrameState{begin, 0.0, false});
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Solver threw unknown exception");
    return;
  }

  if (!solve_success) {
    ++solve_failure_count_;
    frame_history_.push_back(FrameState{begin, 0.0, false});
    return;
  }

  const auto end = now();
  const auto elapsed = end - begin;
  const double elapsed_ms = static_cast<double>(elapsed.nanoseconds()) / 1e6;
  const rclcpp::Time image_stamp(image_msg->header.stamp);
  const rclcpp::Time pointcloud_stamp(pointcloud_msg->header.stamp);
  const auto source_stamp = image_stamp > pointcloud_stamp ? image_stamp : pointcloud_stamp;
  const double end_to_end_latency_ms = static_cast<double>((end - source_stamp).nanoseconds()) / 1e6;

  latency_samples_ms_.push_back(elapsed_ms);
  end_to_end_latency_samples_ms_.push_back(end_to_end_latency_ms);
  ++solved_frame_count_;
  frame_history_.push_back(FrameState{end, elapsed_ms, true});

  target3d_pub_->publish(std::move(target3d_msg));
}

size_t PerceptionNode::getFrameCacheSize() const
{
  return frame_history_.size();
}

size_t PerceptionNode::getDroppedFrameCount() const
{
  return dropped_frame_count_;
}

size_t PerceptionNode::getSolvedFrameCount() const
{
  return solved_frame_count_;
}

size_t PerceptionNode::getSolveFailureCount() const
{
  return solve_failure_count_;
}

size_t PerceptionNode::getLatencySampleCount() const
{
  return latency_samples_ms_.size();
}

bool PerceptionNode::isQosCompatible() const
{
  return qos_compatible_;
}

double PerceptionNode::getLatencyP95Ms() const
{
  if (latency_samples_ms_.empty()) {
    return 0.0;
  }

  std::vector<double> sorted(latency_samples_ms_.begin(), latency_samples_ms_.end());
  std::sort(sorted.begin(), sorted.end());
  const auto index = static_cast<size_t>(
    std::ceil(static_cast<double>(sorted.size()) * 0.95) - 1.0);
  return sorted[std::min(index, sorted.size() - 1)];
}

double PerceptionNode::getEndToEndLatencyP95Ms() const
{
  if (end_to_end_latency_samples_ms_.empty()) {
    return 0.0;
  }

  std::vector<double> sorted(end_to_end_latency_samples_ms_.begin(), end_to_end_latency_samples_ms_.end());
  std::sort(sorted.begin(), sorted.end());
  const auto index = static_cast<size_t>(
    std::ceil(static_cast<double>(sorted.size()) * 0.95) - 1.0);
  return sorted[std::min(index, sorted.size() - 1)];
}

}  // namespace dog_perception