#include "dog_perception/perception_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rmw/qos_profiles.h>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace dog_perception
{

namespace
{

/// @brief 读取并校验 YAML 中必填且非空的字符串字段。
/// @param node YAML 节点。
/// @param field_name 字段名。
/// @param yaml_path YAML 文件路径，用于错误提示。
/// @return 字段值。
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

/// @brief 读取并校验 YAML 中必填且为有限值的标量字段。
/// @param node YAML 节点。
/// @param field_name 字段名。
/// @param yaml_path YAML 文件路径，用于错误提示。
/// @return 字段数值。
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

/// @brief 将字符串形式的可靠性参数解析为 RMW 可靠性策略。
/// @param reliability 可靠性配置字符串。
/// @param logger 日志器。
/// @return RMW 可靠性策略枚举值。
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

/// @brief 将可靠性配置字符串映射为 rclcpp 可靠性策略。
/// @param reliability 可靠性配置字符串。
/// @return rclcpp 可靠性策略。
rclcpp::ReliabilityPolicy toReliabilityPolicy(const std::string & reliability)
{
  if (reliability == "best_effort") {
    return rclcpp::ReliabilityPolicy::BestEffort;
  }
  return rclcpp::ReliabilityPolicy::Reliable;
}

/// @brief 将可靠性策略转换为可读字符串。
/// @param reliability 可靠性策略。
/// @return 可靠性文本表示。
std::string_view reliabilityToString(const rclcpp::ReliabilityPolicy reliability)
{
  return reliability == rclcpp::ReliabilityPolicy::BestEffort ? "best_effort" : "reliable";
}

/// @brief 计算样本集合的 95 分位值。
/// @param samples 时延样本集合。
/// @return 95 分位值；样本为空时返回 0。
double percentile95(const boost::circular_buffer<double> & samples)
{
  if (samples.empty()) {
    return 0.0;
  }

  std::vector<double> sorted(samples.begin(), samples.end());
  std::sort(sorted.begin(), sorted.end());
  const auto index = static_cast<size_t>(
    std::ceil(static_cast<double>(sorted.size()) * 0.95) - 1.0);
  return sorted[std::min(index, sorted.size() - 1)];
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
, single_side_dropout_timeout_ms_(150)
, extrapolation_watchdog_ms_(20)
, extrapolation_max_window_ms_(300)
, extrapolation_min_interval_ms_(40)
, idle_spinning_publish_ms_(200)
, digit_roi_x_(0)
, digit_roi_y_(0)
, digit_roi_width_(64)
, digit_roi_height_(64)
, digit_min_confidence_(0.30)
, digit_glare_brightness_threshold_(245.0)
, digit_glare_ratio_threshold_(0.35)
, digit_yolo_model_path_("yolo11n.pt")
, digit_temporal_window_(5)
, digit_temporal_confirm_count_(2)
, qos_compatible_(true)
, idle_spinning_mode_(false)
, extrapolation_active_(false)
, has_last_image_stamp_(false)
, has_last_pointcloud_stamp_(false)
, has_last_extrapolation_pub_time_(false)
, has_last_idle_publish_time_(false)
, has_mode_enter_time_(false)
, dropped_frame_count_(0)
, solved_frame_count_(0)
, solve_failure_count_(0)
, extrapolation_trigger_count_(0)
, extrapolation_recovery_count_(0)
, idle_spinning_trigger_count_(0)
, frame_history_(8)
, pose_history_(8)
, latency_samples_ms_(8)
, end_to_end_latency_samples_ms_(8)
, digit_label_history_(5)
, digit_latency_samples_ms_(8)
{
  constexpr int kMaxSyncQueueSize = 1024;
  constexpr int kMaxFrameCacheSize = 4096;
  constexpr int kMaxTemporalWindow = 120;

  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
  pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
  target3d_topic_ = declare_parameter<std::string>("target3d_topic", "/target/target_3d");
  digit_result_topic_ = declare_parameter<std::string>("digit_result_topic", "/target/digit_result");
  lifecycle_mode_topic_ = declare_parameter<std::string>("lifecycle_mode_topic", "/lifecycle/system_mode");
  sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
  sync_slop_ms_ = declare_parameter<int>("sync_slop_ms", 25);
  stale_frame_timeout_ms_ = declare_parameter<int>("stale_frame_timeout_ms", 50);
  max_future_skew_ms_ = declare_parameter<int>("max_future_skew_ms", 5);
  frame_cache_size_ = declare_parameter<int>("frame_cache_size", 32);
  single_side_dropout_timeout_ms_ = declare_parameter<int>("single_side_dropout_timeout_ms", 150);
  extrapolation_watchdog_ms_ = declare_parameter<int>("extrapolation_watchdog_ms", 20);
  extrapolation_max_window_ms_ = declare_parameter<int>("extrapolation_max_window_ms", 300);
  extrapolation_min_interval_ms_ = declare_parameter<int>("extrapolation_min_interval_ms", 40);
  idle_spinning_publish_ms_ = declare_parameter<int>("idle_spinning_publish_ms", 200);
  qos_reliability_ = declare_parameter<std::string>("qos_reliability", "best_effort");
  solver_type_ = declare_parameter<std::string>("solver_type", "minimal_pnp");
  digit_recognizer_type_ = declare_parameter<std::string>("digit_recognizer_type", "heuristic");
  extrinsics_yaml_path_ =
    declare_parameter<std::string>("extrinsics_yaml_path", getDefaultExtrinsicsYamlPath());
  digit_roi_x_ = declare_parameter<int>("digit_roi_x", 0);
  digit_roi_y_ = declare_parameter<int>("digit_roi_y", 0);
  digit_roi_width_ = declare_parameter<int>("digit_roi_width", 64);
  digit_roi_height_ = declare_parameter<int>("digit_roi_height", 64);
  digit_min_confidence_ = declare_parameter<double>("digit_min_confidence", 0.30);
  digit_glare_brightness_threshold_ =
    declare_parameter<double>("digit_glare_brightness_threshold", 245.0);
  digit_glare_ratio_threshold_ = declare_parameter<double>("digit_glare_ratio_threshold", 0.35);
  digit_yolo_model_path_ = declare_parameter<std::string>("digit_yolo_model_path", "yolo11n.pt");
  digit_temporal_window_ = declare_parameter<int>("digit_temporal_window", 5);
  digit_temporal_confirm_count_ = declare_parameter<int>("digit_temporal_confirm_count", 2);

  if (sync_queue_size_ <= 0 || sync_slop_ms_ <= 0 || stale_frame_timeout_ms_ <= 0 ||
    max_future_skew_ms_ < 0 || frame_cache_size_ <= 0 || digit_temporal_window_ <= 0 ||
    single_side_dropout_timeout_ms_ <= 0 || extrapolation_watchdog_ms_ <= 0 ||
    extrapolation_max_window_ms_ <= 0 || extrapolation_min_interval_ms_ <= 0 ||
    idle_spinning_publish_ms_ <= 0 ||
    digit_temporal_confirm_count_ <= 0)
  {
    throw std::runtime_error(
            "invalid positive parameters: sync/slop/stale/cache/temporal/dropout/extrapolation/idle values must be > 0, max_future_skew_ms must be >= 0");
  }

  if (sync_queue_size_ > kMaxSyncQueueSize || frame_cache_size_ > kMaxFrameCacheSize ||
    digit_temporal_window_ > kMaxTemporalWindow)
  {
    throw std::runtime_error(
            "sync_queue_size/frame_cache_size/digit_temporal_window exceed safety upper bound");
  }

  if (digit_temporal_confirm_count_ > digit_temporal_window_) {
    throw std::runtime_error(
            "digit_temporal_confirm_count must be <= digit_temporal_window");
  }

  frame_history_.set_capacity(static_cast<size_t>(frame_cache_size_));
  pose_history_.set_capacity(static_cast<size_t>(frame_cache_size_));
  latency_samples_ms_.set_capacity(static_cast<size_t>(frame_cache_size_));
  end_to_end_latency_samples_ms_.set_capacity(static_cast<size_t>(frame_cache_size_));
  digit_label_history_.set_capacity(static_cast<size_t>(digit_temporal_window_));
  digit_latency_samples_ms_.set_capacity(static_cast<size_t>(frame_cache_size_));

  solver_ = Target3DSolverFactory::create(solver_type_, get_logger());
  const DigitRecognizerParams recognizer_params{
    digit_roi_x_,
    digit_roi_y_,
    digit_roi_width_,
    digit_roi_height_,
    digit_min_confidence_,
    digit_glare_brightness_threshold_,
    digit_glare_ratio_threshold_,
    digit_yolo_model_path_};
  digit_recognizer_ = DigitRecognizerFactory::create(
    digit_recognizer_type_,
    recognizer_params,
    get_logger());

  target3d_pub_ = create_publisher<dog_interfaces::msg::Target3DArray>(
    target3d_topic_,
    rclcpp::SensorDataQoS());
  digit_result_pub_ = create_publisher<dog_interfaces::msg::Target3DArray>(
    digit_result_topic_,
    rclcpp::SensorDataQoS());

  initializeStaticTransform();
  setupSynchronizedPipeline();

  image_stamp_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PerceptionNode::imageStampCallback, this, std::placeholders::_1));
  pointcloud_stamp_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PerceptionNode::pointcloudStampCallback, this, std::placeholders::_1));
  lifecycle_mode_sub_ = create_subscription<std_msgs::msg::String>(
    lifecycle_mode_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&PerceptionNode::lifecycleModeCallback, this, std::placeholders::_1));
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(extrapolation_watchdog_ms_),
    std::bind(&PerceptionNode::watchdogCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "dog_perception initialized, image_topic=%s, pointcloud_topic=%s, target3d_topic=%s, digit_result_topic=%s, mode_topic=%s, extrinsics_yaml_path=%s",
    image_topic_.c_str(),
    pointcloud_topic_.c_str(),
    target3d_topic_.c_str(),
    digit_result_topic_.c_str(),
    lifecycle_mode_topic_.c_str(),
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
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "No publishers discovered yet for topic=%s, defer QoS compatibility decision",
          topic_name.c_str());
        return true;
      }

      for (const auto & endpoint : publishers) {
        const auto actual = endpoint.qos_profile().reliability();
        if (actual != expected) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
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
  imageStampCallback(image_msg);
  pointcloudStampCallback(pointcloud_msg);

  if (idle_spinning_mode_) {
    frame_history_.push_back(FrameState{begin, 0.0, false});
    return;
  }

  qos_compatible_ = evaluateRuntimeQosCompatibility();
  if (!qos_compatible_) {
    ++dropped_frame_count_;
    frame_history_.push_back(FrameState{begin, 0.0, false});
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Dropped synced frame due to runtime QoS incompatibility");
    return;
  }

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

  processDigitRecognition(image_msg);

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
  pose_history_.push_back(PoseState{end, *target3d_msg});

  if (extrapolation_active_) {
    extrapolation_active_ = false;
    ++extrapolation_recovery_count_;
    RCLCPP_INFO(
      get_logger(),
      "dropout recovered, switching back to synchronized solving");
  }

  frame_history_.push_back(FrameState{end, elapsed_ms, true});

  dog_interfaces::msg::Target3DArray target3d_array_msg;
  target3d_array_msg.header = target3d_msg->header;
  target3d_array_msg.targets.push_back(*target3d_msg);
  target3d_pub_->publish(std::move(target3d_array_msg));
}

void PerceptionNode::imageStampCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  if (!image_msg) {
    return;
  }
  last_image_stamp_ = rclcpp::Time(image_msg->header.stamp);
  last_image_receive_time_ = now();
  has_last_image_stamp_ = true;
}

void PerceptionNode::pointcloudStampCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg)
{
  if (!pointcloud_msg) {
    return;
  }
  last_pointcloud_stamp_ = rclcpp::Time(pointcloud_msg->header.stamp);
  last_pointcloud_receive_time_ = now();
  has_last_pointcloud_stamp_ = true;
}

void PerceptionNode::lifecycleModeCallback(const std_msgs::msg::String::ConstSharedPtr & msg)
{
  if (!msg) {
    return;
  }

  const auto payload = msg->data;
  const auto mode_position = payload.find("mode=");
  if (mode_position == std::string::npos) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "ignore lifecycle mode payload without mode key: %s",
      payload.c_str());
    return;
  }

  const auto value_start = mode_position + 5;
  const auto value_end = payload.find(';', value_start);
  std::string mode_token = payload.substr(
    value_start,
    value_end == std::string::npos ? std::string::npos : value_end - value_start);

  std::transform(
    mode_token.begin(), mode_token.end(), mode_token.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});

  if (mode_token != "idle_spinning" && mode_token != "normal" && mode_token != "degraded") {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "ignore unknown lifecycle mode=%s payload=%s",
      mode_token.c_str(),
      payload.c_str());
    return;
  }

  const bool to_idle_spinning = (mode_token == "idle_spinning" || mode_token == "degraded");
  if (to_idle_spinning == idle_spinning_mode_) {
    return;
  }

  const auto switch_time = now();
  idle_spinning_mode_ = to_idle_spinning;

  if (idle_spinning_mode_) {
    ++idle_spinning_trigger_count_;
    has_mode_enter_time_ = true;
    mode_enter_time_ = switch_time;
    RCLCPP_WARN(get_logger(), "enter idle_spinning mode, payload=%s", payload.c_str());
    return;
  }

  if (has_mode_enter_time_) {
    const auto duration_ms = (switch_time - mode_enter_time_).nanoseconds() / 1000000;
    RCLCPP_INFO(
      get_logger(),
      "exit idle_spinning mode after %ld ms, payload=%s",
      duration_ms,
      payload.c_str());
    has_mode_enter_time_ = false;
  }
}

void PerceptionNode::watchdogCallback()
{
  const auto current_time = now();

  if (idle_spinning_mode_) {
    publishIdleSpinningPose(current_time);
    return;
  }

  std::string reason;
  if (!shouldTriggerExtrapolation(current_time, reason)) {
    return;
  }

  (void)publishExtrapolatedTarget(current_time, reason);
}

bool PerceptionNode::shouldTriggerExtrapolation(
  const rclcpp::Time & current_time,
  std::string & reason) const
{
  if (!has_last_image_stamp_ || !has_last_pointcloud_stamp_) {
    return false;
  }

  const auto image_gap_ms = (current_time - last_image_receive_time_).nanoseconds() / 1000000;
  const auto pointcloud_gap_ms = (current_time - last_pointcloud_receive_time_).nanoseconds() / 1000000;
  const auto stamp_delta_ms =
    std::llabs((last_image_stamp_ - last_pointcloud_stamp_).nanoseconds()) / 1000000;

  const bool one_side_stream_timeout =
    (image_gap_ms > single_side_dropout_timeout_ms_ &&
    pointcloud_gap_ms <= single_side_dropout_timeout_ms_) ||
    (pointcloud_gap_ms > single_side_dropout_timeout_ms_ &&
    image_gap_ms <= single_side_dropout_timeout_ms_);
  const bool stamp_skew_timeout = stamp_delta_ms > single_side_dropout_timeout_ms_;

  if (!one_side_stream_timeout && !stamp_skew_timeout) {
    return false;
  }

  if (has_last_extrapolation_pub_time_) {
    const auto since_last_pub_ms = (current_time - last_extrapolation_pub_time_).nanoseconds() / 1000000;
    if (since_last_pub_ms < extrapolation_min_interval_ms_) {
      return false;
    }
  }

  reason = one_side_stream_timeout ? "single_side_dropout" : "timestamp_skew";
  return true;
}

bool PerceptionNode::publishExtrapolatedTarget(
  const rclcpp::Time & current_time,
  const std::string & reason)
{
  if (pose_history_.empty()) {
    return false;
  }

  dog_interfaces::msg::Target3D extrapolated = pose_history_.back().target;
  extrapolated.header.stamp = current_time;
  extrapolated.target_id = "extrapolated_target";
  extrapolated.confidence = std::max(0.0F, extrapolated.confidence * 0.8F);

  if (pose_history_.size() >= 2) {
    const auto & previous = pose_history_[pose_history_.size() - 2];
    const auto & latest = pose_history_.back();
    const double dt_sec = static_cast<double>((latest.stamp - previous.stamp).nanoseconds()) / 1e9;
    if (dt_sec > 1e-6) {
      const double velocity_x = (latest.target.position.x - previous.target.position.x) / dt_sec;
      const double velocity_y = (latest.target.position.y - previous.target.position.y) / dt_sec;
      const double velocity_z = (latest.target.position.z - previous.target.position.z) / dt_sec;

      const auto bounded_ns = std::min<int64_t>(
        (current_time - latest.stamp).nanoseconds(),
        static_cast<int64_t>(extrapolation_max_window_ms_) * 1000 * 1000);
      const double prediction_dt_sec = std::max(0.0, static_cast<double>(bounded_ns) / 1e9);
      extrapolated.position.x = latest.target.position.x + velocity_x * prediction_dt_sec;
      extrapolated.position.y = latest.target.position.y + velocity_y * prediction_dt_sec;
      extrapolated.position.z = latest.target.position.z + velocity_z * prediction_dt_sec;
    }
  }

  dog_interfaces::msg::Target3DArray message;
  message.header = extrapolated.header;
  message.targets.push_back(extrapolated);
  target3d_pub_->publish(std::move(message));
  extrapolation_active_ = true;
  ++extrapolation_trigger_count_;
  has_last_extrapolation_pub_time_ = true;
  last_extrapolation_pub_time_ = current_time;

  RCLCPP_WARN_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "published extrapolated target due to %s, trigger_count=%zu",
    reason.c_str(),
    extrapolation_trigger_count_);
  return true;
}

void PerceptionNode::publishIdleSpinningPose(const rclcpp::Time & current_time)
{
  if (has_last_idle_publish_time_) {
    const auto elapsed_ms = (current_time - last_idle_publish_time_).nanoseconds() / 1000000;
    if (elapsed_ms < idle_spinning_publish_ms_) {
      return;
    }
  }

  dog_interfaces::msg::Target3D target;
  target.header.stamp = current_time;
  target.header.frame_id = camera_extrinsics_.frame_id;
  target.target_id = "idle_spinning";
  target.confidence = 0.0F;

  dog_interfaces::msg::Target3DArray message;
  message.header = target.header;
  message.targets.push_back(target);
  target3d_pub_->publish(std::move(message));
  has_last_idle_publish_time_ = true;
  last_idle_publish_time_ = current_time;
}

void PerceptionNode::processDigitRecognition(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  const auto begin = now();

  DigitRecognitionResult result;
  try {
    result = digit_recognizer_->infer(ImageView{image_msg});
  } catch (const std::exception & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Digit recognizer exception: %s",
      exception.what());
    result = DigitRecognitionResult{false, -1, 0.0F, "recognizer_exception"};
  }

  digit_label_history_.push_back(result.has_feature ? result.label : -1);

  if (result.has_feature) {
    int confirmed_count = 0;
    for (const int label : digit_label_history_) {
      if (label == result.label) {
        ++confirmed_count;
      }
    }

    if (confirmed_count < digit_temporal_confirm_count_) {
      result.has_feature = false;
      result.reason = "temporal_unconfirmed";
    }
  }

  auto message = toDigitTarget3D(image_msg, camera_extrinsics_.frame_id, result);
  dog_interfaces::msg::Target3DArray message_array;
  message_array.header = message.header;
  message_array.targets.push_back(message);
  digit_result_pub_->publish(std::move(message_array));

  const auto end = now();
  const double elapsed_ms = static_cast<double>((end - begin).nanoseconds()) / 1e6;
  digit_latency_samples_ms_.push_back(elapsed_ms);

  const auto input_stamp = image_msg ? rclcpp::Time(image_msg->header.stamp) : begin;
  const double input_age_ms = static_cast<double>((end - input_stamp).nanoseconds()) / 1e6;

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "Digit result stamp=%u.%u, infer_ms=%.3f, confidence=%.3f, output=%s, reason=%s, input_age_ms=%.3f",
    message.header.stamp.sec,
    message.header.stamp.nanosec,
    elapsed_ms,
    result.confidence,
    message.target_id.c_str(),
    result.reason.c_str(),
    input_age_ms);
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

size_t PerceptionNode::getDigitLatencySampleCount() const
{
  return digit_latency_samples_ms_.size();
}

size_t PerceptionNode::getExtrapolationTriggerCount() const
{
  return extrapolation_trigger_count_;
}

size_t PerceptionNode::getExtrapolationRecoveryCount() const
{
  return extrapolation_recovery_count_;
}

size_t PerceptionNode::getIdleSpinningTriggerCount() const
{
  return idle_spinning_trigger_count_;
}

bool PerceptionNode::isQosCompatible() const
{
  return qos_compatible_;
}

bool PerceptionNode::isIdleSpinningMode() const
{
  return idle_spinning_mode_;
}

double PerceptionNode::getLatencyP95Ms() const
{
  return percentile95(latency_samples_ms_);
}

double PerceptionNode::getEndToEndLatencyP95Ms() const
{
  return percentile95(end_to_end_latency_samples_ms_);
}

double PerceptionNode::getDigitLatencyP95Ms() const
{
  return percentile95(digit_latency_samples_ms_);
}

}  // namespace dog_perception