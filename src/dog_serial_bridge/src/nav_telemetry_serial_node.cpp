#include "dog_serial_bridge/nav_telemetry_serial_node.hpp"

#include "dog_serial_bridge/system_serial_connection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <utility>

namespace dog_serial_bridge
{

namespace
{

bool isFinitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool hasValidQuaternionNorm(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::isfinite(norm) && norm > 1.0e-6;
}

double yawFromPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool isValidFrameChar(const char value)
{
  return (value >= 'a' && value <= 'z') ||
         (value >= 'A' && value <= 'Z') ||
         (value >= '0' && value <= '9') ||
         value == '_' ||
         value == '-' ||
         value == '/';
}

std::string sanitizeFrameId(const std::string & frame_id)
{
  if (frame_id.empty()) {
    return "unknown";
  }

  std::string sanitized;
  sanitized.reserve(std::min<size_t>(frame_id.size(), 48U));
  for (const char value : frame_id) {
    sanitized.push_back(isValidFrameChar(value) ? value : '_');
    if (sanitized.size() >= 48U) {
      break;
    }
  }
  return sanitized.empty() ? "unknown" : sanitized;
}

void appendPoseFields(
  std::ostringstream & stream,
  const char * prefix,
  const NavTelemetrySerialNode::PoseCache & cache)
{
  stream << ';' << prefix << "_valid=" << (cache.valid ? 1 : 0);
  stream << ';' << prefix << "_frame=" << sanitizeFrameId(cache.pose.header.frame_id);
  if (!cache.valid) {
    stream << ';' << prefix << "_x=0.000"
           << ';' << prefix << "_y=0.000"
           << ';' << prefix << "_z=0.000"
           << ';' << prefix << "_yaw=0.000";
    return;
  }

  stream << ';' << prefix << "_x=" << cache.pose.pose.position.x
         << ';' << prefix << "_y=" << cache.pose.pose.position.y
         << ';' << prefix << "_z=" << cache.pose.pose.position.z
         << ';' << prefix << "_yaw=" << yawFromPose(cache.pose.pose);
}

}  // namespace

NavTelemetrySerialNode::NavTelemetrySerialNode(
  const rclcpp::NodeOptions & options,
  std::shared_ptr<SerialConnection> serial_connection)
: rclcpp::Node("dog_serial_bridge_nav_telemetry", options)
, serial_connection_(serial_connection ? std::move(serial_connection) : std::make_shared<SystemSerialConnection>())
, serial_ready_(false)
, write_newline_(true)
, publish_period_ms_(100)
, reconnect_period_ms_(1000)
, sequence_(0U)
{
  declareParameters();
  loadParameters();

  const auto current_pose_topic = declare_parameter<std::string>("current_pose_topic", "/dog/global_pose");
  const auto goal_pose_topic = declare_parameter<std::string>("goal_pose_topic", "/behavior/nav_goal");

  auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(20));
  pose_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic,
    pose_qos,
    std::bind(&NavTelemetrySerialNode::currentPoseCallback, this, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic,
    pose_qos,
    std::bind(&NavTelemetrySerialNode::goalPoseCallback, this, std::placeholders::_1));

  initializeSerial();

  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_),
    std::bind(&NavTelemetrySerialNode::publishTimerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "NavTelemetrySerialNode initialized, current_pose_topic=%s, goal_pose_topic=%s, serial_port=%s, baud=%d, period_ms=%d",
    current_pose_topic.c_str(),
    goal_pose_topic.c_str(),
    serial_config_.port.c_str(),
    serial_config_.baud_rate,
    publish_period_ms_);
}

NavTelemetrySerialNode::~NavTelemetrySerialNode()
{
  closeSerial();
}

void NavTelemetrySerialNode::declareParameters()
{
  declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
  declare_parameter<int>("baud_rate", 115200);
  declare_parameter<int>("publish_period_ms", 100);
  declare_parameter<int>("reconnect_period_ms", 1000);
  declare_parameter<bool>("write_newline", true);
  declare_parameter<std::string>("read_line_delimiter", "\\n");
}

void NavTelemetrySerialNode::loadParameters()
{
  serial_config_.port = get_parameter("serial_port").as_string();
  serial_config_.baud_rate = get_parameter("baud_rate").as_int();
  publish_period_ms_ = get_parameter("publish_period_ms").as_int();
  reconnect_period_ms_ = get_parameter("reconnect_period_ms").as_int();
  write_newline_ = get_parameter("write_newline").as_bool();
  serial_config_.line_delimiter = decodeDelimiter(get_parameter("read_line_delimiter").as_string());

  if (publish_period_ms_ <= 0) {
    RCLCPP_WARN(get_logger(), "Invalid publish_period_ms=%d, fallback to 100", publish_period_ms_);
    publish_period_ms_ = 100;
  }
  if (reconnect_period_ms_ <= 0) {
    RCLCPP_WARN(get_logger(), "Invalid reconnect_period_ms=%d, fallback to 1000", reconnect_period_ms_);
    reconnect_period_ms_ = 1000;
  }
}

char NavTelemetrySerialNode::decodeDelimiter(const std::string & text) const
{
  if (text == "\\n") {
    return '\n';
  }
  if (text == "\\r") {
    return '\r';
  }
  if (text == "\\t") {
    return '\t';
  }
  if (text.empty()) {
    return '\n';
  }
  return text.front();
}

void NavTelemetrySerialNode::initializeSerial()
{
  std::string error;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_ready_ = serial_connection_ && serial_connection_->open(serial_config_, error);
    serial_error_ = serial_ready_ ? "" : (error.empty() ? "serial_open_failed" : error);
    next_reconnect_time_ = now() + rclcpp::Duration::from_seconds(reconnect_period_ms_ / 1000.0);
  }

  if (!serial_ready_) {
    RCLCPP_ERROR(
      get_logger(),
      "nav_telemetry_serial_open_failed port=%s baud=%d detail=%s",
      serial_config_.port.c_str(),
      serial_config_.baud_rate,
      serial_error_.c_str());
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "nav_telemetry_serial_ready port=%s baud=%d",
    serial_config_.port.c_str(),
    serial_config_.baud_rate);
}

void NavTelemetrySerialNode::closeSerial()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_connection_) {
    serial_connection_->close();
  }
  serial_ready_ = false;
}

bool NavTelemetrySerialNode::isSerialReady() const
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return serial_ready_;
}

void NavTelemetrySerialNode::markSerialNotReady(const std::string & detail)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  serial_ready_ = false;
  serial_error_ = detail;
  if (serial_connection_) {
    serial_connection_->close();
  }
  next_reconnect_time_ = now() + rclcpp::Duration::from_seconds(reconnect_period_ms_ / 1000.0);
}

void NavTelemetrySerialNode::currentPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!msg || !isFinitePose(msg->pose) || !hasValidQuaternionNorm(msg->pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop invalid current pose for nav telemetry");
    return;
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_pose_.valid = true;
  current_pose_.pose = *msg;
}

void NavTelemetrySerialNode::goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!msg || !isFinitePose(msg->pose) || !hasValidQuaternionNorm(msg->pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop invalid goal pose for nav telemetry");
    return;
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);
  goal_pose_.valid = true;
  goal_pose_.pose = *msg;
}

void NavTelemetrySerialNode::publishTimerCallback()
{
  if (!isSerialReady() && !maybeReconnect()) {
    return;
  }

  PoseCache current;
  PoseCache goal;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current = current_pose_;
    goal = goal_pose_;
  }

  const auto frame = appendConfiguredNewline(buildFrame(now(), current, goal));
  std::shared_ptr<SerialConnection> serial_connection;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_ready_ || !serial_connection_) {
      return;
    }
    serial_connection = serial_connection_;
  }

  std::string error;
  if (!serial_connection->write(frame, error)) {
    const auto detail = error.empty() ? "nav_telemetry_write_failed" : error;
    RCLCPP_ERROR(get_logger(), "nav_telemetry_write_failed detail=%s", detail.c_str());
    markSerialNotReady(detail);
  }
}

bool NavTelemetrySerialNode::maybeReconnect()
{
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_ready_) {
      return true;
    }
    if (now() < next_reconnect_time_) {
      return false;
    }
  }

  initializeSerial();
  return isSerialReady();
}

std::string NavTelemetrySerialNode::buildFrame(
  const rclcpp::Time & stamp,
  const PoseCache & current,
  const PoseCache & goal)
{
  ++sequence_;

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3);
  stream << "RCNAV;seq=" << sequence_
         << ";stamp_ms=" << (stamp.nanoseconds() / 1000000);
  appendPoseFields(stream, "cur", current);
  appendPoseFields(stream, "goal", goal);
  return stream.str();
}

std::string NavTelemetrySerialNode::appendConfiguredNewline(const std::string & frame) const
{
  if (!write_newline_) {
    return frame;
  }
  return frame + '\n';
}

}  // namespace dog_serial_bridge
