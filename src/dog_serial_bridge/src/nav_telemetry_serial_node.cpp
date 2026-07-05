#include "dog_serial_bridge/nav_telemetry_serial_node.hpp"

#include "dog_serial_bridge/serial_protocol.hpp"
#include "dog_serial_bridge/system_serial_connection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>

namespace dog_serial_bridge
{

namespace
{

constexpr double kMetersToCentimeters = 100.0;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

std::string escapeSerialPayload(const std::string & payload)
{
  std::string escaped;
  escaped.reserve(payload.size());
  for (const char value : payload) {
    switch (value) {
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped.push_back(value);
        break;
    }
  }
  return escaped;
}

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

double normalizeAngleRadians(double angle)
{
  while (angle > 3.14159265358979323846) {
    angle -= 2.0 * 3.14159265358979323846;
  }
  while (angle < -3.14159265358979323846) {
    angle += 2.0 * 3.14159265358979323846;
  }
  return angle;
}

void appendPoseFields(
  std::ostringstream & stream,
  const char * prefix,
  const NavTelemetrySerialNode::PoseCache & cache)
{
  stream << ';' << prefix << "_valid=" << (cache.valid ? 1 : 0);
  if (!cache.valid) {
    stream << ';' << prefix << "_x=0.000"
           << ';' << prefix << "_y=0.000"
           << ';' << prefix << "_yaw=0.000";
    return;
  }

  stream << ';' << prefix << "_x=" << cache.pose.pose.position.x * kMetersToCentimeters
         << ';' << prefix << "_y=" << cache.pose.pose.position.y * kMetersToCentimeters
         << ';' << prefix << "_yaw=" << yawFromPose(cache.pose.pose) * kRadiansToDegrees;
}

}  // namespace

NavTelemetrySerialNode::NavTelemetrySerialNode(
  const rclcpp::NodeOptions & options,
  std::shared_ptr<SerialConnection> serial_connection)
: rclcpp::Node("dog_serial_bridge_nav_telemetry", options)
, serial_connection_(serial_connection ? std::move(serial_connection) : std::make_shared<SystemSerialConnection>())
, serial_ready_(false)
, write_newline_(true)
, ack_timeout_ms_(10000)
, reconnect_period_ms_(1000)
, continuous_send_enabled_(true)
, continuous_send_period_ms_(100)
, arrival_check_mode_("host_pose")
, arrival_xy_tolerance_m_(0.15)
, arrival_yaw_tolerance_deg_(10.0)
, arrival_check_period_ms_(50)
, goal_reserved_(false)
, send_shutdown_arrival_on_exit_(false)
, shutdown_arrival_repeat_count_(1)
, timeout_stop_repeat_count_(3)
, timeout_stop_interval_ms_(20)
, shutdown_arrival_sent_(false)
{
  declareParameters();
  loadParameters();

  const auto current_pose_topic = declare_parameter<std::string>("current_pose_topic", "/dog/global_pose");
  const auto goal_pose_topic = declare_parameter<std::string>("goal_pose_topic", "/behavior/nav_goal");
  const auto action_name = declare_parameter<std::string>("action_name", "/behavior/nav_execute");

  auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(20));
  pose_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic,
    pose_qos,
    std::bind(&NavTelemetrySerialNode::currentPoseCallback, this, std::placeholders::_1));
  goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, pose_qos);

  initializeSerial();

  if (continuous_send_enabled_) {
    continuous_send_timer_ = create_wall_timer(
      std::chrono::milliseconds(continuous_send_period_ms_),
      std::bind(&NavTelemetrySerialNode::continuousSendTimerCallback, this));
  }

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<NavigateWaypoint>(
    this,
    action_name,
    std::bind(&NavTelemetrySerialNode::handleGoal, this, _1, _2),
    std::bind(&NavTelemetrySerialNode::handleCancel, this, _1),
    std::bind(&NavTelemetrySerialNode::handleAccepted, this, _1));

  RCLCPP_INFO(
    get_logger(),
    "NavTelemetrySerialNode initialized, action_name=%s, current_pose_topic=%s, goal_pose_topic=%s, serial_port=%s, baud=%d, ack_timeout_ms=%d, continuous_send_enabled=%d, continuous_send_period_ms=%d",
    action_name.c_str(),
    current_pose_topic.c_str(),
    goal_pose_topic.c_str(),
    serial_config_.port.c_str(),
    serial_config_.baud_rate,
    ack_timeout_ms_,
    continuous_send_enabled_ ? 1 : 0,
    continuous_send_period_ms_);
}

NavTelemetrySerialNode::~NavTelemetrySerialNode()
{
  if (continuous_send_timer_) {
    continuous_send_timer_->cancel();
  }
  closeSerial();
}

void NavTelemetrySerialNode::declareParameters()
{
  declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  declare_parameter<int>("baud_rate", 115200);
  declare_parameter<int>("ack_timeout_ms", 10000);
  declare_parameter<int>("reconnect_period_ms", 1000);
  declare_parameter<bool>("continuous_send_enabled", true);
  declare_parameter<int>("continuous_send_period_ms", 100);
  declare_parameter<std::string>("arrival_check_mode", "host_pose");
  declare_parameter<double>("arrival_xy_tolerance_m", 0.15);
  declare_parameter<double>("arrival_yaw_tolerance_deg", 10.0);
  declare_parameter<int>("arrival_check_period_ms", 50);
  declare_parameter<bool>("write_newline", true);
  declare_parameter<bool>("send_shutdown_arrival_on_exit", false);
  declare_parameter<int>("shutdown_arrival_repeat_count", 1);
  declare_parameter<int>("timeout_stop_repeat_count", 3);
  declare_parameter<int>("timeout_stop_interval_ms", 20);
  declare_parameter<std::string>("read_line_delimiter", "\\n");
}

void NavTelemetrySerialNode::loadParameters()
{
  serial_config_.port = get_parameter("serial_port").as_string();
  serial_config_.baud_rate = get_parameter("baud_rate").as_int();
  ack_timeout_ms_ = get_parameter("ack_timeout_ms").as_int();
  reconnect_period_ms_ = get_parameter("reconnect_period_ms").as_int();
  continuous_send_enabled_ = get_parameter("continuous_send_enabled").as_bool();
  continuous_send_period_ms_ = get_parameter("continuous_send_period_ms").as_int();
  arrival_check_mode_ = get_parameter("arrival_check_mode").as_string();
  arrival_xy_tolerance_m_ = get_parameter("arrival_xy_tolerance_m").as_double();
  arrival_yaw_tolerance_deg_ = get_parameter("arrival_yaw_tolerance_deg").as_double();
  arrival_check_period_ms_ = get_parameter("arrival_check_period_ms").as_int();
  write_newline_ = get_parameter("write_newline").as_bool();
  send_shutdown_arrival_on_exit_ = get_parameter("send_shutdown_arrival_on_exit").as_bool();
  shutdown_arrival_repeat_count_ = get_parameter("shutdown_arrival_repeat_count").as_int();
  timeout_stop_repeat_count_ = get_parameter("timeout_stop_repeat_count").as_int();
  timeout_stop_interval_ms_ = get_parameter("timeout_stop_interval_ms").as_int();
  serial_config_.line_delimiter = decodeDelimiter(get_parameter("read_line_delimiter").as_string());

  if (ack_timeout_ms_ <= 0) {
    RCLCPP_WARN(get_logger(), "Invalid ack_timeout_ms=%d, fallback to 10000", ack_timeout_ms_);
    ack_timeout_ms_ = 10000;
  }
  if (reconnect_period_ms_ <= 0) {
    RCLCPP_WARN(get_logger(), "Invalid reconnect_period_ms=%d, fallback to 1000", reconnect_period_ms_);
    reconnect_period_ms_ = 1000;
  }
  if (continuous_send_period_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid continuous_send_period_ms=%d, fallback to 100",
      continuous_send_period_ms_);
    continuous_send_period_ms_ = 100;
  }
  if (arrival_check_mode_ != "host_pose" && arrival_check_mode_ != "serial_reply") {
    RCLCPP_WARN(
      get_logger(),
      "Invalid arrival_check_mode=%s, fallback to host_pose",
      arrival_check_mode_.c_str());
    arrival_check_mode_ = "host_pose";
  }
  if (!std::isfinite(arrival_xy_tolerance_m_) || arrival_xy_tolerance_m_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid arrival_xy_tolerance_m=%.3f, fallback to 0.15",
      arrival_xy_tolerance_m_);
    arrival_xy_tolerance_m_ = 0.15;
  }
  if (!std::isfinite(arrival_yaw_tolerance_deg_) || arrival_yaw_tolerance_deg_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid arrival_yaw_tolerance_deg=%.3f, fallback to 10.0",
      arrival_yaw_tolerance_deg_);
    arrival_yaw_tolerance_deg_ = 10.0;
  }
  if (arrival_check_period_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid arrival_check_period_ms=%d, fallback to 50",
      arrival_check_period_ms_);
    arrival_check_period_ms_ = 50;
  }
  if (shutdown_arrival_repeat_count_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid shutdown_arrival_repeat_count=%d, fallback to 1",
      shutdown_arrival_repeat_count_);
    shutdown_arrival_repeat_count_ = 1;
  }
  if (timeout_stop_repeat_count_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid timeout_stop_repeat_count=%d, fallback to 3",
      timeout_stop_repeat_count_);
    timeout_stop_repeat_count_ = 3;
  }
  if (timeout_stop_interval_ms_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid timeout_stop_interval_ms=%d, fallback to 20",
      timeout_stop_interval_ms_);
    timeout_stop_interval_ms_ = 20;
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

bool NavTelemetrySerialNode::isValidGoal(const geometry_msgs::msg::PoseStamped & goal) const
{
  return isFinitePose(goal.pose) && hasValidQuaternionNorm(goal.pose);
}

void NavTelemetrySerialNode::publishGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  if (goal_pose_pub_) {
    goal_pose_pub_->publish(goal);
  }
}

void NavTelemetrySerialNode::continuousSendTimerCallback()
{
  PoseCache current;
  PoseCache goal;
  if (!getContinuousTelemetrySnapshot(current, goal)) {
    return;
  }

  const auto frame = appendConfiguredNewline(buildFrame(now(), current, goal, "continuous"));
  std::string detail;
  if (!writeFrame(frame, "nav_telemetry_continuous_tx", detail)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "nav_telemetry_continuous_tx_failed detail=%s",
      detail.c_str());
  }
}

void NavTelemetrySerialNode::updateLastGoalPose(const PoseCache & goal)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  last_goal_pose_ = goal;
}

bool NavTelemetrySerialNode::getContinuousTelemetrySnapshot(PoseCache & current, PoseCache & goal) const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!current_pose_.valid || !last_goal_pose_.valid) {
    return false;
  }
  current = current_pose_;
  goal = last_goal_pose_;
  return true;
}

bool NavTelemetrySerialNode::writeFrame(
  const std::string & frame,
  const char * log_prefix,
  std::string & detail)
{
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  if (!isSerialReady() && !maybeReconnect()) {
    detail = "serial_not_ready";
    return false;
  }

  std::shared_ptr<SerialConnection> serial_connection;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_ready_ || !serial_connection_) {
      detail = "serial_not_ready";
      return false;
    }
    serial_connection = serial_connection_;
  }

  RCLCPP_INFO(
    get_logger(),
    "%s port=%s data=%s",
    log_prefix,
    serial_config_.port.c_str(),
    escapeSerialPayload(frame).c_str());

  std::string error;
  if (!serial_connection->write(frame, error)) {
    detail = error.empty() ? "serial_write_error" : "serial_write_error:" + error;
    markSerialNotReady(detail);
    return false;
  }

  detail.clear();
  return true;
}

rclcpp_action::GoalResponse NavTelemetrySerialNode::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateWaypoint::Goal> goal)
{
  (void)uuid;
  if (!goal || !isValidGoal(goal->target_pose)) {
    RCLCPP_WARN(get_logger(), "nav_goal_rejected invalid_target_pose");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!reserveGoalSlot()) {
    RCLCPP_WARN(get_logger(), "nav_goal_rejected_busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavTelemetrySerialNode::handleCancel(const std::shared_ptr<GoalHandle>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavTelemetrySerialNode::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto self = std::static_pointer_cast<NavTelemetrySerialNode>(shared_from_this());
  std::thread([self, goal_handle]() { self->executeGoal(goal_handle); }).detach();
}

void NavTelemetrySerialNode::executeGoal(const std::shared_ptr<GoalHandle> goal_handle)
{
  if (!goal_handle) {
    releaseGoalSlot();
    return;
  }

  auto feedback = std::make_shared<NavigateWaypoint::Feedback>();
  feedback->progress = 0.1F;
  feedback->state = "sending_goal";
  goal_handle->publish_feedback(feedback);

  const auto goal_pose = goal_handle->get_goal()->target_pose;
  publishGoal(goal_pose);

  PoseCache current;
  PoseCache goal;
  goal.valid = true;
  goal.pose = goal_pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current = current_pose_;
  }

  const auto frame = appendConfiguredNewline(buildFrame(now(), current, goal));
  std::string error;
  if (!writeFrame(frame, "nav_telemetry_serial_tx", error)) {
    RCLCPP_ERROR(get_logger(), "nav_goal_write_failed detail=%s", error.c_str());
    releaseGoalSlot();
    auto result = std::make_shared<NavigateWaypoint::Result>();
    result->accepted = false;
    result->detail = error;
    goal_handle->abort(result);
    return;
  }

  updateLastGoalPose(goal);

  feedback->progress = 0.5F;
  feedback->state = "waiting_arrival";
  goal_handle->publish_feedback(feedback);

  std::string detail;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ack_timeout_ms_);
  const bool arrived = waitForArrival(deadline, goal, detail);
  if (!arrived && detail.rfind("arrival_timeout", 0) == 0) {
    const bool stop_sent = sendCurrentPoseStopFrames(
      "timeout_stop",
      "nav_timeout_stop",
      timeout_stop_repeat_count_);
    detail += stop_sent ? ";stop_sent=1" : ";stop_sent=0";
  }
  releaseGoalSlot();

  auto result = std::make_shared<NavigateWaypoint::Result>();
  result->accepted = arrived;
  result->detail = detail;

  if (goal_handle->is_canceling()) {
    result->accepted = false;
    result->detail = "goal_canceled";
    goal_handle->canceled(result);
    return;
  }

  if (arrived) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
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
  const PoseCache & goal,
  const std::string & event)
{
  (void)stamp;
  (void)event;

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3);
  stream << "RCNAV";
  appendPoseFields(stream, "cur", current);
  appendPoseFields(stream, "goal", goal);
  return stream.str();
}

std::string NavTelemetrySerialNode::appendConfiguredNewline(const std::string & frame) const
{
  if (!write_newline_) {
    return frame;
  }
  return frame + "\r\n";
}

bool NavTelemetrySerialNode::reserveGoalSlot()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (goal_reserved_) {
    return false;
  }
  goal_reserved_ = true;
  return true;
}

void NavTelemetrySerialNode::releaseGoalSlot()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  goal_reserved_ = false;
}

void NavTelemetrySerialNode::SendShutdownArrivalFrame()
{
  if (!send_shutdown_arrival_on_exit_ || shutdown_arrival_sent_) {
    return;
  }
  shutdown_arrival_sent_ = true;

  sendCurrentPoseStopFrames("shutdown_arrived", "nav_shutdown_arrival", shutdown_arrival_repeat_count_);
}

bool NavTelemetrySerialNode::sendCurrentPoseStopFrames(
  const std::string & event,
  const char * log_prefix,
  const int repeat_count)
{
  std::shared_ptr<SerialConnection> serial;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_ready_ || !serial_connection_) {
      RCLCPP_INFO(get_logger(), "%s_skip serial_not_ready", log_prefix);
      return false;
    }
    serial = serial_connection_;
  }

  PoseCache current;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current = current_pose_;
  }

  if (!current.valid) {
    RCLCPP_INFO(get_logger(), "%s_skip no_current_pose", log_prefix);
    return false;
  }

  updateLastGoalPose(current);

  bool any_sent = false;
  for (int attempt = 1; attempt <= repeat_count; ++attempt) {
    const auto frame = appendConfiguredNewline(buildFrame(now(), current, current, event));
    std::string error;
    RCLCPP_INFO(
      get_logger(),
      "%s_tx port=%s attempt=%d/%d data=%s",
      log_prefix,
      serial_config_.port.c_str(),
      attempt,
      repeat_count,
      escapeSerialPayload(frame).c_str());
    {
      std::lock_guard<std::mutex> write_lock(write_mutex_);
      if (!serial->write(frame, error)) {
        RCLCPP_WARN(
          get_logger(),
          "%s_tx_failed attempt=%d/%d detail=%s",
          log_prefix,
          attempt,
          repeat_count,
          error.empty() ? "write_error" : error.c_str());
      } else {
        any_sent = true;
        RCLCPP_INFO(
          get_logger(),
          "%s_tx_done attempt=%d/%d",
          log_prefix,
          attempt,
          repeat_count);
      }
    }

    if (timeout_stop_interval_ms_ > 0 && attempt < repeat_count) {
      std::this_thread::sleep_for(std::chrono::milliseconds(timeout_stop_interval_ms_));
    }
  }

  return any_sent;
}

bool NavTelemetrySerialNode::hasHostPoseArrived(const PoseCache & goal, std::string & detail) const
{
  PoseCache current;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current = current_pose_;
  }

  if (!current.valid) {
    detail = "arrival_timeout;reason=no_current_pose";
    return false;
  }

  const double dx = current.pose.pose.position.x - goal.pose.pose.position.x;
  const double dy = current.pose.pose.position.y - goal.pose.pose.position.y;
  const double xy_distance = std::sqrt(dx * dx + dy * dy);
  const double yaw_error_deg = std::abs(
    normalizeAngleRadians(yawFromPose(current.pose.pose) - yawFromPose(goal.pose.pose)) *
    kRadiansToDegrees);

  if (xy_distance <= arrival_xy_tolerance_m_ && yaw_error_deg <= arrival_yaw_tolerance_deg_) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "host_arrival_ok;xy_error_m=" << xy_distance
           << ";yaw_error_deg=" << yaw_error_deg;
    detail = stream.str();
    return true;
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "arrival_timeout;xy_error_m=" << xy_distance
         << ";yaw_error_deg=" << yaw_error_deg;
  detail = stream.str();
  return false;
}

bool NavTelemetrySerialNode::waitForArrival(
  const std::chrono::steady_clock::time_point & deadline,
  const PoseCache & goal,
  std::string & detail)
{
  std::string receive_buffer;
  std::string last_observed;
  size_t debug_frame_count = 0U;
  while (std::chrono::steady_clock::now() < deadline) {
    if (arrival_check_mode_ == "host_pose" && hasHostPoseArrived(goal, detail)) {
      return true;
    }

    std::shared_ptr<SerialConnection> serial_connection;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (!serial_ready_ || !serial_connection_) {
        detail = "serial_not_ready";
        return false;
      }
      serial_connection = serial_connection_;
    }

    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    const auto read_timeout = std::min(
      std::chrono::milliseconds(arrival_check_period_ms_),
      remaining);
    const auto read_result = serial_connection->readBytes(read_timeout);
    if (read_result.status == SerialConnection::ReadStatus::kTimeout) {
      continue;
    }
    if (read_result.status == SerialConnection::ReadStatus::kLine && !read_result.data.empty()) {
      receive_buffer += read_result.data;
      last_observed = read_result.data;
      debug_frame_count += eraseCompleteDebugFrames(receive_buffer);
      RCLCPP_INFO(
        get_logger(),
        "nav_telemetry_serial_rx port=%s bytes=%zu data=%s",
        serial_config_.port.c_str(),
        read_result.data.size(),
        escapeSerialPayload(read_result.data).c_str());
      if (arrival_check_mode_ == "serial_reply" && containsArrivalReply(receive_buffer)) {
        serial_connection->clearReadBuffer();
        detail = "arrival_ok";
        return true;
      }
      if (receive_buffer.size() > 512U) {
        receive_buffer.erase(0, receive_buffer.size() - 64U);
      }
      continue;
    }

    detail = read_result.error.empty() ? "serial_read_error" : "serial_read_error:" + read_result.error;
    markSerialNotReady(detail);
    return false;
  }

  if (arrival_check_mode_ == "host_pose") {
    std::string host_detail;
    hasHostPoseArrived(goal, host_detail);
    detail = host_detail.empty() ? "arrival_timeout" : host_detail;
  } else {
    detail = "arrival_timeout";
  }
  if (!last_observed.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "nav_arrival_timeout debug_frames=%zu last_observed=%s",
      debug_frame_count,
      escapeSerialPayload(last_observed).c_str());
  } else {
    RCLCPP_WARN(get_logger(), "nav_arrival_timeout debug_frames=%zu no_serial_reply", debug_frame_count);
  }
  return false;
}

}  // namespace dog_serial_bridge
