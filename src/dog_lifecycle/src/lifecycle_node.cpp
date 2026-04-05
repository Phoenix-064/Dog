#include "dog_lifecycle/lifecycle_node.hpp"

#include "dog_lifecycle/yaml_state_store.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace dog_lifecycle
{

namespace
{

/// @brief 对字符串进行百分号编码，适用于键值负载安全传输。
/// @param value 原始字符串。
/// @return 百分号编码后的字符串。
std::string percentEncode(const std::string & value)
{
  static constexpr char kHex[] = "0123456789ABCDEF";
  std::string encoded;
  encoded.reserve(value.size());

  for (const auto byte_value : value) {
    const auto c = static_cast<unsigned char>(byte_value);
    if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded.push_back(static_cast<char>(c));
      continue;
    }

    encoded.push_back('%');
    encoded.push_back(kHex[(c >> 4) & 0x0F]);
    encoded.push_back(kHex[c & 0x0F]);
  }

  return encoded;
}

}  // namespace

LifecycleNode::LifecycleNode()
: LifecycleNode(rclcpp::NodeOptions())
{
}

LifecycleNode::LifecycleNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dog_lifecycle", options)
{
  const auto recover_total_start = std::chrono::steady_clock::now();
  const auto state_file_path = declare_parameter<std::string>(
    "persistence.state_file_path", "/tmp/dog_lifecycle/state.yaml");
  const auto backup_file_path = declare_parameter<std::string>(
    "persistence.backup_file_path", "/tmp/dog_lifecycle/state.bak.yaml");
  const auto max_write_bytes = declare_parameter<int64_t>("persistence.max_write_bytes", 16384);
  const auto min_write_interval_ms = declare_parameter<int64_t>(
    "persistence.min_write_interval_ms", 100);
  const auto supported_version = declare_parameter<int64_t>("persistence.state_version", 1);
  empty_grasp_threshold_ = declare_parameter<int64_t>("empty_grasp_threshold", 2);
  degrade_timeout_ms_ = declare_parameter<int64_t>("degrade_timeout_ms", 800);
  breaker_reset_window_ms_ = declare_parameter<int64_t>("breaker_reset_window_ms", 3000);
  feedback_dedup_window_ms_ = declare_parameter<int64_t>("feedback_dedup_window_ms", 10);
  heartbeat_timeout_ms_ = declare_parameter<int64_t>("heartbeat_timeout_ms", 2000);
  heartbeat_check_period_ms_ = declare_parameter<int64_t>("heartbeat_check_period_ms", 100);
  reconnect_min_interval_ms_ = declare_parameter<int64_t>("reconnect_min_interval_ms", 500);
  reconnect_pending_timeout_ms_ = declare_parameter<int64_t>("reconnect_pending_timeout_ms", 0);
  max_restart_attempts_ = declare_parameter<int64_t>("max_restart_attempts", 3);
  restart_window_ms_ = declare_parameter<int64_t>("restart_window_ms", 10000);
  valid_frame_recovery_consecutive_ = declare_parameter<int64_t>("valid_frame_recovery_consecutive", 2);
  grasp_feedback_topic_ = declare_parameter<std::string>("grasp_feedback_topic", "/behavior/grasp_feedback");
  degrade_command_topic_ = declare_parameter<std::string>("degrade_command_topic", "/lifecycle/degrade_command");
  degrade_ack_topic_ = declare_parameter<std::string>("degrade_ack_topic", "/lifecycle/degrade_ack");
  recovery_context_topic_ = declare_parameter<std::string>(
    "recovery_context_topic", "/lifecycle/recovery_context");
  estop_topic_ = declare_parameter<std::string>("estop_topic", "/system/estop");
  system_mode_topic_ = declare_parameter<std::string>("system_mode_topic", "/lifecycle/system_mode");
  valid_frame_topic_ = declare_parameter<std::string>("valid_frame_topic", "/perception/target3d");
  lifecycle_transition_topic_ = declare_parameter<std::string>(
    "lifecycle_transition_topic", "/lifecycle/transition_command");
  lifecycle_transition_status_topic_ = declare_parameter<std::string>(
    "lifecycle_transition_status_topic", "/lifecycle/transition_status");
  health_alarm_topic_ = declare_parameter<std::string>("health_alarm_topic", "/lifecycle/health_alarm");
  monitored_node_name_ = declare_parameter<std::string>("monitored_node_name", "dog_perception");

  int64_t validated_max_write_bytes = max_write_bytes;
  if (validated_max_write_bytes <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid persistence.max_write_bytes=%ld, fallback to 16384",
      validated_max_write_bytes);
    validated_max_write_bytes = 16384;
  }

  int64_t validated_min_write_interval_ms = min_write_interval_ms;
  if (validated_min_write_interval_ms < 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid persistence.min_write_interval_ms=%ld, fallback to 100",
      validated_min_write_interval_ms);
    validated_min_write_interval_ms = 100;
  }

  int64_t validated_version = supported_version;
  if (validated_version <= 0 || validated_version > static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
    RCLCPP_WARN(
      get_logger(),
      "invalid persistence.state_version=%ld, fallback to 1",
      validated_version);
    validated_version = 1;
  }

  supported_state_version_ = static_cast<uint32_t>(validated_version);

  if (empty_grasp_threshold_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid empty_grasp_threshold=%ld, fallback to 2",
      empty_grasp_threshold_);
    empty_grasp_threshold_ = 2;
  }

  if (degrade_timeout_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid degrade_timeout_ms=%ld, fallback to 800",
      degrade_timeout_ms_);
    degrade_timeout_ms_ = 800;
  }

  if (breaker_reset_window_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid breaker_reset_window_ms=%ld, fallback to 3000",
      breaker_reset_window_ms_);
    breaker_reset_window_ms_ = 3000;
  }

  if (feedback_dedup_window_ms_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid feedback_dedup_window_ms=%ld, fallback to 10",
      feedback_dedup_window_ms_);
    feedback_dedup_window_ms_ = 10;
  }

  if (heartbeat_timeout_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid heartbeat_timeout_ms=%ld, fallback to 2000",
      heartbeat_timeout_ms_);
    heartbeat_timeout_ms_ = 2000;
  }

  if (heartbeat_check_period_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid heartbeat_check_period_ms=%ld, fallback to 100",
      heartbeat_check_period_ms_);
    heartbeat_check_period_ms_ = 100;
  }

  if (reconnect_min_interval_ms_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid reconnect_min_interval_ms=%ld, fallback to 500",
      reconnect_min_interval_ms_);
    reconnect_min_interval_ms_ = 500;
  }

  if (max_restart_attempts_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid max_restart_attempts=%ld, fallback to 3",
      max_restart_attempts_);
    max_restart_attempts_ = 3;
  }

  if (restart_window_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid restart_window_ms=%ld, fallback to 10000",
      restart_window_ms_);
    restart_window_ms_ = 10000;
  }

  if (reconnect_pending_timeout_ms_ <= 0) {
    const auto adaptive_timeout_ms = std::max<int64_t>(heartbeat_timeout_ms_ * 4, 300);
    // Keep pending timeout within restart window so attempts can accumulate for degrade.
    const auto window_bound_timeout_ms = std::max<int64_t>(restart_window_ms_ - 1, 1);
    reconnect_pending_timeout_ms_ = std::min<int64_t>(adaptive_timeout_ms, window_bound_timeout_ms);
  }

  if (valid_frame_recovery_consecutive_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "invalid valid_frame_recovery_consecutive=%ld, fallback to 2",
      valid_frame_recovery_consecutive_);
    valid_frame_recovery_consecutive_ = 2;
  }

  StateStoreConfig config;
  config.state_file_path = state_file_path;
  config.backup_file_path = backup_file_path;
  config.max_write_bytes = static_cast<size_t>(validated_max_write_bytes);
  config.min_write_interval = std::chrono::milliseconds(validated_min_write_interval_ms);
  config.supported_version = supported_state_version_;

  state_store_ = std::make_unique<YamlStateStore>(config);

  recovery_context_pub_ = create_publisher<std_msgs::msg::String>(
    recovery_context_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal));
  system_mode_pub_ = create_publisher<std_msgs::msg::String>(
    system_mode_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal));
  lifecycle_transition_pub_ = create_publisher<std_msgs::msg::String>(
    lifecycle_transition_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  health_alarm_pub_ = create_publisher<std_msgs::msg::String>(
    health_alarm_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  const auto load_start = std::chrono::steady_clock::now();
  const auto load_result = state_store_->Load();
  const auto load_end = std::chrono::steady_clock::now();
  const auto map_start = std::chrono::steady_clock::now();
  if (load_result.result.ok && load_result.state.has_value()) {
    RCLCPP_INFO(
      get_logger(),
      "loaded persisted state: phase=%s target=%s ts=%ld version=%u",
      load_result.state->task_phase.c_str(),
      load_result.state->target_state.c_str(),
      load_result.state->timestamp_ms,
      load_result.state->version);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "no valid persisted state loaded: %s",
      load_result.result.message.c_str());
  }
  const auto map_end = std::chrono::steady_clock::now();
  const auto recover_total_end = std::chrono::steady_clock::now();
  const auto load_cost_ms = std::chrono::duration_cast<std::chrono::milliseconds>(load_end - load_start).count();
  const auto map_cost_ms = std::chrono::duration_cast<std::chrono::milliseconds>(map_end - map_start).count();
  const auto total_cost_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(recover_total_end - recover_total_start).count();
  publishRecoveryContext(load_result, load_cost_ms, map_cost_ms, total_cost_ms);

  degrade_command_pub_ = create_publisher<std_msgs::msg::String>(
    degrade_command_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  grasp_feedback_sub_ = create_subscription<std_msgs::msg::String>(
    grasp_feedback_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    std::bind(&LifecycleNode::graspFeedbackCallback, this, std::placeholders::_1));
  degrade_ack_sub_ = create_subscription<std_msgs::msg::String>(
    degrade_ack_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    std::bind(&LifecycleNode::degradeAckCallback, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::String>(
    estop_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    std::bind(&LifecycleNode::estopCallback, this, std::placeholders::_1));
  valid_frame_sub_ = create_subscription<dog_interfaces::msg::Target3D>(
    valid_frame_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LifecycleNode::validFrameCallback, this, std::placeholders::_1));
  lifecycle_transition_status_sub_ = create_subscription<std_msgs::msg::String>(
    lifecycle_transition_status_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    std::bind(&LifecycleNode::lifecycleTransitionStatusCallback, this, std::placeholders::_1));
  heartbeat_timer_ = create_wall_timer(
    std::chrono::milliseconds(heartbeat_check_period_ms_),
    std::bind(&LifecycleNode::heartbeatTimerCallback, this));

  {
    std::lock_guard<std::mutex> lock(breaker_mutex_);
    has_last_valid_frame_time_ = true;
    last_valid_frame_time_ = now();
  }

  publishSystemMode("normal", "startup_default");

  RCLCPP_INFO(
    get_logger(),
    "dog_lifecycle initialized, feedback_topic=%s, degrade_topic=%s, estop_topic=%s, mode_topic=%s, threshold=%ld, degrade_timeout_ms=%ld, reset_window_ms=%ld, dedup_window_ms=%ld",
    grasp_feedback_topic_.c_str(),
    degrade_command_topic_.c_str(),
    estop_topic_.c_str(),
    system_mode_topic_.c_str(),
    empty_grasp_threshold_,
    degrade_timeout_ms_,
    breaker_reset_window_ms_,
    feedback_dedup_window_ms_);

  RCLCPP_INFO(
    get_logger(),
    "heartbeat_guard initialized, valid_frame_topic=%s, transition_topic=%s, alarm_topic=%s, monitored_node=%s, timeout_ms=%ld, check_period_ms=%ld, min_interval_ms=%ld, pending_timeout_ms=%ld, max_restart_attempts=%ld, restart_window_ms=%ld",
    valid_frame_topic_.c_str(),
    lifecycle_transition_topic_.c_str(),
    health_alarm_topic_.c_str(),
    monitored_node_name_.c_str(),
    heartbeat_timeout_ms_,
    heartbeat_check_period_ms_,
    reconnect_min_interval_ms_,
    reconnect_pending_timeout_ms_,
    max_restart_attempts_,
    restart_window_ms_);
}

LifecycleNode::~LifecycleNode()
{
  if (!ClearPersistentState()) {
    RCLCPP_ERROR(get_logger(), "failed to clear persistent state during shutdown");
  }
}

bool LifecycleNode::PersistTransition(const std::string & task_phase, const std::string & target_state)
{
  if (!state_store_) {
    RCLCPP_ERROR(get_logger(), "state store is not initialized");
    return false;
  }

  RecoverableState state;
  state.task_phase = task_phase;
  state.target_state = target_state;
  state.version = supported_state_version_;
  state.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count();

  const auto start = std::chrono::steady_clock::now();
  const auto save_result = state_store_->Save(state);
  const auto end = std::chrono::steady_clock::now();
  const auto cost_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();

  if (!save_result.ok) {
    RCLCPP_ERROR(
      get_logger(), "persist transition failed: %s, write_cost=%.3fms",
      save_result.message.c_str(), cost_ms);
    return false;
  }

  RCLCPP_INFO(
    get_logger(), "persisted transition phase=%s target=%s write_cost=%.3fms",
    task_phase.c_str(), target_state.c_str(), cost_ms);
  return true;
}

bool LifecycleNode::ClearPersistentState()
{
  if (!state_store_) {
    return true;
  }

  const auto clear_result = state_store_->Clear();
  if (!clear_result.ok) {
    RCLCPP_ERROR(get_logger(), "clear persistent state failed: %s", clear_result.message.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "persistent state files cleared");
  return true;
}

void LifecycleNode::InjectGraspFeedbackForTest(const std::string & raw_feedback)
{
  const auto event = parseGraspFeedback(raw_feedback, now());
  processGraspFeedback(event);
}

bool LifecycleNode::IsBreakerOpen() const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return breaker_open_;
}

bool LifecycleNode::IsTaskBlocked(const std::string & task_id) const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return isTaskBlockedLocked(task_id);
}

size_t LifecycleNode::GetConsecutiveEmptyCount() const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return consecutive_empty_count_;
}

bool LifecycleNode::IsDegradePending() const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return degrade_pending_;
}

int64_t LifecycleNode::GetLastBreakerToDegradeLatencyMs() const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return last_breaker_to_degrade_latency_ms_;
}

std::string LifecycleNode::GetLastRecoveryContextForTest() const
{
  std::lock_guard<std::mutex> lock(recovery_context_mutex_);
  return last_recovery_context_payload_;
}

bool LifecycleNode::IsIdleSpinningForTest() const
{
  std::lock_guard<std::mutex> lock(recovery_context_mutex_);
  return idle_spinning_active_;
}

std::string LifecycleNode::GetLastSystemModePayloadForTest() const
{
  std::lock_guard<std::mutex> lock(recovery_context_mutex_);
  return last_system_mode_payload_;
}

bool LifecycleNode::IsReconnectPendingForTest() const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return reconnect_pending_;
}

bool LifecycleNode::IsReconnectTransitionCompletedForTest() const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return reconnect_transition_inactive_confirmed_ && reconnect_transition_active_confirmed_;
}

bool LifecycleNode::IsControlledDegradeModeForTest() const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return controlled_degrade_mode_;
}

int64_t LifecycleNode::GetLastReconnectRecoveryLatencyMsForTest() const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return last_reconnect_recovery_latency_ms_;
}

size_t LifecycleNode::GetRestartAttemptsInWindowForTest() const
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  return restart_attempt_timestamps_.size();
}

void LifecycleNode::graspFeedbackCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  const auto event = parseGraspFeedback(msg->data, now());
  processGraspFeedback(event);
}

void LifecycleNode::degradeAckCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  const auto ack = parseDegradeAck(msg->data);
  if (ack.status != "ok" && ack.status != "success") {
    return;
  }

  std::lock_guard<std::mutex> lock(breaker_mutex_);
  if (!degrade_pending_) {
    return;
  }

  if (!ack.task_id.empty() && ack.task_id != blocked_task_id_) {
    RCLCPP_WARN(
      get_logger(),
      "degrade_ack_ignored task_id_mismatch expected=%s actual=%s",
      blocked_task_id_.c_str(),
      ack.task_id.c_str());
    return;
  }

  if (ack.has_request_id && ack.request_id != pending_degrade_request_id_) {
    RCLCPP_WARN(
      get_logger(),
      "degrade_ack_ignored request_id_mismatch expected=%lu actual=%lu",
      pending_degrade_request_id_,
      ack.request_id);
    return;
  }

  degrade_pending_ = false;
  if (degrade_timeout_timer_) {
    degrade_timeout_timer_->cancel();
  }

  RCLCPP_INFO(
    get_logger(),
    "degrade_ack_received task_id=%s request_id=%lu ack=%s",
    blocked_task_id_.c_str(),
    pending_degrade_request_id_,
    msg->data.c_str());
}

void LifecycleNode::estopCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto estop_active = parseEstopActive(msg->data);
  if (!estop_active.has_value()) {
    RCLCPP_WARN(
      get_logger(),
      "ignore ambiguous estop payload=%s",
      msg->data.c_str());
    return;
  }

  bool should_publish = false;

  {
    std::lock_guard<std::mutex> lock(recovery_context_mutex_);
    if (estop_active.value() == idle_spinning_active_) {
      return;
    }
    idle_spinning_active_ = estop_active.value();
    should_publish = true;
  }

  if (!should_publish) {
    return;
  }

  if (estop_active.value()) {
    publishSystemMode("idle_spinning", "estop_triggered");
  } else {
    publishSystemMode("normal", "estop_released");
  }
}

LifecycleNode::DegradeAckEvent LifecycleNode::parseDegradeAck(const std::string & payload) const
{
  DegradeAckEvent event;
  event.task_id = parseKeyValuePayload(payload, "task_id");
  event.status = parseKeyValuePayload(payload, "status");
  if (event.status.empty()) {
    event.status = normalizeToken(payload);
  }

  const auto request_id_value = parseKeyValuePayload(payload, "request_id");
  if (!request_id_value.empty()) {
    try {
      event.request_id = static_cast<uint64_t>(std::stoull(request_id_value));
      event.has_request_id = true;
    } catch (const std::invalid_argument &) {
      event.has_request_id = false;
    } catch (const std::out_of_range &) {
      event.has_request_id = false;
    }
  }

  return event;
}

std::optional<bool> LifecycleNode::parseEstopActive(const std::string & payload) const
{
  const auto normalized_payload = normalizeToken(payload);
  const auto active_token = parseKeyValuePayload(payload, "active");
  if (!active_token.empty()) {
    if (active_token == "1" || active_token == "true" || active_token == "on" ||
      active_token == "pressed")
    {
      return true;
    }
    if (active_token == "0" || active_token == "false" || active_token == "off" ||
      active_token == "released" || active_token == "clear")
    {
      return false;
    }
    return std::nullopt;
  }

  if (normalized_payload == "1" || normalized_payload == "true" ||
    normalized_payload == "on" || normalized_payload == "pressed" ||
    normalized_payload == "estop" || normalized_payload == "emergency_stop")
  {
    return true;
  }

  if (normalized_payload == "0" || normalized_payload == "false" ||
    normalized_payload == "off" || normalized_payload == "released" ||
    normalized_payload == "clear")
  {
    return false;
  }

  const auto mode_token = parseKeyValuePayload(payload, "mode");
  if (mode_token == "estop" || mode_token == "idle_spinning") {
    return true;
  }
  if (mode_token == "normal") {
    return false;
  }

  return std::nullopt;
}

LifecycleNode::GraspFeedbackEvent LifecycleNode::parseGraspFeedback(
  const std::string & payload,
  const rclcpp::Time & stamp) const
{
  GraspFeedbackEvent event;
  event.stamp = stamp;

  const auto delimiter = payload.find('|');
  std::string raw_task_id;
  std::string raw_event;
  if (delimiter == std::string::npos) {
    raw_task_id = "unknown_task";
    raw_event = payload;
  } else {
    raw_task_id = payload.substr(0, delimiter);
    raw_event = payload.substr(delimiter + 1);
  }

  event.task_id = normalizeToken(raw_task_id);
  if (event.task_id.empty()) {
    event.task_id = "unknown_task";
  }

  const auto token = normalizeToken(raw_event);
  if (token == "success" || token == "succeeded" || token == "ok") {
    event.type = GraspFeedbackType::kSuccess;
  } else if (token == "empty" || token == "empty_grasp" || token == "grasp_empty") {
    event.type = GraspFeedbackType::kEmpty;
  } else if (
    token == "exception" || token == "error" || token == "timeout" || token == "aborted" ||
    token == "failed")
  {
    event.type = GraspFeedbackType::kException;
  } else {
    event.type = GraspFeedbackType::kUnknown;
  }

  return event;
}

void LifecycleNode::processGraspFeedback(const GraspFeedbackEvent & event)
{
  bool should_publish_degrade = false;
  std::string degrade_task_id;

  {
    std::lock_guard<std::mutex> lock(breaker_mutex_);
    resetBreakerIfWindowElapsed(event.stamp);

    if (has_last_feedback_stamp_) {
      const auto duplicate_elapsed_ms = (event.stamp - last_feedback_stamp_).nanoseconds() / 1000000;
      if (
        duplicate_elapsed_ms >= 0 && duplicate_elapsed_ms <= feedback_dedup_window_ms_ &&
        event.task_id == last_feedback_task_id_ && event.type == last_feedback_type_)
      {
        RCLCPP_WARN(
          get_logger(),
          "feedback_duplicate_ignored task_id=%s type=%s elapsed_ms=%ld",
          event.task_id.c_str(),
          feedbackTypeToString(event.type),
          duplicate_elapsed_ms);
        return;
      }
    }
    has_last_feedback_stamp_ = true;
    last_feedback_stamp_ = event.stamp;
    last_feedback_task_id_ = event.task_id;
    last_feedback_type_ = event.type;

    switch (event.type) {
      case GraspFeedbackType::kSuccess:
        consecutive_empty_count_ = 0;
        has_last_empty_time_ = false;
        current_empty_task_id_.clear();
        RCLCPP_INFO(
          get_logger(),
          "grasp_success task_id=%s breaker_open=%s",
          event.task_id.c_str(),
          breaker_open_ ? "true" : "false");
        break;

      case GraspFeedbackType::kEmpty:
        if (current_empty_task_id_ != event.task_id) {
          current_empty_task_id_ = event.task_id;
          consecutive_empty_count_ = 0;
          has_last_empty_time_ = false;
        }

        if (isTaskBlockedLocked(event.task_id)) {
          RCLCPP_WARN(
            get_logger(),
            "breaker_reject_retry task_id=%s count=%zu threshold=%ld",
            event.task_id.c_str(),
            consecutive_empty_count_,
            empty_grasp_threshold_);
          return;
        }

        ++consecutive_empty_count_;
        has_last_empty_time_ = true;
        last_empty_time_ = event.stamp;
        if (consecutive_empty_count_ < static_cast<size_t>(empty_grasp_threshold_)) {
          RCLCPP_INFO(
            get_logger(),
            "retry_continue task_id=%s count=%zu threshold=%ld",
            event.task_id.c_str(),
            consecutive_empty_count_,
            empty_grasp_threshold_);
          return;
        }

        breaker_open_ = true;
        blocked_task_id_ = event.task_id;
        has_breaker_trigger_time_ = true;
        breaker_trigger_time_ = event.stamp;
        should_publish_degrade = true;
        degrade_task_id = event.task_id;

        RCLCPP_WARN(
          get_logger(),
          "breaker_open task_id=%s count=%zu threshold=%ld",
          event.task_id.c_str(),
          consecutive_empty_count_,
          empty_grasp_threshold_);
        break;

      case GraspFeedbackType::kException:
        RCLCPP_ERROR(
          get_logger(),
          "grasp_exception task_id=%s breaker_open=%s",
          event.task_id.c_str(),
          breaker_open_ ? "true" : "false");
        break;

      case GraspFeedbackType::kUnknown:
      default:
        RCLCPP_WARN(
          get_logger(),
          "grasp_feedback_unknown task_id=%s raw_type=%s",
          event.task_id.c_str(),
          feedbackTypeToString(event.type));
        break;
    }
  }

  if (should_publish_degrade) {
    publishDegradeCommand(degrade_task_id, "empty_threshold_reached");
  }
}

void LifecycleNode::validFrameCallback(const dog_interfaces::msg::Target3D::ConstSharedPtr msg)
{
  if (!isValidFrameMessage(msg)) {
    return;
  }

  int64_t recovery_latency_ms = -1;
  bool recovered_from_reconnect = false;
  {
    std::lock_guard<std::mutex> lock(breaker_mutex_);
    const auto now_time = now();
    has_last_valid_frame_time_ = true;
    last_valid_frame_time_ = now_time;

    if (reconnect_pending_) {
      if (!(reconnect_transition_inactive_confirmed_ && reconnect_transition_active_confirmed_)) {
        return;
      }
      ++reconnect_valid_frame_consecutive_;
      if (reconnect_valid_frame_consecutive_ < static_cast<size_t>(valid_frame_recovery_consecutive_)) {
        return;
      }
      recovery_latency_ms = (now_time - last_reconnect_start_time_).nanoseconds() / 1000000;
      last_reconnect_recovery_latency_ms_ = recovery_latency_ms;
      has_last_reconnect_recovery_time_ = true;
      last_reconnect_recovery_time_ = now_time;
      reconnect_pending_ = false;
      reconnect_valid_frame_consecutive_ = 0U;
      recovered_from_reconnect = true;
    }
  }

  if (recovered_from_reconnect) {
    RCLCPP_INFO(
      get_logger(),
      "heartbeat_reconnect_recovered node=%s latency_ms=%ld requirement_lt_ms=2000",
      monitored_node_name_.c_str(),
      recovery_latency_ms);
  }
}

void LifecycleNode::lifecycleTransitionStatusCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto status = parseKeyValuePayload(msg->data, "status");
  if (
    status != "ok" && status != "success" && status != "succeeded" &&
    status != "done" && status != "completed")
  {
    return;
  }

  const auto node_name = parseKeyValuePayload(msg->data, "node");
  if (!node_name.empty() && node_name != normalizeToken(monitored_node_name_)) {
    return;
  }

  const auto from_state = parseKeyValuePayload(msg->data, "from");
  const auto to_state = parseKeyValuePayload(msg->data, "to");
  if (from_state.empty() || to_state.empty()) {
    return;
  }

  const auto attempt_value = parseKeyValuePayload(msg->data, "attempt");
  if (attempt_value.empty()) {
    return;
  }

  uint32_t attempt = 0U;
  try {
    attempt = static_cast<uint32_t>(std::stoul(attempt_value));
  } catch (const std::exception &) {
    return;
  }

  bool transition_completed = false;
  {
    std::lock_guard<std::mutex> lock(breaker_mutex_);
    if (!reconnect_pending_ || attempt != reconnect_transition_attempt_) {
      return;
    }

    if (from_state == "active" && to_state == "inactive") {
      reconnect_transition_inactive_confirmed_ = true;
    } else if (from_state == "inactive" && to_state == "active") {
      if (!reconnect_transition_inactive_confirmed_) {
        return;
      }
      reconnect_transition_active_confirmed_ = true;
    } else {
      return;
    }

    transition_completed = reconnect_transition_inactive_confirmed_ && reconnect_transition_active_confirmed_;
    if (transition_completed) {
      has_last_transition_verified_time_ = true;
      last_transition_verified_time_ = now();
    }
  }

  if (transition_completed) {
    RCLCPP_INFO(
      get_logger(),
      "heartbeat_transition_verified node=%s attempt=%u",
      monitored_node_name_.c_str(),
      attempt);
  }
}

void LifecycleNode::heartbeatTimerCallback()
{
  const auto now_time = now();

  bool should_trigger_reconnect = false;
  bool should_enter_degrade = false;
  uint32_t attempts = 0U;
  int64_t since_last_valid_frame_ms = -1;

  {
    std::lock_guard<std::mutex> lock(breaker_mutex_);
    if (controlled_degrade_mode_ || !has_last_valid_frame_time_) {
      return;
    }

    since_last_valid_frame_ms = (now_time - last_valid_frame_time_).nanoseconds() / 1000000;
    if (since_last_valid_frame_ms <= heartbeat_timeout_ms_) {
      return;
    }

    // Keep a single reconnect attempt in flight to avoid attempt inflation while
    // waiting for transition acks or valid-frame latch completion.
    if (reconnect_pending_) {
      const auto pending_elapsed_ms = (now_time - last_reconnect_start_time_).nanoseconds() / 1000000;
      if (pending_elapsed_ms < reconnect_pending_timeout_ms_) {
        return;
      }

      reconnect_pending_ = false;
      reconnect_valid_frame_consecutive_ = 0U;
      reconnect_transition_inactive_confirmed_ = false;
      reconnect_transition_active_confirmed_ = false;
      reconnect_transition_attempt_ = 0U;

      RCLCPP_WARN(
        get_logger(),
        "heartbeat_reconnect_pending_timeout node=%s elapsed_ms=%ld timeout_ms=%ld",
        monitored_node_name_.c_str(),
        pending_elapsed_ms,
        reconnect_pending_timeout_ms_);
    }

    if (has_last_reconnect_trigger_time_) {
      const auto elapsed_since_last_reconnect_ms =
        (now_time - last_reconnect_trigger_time_).nanoseconds() / 1000000;
      if (elapsed_since_last_reconnect_ms < reconnect_min_interval_ms_) {
        return;
      }
    }

    while (!restart_attempt_timestamps_.empty()) {
      const auto elapsed_in_window_ms =
        (now_time - restart_attempt_timestamps_.front()).nanoseconds() / 1000000;
      if (elapsed_in_window_ms <= restart_window_ms_) {
        break;
      }
      restart_attempt_timestamps_.pop_front();
    }

    restart_attempt_timestamps_.push_back(now_time);
    attempts = static_cast<uint32_t>(restart_attempt_timestamps_.size());
    has_last_reconnect_trigger_time_ = true;
    last_reconnect_trigger_time_ = now_time;

    if (attempts > static_cast<uint32_t>(max_restart_attempts_)) {
      reconnect_pending_ = false;
      reconnect_valid_frame_consecutive_ = 0U;
      reconnect_transition_inactive_confirmed_ = false;
      reconnect_transition_active_confirmed_ = false;
      reconnect_transition_attempt_ = 0U;
      controlled_degrade_mode_ = true;
      should_enter_degrade = true;
    } else {
      reconnect_pending_ = true;
      reconnect_valid_frame_consecutive_ = 0U;
      reconnect_transition_inactive_confirmed_ = false;
      reconnect_transition_active_confirmed_ = false;
      reconnect_transition_attempt_ = attempts;
      last_reconnect_start_time_ = now_time;
      should_trigger_reconnect = true;
    }
  }

  if (should_enter_degrade) {
    publishHealthAlarm("heartbeat_restart_limit_exceeded", attempts, since_last_valid_frame_ms);
    publishSystemMode("degraded", "heartbeat_restart_limit_exceeded");
    return;
  }

  if (!should_trigger_reconnect) {
    return;
  }

  publishLifecycleTransition("active", "inactive", "heartbeat_timeout", attempts);
  publishLifecycleTransition("inactive", "active", "heartbeat_timeout", attempts);
  RCLCPP_WARN(
    get_logger(),
    "heartbeat_timeout_reconnect_triggered node=%s since_last_valid_frame_ms=%ld attempt=%u",
    monitored_node_name_.c_str(),
    since_last_valid_frame_ms,
    attempts);
}

void LifecycleNode::resetBreakerIfWindowElapsed(const rclcpp::Time & now)
{
  if (!has_last_empty_time_ || breaker_reset_window_ms_ <= 0) {
    return;
  }

  const auto elapsed_ms = (now - last_empty_time_).nanoseconds() / 1000000;
  if (elapsed_ms < breaker_reset_window_ms_) {
    return;
  }

  if (consecutive_empty_count_ > 0 || breaker_open_) {
    RCLCPP_INFO(
      get_logger(),
      "breaker_reset_window_elapsed previous_count=%zu window_ms=%ld",
      consecutive_empty_count_,
      breaker_reset_window_ms_);
  }

  consecutive_empty_count_ = 0;
  breaker_open_ = false;
  blocked_task_id_.clear();
  current_empty_task_id_.clear();
  has_last_empty_time_ = false;
}

void LifecycleNode::publishDegradeCommand(const std::string & task_id, const char * reason)
{
  int64_t latency_ms = -1;
  uint64_t request_id = 0U;
  {
    std::lock_guard<std::mutex> lock(breaker_mutex_);
    if (has_breaker_trigger_time_) {
      latency_ms = (now() - breaker_trigger_time_).nanoseconds() / 1000000;
      last_breaker_to_degrade_latency_ms_ = latency_ms;
    }

    blocked_task_id_ = normalizeToken(task_id);
    degrade_pending_ = true;
    request_id = ++next_degrade_request_id_;
    pending_degrade_request_id_ = request_id;

    if (degrade_timeout_timer_) {
      degrade_timeout_timer_->cancel();
    }

    degrade_timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(degrade_timeout_ms_),
      [this, request_id]() {
        degradeTimeoutCallback(request_id);
      });
  }

  std_msgs::msg::String command;
  command.data =
    "task_id=" + task_id + ";mode=deliver_completed_boxes;reason=" + reason +
    ";request_id=" + std::to_string(request_id);

  degrade_command_pub_->publish(command);
  RCLCPP_WARN(
    get_logger(),
    "degrade_issued task_id=%s request_id=%lu latency_ms=%ld command=%s",
    task_id.c_str(),
    request_id,
    latency_ms,
    command.data.c_str());
}

void LifecycleNode::publishRecoveryContext(
  const StateStoreLoadResult & load_result,
  int64_t load_cost_ms,
  int64_t map_cost_ms,
  int64_t total_cost_ms)
{
  std_msgs::msg::String recovery_context;
  std::ostringstream payload;

  if (load_result.result.ok && load_result.state.has_value()) {
    payload
      << "mode=recovered"
      << ";task_phase=" << percentEncode(load_result.state->task_phase)
      << ";target_state=" << percentEncode(load_result.state->target_state)
      << ";timestamp_ms=" << load_result.state->timestamp_ms
      << ";version=" << load_result.state->version;
  } else {
    payload
      << "mode=cold_start"
      << ";task_phase="
      << ";target_state="
      << ";timestamp_ms=0"
      << ";version=" << supported_state_version_
        << ";reason=" << percentEncode(normalizeToken(load_result.result.message));
  }

  payload
    << ";load_ms=" << load_cost_ms
    << ";map_ms=" << map_cost_ms
    << ";total_ms=" << total_cost_ms;
  recovery_context.data = payload.str();

  {
    std::lock_guard<std::mutex> lock(recovery_context_mutex_);
    last_recovery_context_payload_ = recovery_context.data;
  }

  recovery_context_pub_->publish(recovery_context);
  RCLCPP_INFO(
    get_logger(),
    "recovery_context_published topic=%s payload=%s",
    recovery_context_topic_.c_str(),
    recovery_context.data.c_str());
}

void LifecycleNode::degradeTimeoutCallback(uint64_t request_id)
{
  std::lock_guard<std::mutex> lock(breaker_mutex_);
  if (!degrade_pending_) {
    return;
  }

  if (request_id != pending_degrade_request_id_) {
    return;
  }

  degrade_pending_ = false;
  RCLCPP_ERROR(
    get_logger(),
    "degrade_failed_timeout task_id=%s request_id=%lu timeout_ms=%ld",
    blocked_task_id_.c_str(),
    pending_degrade_request_id_,
    degrade_timeout_ms_);
  if (degrade_timeout_timer_) {
    degrade_timeout_timer_->cancel();
  }
}

void LifecycleNode::publishSystemMode(const std::string & mode, const char * reason)
{
  std_msgs::msg::String mode_message;
  mode_message.data = "mode=" + mode + ";reason=" + std::string(reason);

  {
    std::lock_guard<std::mutex> lock(recovery_context_mutex_);
    last_system_mode_payload_ = mode_message.data;
  }

  system_mode_pub_->publish(mode_message);
  RCLCPP_WARN(
    get_logger(),
    "system_mode_published mode=%s reason=%s payload=%s",
    mode.c_str(),
    reason,
    mode_message.data.c_str());
}

void LifecycleNode::publishLifecycleTransition(
  const char * from_state,
  const char * to_state,
  const char * reason,
  uint32_t attempt)
{
  std_msgs::msg::String transition_message;
  transition_message.data =
    "node=" + monitored_node_name_ +
    ";from=" + std::string(from_state) +
    ";to=" + std::string(to_state) +
    ";reason=" + std::string(reason) +
    ";attempt=" + std::to_string(attempt);

  lifecycle_transition_pub_->publish(transition_message);
  RCLCPP_WARN(
    get_logger(),
    "lifecycle_transition_issued node=%s from=%s to=%s reason=%s attempt=%u",
    monitored_node_name_.c_str(),
    from_state,
    to_state,
    reason,
    attempt);
}

void LifecycleNode::publishHealthAlarm(
  const char * reason,
  uint32_t attempts,
  int64_t since_last_valid_frame_ms)
{
  int64_t last_valid_frame_unix_ms = -1;
  int64_t last_reconnect_success_unix_ms = -1;
  {
    std::lock_guard<std::mutex> lock(breaker_mutex_);
    if (has_last_valid_frame_time_) {
      last_valid_frame_unix_ms = last_valid_frame_time_.nanoseconds() / 1000000;
    }
    if (has_last_reconnect_recovery_time_) {
      last_reconnect_success_unix_ms = last_reconnect_recovery_time_.nanoseconds() / 1000000;
    }
  }

  std_msgs::msg::String alarm_message;
  alarm_message.data =
    "type=heartbeat_restart_limit"
    ";reason=" + std::string(reason) +
    ";node=" + monitored_node_name_ +
    ";attempts=" + std::to_string(attempts) +
    ";max_attempts=" + std::to_string(max_restart_attempts_) +
    ";restart_window_ms=" + std::to_string(restart_window_ms_) +
    ";last_reconnect_success_unix_ms=" + std::to_string(last_reconnect_success_unix_ms) +
    ";last_valid_frame_unix_ms=" + std::to_string(last_valid_frame_unix_ms) +
    ";since_last_valid_frame_ms=" + std::to_string(since_last_valid_frame_ms);

  health_alarm_pub_->publish(alarm_message);
  RCLCPP_ERROR(
    get_logger(),
    "heartbeat_controlled_degrade reason=%s node=%s attempts=%u max_attempts=%ld restart_window_ms=%ld since_last_valid_frame_ms=%ld",
    reason,
    monitored_node_name_.c_str(),
    attempts,
    max_restart_attempts_,
    restart_window_ms_,
    since_last_valid_frame_ms);
}

bool LifecycleNode::isValidFrameMessage(const dog_interfaces::msg::Target3D::ConstSharedPtr & msg) const
{
  if (!msg) {
    return false;
  }

  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    return false;
  }

  if (
    !std::isfinite(msg->position.x) ||
    !std::isfinite(msg->position.y) ||
    !std::isfinite(msg->position.z))
  {
    return false;
  }

  if (!std::isfinite(msg->confidence) || msg->confidence <= 0.0F) {
    return false;
  }

  return true;
}

const char * LifecycleNode::feedbackTypeToString(GraspFeedbackType type)
{
  switch (type) {
    case GraspFeedbackType::kSuccess:
      return "success";
    case GraspFeedbackType::kEmpty:
      return "empty";
    case GraspFeedbackType::kException:
      return "exception";
    case GraspFeedbackType::kUnknown:
    default:
      return "unknown";
  }
}

std::string LifecycleNode::normalizeToken(const std::string & value)
{
  std::string normalized;
  normalized.reserve(value.size());
  for (const auto ch : value) {
    if (std::isspace(static_cast<unsigned char>(ch))) {
      continue;
    }
    normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }
  return normalized;
}

std::string LifecycleNode::parseKeyValuePayload(const std::string & payload, const std::string & key)
{
  static const std::string delimiter = ";";
  const auto normalized_key = normalizeToken(key);

  size_t start = 0;
  while (start <= payload.size()) {
    const auto end = payload.find(delimiter, start);
    const auto token = payload.substr(start, end == std::string::npos ? std::string::npos : end - start);
    const auto equal_pos = token.find('=');
    if (equal_pos != std::string::npos) {
      const auto token_key = normalizeToken(token.substr(0, equal_pos));
      if (token_key == normalized_key) {
        return normalizeToken(token.substr(equal_pos + 1));
      }
    }

    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }

  return "";
}

bool LifecycleNode::isTaskBlockedLocked(const std::string & task_id) const
{
  if (!breaker_open_) {
    return false;
  }
  if (blocked_task_id_.empty()) {
    return false;
  }
  return blocked_task_id_ == normalizeToken(task_id);
}

}  // namespace dog_lifecycle