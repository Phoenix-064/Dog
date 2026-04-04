#include "dog_lifecycle/lifecycle_node.hpp"

#include "dog_lifecycle/yaml_state_store.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace dog_lifecycle
{

LifecycleNode::LifecycleNode()
: LifecycleNode(rclcpp::NodeOptions())
{
}

LifecycleNode::LifecycleNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dog_lifecycle", options)
{
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
  grasp_feedback_topic_ = declare_parameter<std::string>("grasp_feedback_topic", "/behavior/grasp_feedback");
  degrade_command_topic_ = declare_parameter<std::string>("degrade_command_topic", "/lifecycle/degrade_command");
  degrade_ack_topic_ = declare_parameter<std::string>("degrade_ack_topic", "/lifecycle/degrade_ack");

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

  StateStoreConfig config;
  config.state_file_path = state_file_path;
  config.backup_file_path = backup_file_path;
  config.max_write_bytes = static_cast<size_t>(validated_max_write_bytes);
  config.min_write_interval = std::chrono::milliseconds(validated_min_write_interval_ms);
  config.supported_version = supported_state_version_;

  state_store_ = std::make_unique<YamlStateStore>(config);

  const auto load_result = state_store_->Load();
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

  RCLCPP_INFO(
    get_logger(),
    "dog_lifecycle initialized, feedback_topic=%s, degrade_topic=%s, threshold=%ld, degrade_timeout_ms=%ld, reset_window_ms=%ld, dedup_window_ms=%ld",
    grasp_feedback_topic_.c_str(),
    degrade_command_topic_.c_str(),
    empty_grasp_threshold_,
    degrade_timeout_ms_,
    breaker_reset_window_ms_,
    feedback_dedup_window_ms_);
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