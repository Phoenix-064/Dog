#include "dog_lifecycle/lifecycle_node.hpp"

#include "dog_lifecycle/yaml_state_store.hpp"

#include <chrono>
#include <limits>

namespace dog_lifecycle
{

LifecycleNode::LifecycleNode()
: rclcpp::Node("dog_lifecycle")
{
  const auto state_file_path = declare_parameter<std::string>(
    "persistence.state_file_path", "/tmp/dog_lifecycle/state.yaml");
  const auto backup_file_path = declare_parameter<std::string>(
    "persistence.backup_file_path", "/tmp/dog_lifecycle/state.bak.yaml");
  const auto max_write_bytes = declare_parameter<int64_t>("persistence.max_write_bytes", 16384);
  const auto min_write_interval_ms = declare_parameter<int64_t>(
    "persistence.min_write_interval_ms", 100);
  const auto supported_version = declare_parameter<int64_t>("persistence.state_version", 1);

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

  RCLCPP_INFO(get_logger(), "dog_lifecycle node initialized with persistence service");
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

}  // namespace dog_lifecycle