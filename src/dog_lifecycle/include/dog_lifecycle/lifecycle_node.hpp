#pragma once

#include "dog_lifecycle/state_store.hpp"

#include <dog_interfaces/msg/target3_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

namespace dog_lifecycle
{

class LifecycleNode : public rclcpp::Node
{
public:
  /// @brief Construct lifecycle node with default node options.
  LifecycleNode();
  /// @brief Construct lifecycle node with explicit ROS node options.
  /// @param options ROS node options.
  explicit LifecycleNode(const rclcpp::NodeOptions & options);
  /// @brief Shutdown hook that clears persistent recovery state.
  ~LifecycleNode() override;

  /// @brief Persist a lifecycle transition for recovery on next startup.
  /// @param task_phase Transition task phase.
  /// @param target_state Transition target state.
  /// @return True when the state is persisted successfully.
  bool PersistTransition(const std::string & task_phase, const std::string & target_state);
  /// @brief Remove persisted lifecycle state files.
  /// @return True when persistent state is cleared or absent.
  bool ClearPersistentState();

  /// @brief Inject raw grasp feedback for deterministic unit tests.
  /// @param raw_feedback Simulated feedback payload.
  void InjectGraspFeedbackForTest(const std::string & raw_feedback);
  /// @brief Test helper for breaker-open status.
  /// @return True if breaker is currently open.
  bool IsBreakerOpen() const;
  /// @brief Test helper for blocked task status.
  /// @param task_id Task identifier.
  /// @return True when the task is blocked by breaker state.
  bool IsTaskBlocked(const std::string & task_id) const;
  /// @brief Test helper for consecutive empty-grasp counter.
  /// @return Current consecutive empty-grasp count.
  size_t GetConsecutiveEmptyCount() const;
  /// @brief Test helper for degrade pending state.
  /// @return True when waiting for degrade acknowledgement.
  bool IsDegradePending() const;
  /// @brief Test helper for breaker-to-degrade latency metric.
  /// @return Last measured latency in milliseconds, or -1 when unavailable.
  int64_t GetLastBreakerToDegradeLatencyMs() const;
  /// @brief Test helper exposing last published recovery context payload.
  /// @return Recovery context payload string.
  std::string GetLastRecoveryContextForTest() const;
  /// @brief Test helper for idle-spinning state.
  /// @return True when idle-spinning mode is active.
  bool IsIdleSpinningForTest() const;
  /// @brief Test helper exposing last system-mode payload.
  /// @return Last published system-mode payload.
  std::string GetLastSystemModePayloadForTest() const;
  /// @brief Test helper for reconnect pending state.
  /// @return True when reconnect sequence is in progress.
  bool IsReconnectPendingForTest() const;
  /// @brief Test helper for reconnect transition completion state.
  /// @return True when both reconnect transition phases are confirmed.
  bool IsReconnectTransitionCompletedForTest() const;
  /// @brief Test helper for controlled degrade mode state.
  /// @return True when node entered controlled degrade mode.
  bool IsControlledDegradeModeForTest() const;
  /// @brief Test helper for reconnect recovery latency metric.
  /// @return Last reconnect recovery latency in milliseconds.
  int64_t GetLastReconnectRecoveryLatencyMsForTest() const;
  /// @brief Test helper for restart attempts currently tracked in window.
  /// @return Number of restart attempts in sliding restart window.
  size_t GetRestartAttemptsInWindowForTest() const;

private:
  enum class GraspFeedbackType
  {
    kSuccess,
    kEmpty,
    kException,
    kUnknown,
  };

  struct GraspFeedbackEvent
  {
    std::string task_id;
    GraspFeedbackType type{GraspFeedbackType::kUnknown};
    rclcpp::Time stamp;
  };

  struct DegradeAckEvent
  {
    std::string task_id;
    std::string status;
    uint64_t request_id{0U};
    bool has_request_id{false};
  };

  /// @brief Consume grasp feedback stream and drive breaker/degrade logic.
  /// @param msg Grasp feedback payload message.
  void graspFeedbackCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief Consume degrade acknowledgement messages.
  /// @param msg Degrade acknowledgement payload.
  void degradeAckCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief Consume e-stop mode updates.
  /// @param msg E-stop payload.
  void estopCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief Track valid perception frames for heartbeat recovery gating.
  /// @param msg Perception target frame.
  void validFrameCallback(const dog_interfaces::msg::Target3D::ConstSharedPtr msg);
  /// @brief Consume transition status acknowledgements for reconnect flow.
  /// @param msg Transition status payload.
  void lifecycleTransitionStatusCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief Validate whether an incoming target frame qualifies as healthy data.
  /// @param msg Target frame message.
  /// @return True when frame content is valid.
  bool isValidFrameMessage(const dog_interfaces::msg::Target3D::ConstSharedPtr & msg) const;
  /// @brief Parse raw grasp feedback payload into structured event.
  /// @param payload Raw feedback payload.
  /// @param stamp Reception timestamp.
  /// @return Parsed grasp feedback event.
  GraspFeedbackEvent parseGraspFeedback(const std::string & payload, const rclcpp::Time & stamp) const;
  /// @brief Parse degrade acknowledgement payload.
  /// @param payload Raw ack payload.
  /// @return Parsed degrade acknowledgement event.
  DegradeAckEvent parseDegradeAck(const std::string & payload) const;
  /// @brief Parse e-stop payload into boolean active state.
  /// @param payload Raw e-stop payload.
  /// @return Parsed active flag, or nullopt when ambiguous.
  std::optional<bool> parseEstopActive(const std::string & payload) const;
  /// @brief Apply parsed grasp feedback to breaker/degrade state machine.
  /// @param event Parsed grasp feedback event.
  void processGraspFeedback(const GraspFeedbackEvent & event);
  /// @brief Periodic heartbeat guard callback.
  void heartbeatTimerCallback();
  /// @brief Reset breaker counters if no recent empty-grasp events are observed.
  /// @param now Current node time.
  void resetBreakerIfWindowElapsed(const rclcpp::Time & now);
  /// @brief Publish degrade command message and arm degrade timeout tracking.
  /// @param task_id Blocked task identifier.
  /// @param reason Degrade trigger reason.
  void publishDegradeCommand(const std::string & task_id, const char * reason);
  /// @brief Publish lifecycle transition command payload.
  /// @param from_state Transition source state.
  /// @param to_state Transition destination state.
  /// @param reason Transition reason.
  /// @param attempt Reconnect attempt index.
  void publishLifecycleTransition(
    const char * from_state,
    const char * to_state,
    const char * reason,
    uint32_t attempt);
  /// @brief Publish heartbeat-related health alarm payload.
  /// @param reason Alarm reason.
  /// @param attempts Restart attempt count.
  /// @param since_last_valid_frame_ms Elapsed ms since last valid frame.
  void publishHealthAlarm(const char * reason, uint32_t attempts, int64_t since_last_valid_frame_ms);
  /// @brief Publish startup recovery context after loading persisted state.
  /// @param load_result Persisted state load result.
  /// @param load_cost_ms Load stage latency in milliseconds.
  /// @param map_cost_ms Mapping stage latency in milliseconds.
  /// @param total_cost_ms Total recovery publish latency in milliseconds.
  void publishRecoveryContext(
    const StateStoreLoadResult & load_result,
    int64_t load_cost_ms,
    int64_t map_cost_ms,
    int64_t total_cost_ms);
  /// @brief Handle degrade timeout expiry for request tracking.
  /// @param request_id Request id associated with timeout timer.
  void degradeTimeoutCallback(uint64_t request_id);
  /// @brief Publish lifecycle system mode payload.
  /// @param mode Target mode token.
  /// @param reason Mode transition reason.
  void publishSystemMode(const std::string & mode, const char * reason);
  /// @brief Extract value for key from semicolon-separated key-value payload.
  /// @param payload Input payload.
  /// @param key Key name to query.
  /// @return Normalized value or empty string.
  static std::string parseKeyValuePayload(const std::string & payload, const std::string & key);
  /// @brief Convert feedback type enum to stable lowercase text.
  /// @param type Feedback enum value.
  /// @return Feedback type string.
  static const char * feedbackTypeToString(GraspFeedbackType type);
  /// @brief Normalize tokens by removing spaces and lowercasing.
  /// @param value Raw token.
  /// @return Normalized token.
  static std::string normalizeToken(const std::string & value);
  /// @brief Check whether a task is currently blocked while lock is held.
  /// @param task_id Task identifier.
  /// @return True when blocked by breaker logic.
  bool isTaskBlockedLocked(const std::string & task_id) const;

  std::unique_ptr<IStateStore> state_store_;
  uint32_t supported_state_version_{1U};

  int64_t empty_grasp_threshold_{2};
  int64_t degrade_timeout_ms_{1000};
  int64_t breaker_reset_window_ms_{3000};
  int64_t heartbeat_timeout_ms_{2000};
  int64_t heartbeat_check_period_ms_{100};
  int64_t reconnect_min_interval_ms_{500};
  int64_t reconnect_pending_timeout_ms_{0};
  int64_t max_restart_attempts_{3};
  int64_t restart_window_ms_{10000};
  int64_t valid_frame_recovery_consecutive_{2};

  std::string grasp_feedback_topic_;
  std::string degrade_command_topic_;
  std::string degrade_ack_topic_;
  std::string recovery_context_topic_;
  std::string estop_topic_;
  std::string system_mode_topic_;
  std::string valid_frame_topic_;
  std::string lifecycle_transition_topic_;
  std::string lifecycle_transition_status_topic_;
  std::string health_alarm_topic_;
  std::string monitored_node_name_;

  mutable std::mutex breaker_mutex_;
  size_t consecutive_empty_count_{0U};
  std::string current_empty_task_id_;
  bool breaker_open_{false};
  std::string blocked_task_id_;
  bool has_last_empty_time_{false};
  rclcpp::Time last_empty_time_;
  bool degrade_pending_{false};
  uint64_t next_degrade_request_id_{0U};
  uint64_t pending_degrade_request_id_{0U};
  bool has_breaker_trigger_time_{false};
  rclcpp::Time breaker_trigger_time_;
  int64_t last_breaker_to_degrade_latency_ms_{-1};
  int64_t feedback_dedup_window_ms_{10};
  bool has_last_feedback_stamp_{false};
  rclcpp::Time last_feedback_stamp_;
  std::string last_feedback_task_id_;
  GraspFeedbackType last_feedback_type_{GraspFeedbackType::kUnknown};
  bool has_last_valid_frame_time_{false};
  rclcpp::Time last_valid_frame_time_;
  bool reconnect_pending_{false};
  bool has_last_reconnect_trigger_time_{false};
  rclcpp::Time last_reconnect_trigger_time_;
  rclcpp::Time last_reconnect_start_time_;
  int64_t last_reconnect_recovery_latency_ms_{-1};
  size_t reconnect_valid_frame_consecutive_{0U};
  bool reconnect_transition_inactive_confirmed_{false};
  bool reconnect_transition_active_confirmed_{false};
  uint32_t reconnect_transition_attempt_{0U};
  bool has_last_transition_verified_time_{false};
  rclcpp::Time last_transition_verified_time_;
  bool has_last_reconnect_recovery_time_{false};
  rclcpp::Time last_reconnect_recovery_time_;
  bool controlled_degrade_mode_{false};
  std::deque<rclcpp::Time> restart_attempt_timestamps_;

  mutable std::mutex recovery_context_mutex_;
  std::string last_recovery_context_payload_;
  bool idle_spinning_active_{false};
  std::string last_system_mode_payload_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr degrade_ack_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr estop_sub_;
  rclcpp::Subscription<dog_interfaces::msg::Target3D>::SharedPtr valid_frame_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lifecycle_transition_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr degrade_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_context_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_transition_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_alarm_pub_;
  rclcpp::TimerBase::SharedPtr degrade_timeout_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace dog_lifecycle