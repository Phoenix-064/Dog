#pragma once

#include "dog_lifecycle/state_store.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <cstddef>
#include <cstdint>
#include <string>

namespace dog_lifecycle
{

class LifecycleNode : public rclcpp::Node
{
public:
  LifecycleNode();
  explicit LifecycleNode(const rclcpp::NodeOptions & options);
  ~LifecycleNode() override;

  bool PersistTransition(const std::string & task_phase, const std::string & target_state);
  bool ClearPersistentState();

  void InjectGraspFeedbackForTest(const std::string & raw_feedback);
  bool IsBreakerOpen() const;
  bool IsTaskBlocked(const std::string & task_id) const;
  size_t GetConsecutiveEmptyCount() const;
  bool IsDegradePending() const;
  int64_t GetLastBreakerToDegradeLatencyMs() const;
  std::string GetLastRecoveryContextForTest() const;
  bool IsIdleSpinningForTest() const;
  std::string GetLastSystemModePayloadForTest() const;

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

  void graspFeedbackCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void degradeAckCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void estopCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  GraspFeedbackEvent parseGraspFeedback(const std::string & payload, const rclcpp::Time & stamp) const;
  DegradeAckEvent parseDegradeAck(const std::string & payload) const;
  std::optional<bool> parseEstopActive(const std::string & payload) const;
  void processGraspFeedback(const GraspFeedbackEvent & event);
  void resetBreakerIfWindowElapsed(const rclcpp::Time & now);
  void publishDegradeCommand(const std::string & task_id, const char * reason);
  void publishRecoveryContext(
    const StateStoreLoadResult & load_result,
    int64_t load_cost_ms,
    int64_t map_cost_ms,
    int64_t total_cost_ms);
  void degradeTimeoutCallback(uint64_t request_id);
  void publishSystemMode(const std::string & mode, const char * reason);
  static std::string parseKeyValuePayload(const std::string & payload, const std::string & key);
  static const char * feedbackTypeToString(GraspFeedbackType type);
  static std::string normalizeToken(const std::string & value);
  bool isTaskBlockedLocked(const std::string & task_id) const;

  std::unique_ptr<IStateStore> state_store_;
  uint32_t supported_state_version_{1U};

  int64_t empty_grasp_threshold_{2};
  int64_t degrade_timeout_ms_{1000};
  int64_t breaker_reset_window_ms_{3000};

  std::string grasp_feedback_topic_;
  std::string degrade_command_topic_;
  std::string degrade_ack_topic_;
  std::string recovery_context_topic_;
  std::string estop_topic_;
  std::string system_mode_topic_;

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

  mutable std::mutex recovery_context_mutex_;
  std::string last_recovery_context_payload_;
  bool idle_spinning_active_{false};
  std::string last_system_mode_payload_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr degrade_ack_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr estop_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr degrade_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_context_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_mode_pub_;
  rclcpp::TimerBase::SharedPtr degrade_timeout_timer_;
};

}  // namespace dog_lifecycle