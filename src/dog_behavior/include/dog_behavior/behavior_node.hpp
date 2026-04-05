#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <dog_interfaces/action/execute_behavior.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <unordered_set>

namespace dog_behavior
{

class BehaviorNode : public rclcpp::Node
{
public:
  /// @brief Construct a behavior node with default options.
  BehaviorNode();
  /// @brief Construct a behavior node with explicit ROS node options.
  /// @param options ROS node options used for node initialization.
  explicit BehaviorNode(const rclcpp::NodeOptions & options);

  /// @brief Trigger an ExecuteBehavior action goal using the latest cached pose.
  /// @param behavior_name Behavior identifier forwarded to the action server.
  /// @return True when the goal request is accepted for sending.
  bool triggerExecuteBehavior(const std::string & behavior_name);
  /// @brief Get the current execution state as a string token.
  /// @return Lowercase execution state text.
  std::string getExecutionState() const;
  /// @brief Test helper that checks whether a task phase is blocked after recovery.
  /// @param task_phase Task phase token to query.
  /// @return True if the recovered completed phase is tracked.
  bool IsTaskPhaseRecoveredForTest(const std::string & task_phase) const;
  /// @brief Test helper that reports whether idle-spinning mode is active.
  /// @return True when behavior execution is gated by idle-spinning mode.
  bool IsIdleSpinningForTest() const;

private:
  using ExecuteBehavior = dog_interfaces::action::ExecuteBehavior;
  using ExecuteBehaviorGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteBehavior>;

  enum class ExecutionState
  {
    kIdle,
    kWaitingServer,
    kServerUnavailable,
    kSendingGoal,
    kRunning,
    kSucceeded,
    kFailed,
    kRejected,
    kTimeout,
  };

  /// @brief Convert odometry updates to global pose publications and cache latest pose.
  /// @param msg Incoming odometry message.
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  /// @brief Handle behavior trigger messages and forward them to action dispatch.
  /// @param msg Incoming trigger payload.
  void executeTriggerCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief Update recovery-derived execution filters.
  /// @param msg Recovery context payload.
  void recoveryContextCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief React to lifecycle system mode transitions.
  /// @param msg System mode payload.
  void systemModeCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief Periodically probe action server readiness.
  void actionServerWaitTimerCallback();
  /// @brief Cancel active goals when action feedback is stale beyond timeout.
  void feedbackWatchdogTimerCallback();
  /// @brief Handle action goal acceptance or rejection responses.
  /// @param goal_handle Goal handle returned by the action client.
  void goalResponseCallback(ExecuteBehaviorGoalHandle::SharedPtr goal_handle);
  /// @brief Track action feedback to refresh watchdog timestamps.
  /// @param goal_handle Goal handle associated with feedback.
  /// @param feedback Action feedback payload.
  void feedbackCallback(
    ExecuteBehaviorGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const ExecuteBehavior::Feedback> feedback);
  /// @brief Handle terminal action result and update execution state.
  /// @param result Wrapped action result.
  void resultCallback(const ExecuteBehaviorGoalHandle::WrappedResult & result);
  /// @brief Check whether current internal state allows sending a new goal.
  /// @return True when action server is ready and no goal is in-flight.
  bool canSendGoalLocked() const;
  /// @brief Convert internal execution state enum to stable text representation.
  /// @param state Execution state enum value.
  /// @return Lowercase state token.
  std::string executionStateToString(ExecutionState state) const;
  /// @brief Normalize free-form tokens by lowercasing and removing spaces.
  /// @param value Raw token text.
  /// @return Normalized token.
  static std::string normalizeToken(const std::string & value);
  /// @brief Parse key-value payload text and decode percent-encoded values.
  /// @param payload Input payload using semicolon-separated key-value pairs.
  /// @param key Target key to extract.
  /// @return Decoded value for the key, or empty string when missing.
  static std::string parseKeyValuePayload(const std::string & payload, const std::string & key);
  /// @brief Determine whether a target state represents completion.
  /// @param target_state State token.
  /// @return True for completed terminal states.
  bool isCompletedState(const std::string & target_state) const;
  /// @brief Validate that all pose components are finite.
  /// @param pose Pose to validate.
  /// @return True when no NaN/Inf values are present.
  bool isFinitePose(const geometry_msgs::msg::Pose & pose) const;
  /// @brief Validate quaternion norm against expected unit norm tolerance.
  /// @param pose Pose whose orientation quaternion is checked.
  /// @return True when quaternion norm is within accepted bounds.
  bool hasValidQuaternionNorm(const geometry_msgs::msg::Pose & pose) const;

  std::string default_frame_id_;
  std::string execute_behavior_action_name_;
  std::string execute_behavior_trigger_topic_;
  std::string recovery_context_topic_;
  std::string system_mode_topic_;
  double action_server_wait_timeout_sec_;
  double feedback_timeout_sec_;

  mutable std::mutex state_mutex_;
  ExecutionState execution_state_;
  bool action_server_ready_;
  bool action_goal_pending_;
  bool action_goal_active_;
  bool cancel_due_to_idle_spinning_;
  bool idle_spinning_mode_active_;
  bool has_latest_pose_;
  std::unordered_set<std::string> recovered_completed_task_phases_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  rclcpp::Time action_server_wait_start_time_;
  rclcpp::Time last_feedback_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execute_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_context_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
  rclcpp::TimerBase::SharedPtr action_server_wait_timer_;
  rclcpp::TimerBase::SharedPtr feedback_watchdog_timer_;
  rclcpp_action::Client<ExecuteBehavior>::SharedPtr execute_behavior_client_;
  ExecuteBehaviorGoalHandle::SharedPtr active_goal_handle_;
};

}  // namespace dog_behavior