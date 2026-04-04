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
  BehaviorNode();
  explicit BehaviorNode(const rclcpp::NodeOptions & options);

  bool triggerExecuteBehavior(const std::string & behavior_name);
  std::string getExecutionState() const;
  bool IsTaskPhaseRecoveredForTest(const std::string & task_phase) const;
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

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void executeTriggerCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void recoveryContextCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void systemModeCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void actionServerWaitTimerCallback();
  void feedbackWatchdogTimerCallback();
  void goalResponseCallback(ExecuteBehaviorGoalHandle::SharedPtr goal_handle);
  void feedbackCallback(
    ExecuteBehaviorGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const ExecuteBehavior::Feedback> feedback);
  void resultCallback(const ExecuteBehaviorGoalHandle::WrappedResult & result);
  bool canSendGoalLocked() const;
  std::string executionStateToString(ExecutionState state) const;
  static std::string normalizeToken(const std::string & value);
  static std::string parseKeyValuePayload(const std::string & payload, const std::string & key);
  bool isCompletedState(const std::string & target_state) const;
  bool isFinitePose(const geometry_msgs::msg::Pose & pose) const;
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