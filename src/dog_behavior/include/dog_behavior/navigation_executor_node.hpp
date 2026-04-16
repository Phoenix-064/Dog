#pragma once

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>

namespace dog_behavior
{

class NavigationExecutorNode : public rclcpp::Node
{
public:
  NavigationExecutorNode();
  explicit NavigationExecutorNode(const rclcpp::NodeOptions & options);

  std::string getExecutionState() const;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandleClient = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using NavigateGoalHandleServer = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  enum class ExecutionState
  {
    kIdle,
    kWaitingNav2Server,
    kNav2ServerUnavailable,
    kForwardingGoal,
    kRunning,
    kSucceeded,
    kFailed,
    kRejected,
    kTimeout,
  };

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<NavigateGoalHandleServer> goal_handle);
  void handleAccepted(const std::shared_ptr<NavigateGoalHandleServer> goal_handle);

  void nav2GoalResponseCallback(NavigateGoalHandleClient::SharedPtr goal_handle);
  void nav2FeedbackCallback(
    NavigateGoalHandleClient::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void nav2ResultCallback(const NavigateGoalHandleClient::WrappedResult & result);

  void nav2ServerWaitTimerCallback();
  void feedbackWatchdogTimerCallback();
  void systemModeCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void recoveryContextCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  bool canAcceptGoalLocked() const;
  ExecutionState mapNav2ResultState(
    rclcpp_action::ResultCode result_code,
    bool timeout_terminal,
    bool canceled_by_idle) const;
  void publishExecutionState(ExecutionState state);
  std::string executionStateToString(ExecutionState state) const;

  std::string navigate_execute_action_name_;
  std::string nav2_action_name_;
  std::string execution_state_topic_;
  std::string recovery_context_topic_;
  std::string system_mode_topic_;
  double nav2_server_wait_timeout_sec_;
  double feedback_timeout_sec_;

  mutable std::mutex state_mutex_;
  ExecutionState execution_state_;
  bool nav2_server_ready_;
  bool internal_goal_active_;
  bool forwarding_goal_pending_;
  bool timeout_terminal_;
  bool cancel_due_to_idle_spinning_;
  bool idle_spinning_mode_active_;
  bool navigation_blocked_by_recovery_;
  rclcpp::Time nav2_server_wait_start_time_;
  rclcpp::Time last_feedback_time_;

  std::shared_ptr<const NavigateToPose::Goal> active_internal_goal_request_;
  std::shared_ptr<NavigateGoalHandleServer> active_internal_goal_handle_;
  NavigateGoalHandleClient::SharedPtr active_nav2_goal_handle_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_context_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_state_pub_;
  rclcpp::TimerBase::SharedPtr nav2_server_wait_timer_;
  rclcpp::TimerBase::SharedPtr feedback_watchdog_timer_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_execute_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
};

}  // namespace dog_behavior
