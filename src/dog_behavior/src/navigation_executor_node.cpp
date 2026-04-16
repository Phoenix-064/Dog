#include "dog_behavior/navigation_executor_node.hpp"
#include "dog_behavior/common/payload_utils.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <utility>

namespace dog_behavior
{

namespace
{
}  // namespace

NavigationExecutorNode::NavigationExecutorNode()
: NavigationExecutorNode(rclcpp::NodeOptions())
{
}

NavigationExecutorNode::NavigationExecutorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dog_navigation_executor", options)
, execution_state_(ExecutionState::kIdle)
, nav2_server_ready_(false)
, internal_goal_active_(false)
, forwarding_goal_pending_(false)
, timeout_terminal_(false)
, cancel_due_to_idle_spinning_(false)
, idle_spinning_mode_active_(false)
, navigation_blocked_by_recovery_(false)
{
  navigate_execute_action_name_ = declare_parameter<std::string>(
    "navigate_execute_action_name",
    "/behavior/navigate_execute");
  nav2_action_name_ = declare_parameter<std::string>("nav2_action_name", "/navigate_to_pose");
  execution_state_topic_ = declare_parameter<std::string>(
    "navigate_execution_state_topic",
    "/behavior/nav_exec_state");
  recovery_context_topic_ = declare_parameter<std::string>(
    "recovery_context_topic",
    "/lifecycle/recovery_context");
  system_mode_topic_ = declare_parameter<std::string>(
    "system_mode_topic",
    "/lifecycle/system_mode");
  nav2_server_wait_timeout_sec_ = declare_parameter<double>("nav2_server_wait_timeout_sec", 10.0);
  feedback_timeout_sec_ = declare_parameter<double>("navigate_feedback_timeout_sec", 10.0);

  if (nav2_server_wait_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid nav2_server_wait_timeout_sec=%.3f, fallback to 10.0s",
      nav2_server_wait_timeout_sec_);
    nav2_server_wait_timeout_sec_ = 10.0;
  }

  if (feedback_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid navigate_feedback_timeout_sec=%.3f, fallback to 10.0s",
      feedback_timeout_sec_);
    feedback_timeout_sec_ = 10.0;
  }

  nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);
  navigate_execute_server_ = rclcpp_action::create_server<NavigateToPose>(
    this,
    navigate_execute_action_name_,
    std::bind(&NavigationExecutorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavigationExecutorNode::handleCancel, this, std::placeholders::_1),
    std::bind(&NavigationExecutorNode::handleAccepted, this, std::placeholders::_1));

  execution_state_pub_ = create_publisher<std_msgs::msg::String>(
    execution_state_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  recovery_context_sub_ = create_subscription<std_msgs::msg::String>(
    recovery_context_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&NavigationExecutorNode::recoveryContextCallback, this, std::placeholders::_1));

  system_mode_sub_ = create_subscription<std_msgs::msg::String>(
    system_mode_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&NavigationExecutorNode::systemModeCallback, this, std::placeholders::_1));

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    execution_state_ = ExecutionState::kWaitingNav2Server;
    nav2_server_wait_start_time_ = now();
  }

  nav2_server_wait_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&NavigationExecutorNode::nav2ServerWaitTimerCallback, this));

  feedback_watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&NavigationExecutorNode::feedbackWatchdogTimerCallback, this));

  publishExecutionState(ExecutionState::kWaitingNav2Server);

  RCLCPP_INFO(
    get_logger(),
    "dog_navigation_executor initialized, internal_action=%s, nav2_action=%s, state_topic=%s, recovery_topic=%s, system_mode_topic=%s",
    navigate_execute_action_name_.c_str(),
    nav2_action_name_.c_str(),
    execution_state_topic_.c_str(),
    recovery_context_topic_.c_str(),
    system_mode_topic_.c_str());
}

rclcpp_action::GoalResponse NavigationExecutorNode::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  if (!goal) {
    RCLCPP_WARN(get_logger(), "Reject null navigation goal request");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!canAcceptGoalLocked()) {
    RCLCPP_WARN(
      get_logger(),
      "Reject navigation goal because executor is busy or unavailable: state=%s",
      executionStateToString(execution_state_).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (idle_spinning_mode_active_) {
    RCLCPP_WARN(get_logger(), "Reject navigation goal in idle/degraded mode");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (navigation_blocked_by_recovery_) {
    RCLCPP_WARN(get_logger(), "Reject navigation goal due to recovery_context completed state");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!utils::isFinitePose(goal->pose) || !utils::hasValidQuaternionNorm(goal->pose)) {
    RCLCPP_WARN(get_logger(), "Reject navigation goal due to invalid pose values");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationExecutorNode::handleCancel(
  const std::shared_ptr<NavigateGoalHandleServer>)
{
  NavigateGoalHandleClient::SharedPtr nav2_goal_handle;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    nav2_goal_handle = active_nav2_goal_handle_;
    internal_goal_active_ = false;
    forwarding_goal_pending_ = false;
    timeout_terminal_ = false;
    active_internal_goal_request_.reset();
    active_internal_goal_handle_.reset();
    active_nav2_goal_handle_.reset();
    execution_state_ = ExecutionState::kIdle;
  }

  publishExecutionState(ExecutionState::kIdle);
  if (nav2_goal_handle) {
    nav2_client_->async_cancel_goal(nav2_goal_handle);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationExecutorNode::handleAccepted(const std::shared_ptr<NavigateGoalHandleServer> goal_handle)
{
  std::shared_ptr<const NavigateToPose::Goal> goal;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    goal = goal_handle->get_goal();
    active_internal_goal_request_ = goal;
    active_internal_goal_handle_ = goal_handle;
    internal_goal_active_ = true;
    forwarding_goal_pending_ = true;
    timeout_terminal_ = false;
    execution_state_ = ExecutionState::kForwardingGoal;
  }
  publishExecutionState(ExecutionState::kForwardingGoal);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback =
    [this](NavigateGoalHandleClient::SharedPtr nav2_goal_handle) {
      this->nav2GoalResponseCallback(nav2_goal_handle);
    };
  send_goal_options.feedback_callback =
    std::bind(&NavigationExecutorNode::nav2FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&NavigationExecutorNode::nav2ResultCallback, this, std::placeholders::_1);

  nav2_client_->async_send_goal(*goal, send_goal_options);
}

void NavigationExecutorNode::nav2GoalResponseCallback(NavigateGoalHandleClient::SharedPtr goal_handle)
{
  std::shared_ptr<NavigateGoalHandleServer> internal_goal_handle;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    internal_goal_handle = active_internal_goal_handle_;
    forwarding_goal_pending_ = false;
    if (!goal_handle) {
      execution_state_ = ExecutionState::kRejected;
      internal_goal_active_ = false;
      active_internal_goal_request_.reset();
      active_internal_goal_handle_.reset();
      active_nav2_goal_handle_.reset();
      timeout_terminal_ = false;
    } else {
      active_nav2_goal_handle_ = goal_handle;
      last_feedback_time_ = now();
      execution_state_ = ExecutionState::kRunning;
    }
  }

  if (!goal_handle) {
    publishExecutionState(ExecutionState::kRejected);
    if (internal_goal_handle && internal_goal_handle->is_active()) {
      auto result = std::make_shared<NavigateToPose::Result>();
      internal_goal_handle->abort(result);
    }
    return;
  }

  publishExecutionState(ExecutionState::kRunning);
}

void NavigationExecutorNode::nav2FeedbackCallback(
  NavigateGoalHandleClient::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  std::shared_ptr<NavigateGoalHandleServer> internal_goal_handle;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!internal_goal_active_) {
      return;
    }

    last_feedback_time_ = now();
    internal_goal_handle = active_internal_goal_handle_;
  }

  if (internal_goal_handle && internal_goal_handle->is_active()) {
    internal_goal_handle->publish_feedback(std::make_shared<NavigateToPose::Feedback>(*feedback));
  }
}

void NavigationExecutorNode::nav2ResultCallback(const NavigateGoalHandleClient::WrappedResult & result)
{
  std::shared_ptr<NavigateGoalHandleServer> internal_goal_handle;
  bool timeout_terminal = false;
  bool canceled_by_idle = false;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    internal_goal_handle = active_internal_goal_handle_;
    timeout_terminal = timeout_terminal_;
    canceled_by_idle = cancel_due_to_idle_spinning_;
    cancel_due_to_idle_spinning_ = false;

    internal_goal_active_ = false;
    forwarding_goal_pending_ = false;
    timeout_terminal_ = false;
    active_internal_goal_request_.reset();
    active_internal_goal_handle_.reset();
    active_nav2_goal_handle_.reset();
    execution_state_ = mapNav2ResultState(result.code, timeout_terminal, canceled_by_idle);
  }

  const auto final_state = mapNav2ResultState(result.code, timeout_terminal, canceled_by_idle);

  publishExecutionState(final_state);

  if (!internal_goal_handle || !internal_goal_handle->is_active()) {
    return;
  }

  auto internal_result = std::make_shared<NavigateToPose::Result>();
  if (result.result) {
    *internal_result = *result.result;
  }

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    internal_goal_handle->succeed(internal_result);
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    internal_goal_handle->canceled(internal_result);
  } else {
    internal_goal_handle->abort(internal_result);
  }
}

NavigationExecutorNode::ExecutionState NavigationExecutorNode::mapNav2ResultState(
  const rclcpp_action::ResultCode result_code,
  const bool timeout_terminal,
  const bool canceled_by_idle) const
{
  switch (result_code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return ExecutionState::kSucceeded;
    case rclcpp_action::ResultCode::CANCELED:
      if (canceled_by_idle) {
        return ExecutionState::kIdle;
      }
      if (timeout_terminal) {
        return ExecutionState::kTimeout;
      }
      return ExecutionState::kFailed;
    case rclcpp_action::ResultCode::ABORTED:
    default:
      return ExecutionState::kFailed;
  }
}

void NavigationExecutorNode::nav2ServerWaitTimerCallback()
{
  if (nav2_client_->wait_for_action_server(std::chrono::seconds(0))) {
    bool changed_to_ready = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!nav2_server_ready_) {
        nav2_server_ready_ = true;
        execution_state_ = ExecutionState::kIdle;
        changed_to_ready = true;
      }
    }
    if (changed_to_ready) {
      publishExecutionState(ExecutionState::kIdle);
      RCLCPP_INFO(get_logger(), "Nav2 action server is ready: action_name=%s", nav2_action_name_.c_str());
    }
    nav2_server_wait_timer_.reset();
    return;
  }

  const double elapsed_sec = (now() - nav2_server_wait_start_time_).seconds();
  if (elapsed_sec >= nav2_server_wait_timeout_sec_) {
    bool changed_to_unavailable = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!nav2_server_ready_) {
        execution_state_ = ExecutionState::kNav2ServerUnavailable;
        changed_to_unavailable = true;
      }
    }
    if (changed_to_unavailable) {
      publishExecutionState(ExecutionState::kNav2ServerUnavailable);
      RCLCPP_ERROR(
        get_logger(),
        "Nav2 action server unavailable after timeout: action_name=%s timeout=%.2fs",
        nav2_action_name_.c_str(),
        nav2_server_wait_timeout_sec_);
    }
    nav2_server_wait_timer_.reset();
  }
}

void NavigationExecutorNode::feedbackWatchdogTimerCallback()
{
  NavigateGoalHandleClient::SharedPtr nav2_goal_handle;
  std::shared_ptr<NavigateGoalHandleServer> internal_goal_handle;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!internal_goal_active_ || forwarding_goal_pending_) {
      return;
    }

    const double feedback_elapsed_sec = (now() - last_feedback_time_).seconds();
    if (feedback_elapsed_sec <= feedback_timeout_sec_) {
      return;
    }

    execution_state_ = ExecutionState::kTimeout;
    timeout_terminal_ = true;
    internal_goal_active_ = false;
    forwarding_goal_pending_ = false;

    nav2_goal_handle = active_nav2_goal_handle_;
    internal_goal_handle = active_internal_goal_handle_;

    active_internal_goal_request_.reset();
    active_internal_goal_handle_.reset();
    active_nav2_goal_handle_.reset();
  }

  publishExecutionState(ExecutionState::kTimeout);

  if (nav2_goal_handle) {
    nav2_client_->async_cancel_goal(nav2_goal_handle);
  }

  if (internal_goal_handle && internal_goal_handle->is_active()) {
    auto result = std::make_shared<NavigateToPose::Result>();
    internal_goal_handle->abort(result);
  }
}

void NavigationExecutorNode::systemModeCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto mode = utils::normalizeToken(utils::parseKeyValuePayload(msg->data, "mode"));
  if (mode.empty()) {
    return;
  }

  const bool is_idle_spinning = (mode == "idle_spinning" || mode == "degraded");
  NavigateGoalHandleClient::SharedPtr nav2_goal_handle;
  std::shared_ptr<NavigateGoalHandleServer> internal_goal_handle;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (idle_spinning_mode_active_ == is_idle_spinning) {
      return;
    }

    idle_spinning_mode_active_ = is_idle_spinning;
    if (is_idle_spinning && internal_goal_active_) {
      cancel_due_to_idle_spinning_ = true;
      nav2_goal_handle = active_nav2_goal_handle_;
      internal_goal_handle = active_internal_goal_handle_;

      internal_goal_active_ = false;
      forwarding_goal_pending_ = false;
      timeout_terminal_ = false;
      execution_state_ = ExecutionState::kIdle;
      active_internal_goal_request_.reset();
      active_internal_goal_handle_.reset();
      active_nav2_goal_handle_.reset();
    }
  }

  if (is_idle_spinning) {
    publishExecutionState(ExecutionState::kIdle);
  }

  if (nav2_goal_handle) {
    nav2_client_->async_cancel_goal(nav2_goal_handle);
  }

  if (internal_goal_handle && internal_goal_handle->is_active()) {
    auto result = std::make_shared<NavigateToPose::Result>();
    internal_goal_handle->abort(result);
  }

  RCLCPP_WARN(get_logger(), "system_mode_updated mode=%s payload=%s", mode.c_str(), msg->data.c_str());
}

void NavigationExecutorNode::recoveryContextCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  const auto mode = utils::normalizeToken(utils::parseKeyValuePayload(msg->data, "mode"));
  const auto target_state = utils::normalizeToken(utils::parseKeyValuePayload(msg->data, "target_state"));

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (mode == "cold_start") {
    navigation_blocked_by_recovery_ = false;
    return;
  }

  if (mode != "recovered") {
    return;
  }

  navigation_blocked_by_recovery_ = utils::isCompletedState(target_state);
}

bool NavigationExecutorNode::canAcceptGoalLocked() const
{
  return nav2_server_ready_ && !internal_goal_active_ && !forwarding_goal_pending_;
}

void NavigationExecutorNode::publishExecutionState(ExecutionState state)
{
  std_msgs::msg::String msg;
  msg.data = executionStateToString(state);
  execution_state_pub_->publish(msg);
}

std::string NavigationExecutorNode::executionStateToString(ExecutionState state) const
{
  switch (state) {
    case ExecutionState::kIdle:
      return "idle";
    case ExecutionState::kWaitingNav2Server:
      return "waiting_nav2_server";
    case ExecutionState::kNav2ServerUnavailable:
      return "nav2_server_unavailable";
    case ExecutionState::kForwardingGoal:
      return "forwarding_goal";
    case ExecutionState::kRunning:
      return "running";
    case ExecutionState::kSucceeded:
      return "succeeded";
    case ExecutionState::kFailed:
      return "failed";
    case ExecutionState::kRejected:
      return "rejected";
    case ExecutionState::kTimeout:
      return "timeout";
    default:
      return "unknown";
  }
}

std::string NavigationExecutorNode::getExecutionState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return executionStateToString(execution_state_);
}

}  // namespace dog_behavior
