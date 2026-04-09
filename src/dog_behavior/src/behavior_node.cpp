#include "dog_behavior/behavior_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <cctype>
#include <chrono>
#include <functional>
#include <utility>

namespace dog_behavior
{

namespace
{

/// @brief 将十六进制字符转换为对应数值。
/// @param c 十六进制字符。
/// @return 有效时返回 0-15，无效时返回 -1。
int hexDigitValue(const char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return 10 + (c - 'a');
  }
  if (c >= 'A' && c <= 'F') {
    return 10 + (c - 'A');
  }
  return -1;
}

/// @brief 对百分号编码字符串执行解码。
/// @param value 可能包含 `%xx` 编码片段的字符串。
/// @return 解码后的字符串。
std::string percentDecode(const std::string & value)
{
  std::string decoded;
  decoded.reserve(value.size());

  size_t index = 0;
  while (index < value.size()) {
    if (value[index] == '%' && (index + 2) < value.size()) {
      const auto high = hexDigitValue(value[index + 1]);
      const auto low = hexDigitValue(value[index + 2]);
      if (high >= 0 && low >= 0) {
        const auto byte_value = static_cast<unsigned char>((high << 4) | low);
        decoded.push_back(static_cast<char>(byte_value));
        index += 3;
        continue;
      }
    }

    decoded.push_back(value[index]);
    ++index;
  }

  return decoded;
}

std::string getBehaviorTreeXmlPath()
{
  const auto package_share = ament_index_cpp::get_package_share_directory("dog_behavior");
  return package_share + "/config/execute_trigger_tree.xml";
}

}  // namespace

BehaviorNode::BehaviorNode()
: BehaviorNode(rclcpp::NodeOptions())
{
}

BehaviorNode::BehaviorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dog_behavior", options)
, execution_state_(ExecutionState::kIdle)
, action_server_ready_(false)
, action_goal_pending_(false)
, action_goal_active_(false)
, navigate_goal_pending_(false)
, navigate_goal_active_(false)
, navigate_server_ready_(false)
, cancel_due_to_idle_spinning_(false)
, idle_spinning_mode_active_(false)
, has_latest_pose_(false)
, behavior_tree_(
    [this](const std::string & behavior_name) {
      return this->triggerExecuteBehavior(behavior_name);
    },
    [this](const std::string & behavior_name) {
      return this->triggerNavigateGoal(behavior_name);
    },
    []() {},
    getBehaviorTreeXmlPath())
{
  const auto global_pose_topic = declare_parameter<std::string>("global_pose_topic", "/dog/global_pose");
  const auto localization_topic = declare_parameter<std::string>("localization_topic", "/localization/dog");
  default_frame_id_ = declare_parameter<std::string>("default_frame_id", "base_link");
  execute_behavior_action_name_ = declare_parameter<std::string>(
    "execute_behavior_action_name",
    "/behavior/execute");
  execute_behavior_trigger_topic_ = declare_parameter<std::string>(
    "execute_behavior_trigger_topic",
    "/behavior/execute_trigger");
  navigate_execute_action_name_ = declare_parameter<std::string>(
    "navigate_execute_action_name",
    "/behavior/navigate_execute");
  recovery_context_topic_ = declare_parameter<std::string>(
    "recovery_context_topic",
    "/lifecycle/recovery_context");
  system_mode_topic_ = declare_parameter<std::string>(
    "system_mode_topic",
    "/lifecycle/system_mode");
  action_server_wait_timeout_sec_ = declare_parameter<double>("action_server_wait_timeout_sec", 5.0);
  feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 2.0);

  if (action_server_wait_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid action_server_wait_timeout_sec=%.3f, fallback to 5.0s",
      action_server_wait_timeout_sec_);
    action_server_wait_timeout_sec_ = 5.0;
  }

  if (feedback_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid feedback_timeout_sec=%.3f, fallback to 2.0s",
      feedback_timeout_sec_);
    feedback_timeout_sec_ = 2.0;
  }

  auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(20));
  pose_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  global_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    global_pose_topic,
    pose_qos);

  auto odom_qos = rclcpp::SensorDataQoS();
  odom_qos.keep_last(20);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    localization_topic,
    odom_qos,
    std::bind(&BehaviorNode::odomCallback, this, std::placeholders::_1));

  execute_behavior_client_ = rclcpp_action::create_client<ExecuteBehavior>(
    this,
    execute_behavior_action_name_);
  navigate_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    navigate_execute_action_name_);

  execute_trigger_sub_ = create_subscription<std_msgs::msg::String>(
    execute_behavior_trigger_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    std::bind(&BehaviorNode::executeTriggerCallback, this, std::placeholders::_1));

  recovery_context_sub_ = create_subscription<std_msgs::msg::String>(
    recovery_context_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&BehaviorNode::recoveryContextCallback, this, std::placeholders::_1));
  system_mode_sub_ = create_subscription<std_msgs::msg::String>(
    system_mode_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&BehaviorNode::systemModeCallback, this, std::placeholders::_1));

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    execution_state_ = ExecutionState::kWaitingServer;
    action_server_wait_start_time_ = now();
    navigate_server_wait_start_time_ = now();
  }

  action_server_wait_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&BehaviorNode::actionServerWaitTimerCallback, this));
  navigate_server_wait_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    [this]() {
      if (navigate_client_->wait_for_action_server(std::chrono::seconds(0))) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        navigate_server_ready_ = true;
        navigate_server_wait_timer_.reset();
      }
    });

  feedback_watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&BehaviorNode::feedbackWatchdogTimerCallback, this));
  navigate_feedback_watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    [this]() {
      NavigateGoalHandle::SharedPtr goal_handle_to_cancel;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!navigate_goal_active_) {
          return;
        }

        const double feedback_elapsed_sec = (now() - navigate_last_feedback_time_).seconds();
        if (feedback_elapsed_sec <= feedback_timeout_sec_) {
          return;
        }

        execution_state_ = ExecutionState::kTimeout;
        navigate_goal_active_ = false;
        navigate_goal_pending_ = false;
        goal_handle_to_cancel = active_navigate_goal_handle_;
        active_navigate_goal_handle_.reset();
      }

      if (goal_handle_to_cancel) {
        navigate_client_->async_cancel_goal(goal_handle_to_cancel);
      }
    });

  RCLCPP_INFO(
    get_logger(),
    "dog_behavior initialized, localization_topic=%s, global_pose_topic=%s, default_frame_id=%s, action_name=%s, trigger_topic=%s, recovery_topic=%s, system_mode_topic=%s",
    localization_topic.c_str(),
    global_pose_topic.c_str(),
    default_frame_id_.c_str(),
    execute_behavior_action_name_.c_str(),
    execute_behavior_trigger_topic_.c_str(),
    recovery_context_topic_.c_str(),
    system_mode_topic_.c_str());
}

bool BehaviorNode::triggerNavigateGoal(const std::string & behavior_name)
{
  (void)behavior_name;

  NavigateToPose::Goal goal;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (idle_spinning_mode_active_) {
      RCLCPP_WARN(get_logger(), "skip navigate goal during idle_spinning mode");
      return false;
    }

    if (!navigate_server_ready_) {
      RCLCPP_WARN(get_logger(), "Cannot send navigate goal because navigation executor is unavailable");
      return false;
    }

    if (navigate_goal_pending_ || navigate_goal_active_) {
      RCLCPP_WARN(get_logger(), "Cannot send navigate goal because one goal is in-flight");
      return false;
    }

    if (!has_latest_pose_) {
      RCLCPP_WARN(get_logger(), "Cannot send navigate goal because latest pose is unavailable");
      return false;
    }

    goal.pose = latest_pose_;
    navigate_goal_pending_ = true;
    execution_state_ = ExecutionState::kSendingGoal;
  }

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback =
    [this](NavigateGoalHandle::SharedPtr goal_handle) {
      this->navigateGoalResponseCallback(goal_handle);
    };
  send_goal_options.feedback_callback =
    std::bind(&BehaviorNode::navigateFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&BehaviorNode::navigateResultCallback, this, std::placeholders::_1);

  navigate_client_->async_send_goal(goal, send_goal_options);
  RCLCPP_INFO(
    get_logger(),
    "Sent navigate goal to internal executor: action_name=%s frame_id=%s",
    navigate_execute_action_name_.c_str(),
    goal.pose.header.frame_id.c_str());
  return true;
}

void BehaviorNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = msg->header.frame_id;
  pose_msg.pose = msg->pose.pose;

  if (pose_msg.header.frame_id.empty()) {
    if (default_frame_id_.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Drop pose publish because input frame_id is empty and default_frame_id is not configured (stamp=%u.%u)",
        pose_msg.header.stamp.sec,
        pose_msg.header.stamp.nanosec);
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Input frame_id is empty at stamp=%u.%u, fallback to default_frame_id=%s",
      pose_msg.header.stamp.sec,
      pose_msg.header.stamp.nanosec,
      default_frame_id_.c_str());
    pose_msg.header.frame_id = default_frame_id_;
  }

  if (!isFinitePose(pose_msg.pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop pose publish because pose contains non-finite values (frame_id=%s)",
      pose_msg.header.frame_id.c_str());
    return;
  }

  if (!hasValidQuaternionNorm(pose_msg.pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop pose publish because quaternion norm is invalid (frame_id=%s)",
      pose_msg.header.frame_id.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_pose_ = pose_msg;
    has_latest_pose_ = true;
  }

  global_pose_pub_->publish(pose_msg);
}

void BehaviorNode::executeTriggerCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!behavior_tree_.execute(msg->data)) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to execute behavior tree from topic=%s behavior_name=%s",
      execute_behavior_trigger_topic_.c_str(),
      msg->data.c_str());
  }
}

void BehaviorNode::recoveryContextCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  const auto mode = normalizeToken(parseKeyValuePayload(msg->data, "mode"));
  const auto task_phase = normalizeToken(parseKeyValuePayload(msg->data, "task_phase"));
  const auto target_state = normalizeToken(parseKeyValuePayload(msg->data, "target_state"));

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (mode == "cold_start") {
    recovered_completed_task_phases_.clear();
    RCLCPP_INFO(get_logger(), "recovery_context_cold_start_received, cleared recovered task filters");
    return;
  }

  if (mode != "recovered") {
    RCLCPP_WARN(get_logger(), "ignore unknown recovery mode: %s", mode.c_str());
    return;
  }

  if (task_phase.empty()) {
    RCLCPP_WARN(get_logger(), "ignore recovered context with empty task_phase");
    return;
  }

  if (!isCompletedState(target_state)) {
    recovered_completed_task_phases_.erase(task_phase);
    RCLCPP_INFO(
      get_logger(),
      "recovered task remains executable, task_phase=%s target_state=%s, removed from recovered block list",
      task_phase.c_str(),
      target_state.c_str());
    return;
  }

  recovered_completed_task_phases_.insert(task_phase);
  RCLCPP_INFO(
    get_logger(),
    "recovered completed task phase is blocked from re-execution: %s",
    task_phase.c_str());
}

void BehaviorNode::systemModeCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto mode = normalizeToken(parseKeyValuePayload(msg->data, "mode"));
  if (mode.empty()) {
    return;
  }

  const bool is_idle_spinning = (mode == "idle_spinning" || mode == "degraded");
  ExecuteBehaviorGoalHandle::SharedPtr goal_handle_to_cancel;
  NavigateGoalHandle::SharedPtr navigate_goal_handle_to_cancel;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (idle_spinning_mode_active_ == is_idle_spinning) {
      return;
    }

    idle_spinning_mode_active_ = is_idle_spinning;
    if (is_idle_spinning && action_goal_active_ && active_goal_handle_) {
      goal_handle_to_cancel = active_goal_handle_;
      cancel_due_to_idle_spinning_ = true;
      action_goal_active_ = false;
      action_goal_pending_ = false;
      active_goal_handle_.reset();
      execution_state_ = ExecutionState::kIdle;
    }

    if (is_idle_spinning && navigate_goal_active_ && active_navigate_goal_handle_) {
      navigate_goal_handle_to_cancel = active_navigate_goal_handle_;
      navigate_goal_active_ = false;
      navigate_goal_pending_ = false;
      active_navigate_goal_handle_.reset();
      execution_state_ = ExecutionState::kIdle;
    }
  }

  if (goal_handle_to_cancel) {
    execute_behavior_client_->async_cancel_goal(goal_handle_to_cancel);
  }
  if (navigate_goal_handle_to_cancel) {
    navigate_client_->async_cancel_goal(navigate_goal_handle_to_cancel);
  }

  RCLCPP_WARN(
    get_logger(),
    "system_mode_updated mode=%s payload=%s",
    mode.c_str(),
    msg->data.c_str());
}

void BehaviorNode::navigateGoalResponseCallback(NavigateGoalHandle::SharedPtr goal_handle)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!goal_handle) {
    execution_state_ = ExecutionState::kRejected;
    navigate_goal_pending_ = false;
    navigate_goal_active_ = false;
    RCLCPP_ERROR(get_logger(), "Navigate goal rejected: action_name=%s", navigate_execute_action_name_.c_str());
    return;
  }

  active_navigate_goal_handle_ = goal_handle;
  navigate_goal_pending_ = false;
  navigate_goal_active_ = true;
  navigate_last_feedback_time_ = now();
  execution_state_ = ExecutionState::kRunning;
  RCLCPP_INFO(get_logger(), "Navigate goal accepted: action_name=%s", navigate_execute_action_name_.c_str());
}

void BehaviorNode::navigateFeedbackCallback(
  NavigateGoalHandle::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback>)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!navigate_goal_active_) {
    return;
  }

  navigate_last_feedback_time_ = now();
}

void BehaviorNode::navigateResultCallback(const NavigateGoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  navigate_goal_pending_ = false;
  navigate_goal_active_ = false;
  active_navigate_goal_handle_.reset();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      execution_state_ = ExecutionState::kSucceeded;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      execution_state_ = ExecutionState::kFailed;
      break;
    case rclcpp_action::ResultCode::ABORTED:
    default:
      execution_state_ = ExecutionState::kFailed;
      break;
  }
}

void BehaviorNode::actionServerWaitTimerCallback()
{
  if (execute_behavior_client_->wait_for_action_server(std::chrono::seconds(0))) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!action_server_ready_) {
      action_server_ready_ = true;
      execution_state_ = ExecutionState::kIdle;
      RCLCPP_INFO(
        get_logger(),
        "Action server is ready: action_name=%s",
        execute_behavior_action_name_.c_str());
    }
    action_server_wait_timer_.reset();
    return;
  }

  const double elapsed_sec = (now() - action_server_wait_start_time_).seconds();
  if (elapsed_sec >= action_server_wait_timeout_sec_) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!action_server_ready_) {
      execution_state_ = ExecutionState::kServerUnavailable;
      RCLCPP_ERROR(
        get_logger(),
        "Action server unavailable after timeout: action_name=%s timeout=%.2fs",
        execute_behavior_action_name_.c_str(),
        action_server_wait_timeout_sec_);
    }
    action_server_wait_timer_.reset();
  }
}

void BehaviorNode::feedbackWatchdogTimerCallback()
{
  ExecuteBehaviorGoalHandle::SharedPtr goal_handle_to_cancel;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!action_goal_active_) {
      return;
    }

    const double feedback_elapsed_sec = (now() - last_feedback_time_).seconds();
    if (feedback_elapsed_sec <= feedback_timeout_sec_) {
      return;
    }

    execution_state_ = ExecutionState::kTimeout;
    action_goal_active_ = false;
    action_goal_pending_ = false;
    goal_handle_to_cancel = active_goal_handle_;
    active_goal_handle_.reset();

    RCLCPP_ERROR(
      get_logger(),
      "Action feedback timeout: action_name=%s timeout=%.2fs",
      execute_behavior_action_name_.c_str(),
      feedback_timeout_sec_);
  }

  if (goal_handle_to_cancel) {
    execute_behavior_client_->async_cancel_goal(goal_handle_to_cancel);
  }
}

bool BehaviorNode::triggerExecuteBehavior(const std::string & behavior_name)
{
  const auto normalized_behavior_name = normalizeToken(behavior_name);
  ExecuteBehavior::Goal goal;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (idle_spinning_mode_active_) {
      RCLCPP_WARN(
        get_logger(),
        "skip execute behavior during idle_spinning mode, behavior_name=%s",
        behavior_name.c_str());
      return false;
    }

    if (
      !normalized_behavior_name.empty() &&
      recovered_completed_task_phases_.find(normalized_behavior_name) != recovered_completed_task_phases_.end())
    {
      RCLCPP_INFO(
        get_logger(),
        "skip execute behavior due to recovered completed task phase=%s",
        normalized_behavior_name.c_str());
      return false;
    }

    if (!canSendGoalLocked()) {
      RCLCPP_WARN(
        get_logger(),
        "Cannot send action goal in current state: state=%s",
        executionStateToString(execution_state_).c_str());
      return false;
    }

    if (!has_latest_pose_) {
      RCLCPP_WARN(get_logger(), "Cannot send action goal because latest pose is unavailable");
      return false;
    }

    goal.behavior_name = behavior_name;
    goal.target_pose = latest_pose_;
    execution_state_ = ExecutionState::kSendingGoal;
    action_goal_pending_ = true;
  }

  rclcpp_action::Client<ExecuteBehavior>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback =
    [this](ExecuteBehaviorGoalHandle::SharedPtr goal_handle) {
      this->goalResponseCallback(goal_handle);
    };
  send_goal_options.feedback_callback =
    std::bind(&BehaviorNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&BehaviorNode::resultCallback, this, std::placeholders::_1);

  execute_behavior_client_->async_send_goal(goal, send_goal_options);
  RCLCPP_INFO(
    get_logger(),
    "Sent action goal: action_name=%s behavior_name=%s frame_id=%s",
    execute_behavior_action_name_.c_str(),
    goal.behavior_name.c_str(),
    goal.target_pose.header.frame_id.c_str());
  return true;
}

void BehaviorNode::goalResponseCallback(ExecuteBehaviorGoalHandle::SharedPtr goal_handle)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!goal_handle) {
    execution_state_ = ExecutionState::kRejected;
    action_goal_pending_ = false;
    action_goal_active_ = false;
    RCLCPP_ERROR(get_logger(), "Action goal rejected: action_name=%s", execute_behavior_action_name_.c_str());
    return;
  }

  active_goal_handle_ = goal_handle;
  action_goal_pending_ = false;
  action_goal_active_ = true;
  last_feedback_time_ = now();
  execution_state_ = ExecutionState::kRunning;
  RCLCPP_INFO(get_logger(), "Action goal accepted: action_name=%s", execute_behavior_action_name_.c_str());
}

void BehaviorNode::feedbackCallback(
  ExecuteBehaviorGoalHandle::SharedPtr,
  const std::shared_ptr<const ExecuteBehavior::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!action_goal_active_) {
    return;
  }

  last_feedback_time_ = now();
  RCLCPP_INFO(
    get_logger(),
    "Action feedback received: progress=%.3f state=%s",
    feedback->progress,
    feedback->state.c_str());
}

void BehaviorNode::resultCallback(const ExecuteBehaviorGoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto previous_state = execution_state_;
  const bool timeout_terminal = (previous_state == ExecutionState::kTimeout);
  const bool canceled_by_idle = cancel_due_to_idle_spinning_;
  cancel_due_to_idle_spinning_ = false;

  action_goal_pending_ = false;
  action_goal_active_ = false;
  active_goal_handle_.reset();

  const char * result_detail = (result.result && !result.result->detail.empty()) ?
    result.result->detail.c_str() : "<empty>";

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (result.result && result.result->accepted) {
        execution_state_ = ExecutionState::kSucceeded;
        RCLCPP_INFO(
          get_logger(),
          "Action result succeeded: accepted=%s detail=%s",
          result.result->accepted ? "true" : "false",
          result.result->detail.c_str());
      } else {
        execution_state_ = ExecutionState::kFailed;
        RCLCPP_ERROR(
          get_logger(),
          "Action result failed due to accepted=false: action_name=%s result_code=SUCCEEDED detail=%s",
          execute_behavior_action_name_.c_str(),
          result_detail);
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      execution_state_ = ExecutionState::kFailed;
      RCLCPP_ERROR(
        get_logger(),
        "Action result aborted: action_name=%s result_code=ABORTED detail=%s",
        execute_behavior_action_name_.c_str(),
        result_detail);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      if (canceled_by_idle) {
        execution_state_ = ExecutionState::kIdle;
      } else if (timeout_terminal) {
        execution_state_ = ExecutionState::kTimeout;
      } else {
        execution_state_ = ExecutionState::kFailed;
      }
      RCLCPP_ERROR(
        get_logger(),
        "Action result canceled: action_name=%s result_code=CANCELED detail=%s previous_state=%s",
        execute_behavior_action_name_.c_str(),
        result_detail,
        executionStateToString(previous_state).c_str());
      break;
    default:
      execution_state_ = ExecutionState::kFailed;
      RCLCPP_ERROR(
        get_logger(),
        "Action result unknown code: action_name=%s result_code=%d detail=%s",
        execute_behavior_action_name_.c_str(),
        static_cast<int>(result.code),
        result_detail);
      break;
  }
}

bool BehaviorNode::canSendGoalLocked() const
{
  if (!action_server_ready_) {
    return false;
  }
  if (action_goal_pending_) {
    return false;
  }
  if (action_goal_active_) {
    return false;
  }
  return true;
}

bool BehaviorNode::IsTaskPhaseRecoveredForTest(const std::string & task_phase) const
{
  const auto normalized_task_phase = normalizeToken(task_phase);
  std::lock_guard<std::mutex> lock(state_mutex_);
  return recovered_completed_task_phases_.find(normalized_task_phase) != recovered_completed_task_phases_.end();
}

bool BehaviorNode::IsIdleSpinningForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return idle_spinning_mode_active_;
}

std::string BehaviorNode::getExecutionState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return executionStateToString(execution_state_);
}

std::string BehaviorNode::executionStateToString(ExecutionState state) const
{
  switch (state) {
    case ExecutionState::kIdle:
      return "idle";
    case ExecutionState::kWaitingServer:
      return "waiting_server";
    case ExecutionState::kServerUnavailable:
      return "server_unavailable";
    case ExecutionState::kSendingGoal:
      return "sending_goal";
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

std::string BehaviorNode::normalizeToken(const std::string & value)
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

std::string BehaviorNode::parseKeyValuePayload(const std::string & payload, const std::string & key)
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
        return percentDecode(token.substr(equal_pos + 1));
      }
    }

    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }

  return "";
}

bool BehaviorNode::isCompletedState(const std::string & target_state) const
{
  return target_state == "done" || target_state == "completed" || target_state == "succeeded" ||
         target_state == "success" || target_state == "finished";
}

bool BehaviorNode::isFinitePose(const geometry_msgs::msg::Pose & pose) const
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool BehaviorNode::hasValidQuaternionNorm(const geometry_msgs::msg::Pose & pose) const
{
  const double norm =
    (pose.orientation.x * pose.orientation.x) +
    (pose.orientation.y * pose.orientation.y) +
    (pose.orientation.z * pose.orientation.z) +
    (pose.orientation.w * pose.orientation.w);

  constexpr double min_norm = 1e-6;
  constexpr double target_norm = 1.0;
  constexpr double tolerance = 0.1;
  return norm > min_norm && std::fabs(norm - target_norm) <= tolerance;
}

}  // namespace dog_behavior