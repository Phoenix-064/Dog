#include "dog_behavior/bt_nodes/navigate_to_pose_action.hpp"

#include "dog_behavior/common/payload_utils.hpp"

#include <chrono>
#include <utility>

namespace dog_behavior::bt_nodes
{

NavigateWaypointAction::NavigateWaypointAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
, goal_accepted_(false)
, result_ready_(false)
, canceled_(false)
, result_code_(rclcpp_action::ResultCode::UNKNOWN)
, result_accepted_(false)
, feedback_timeout_sec_(0.0)
{
}

BT::PortsList NavigateWaypointAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<std::string>("action_name", "/behavior/nav_execute"),
    BT::InputPort<std::string>("state_topic", "/behavior/nav_exec_state"),
    BT::InputPort<std::string>("goal_topic", "/behavior/nav_goal"),
    BT::InputPort<double>("feedback_timeout_sec", 0.0, "feedback timeout seconds; <=0 disables timeout"),
    BT::InputPort<double>("server_timeout_sec", 1.0, "action server discovery timeout seconds"),
  };
}

BT::NodeStatus NavigateWaypointAction::onStart()
{
  if (!config().blackboard) {
    return BT::NodeStatus::FAILURE;
  }

  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  } catch (const std::exception &) {
    return BT::NodeStatus::FAILURE;
  }
  if (!node_) {
    return BT::NodeStatus::FAILURE;
  }

  const auto goal_input = getInput<geometry_msgs::msg::PoseStamped>("goal");
  const auto action_name_input = getInput<std::string>("action_name");
  const auto state_topic_input = getInput<std::string>("state_topic");
  const auto goal_topic_input = getInput<std::string>("goal_topic");
  const auto feedback_timeout_input = getInput<double>("feedback_timeout_sec");
  const auto server_timeout_input = getInput<double>("server_timeout_sec");
  if (!goal_input) {
    RCLCPP_WARN(node_->get_logger(), "NavigateWaypointAction missing required goal input port");
    return BT::NodeStatus::FAILURE;
  }

  if (!utils::isFinitePose(goal_input.value()) || !utils::hasValidQuaternionNorm(goal_input.value())) {
    RCLCPP_WARN(node_->get_logger(), "NavigateWaypointAction rejected invalid goal pose");
    return BT::NodeStatus::FAILURE;
  }

  feedback_timeout_sec_ = feedback_timeout_input ? feedback_timeout_input.value() : 0.0;
  const double server_timeout_sec = server_timeout_input && server_timeout_input.value() > 0.0 ?
    server_timeout_input.value() : 1.0;

  const std::string action_name = action_name_input ? action_name_input.value() : "/behavior/nav_execute";
  const std::string state_topic = state_topic_input ? state_topic_input.value() : "/behavior/nav_exec_state";
  const std::string goal_topic = goal_topic_input ? goal_topic_input.value() : "/behavior/nav_goal";
  if (!client_ || action_name_ != action_name) {
    client_ = rclcpp_action::create_client<NavigateWaypoint>(node_, action_name);
    action_name_ = action_name;
  }

  if (!state_pub_ || state_pub_->get_topic_name() != state_topic) {
    state_pub_ = node_->create_publisher<std_msgs::msg::String>(
      state_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  }

  const auto server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(server_timeout_sec));
  if (!client_->wait_for_action_server(server_timeout)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "NavigateWaypointAction action server unavailable action_name=%s timeout_sec=%.3f",
      action_name.c_str(),
      server_timeout_sec);
    publishState("failed");
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_accepted_ = false;
    result_ready_ = false;
    canceled_ = false;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    result_accepted_ = false;
    result_detail_.clear();
    active_goal_handle_.reset();
    last_feedback_time_ = node_->now();
  }

  NavigateWaypoint::Goal goal;
  goal.target_pose = goal_input.value();
  publishGoal(goal.target_pose, goal_topic);

  rclcpp_action::Client<NavigateWaypoint>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback = [this](GoalHandle::SharedPtr goal_handle) {
    this->goalResponseCallback(goal_handle);
  };
  send_goal_options.feedback_callback = [this](
    GoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateWaypoint::Feedback> feedback) {
      this->feedbackCallback(goal_handle, feedback);
    };
  send_goal_options.result_callback = [this](const GoalHandle::WrappedResult & result) {
    this->resultCallback(result);
  };

  client_->async_send_goal(goal, send_goal_options);
  publishState("forwarding_goal");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateWaypointAction::onRunning()
{
  GoalHandle::SharedPtr goal_handle_to_cancel;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (canceled_) {
      publishState("failed");
      return BT::NodeStatus::FAILURE;
    }

    if (!goal_accepted_ && !result_ready_) {
      return BT::NodeStatus::RUNNING;
    }

    if (!goal_accepted_) {
      publishState("failed");
      return BT::NodeStatus::FAILURE;
    }

    if (!result_ready_) {
      const double feedback_elapsed_sec = (node_->now() - last_feedback_time_).seconds();
      if (feedback_timeout_sec_ > 0.0 && feedback_elapsed_sec > feedback_timeout_sec_) {
        goal_handle_to_cancel = active_goal_handle_;
      } else {
        return BT::NodeStatus::RUNNING;
      }
    }
  }

  if (goal_handle_to_cancel) {
    client_->async_cancel_goal(goal_handle_to_cancel);
    std::lock_guard<std::mutex> lock(mutex_);
    canceled_ = true;
    publishState("timeout");
    return BT::NodeStatus::FAILURE;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED && result_accepted_) {
    publishState("succeeded");
    return BT::NodeStatus::SUCCESS;
  }

  if (result_detail_.find("timeout") != std::string::npos) {
    publishState("timeout");
  } else {
    publishState("failed");
  }
  return BT::NodeStatus::FAILURE;
}

void NavigateWaypointAction::onHalted()
{
  GoalHandle::SharedPtr goal_handle_to_cancel;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_to_cancel = active_goal_handle_;
    canceled_ = true;
  }
  if (goal_handle_to_cancel && client_) {
    client_->async_cancel_goal(goal_handle_to_cancel);
  }
  publishState("failed");
}

void NavigateWaypointAction::goalResponseCallback(GoalHandle::SharedPtr goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!goal_handle) {
    goal_accepted_ = false;
    result_ready_ = true;
    result_code_ = rclcpp_action::ResultCode::ABORTED;
    result_accepted_ = false;
    result_detail_ = "goal_rejected";
    return;
  }

  active_goal_handle_ = std::move(goal_handle);
  goal_accepted_ = true;
  last_feedback_time_ = node_->now();
  publishState("running");
}

void NavigateWaypointAction::feedbackCallback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const NavigateWaypoint::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!goal_accepted_) {
    return;
  }

  if (feedback) {
    result_detail_ = feedback->state;
  }
  last_feedback_time_ = node_->now();
}

void NavigateWaypointAction::resultCallback(const GoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  result_ready_ = true;
  result_code_ = result.code;
  if (result.result) {
    result_accepted_ = result.result->accepted;
    result_detail_ = result.result->detail;
  } else {
    result_accepted_ = false;
    result_detail_.clear();
  }
}

void NavigateWaypointAction::publishGoal(const geometry_msgs::msg::PoseStamped & goal, const std::string & topic)
{
  if (!node_) {
    return;
  }

  if (!goal_pub_ || goal_topic_ != topic) {
    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));
    goal_topic_ = topic;
  }

  goal_pub_->publish(goal);
}

void NavigateWaypointAction::publishState(const std::string & state)
{
  if (!state_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = state;
  state_pub_->publish(msg);
}

}  // namespace dog_behavior::bt_nodes
