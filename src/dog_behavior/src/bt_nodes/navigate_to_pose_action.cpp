#include "dog_behavior/bt_nodes/navigate_to_pose_action.hpp"

#include "dog_behavior/common/payload_utils.hpp"

#include <chrono>
#include <utility>

namespace dog_behavior::bt_nodes
{

NavigateToPoseAction::NavigateToPoseAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
, goal_accepted_(false)
, result_ready_(false)
, canceled_(false)
, result_code_(rclcpp_action::ResultCode::UNKNOWN)
, feedback_timeout_sec_(10.0)
{
}

BT::PortsList NavigateToPoseAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<std::string>("action_name", "/navigate_to_pose"),
    BT::InputPort<std::string>("state_topic", "/behavior/nav_exec_state"),
    BT::InputPort<double>("feedback_timeout_sec", 10.0, "feedback timeout seconds"),
  };
}

BT::NodeStatus NavigateToPoseAction::onStart()
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
  const auto feedback_timeout_input = getInput<double>("feedback_timeout_sec");
  if (!goal_input || !action_name_input || !state_topic_input) {
    return BT::NodeStatus::FAILURE;
  }

  if (!utils::isFinitePose(goal_input.value()) || !utils::hasValidQuaternionNorm(goal_input.value())) {
    return BT::NodeStatus::FAILURE;
  }

  feedback_timeout_sec_ = feedback_timeout_input && feedback_timeout_input.value() > 0.0 ?
    feedback_timeout_input.value() : 10.0;

  const auto & action_name = action_name_input.value();
  if (!client_ || action_name_ != action_name) {
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name);
    action_name_ = action_name;
  }

  if (!state_pub_ || state_pub_->get_topic_name() != state_topic_input.value()) {
    state_pub_ = node_->create_publisher<std_msgs::msg::String>(
      state_topic_input.value(),
      rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  }

  if (!client_->wait_for_action_server(std::chrono::seconds(0))) {
    publishState("nav2_server_unavailable");
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_accepted_ = false;
    result_ready_ = false;
    canceled_ = false;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    active_goal_handle_.reset();
    goal_handle_future_ = std::shared_future<GoalHandle::SharedPtr>();
    result_future_ = std::shared_future<GoalHandle::WrappedResult>();
    last_feedback_time_ = node_->now();
  }

  NavigateToPose::Goal goal;
  goal.pose = goal_input.value();

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
  send_goal_options.feedback_callback = [this](GoalHandle::SharedPtr goal_handle, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    this->feedbackCallback(goal_handle, feedback);
  };

  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_future_ = client_->async_send_goal(goal, send_goal_options);
  }
  publishState("forwarding_goal");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onRunning()
{
  GoalHandle::SharedPtr goal_handle_to_cancel;
  bool result_is_ready = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (canceled_) {
      publishState("failed");
      return BT::NodeStatus::FAILURE;
    }

    if (!goal_accepted_) {
      if (!goal_handle_future_.valid()) {
        publishState("failed");
        return BT::NodeStatus::FAILURE;
      }
      if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
      }

      auto goal_handle = goal_handle_future_.get();
      if (!goal_handle) {
        result_ready_ = true;
        result_code_ = rclcpp_action::ResultCode::ABORTED;
        publishState("rejected");
        return BT::NodeStatus::FAILURE;
      }

      active_goal_handle_ = std::move(goal_handle);
      goal_accepted_ = true;
      last_feedback_time_ = node_->now();
      result_future_ = client_->async_get_result(active_goal_handle_);
      publishState("running");
    }

    if (result_future_.valid() &&
      result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      const auto wrapped_result = result_future_.get();
      result_code_ = wrapped_result.code;
      result_ready_ = true;
      result_is_ready = true;
    }

    if (!result_is_ready) {
      const double feedback_elapsed_sec = (node_->now() - last_feedback_time_).seconds();
      if (feedback_elapsed_sec > feedback_timeout_sec_) {
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
  if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
    publishState("succeeded");
    return BT::NodeStatus::SUCCESS;
  }

  if (result_code_ == rclcpp_action::ResultCode::CANCELED) {
    publishState("timeout");
  } else {
    publishState("failed");
  }
  return BT::NodeStatus::FAILURE;
}

void NavigateToPoseAction::onHalted()
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

void NavigateToPoseAction::goalResponseCallback(GoalHandle::SharedPtr goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!goal_handle) {
    goal_accepted_ = false;
    result_ready_ = true;
    result_code_ = rclcpp_action::ResultCode::ABORTED;
    return;
  }

  active_goal_handle_ = std::move(goal_handle);
  goal_accepted_ = true;
  last_feedback_time_ = node_->now();
  publishState("running");
}

void NavigateToPoseAction::feedbackCallback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!goal_accepted_) {
    return;
  }

  (void)feedback;
  last_feedback_time_ = node_->now();
}

void NavigateToPoseAction::resultCallback(const GoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  result_ready_ = true;
  result_code_ = result.code;
}

void NavigateToPoseAction::publishState(const std::string & state)
{
  if (!state_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = state;
  state_pub_->publish(msg);
}

}  // namespace dog_behavior::bt_nodes
