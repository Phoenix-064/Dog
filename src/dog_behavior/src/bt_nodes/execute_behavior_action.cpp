#include "dog_behavior/bt_nodes/execute_behavior_action.hpp"

#include "dog_behavior/common/payload_utils.hpp"

#include <chrono>
#include <utility>

namespace dog_behavior::bt_nodes
{

ExecuteBehaviorAction::ExecuteBehaviorAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
, goal_accepted_(false)
, result_ready_(false)
, canceled_(false)
, result_code_(rclcpp_action::ResultCode::UNKNOWN)
, result_accepted_(false)
, feedback_timeout_sec_(2.0)
{
}

BT::PortsList ExecuteBehaviorAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("behavior_name"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
    BT::InputPort<std::string>("action_name", "/behavior/execute"),
    BT::InputPort<double>("feedback_timeout_sec", 2.0, "feedback timeout seconds"),
  };
}

BT::NodeStatus ExecuteBehaviorAction::onStart()
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

  const auto behavior_name_input = getInput<std::string>("behavior_name");
  const auto target_pose_input = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
  const auto action_name_input = getInput<std::string>("action_name");
  const auto feedback_timeout_input = getInput<double>("feedback_timeout_sec");
  if (!behavior_name_input || !target_pose_input || !action_name_input) {
    return BT::NodeStatus::FAILURE;
  }

  if (!utils::isFinitePose(target_pose_input.value()) || !utils::hasValidQuaternionNorm(target_pose_input.value())) {
    return BT::NodeStatus::FAILURE;
  }

  feedback_timeout_sec_ = feedback_timeout_input && feedback_timeout_input.value() > 0.0 ?
    feedback_timeout_input.value() : 2.0;

  const auto & action_name = action_name_input.value();
  if (!client_ || action_name_ != action_name) {
    client_ = rclcpp_action::create_client<ExecuteBehavior>(node_, action_name);
    action_name_ = action_name;
  }

  if (!client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN(node_->get_logger(), "ExecuteBehavior action server unavailable: %s", action_name.c_str());
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

  ExecuteBehavior::Goal goal;
  goal.behavior_name = behavior_name_input.value();
  goal.target_pose = target_pose_input.value();

  rclcpp_action::Client<ExecuteBehavior>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback = [this](GoalHandle::SharedPtr goal_handle) {
    this->goalResponseCallback(goal_handle);
  };
  send_goal_options.feedback_callback = [this](GoalHandle::SharedPtr goal_handle, const std::shared_ptr<const ExecuteBehavior::Feedback> feedback) {
    this->feedbackCallback(goal_handle, feedback);
  };
  send_goal_options.result_callback = [this](const GoalHandle::WrappedResult & result) {
    this->resultCallback(result);
  };

  client_->async_send_goal(goal, send_goal_options);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteBehaviorAction::onRunning()
{
  GoalHandle::SharedPtr goal_handle_to_cancel;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (canceled_) {
      return BT::NodeStatus::FAILURE;
    }

    if (!goal_accepted_ && !result_ready_) {
      return BT::NodeStatus::RUNNING;
    }

    if (!goal_accepted_) {
      return BT::NodeStatus::FAILURE;
    }

    if (!result_ready_) {
      const auto feedback_elapsed_sec = (node_->now() - last_feedback_time_).seconds();
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
    return BT::NodeStatus::FAILURE;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED && result_accepted_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void ExecuteBehaviorAction::onHalted()
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
}

void ExecuteBehaviorAction::goalResponseCallback(GoalHandle::SharedPtr goal_handle)
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
}

void ExecuteBehaviorAction::feedbackCallback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const ExecuteBehavior::Feedback> feedback)
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

void ExecuteBehaviorAction::resultCallback(const GoalHandle::WrappedResult & result)
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

}  // namespace dog_behavior::bt_nodes
