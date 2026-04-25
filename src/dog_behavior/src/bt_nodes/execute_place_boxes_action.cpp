#include "dog_behavior/bt_nodes/execute_place_boxes_action.hpp"

#include "dog_behavior/common/payload_utils.hpp"

#include <chrono>
#include <sstream>
#include <utility>

namespace dog_behavior::bt_nodes
{

ExecutePlaceBoxesAction::ExecutePlaceBoxesAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
, goal_accepted_(false)
, result_ready_(false)
, canceled_(false)
, result_code_(rclcpp_action::ResultCode::UNKNOWN)
, result_accepted_(false)
, feedback_timeout_sec_(2.0)
, has_target_(false)
, step_counter_(-1)
, count_after_success_(0)
{
}

BT::PortsList ExecutePlaceBoxesAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("box_type"),
    BT::InputPort<std::string>("payload"),
    BT::InputPort<int>("step_counter", -1, "step counter"),
    BT::InputPort<bool>("has_target", true, "has target indices"),
    BT::InputPort<int>("count_after_success", 0, "count to commit on success"),
    BT::InputPort<std::string>("match_type", "left"),
    BT::InputPort<std::vector<int>>("local_indices"),
    BT::InputPort<std::string>("action_name", "/behavior/place_boxes"),
    BT::InputPort<double>("feedback_timeout_sec", 2.0, "feedback timeout seconds"),
    BT::OutputPort<std::string>("result_code_text"),
  };
}

BT::NodeStatus ExecutePlaceBoxesAction::onStart()
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

  const auto box_type_input = getInput<std::string>("box_type");
  const auto payload_input = getInput<std::string>("payload");
  const auto step_counter_input = getInput<int>("step_counter");
  const auto has_target_input = getInput<bool>("has_target");
  const auto count_after_success_input = getInput<int>("count_after_success");
  const auto match_type_input = getInput<std::string>("match_type");
  const auto local_indices_input = getInput<std::vector<int>>("local_indices");
  const auto action_name_input = getInput<std::string>("action_name");
  const auto feedback_timeout_input = getInput<double>("feedback_timeout_sec");

  if (!box_type_input || !payload_input || !step_counter_input || !has_target_input ||
    !count_after_success_input || !match_type_input || !local_indices_input ||
    !action_name_input)
  {
    return BT::NodeStatus::FAILURE;
  }

  has_target_ = has_target_input.value();
  box_type_ = utils::normalizeToken(box_type_input.value());
  payload_ = payload_input.value();
  step_counter_ = step_counter_input.value();
  count_after_success_ = count_after_success_input.value();
  match_type_ = utils::normalizeToken(match_type_input.value());
  local_indices_ = local_indices_input.value();
  feedback_timeout_sec_ = feedback_timeout_input && feedback_timeout_input.value() > 0.0 ?
    feedback_timeout_input.value() : 2.0;

  if (!has_target_) {
    setOutput("result_code_text", std::string("skipped_no_target"));
    logStep("skipped_no_target");
    return BT::NodeStatus::SUCCESS;
  }

  const auto & action_name = action_name_input.value();
  if (!client_ || action_name_ != action_name) {
    client_ = rclcpp_action::create_client<PlaceBoxes>(node_, action_name);
    action_name_ = action_name;
  }

  if (!client_->wait_for_action_server(std::chrono::seconds(0))) {
    setOutput("result_code_text", std::string("server_unavailable"));
    logStep("server_unavailable");
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

  PlaceBoxes::Goal goal;
  goal.box_type = box_type_;
  goal.payload = payload_;
  goal.step_counter = step_counter_;

  rclcpp_action::Client<PlaceBoxes>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback = [this](GoalHandle::SharedPtr goal_handle) {
    this->goalResponseCallback(goal_handle);
  };
  send_goal_options.feedback_callback = [this](GoalHandle::SharedPtr goal_handle, const std::shared_ptr<const PlaceBoxes::Feedback> feedback) {
    this->feedbackCallback(goal_handle, feedback);
  };
  send_goal_options.result_callback = [this](const GoalHandle::WrappedResult & result) {
    this->resultCallback(result);
  };

  client_->async_send_goal(goal, send_goal_options);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecutePlaceBoxesAction::onRunning()
{
  GoalHandle::SharedPtr goal_handle_to_cancel;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (canceled_) {
      setOutput("result_code_text", std::string("canceled"));
      logStep("canceled");
      return BT::NodeStatus::FAILURE;
    }

    if (!goal_accepted_ && !result_ready_) {
      return BT::NodeStatus::RUNNING;
    }

    if (!goal_accepted_) {
      setOutput("result_code_text", std::string("goal_rejected"));
      logStep("goal_rejected");
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
    setOutput("result_code_text", std::string("timeout"));
    logStep("timeout");
    return BT::NodeStatus::FAILURE;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const std::string code_text = resultCodeText();
  setOutput("result_code_text", code_text);

  if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED && result_accepted_) {
    commitTypeCountOnSuccess();
    logStep(code_text);
    return BT::NodeStatus::SUCCESS;
  }

  logStep(code_text);
  return BT::NodeStatus::FAILURE;
}

void ExecutePlaceBoxesAction::onHalted()
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

void ExecutePlaceBoxesAction::goalResponseCallback(GoalHandle::SharedPtr goal_handle)
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

void ExecutePlaceBoxesAction::feedbackCallback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const PlaceBoxes::Feedback> feedback)
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

void ExecutePlaceBoxesAction::resultCallback(const GoalHandle::WrappedResult & result)
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

void ExecutePlaceBoxesAction::commitTypeCountOnSuccess()
{
  if (!config().blackboard) {
    return;
  }

  const std::string key =
    box_type_ == "food" ? "food_box_count" :
    box_type_ == "tool" ? "tool_box_count" :
    box_type_ == "instrument" ? "instrument_box_count" :
    box_type_ == "medical" ? "medical_box_count" : "";

  if (key.empty()) {
    return;
  }

  config().blackboard->set(key, count_after_success_);
}

std::string ExecutePlaceBoxesAction::resultCodeText() const
{
  switch (result_code_) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return result_accepted_ ? "success" : "succeeded_not_accepted";
    case rclcpp_action::ResultCode::ABORTED:
      return "aborted";
    case rclcpp_action::ResultCode::CANCELED:
      return "canceled";
    default:
      return "unknown";
  }
}

std::string ExecutePlaceBoxesAction::localIndicesToText() const
{
  std::ostringstream ss;
  for (size_t i = 0; i < local_indices_.size(); ++i) {
    if (i > 0) {
      ss << ",";
    }
    ss << local_indices_[i];
  }
  return ss.str();
}

void ExecutePlaceBoxesAction::logStep(const std::string & result_code_text) const
{
  if (!node_) {
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "place_step counter=%d match_type=%s target_type=%s local_indices=%s payload=%s result=%s detail=%s",
    step_counter_,
    match_type_.c_str(),
    box_type_.c_str(),
    localIndicesToText().c_str(),
    payload_.c_str(),
    result_code_text.c_str(),
    result_detail_.c_str());
}

}  // namespace dog_behavior::bt_nodes
