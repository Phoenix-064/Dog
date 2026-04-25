#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <dog_interfaces/action/place_boxes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace dog_behavior::bt_nodes
{

class ExecutePlaceBoxesAction : public BT::StatefulActionNode
{
public:
  using PlaceBoxes = dog_interfaces::action::PlaceBoxes;
  using GoalHandle = rclcpp_action::ClientGoalHandle<PlaceBoxes>;

  ExecutePlaceBoxesAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void goalResponseCallback(GoalHandle::SharedPtr goal_handle);
  void feedbackCallback(GoalHandle::SharedPtr, const std::shared_ptr<const PlaceBoxes::Feedback> feedback);
  void resultCallback(const GoalHandle::WrappedResult & result);
  void commitTypeCountOnSuccess();
  std::string resultCodeText() const;
  std::string localIndicesToText() const;
  void logStep(const std::string & result_code_text) const;

  std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PlaceBoxes>::SharedPtr client_;
  GoalHandle::SharedPtr active_goal_handle_;
  bool goal_accepted_;
  bool result_ready_;
  bool canceled_;
  rclcpp_action::ResultCode result_code_;
  bool result_accepted_;
  std::string result_detail_;
  std::string action_name_;
  rclcpp::Time last_feedback_time_;
  double feedback_timeout_sec_;

  bool has_target_;
  std::string box_type_;
  std::string payload_;
  int step_counter_;
  int count_after_success_;
  std::string match_type_;
  std::vector<int> local_indices_;
};

}  // namespace dog_behavior::bt_nodes
