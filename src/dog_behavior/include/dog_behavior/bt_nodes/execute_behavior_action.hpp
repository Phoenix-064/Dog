#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <dog_interfaces/action/execute_behavior.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mutex>
#include <string>

namespace dog_behavior::bt_nodes
{

class ExecuteBehaviorAction : public BT::StatefulActionNode
{
public:
  using ExecuteBehavior = dog_interfaces::action::ExecuteBehavior;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteBehavior>;

  ExecuteBehaviorAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void goalResponseCallback(GoalHandle::SharedPtr goal_handle);
  void feedbackCallback(GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteBehavior::Feedback> feedback);
  void resultCallback(const GoalHandle::WrappedResult & result);

  std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecuteBehavior>::SharedPtr client_;
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
};

}  // namespace dog_behavior::bt_nodes
