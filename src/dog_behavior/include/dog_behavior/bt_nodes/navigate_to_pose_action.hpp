#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <future>
#include <mutex>
#include <string>

namespace dog_behavior::bt_nodes
{

class NavigateToPoseAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void goalResponseCallback(GoalHandle::SharedPtr goal_handle);
  void feedbackCallback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandle::WrappedResult & result);
  void publishState(const std::string & state);

  std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  GoalHandle::SharedPtr active_goal_handle_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  bool goal_accepted_;
  bool result_ready_;
  bool canceled_;
  rclcpp_action::ResultCode result_code_;
  std::string action_name_;
  rclcpp::Time last_feedback_time_;
  double feedback_timeout_sec_;
};

}  // namespace dog_behavior::bt_nodes
