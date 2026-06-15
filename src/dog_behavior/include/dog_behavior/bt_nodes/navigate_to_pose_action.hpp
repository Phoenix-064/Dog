#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <dog_interfaces/action/navigate_waypoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <string>

namespace dog_behavior::bt_nodes
{

class NavigateWaypointAction : public BT::StatefulActionNode
{
public:
  using NavigateWaypoint = dog_interfaces::action::NavigateWaypoint;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateWaypoint>;

  NavigateWaypointAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void goalResponseCallback(GoalHandle::SharedPtr goal_handle);
  void feedbackCallback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateWaypoint::Feedback> feedback);
  void resultCallback(const GoalHandle::WrappedResult & result);
  void publishGoal(const geometry_msgs::msg::PoseStamped & goal, const std::string & topic);
  void publishState(const std::string & state);

  std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateWaypoint>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  GoalHandle::SharedPtr active_goal_handle_;
  bool goal_accepted_;
  bool result_ready_;
  bool canceled_;
  rclcpp_action::ResultCode result_code_;
  bool result_accepted_;
  std::string result_detail_;
  std::string action_name_;
  std::string goal_topic_;
  rclcpp::Time last_feedback_time_;
  double feedback_timeout_sec_;
};

}  // namespace dog_behavior::bt_nodes
