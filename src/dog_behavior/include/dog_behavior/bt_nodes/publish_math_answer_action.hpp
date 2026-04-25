#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace dog_behavior::bt_nodes
{

class PublishMathAnswerAction : public BT::SyncActionNode
{
public:
  PublishMathAnswerAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool canRunAtWaypoint(const std::string & waypoint_name, const std::string & required_waypoint_name) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string topic_name_;
};

}  // namespace dog_behavior::bt_nodes
