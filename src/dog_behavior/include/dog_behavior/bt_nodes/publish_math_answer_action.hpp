#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <dog_interfaces/msg/target3_d_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
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
  bool ensureOcrSubscription(const std::string & digit_result_topic);
  void digitResultCallback(const dog_interfaces::msg::Target3DArray::ConstSharedPtr msg);
  std::string waitForExpression(int timeout_ms);
  bool evaluateExpression(const std::string & expression, int64_t & result) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr ocr_listener_node_;
  rclcpp::executors::SingleThreadedExecutor ocr_executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<dog_interfaces::msg::Target3DArray>::SharedPtr digit_result_sub_;
  std::mutex expression_mutex_;
  std::string digit_result_topic_;
  std::string topic_name_;
  std::string latest_expression_;
};

}  // namespace dog_behavior::bt_nodes
