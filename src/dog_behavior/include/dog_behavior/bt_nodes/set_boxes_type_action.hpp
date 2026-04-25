#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <dog_interfaces/msg/target3_d_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <vector>

namespace dog_behavior::bt_nodes
{

class SetBoxesTypeAction : public BT::SyncActionNode
{
public:
  SetBoxesTypeAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void boxesCallback(const dog_interfaces::msg::Target3DArray::ConstSharedPtr msg);
  std::vector<std::string> sortToTwoRows(const dog_interfaces::msg::Target3DArray & msg) const;
  bool ensureSubscription();

  mutable std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<dog_interfaces::msg::Target3DArray>::SharedPtr subscription_;
  dog_interfaces::msg::Target3DArray::ConstSharedPtr latest_boxes_;
  bool subscription_initialized_;
  bool boxes_ready_once_;
};

}  // namespace dog_behavior::bt_nodes
