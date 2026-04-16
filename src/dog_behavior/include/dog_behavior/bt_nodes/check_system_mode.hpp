#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

namespace dog_behavior::bt_nodes
{

class CheckSystemMode : public BT::ConditionNode
{
public:
  CheckSystemMode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace dog_behavior::bt_nodes
