#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace dog_behavior::bt_nodes
{

class AdvancePlaceCounterAction : public BT::SyncActionNode
{
public:
  AdvancePlaceCounterAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace dog_behavior::bt_nodes
