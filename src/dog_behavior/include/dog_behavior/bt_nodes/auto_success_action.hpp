#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <string>

namespace dog_behavior::bt_nodes
{

class AutoSuccessAction : public BT::SyncActionNode
{
public:
  AutoSuccessAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace dog_behavior::bt_nodes
