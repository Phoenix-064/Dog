#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

namespace dog_behavior::bt_nodes
{

class WaitForPoseCondition : public BT::ConditionNode
{
public:
  WaitForPoseCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool started_waiting_;
  uint64_t first_wait_time_ms_;
};

}  // namespace dog_behavior::bt_nodes
