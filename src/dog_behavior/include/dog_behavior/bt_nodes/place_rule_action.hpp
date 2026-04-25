#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <array>
#include <string>
#include <vector>

namespace dog_behavior::bt_nodes
{

class PlaceRuleAction : public BT::SyncActionNode
{
public:
  PlaceRuleAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::string normalizeMatchType(const std::string & input) const;
  std::string resolveTargetType(const std::string & match_type, int counter) const;
  const std::array<int, 4> & resolveGroup(int counter) const;
};

}  // namespace dog_behavior::bt_nodes
