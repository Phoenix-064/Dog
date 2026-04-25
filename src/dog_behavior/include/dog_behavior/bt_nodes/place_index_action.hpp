#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <array>
#include <string>
#include <vector>

namespace dog_behavior::bt_nodes
{

class PlaceIndexAction : public BT::SyncActionNode
{
public:
  PlaceIndexAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::vector<std::string> selectByGroup(
    const std::vector<std::string> & boxes_type_list,
    const std::vector<int> & group_indices) const;
  std::vector<int> collectLocalIndices(
    const std::vector<std::string> & selected_boxes,
    const std::string & target_type) const;
  int currentTypeCount(
    const std::string & target_type,
    int food_count,
    int tool_count,
    int instrument_count,
    int medical_count) const;
  std::string buildPayload(const std::vector<int> & local_indices, int count_after_success) const;
};

}  // namespace dog_behavior::bt_nodes
