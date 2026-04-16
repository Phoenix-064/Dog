#include "dog_behavior/bt_nodes/check_system_mode.hpp"

#include "dog_behavior/common/payload_utils.hpp"

namespace dog_behavior::bt_nodes
{

CheckSystemMode::CheckSystemMode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckSystemMode::providedPorts()
{
  return {
    BT::InputPort<std::string>("mode"),
    BT::InputPort<std::string>("expected_mode"),
  };
}

BT::NodeStatus CheckSystemMode::tick()
{
  const auto mode = getInput<std::string>("mode");
  const auto expected_mode = getInput<std::string>("expected_mode");
  if (!mode || !expected_mode) {
    return BT::NodeStatus::FAILURE;
  }

  return utils::normalizeToken(mode.value()) == utils::normalizeToken(expected_mode.value()) ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace dog_behavior::bt_nodes
