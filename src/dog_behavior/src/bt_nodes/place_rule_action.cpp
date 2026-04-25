#include "dog_behavior/bt_nodes/place_rule_action.hpp"

#include <algorithm>
#include <array>
#include <cctype>

namespace dog_behavior::bt_nodes
{

namespace
{
const std::array<int, 4> kGroupA{0, 1, 5, 6};
const std::array<int, 4> kGroupB{2, 3, 4, 7};
const std::array<const char *, 8> kLeftTypes{
  "food", "tool", "instrument", "medical", "medical", "instrument", "tool", "food"};
const std::array<const char *, 8> kRightTypes{
  "medical", "instrument", "tool", "food", "food", "tool", "instrument", "medical"};
}  // namespace

PlaceRuleAction::PlaceRuleAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PlaceRuleAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("match_type"),
    BT::InputPort<int>("counter"),
    BT::OutputPort<std::string>("target_type"),
    BT::OutputPort<std::vector<int>>("group_indices"),
  };
}

BT::NodeStatus PlaceRuleAction::tick()
{
  const auto match_type_input = getInput<std::string>("match_type");
  const auto counter_input = getInput<int>("counter");
  if (!match_type_input || !counter_input) {
    return BT::NodeStatus::FAILURE;
  }

  const int counter = counter_input.value();
  if (counter < 0 || counter > 7) {
    setOutput("target_type", std::string(""));
    setOutput("group_indices", std::vector<int>{});
    return BT::NodeStatus::SUCCESS;
  }

  const auto match_type = normalizeMatchType(match_type_input.value());
  const auto target_type = resolveTargetType(match_type, counter);
  if (target_type.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  const auto & group = resolveGroup(counter);
  setOutput("target_type", target_type);
  setOutput("group_indices", std::vector<int>(group.begin(), group.end()));
  return BT::NodeStatus::SUCCESS;
}

std::string PlaceRuleAction::normalizeMatchType(const std::string & input) const
{
  std::string result = input;
  std::transform(
    result.begin(),
    result.end(),
    result.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return result;
}

std::string PlaceRuleAction::resolveTargetType(const std::string & match_type, const int counter) const
{
  if (counter < 0 || counter > 7) {
    return "";
  }
  if (match_type == "left") {
    return kLeftTypes[static_cast<size_t>(counter)];
  }
  if (match_type == "right") {
    return kRightTypes[static_cast<size_t>(counter)];
  }
  return "";
}

const std::array<int, 4> & PlaceRuleAction::resolveGroup(const int counter) const
{
  return counter < 4 ? kGroupA : kGroupB;
}

}  // namespace dog_behavior::bt_nodes
