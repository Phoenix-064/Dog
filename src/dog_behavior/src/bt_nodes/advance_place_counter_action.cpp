#include "dog_behavior/bt_nodes/advance_place_counter_action.hpp"

namespace dog_behavior::bt_nodes
{

AdvancePlaceCounterAction::AdvancePlaceCounterAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList AdvancePlaceCounterAction::providedPorts()
{
  return {
    BT::BidirectionalPort<int>("counter", -1, "step counter"),
    BT::OutputPort<bool>("done"),
  };
}

BT::NodeStatus AdvancePlaceCounterAction::tick()
{
  auto counter_input = getInput<int>("counter");
  int counter = counter_input ? counter_input.value() : -1;

  ++counter;
  const bool done = counter > 7;

  setOutput("counter", counter);
  setOutput("done", done);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace dog_behavior::bt_nodes
