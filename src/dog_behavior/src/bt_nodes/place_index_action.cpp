#include "dog_behavior/bt_nodes/place_index_action.hpp"

#include <sstream>

namespace dog_behavior::bt_nodes
{

PlaceIndexAction::PlaceIndexAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PlaceIndexAction::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("boxes_type_list"),
    BT::InputPort<std::vector<int>>("group_indices"),
    BT::InputPort<std::string>("target_type"),
    BT::InputPort<int>("food_box_count", 0, "food count"),
    BT::InputPort<int>("tool_box_count", 0, "tool count"),
    BT::InputPort<int>("instrument_box_count", 0, "instrument count"),
    BT::InputPort<int>("medical_box_count", 0, "medical count"),
    BT::OutputPort<std::vector<int>>("local_indices"),
    BT::OutputPort<std::string>("payload"),
    BT::OutputPort<int>("count_after_success"),
    BT::OutputPort<bool>("has_target"),
  };
}

BT::NodeStatus PlaceIndexAction::tick()
{
  const auto boxes_type_list_input = getInput<std::vector<std::string>>("boxes_type_list");
  const auto group_indices_input = getInput<std::vector<int>>("group_indices");
  const auto target_type_input = getInput<std::string>("target_type");
  const auto food_count_input = getInput<int>("food_box_count");
  const auto tool_count_input = getInput<int>("tool_box_count");
  const auto instrument_count_input = getInput<int>("instrument_box_count");
  const auto medical_count_input = getInput<int>("medical_box_count");

  if (!boxes_type_list_input || !group_indices_input || !target_type_input ||
    !food_count_input || !tool_count_input || !instrument_count_input || !medical_count_input)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto & boxes_type_list = boxes_type_list_input.value();
  const auto & group_indices = group_indices_input.value();
  const auto & target_type = target_type_input.value();

  if (boxes_type_list.size() < 8 || target_type.empty()) {
    setOutput("local_indices", std::vector<int>{});
    setOutput("payload", std::string(""));
    setOutput("count_after_success", currentTypeCount(
      target_type,
      food_count_input.value(),
      tool_count_input.value(),
      instrument_count_input.value(),
      medical_count_input.value()));
    setOutput("has_target", false);
    return BT::NodeStatus::SUCCESS;
  }

  const auto selected_boxes = selectByGroup(boxes_type_list, group_indices);
  const auto local_indices = collectLocalIndices(selected_boxes, target_type);
  const int current_count = currentTypeCount(
    target_type,
    food_count_input.value(),
    tool_count_input.value(),
    instrument_count_input.value(),
    medical_count_input.value());
  const int count_after_success = current_count + static_cast<int>(local_indices.size());

  setOutput("local_indices", local_indices);
  setOutput("count_after_success", count_after_success);
  setOutput("has_target", !local_indices.empty());

  if (local_indices.empty()) {
    setOutput("payload", std::string(""));
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("payload", buildPayload(local_indices, count_after_success));
  return BT::NodeStatus::SUCCESS;
}

std::vector<std::string> PlaceIndexAction::selectByGroup(
  const std::vector<std::string> & boxes_type_list,
  const std::vector<int> & group_indices) const
{
  std::vector<std::string> selected_boxes;
  selected_boxes.reserve(group_indices.size());

  for (const int index : group_indices) {
    if (index >= 0 && static_cast<size_t>(index) < boxes_type_list.size()) {
      selected_boxes.push_back(boxes_type_list[static_cast<size_t>(index)]);
    }
  }

  return selected_boxes;
}

std::vector<int> PlaceIndexAction::collectLocalIndices(
  const std::vector<std::string> & selected_boxes,
  const std::string & target_type) const
{
  std::vector<int> local_indices;
  for (size_t i = 0; i < selected_boxes.size(); ++i) {
    if (selected_boxes[i] == target_type) {
      local_indices.push_back(static_cast<int>(i));
    }
  }
  return local_indices;
}

int PlaceIndexAction::currentTypeCount(
  const std::string & target_type,
  const int food_count,
  const int tool_count,
  const int instrument_count,
  const int medical_count) const
{
  if (target_type == "food") {
    return food_count;
  }
  if (target_type == "tool") {
    return tool_count;
  }
  if (target_type == "instrument") {
    return instrument_count;
  }
  if (target_type == "medical") {
    return medical_count;
  }
  return 0;
}

std::string PlaceIndexAction::buildPayload(
  const std::vector<int> & local_indices,
  const int count_after_success) const
{
  std::ostringstream ss;
  ss << "place=";
  for (size_t i = 0; i < local_indices.size(); ++i) {
    if (i > 0) {
      ss << ",";
    }
    ss << local_indices[i];
  }
  ss << ",count=" << count_after_success;
  return ss.str();
}

}  // namespace dog_behavior::bt_nodes
