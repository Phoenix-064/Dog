#include "dog_behavior/behavior_tree.hpp"

#include <behaviortree_cpp_v3/basic_types.h>

#include <exception>
#include <utility>

namespace dog_behavior
{

BehaviorTree::BehaviorTree(
  ActionCallback action_callback,
  PlaceholderCallback placeholder_callback,
  std::string tree_xml_file_path)
: tree_xml_file_path_(std::move(tree_xml_file_path))
, action_callback_(std::move(action_callback))
, placeholder_callback_(std::move(placeholder_callback))
{
  factory_.registerSimpleAction(
    "TriggerExecuteBehavior",
    [this](BT::TreeNode & node) {
      if (!action_callback_) {
        return BT::NodeStatus::FAILURE;
      }

      const auto behavior_name = node.getInput<std::string>("behavior_name");
      if (!behavior_name) {
        return BT::NodeStatus::FAILURE;
      }

      return action_callback_(behavior_name.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    },
    {BT::InputPort<std::string>("behavior_name")});

  factory_.registerSimpleAction(
    "PlaceholderBehavior",
    [this](BT::TreeNode &) {
      if (placeholder_callback_) {
        placeholder_callback_();
      }
      return BT::NodeStatus::SUCCESS;
    });
}

bool BehaviorTree::execute(const std::string & behavior_name)
{
  if (tree_xml_file_path_.empty()) {
    return false;
  }

  auto blackboard = BT::Blackboard::create();
  blackboard->set("behavior_name", behavior_name);

  try {
    auto tree = factory_.createTreeFromFile(tree_xml_file_path_, blackboard);
    return tree.tickRoot() == BT::NodeStatus::SUCCESS;
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace dog_behavior
