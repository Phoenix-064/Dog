#include "dog_behavior/bt_nodes/check_system_mode.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>

TEST(CheckSystemModeNodeTest, ReturnsSuccessWhenModesMatch)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::CheckSystemMode>("CheckSystemMode");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("mode", std::string("Normal"));

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <CheckSystemMode mode=\"{mode}\" expected_mode=\"normal\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

TEST(CheckSystemModeNodeTest, ReturnsFailureWhenModesDiffer)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::CheckSystemMode>("CheckSystemMode");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("mode", std::string("idle_spinning"));

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <CheckSystemMode mode=\"{mode}\" expected_mode=\"normal\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}
