#include "dog_behavior/bt_nodes/advance_place_counter_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>

TEST(AdvancePlaceCounterActionNodeTest, IncrementsCounterAndKeepsDoneFalse)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::AdvancePlaceCounterAction>("AdvancePlaceCounterAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("counter", -1);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <AdvancePlaceCounterAction counter=\"{counter}\" done=\"{done}\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(blackboard->get<int>("counter"), 0);
  EXPECT_FALSE(blackboard->get<bool>("done"));
}

TEST(AdvancePlaceCounterActionNodeTest, MarksDoneAfterFinalStep)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::AdvancePlaceCounterAction>("AdvancePlaceCounterAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("counter", 7);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <AdvancePlaceCounterAction counter=\"{counter}\" done=\"{done}\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(blackboard->get<int>("counter"), 8);
  EXPECT_TRUE(blackboard->get<bool>("done"));
}
