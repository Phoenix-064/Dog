#include "dog_behavior/bt_nodes/place_rule_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>

#include <vector>

TEST(PlaceRuleActionNodeTest, ResolvesLeftMatchTypeAndGroupA)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::PlaceRuleAction>("PlaceRuleAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("match_type", std::string("left"));
  blackboard->set("counter", 0);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <PlaceRuleAction match_type=\"{match_type}\" counter=\"{counter}\" target_type=\"{target_type}\" group_indices=\"{group_indices}\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(blackboard->get<std::string>("target_type"), "food");
  EXPECT_EQ(blackboard->get<std::vector<int>>("group_indices"), (std::vector<int>{0, 1, 5, 6}));
}

TEST(PlaceRuleActionNodeTest, ResolvesRightMatchTypeAndGroupB)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::PlaceRuleAction>("PlaceRuleAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("match_type", std::string("right"));
  blackboard->set("counter", 7);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <PlaceRuleAction match_type=\"{match_type}\" counter=\"{counter}\" target_type=\"{target_type}\" group_indices=\"{group_indices}\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(blackboard->get<std::string>("target_type"), "medical");
  EXPECT_EQ(blackboard->get<std::vector<int>>("group_indices"), (std::vector<int>{2, 3, 4, 7}));
}

TEST(PlaceRuleActionNodeTest, InvalidMatchTypeReturnsFailure)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::PlaceRuleAction>("PlaceRuleAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("match_type", std::string("unknown"));
  blackboard->set("counter", 0);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <PlaceRuleAction match_type=\"{match_type}\" counter=\"{counter}\" target_type=\"{target_type}\" group_indices=\"{group_indices}\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}
