#include "dog_behavior/bt_nodes/place_index_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>

#include <vector>

TEST(PlaceIndexActionNodeTest, OutputsLocalIndicesPayloadAndCount)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::PlaceIndexAction>("PlaceIndexAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("boxes_type_list", std::vector<std::string>{
    "food", "tool", "medical", "food", "food", "instrument", "food", "medical"});
  blackboard->set("group_indices", std::vector<int>{0, 1, 5, 6});
  blackboard->set("target_type", std::string("food"));
  blackboard->set("food_box_count", 1);
  blackboard->set("tool_box_count", 0);
  blackboard->set("instrument_box_count", 0);
  blackboard->set("medical_box_count", 0);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <PlaceIndexAction boxes_type_list=\"{boxes_type_list}\" group_indices=\"{group_indices}\" target_type=\"{target_type}\" "
    "food_box_count=\"{food_box_count}\" tool_box_count=\"{tool_box_count}\" instrument_box_count=\"{instrument_box_count}\" medical_box_count=\"{medical_box_count}\" "
    "local_indices=\"{local_indices}\" payload=\"{payload}\" count_after_success=\"{count_after_success}\" has_target=\"{has_target}\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(blackboard->get<std::vector<int>>("local_indices"), (std::vector<int>{0, 3}));
  EXPECT_EQ(blackboard->get<std::string>("payload"), "place=0,3,count=3");
  EXPECT_EQ(blackboard->get<int>("count_after_success"), 3);
  EXPECT_TRUE(blackboard->get<bool>("has_target"));
}

TEST(PlaceIndexActionNodeTest, ReturnsNoTargetWhenTypeMissingInGroup)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::PlaceIndexAction>("PlaceIndexAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("boxes_type_list", std::vector<std::string>{
    "food", "tool", "medical", "food", "food", "instrument", "food", "medical"});
  blackboard->set("group_indices", std::vector<int>{0, 1, 5, 6});
  blackboard->set("target_type", std::string("medical"));
  blackboard->set("food_box_count", 0);
  blackboard->set("tool_box_count", 0);
  blackboard->set("instrument_box_count", 0);
  blackboard->set("medical_box_count", 2);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <PlaceIndexAction boxes_type_list=\"{boxes_type_list}\" group_indices=\"{group_indices}\" target_type=\"{target_type}\" "
    "food_box_count=\"{food_box_count}\" tool_box_count=\"{tool_box_count}\" instrument_box_count=\"{instrument_box_count}\" medical_box_count=\"{medical_box_count}\" "
    "local_indices=\"{local_indices}\" payload=\"{payload}\" count_after_success=\"{count_after_success}\" has_target=\"{has_target}\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  EXPECT_TRUE(blackboard->get<std::vector<int>>("local_indices").empty());
  EXPECT_EQ(blackboard->get<std::string>("payload"), "");
  EXPECT_EQ(blackboard->get<int>("count_after_success"), 2);
  EXPECT_FALSE(blackboard->get<bool>("has_target"));
}
