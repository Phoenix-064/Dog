#include "dog_behavior/bt_nodes/wait_for_pose_condition.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>

TEST(WaitForPoseConditionNodeTest, ReturnsFailureWhenPoseNotReady)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::WaitForPoseCondition>("WaitForPose");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("has_pose", false);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <WaitForPose has_pose=\"{has_pose}\" timeout_ms=\"100\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST(WaitForPoseConditionNodeTest, ReturnsSuccessWhenPoseReady)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::WaitForPoseCondition>("WaitForPose");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("has_pose", true);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <WaitForPose has_pose=\"{has_pose}\" timeout_ms=\"100\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}
