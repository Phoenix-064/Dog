#include "dog_behavior/bt_nodes/select_waypoint_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>

#include <vector>

TEST(SelectWaypointActionNodeTest, OutputsTargetPoseAndRotatesIndex)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::SelectWaypointAction>("SelectWaypoint");

  auto blackboard = BT::Blackboard::create();
  std::vector<dog_behavior::Waypoint> waypoints;
  waypoints.push_back({"A", 1.0, 2.0, 0.0, 0.0});
  waypoints.push_back({"B", 3.0, 4.0, 0.0, 1.57});
  blackboard->set("waypoints", waypoints);
  blackboard->set("waypoint_index", 0);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <SelectWaypoint waypoints=\"{waypoints}\" index=\"{waypoint_index}\" target_pose=\"{target_pose}\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  auto selected_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("target_pose");
  auto next_index = blackboard->get<int>("waypoint_index");

  EXPECT_DOUBLE_EQ(selected_pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(selected_pose.pose.position.y, 2.0);
  EXPECT_EQ(next_index, 1);
}
