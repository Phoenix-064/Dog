#include "dog_behavior/bt_nodes/select_waypoint_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

TEST(SelectWaypointActionNodeTest, OutputsTargetPoseAndRotatesIndex)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::SelectWaypointAction>("SelectWaypoint");

  auto blackboard = BT::Blackboard::create();
  std::vector<dog_behavior::Waypoint> waypoints;
  waypoints.push_back({"A", 1.0, 2.0, 0.0, 0.0});
  waypoints.push_back({"B", 3.0, 4.0, 0.0, 90.0});
  blackboard->set("waypoints", waypoints);
  blackboard->set("waypoint_index", 1);

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

  EXPECT_DOUBLE_EQ(selected_pose.pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(selected_pose.pose.position.y, 4.0);
  EXPECT_NEAR(yawFromQuaternion(selected_pose.pose.orientation), dog_behavior::kPi / 2.0, 1.0e-6);
  EXPECT_EQ(next_index, 0);
}
