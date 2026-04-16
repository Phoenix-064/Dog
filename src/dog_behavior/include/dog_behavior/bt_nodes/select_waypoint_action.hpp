#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

#include "dog_behavior/behavior_tree_node.hpp"

namespace dog_behavior::bt_nodes
{

class SelectWaypointAction : public BT::SyncActionNode
{
public:
  SelectWaypointAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  geometry_msgs::msg::PoseStamped waypointToPose(const Waypoint & waypoint) const;
};

}  // namespace dog_behavior::bt_nodes
