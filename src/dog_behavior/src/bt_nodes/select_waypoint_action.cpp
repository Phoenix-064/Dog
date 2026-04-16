#include "dog_behavior/bt_nodes/select_waypoint_action.hpp"

#include "dog_behavior/common/payload_utils.hpp"

#include <cmath>

namespace dog_behavior::bt_nodes
{

SelectWaypointAction::SelectWaypointAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SelectWaypointAction::providedPorts()
{
  return {
    BT::InputPort<std::vector<Waypoint>>("waypoints"),
    BT::BidirectionalPort<int>("index", 0, "rolling waypoint index"),
    BT::InputPort<std::string>("frame_id", "map"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
  };
}

BT::NodeStatus SelectWaypointAction::tick()
{
  const auto waypoints_input = getInput<std::vector<Waypoint>>("waypoints");
  if (!waypoints_input || waypoints_input->empty()) {
    return BT::NodeStatus::FAILURE;
  }

  auto index_input = getInput<int>("index");
  int index = index_input ? index_input.value() : 0;
  if (index < 0) {
    index = 0;
  }

  const int bounded_index = index % static_cast<int>(waypoints_input->size());
  auto target_pose = waypointToPose(waypoints_input.value().at(static_cast<size_t>(bounded_index)));

  const auto frame_id = getInput<std::string>("frame_id");
  if (frame_id && !frame_id->empty()) {
    target_pose.header.frame_id = frame_id.value();
  }

  if (!utils::isFinitePose(target_pose) || !utils::hasValidQuaternionNorm(target_pose)) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("target_pose", target_pose);
  setOutput("index", (bounded_index + 1) % static_cast<int>(waypoints_input->size()));
  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::PoseStamped SelectWaypointAction::waypointToPose(const Waypoint & waypoint) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = waypoint.z;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(waypoint.yaw / 2.0);
  pose.pose.orientation.w = std::cos(waypoint.yaw / 2.0);
  return pose;
}

}  // namespace dog_behavior::bt_nodes
