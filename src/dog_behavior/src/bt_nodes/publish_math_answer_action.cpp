#include "dog_behavior/bt_nodes/publish_math_answer_action.hpp"

#include "dog_behavior/common/payload_utils.hpp"

namespace dog_behavior::bt_nodes
{

PublishMathAnswerAction::PublishMathAnswerAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PublishMathAnswerAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("waypoint_name", "WayPointGoal3", "current waypoint name"),
    BT::InputPort<std::string>("required_waypoint_name", "WayPointGoal3", "required waypoint name"),
    BT::InputPort<std::string>("answer", "42"),
    BT::InputPort<std::string>("topic_name", "/math_answer"),
  };
}

BT::NodeStatus PublishMathAnswerAction::tick()
{
  if (!config().blackboard) {
    return BT::NodeStatus::FAILURE;
  }

  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  } catch (const std::exception &) {
    return BT::NodeStatus::FAILURE;
  }

  if (!node_) {
    return BT::NodeStatus::FAILURE;
  }

  const auto waypoint_name_input = getInput<std::string>("waypoint_name");
  const auto required_waypoint_name_input = getInput<std::string>("required_waypoint_name");
  const auto answer_input = getInput<std::string>("answer");
  const auto topic_name_input = getInput<std::string>("topic_name");
  if (!waypoint_name_input || !required_waypoint_name_input || !answer_input || !topic_name_input) {
    return BT::NodeStatus::FAILURE;
  }

  if (!canRunAtWaypoint(waypoint_name_input.value(), required_waypoint_name_input.value())) {
    return BT::NodeStatus::FAILURE;
  }

  if (!publisher_ || topic_name_ != topic_name_input.value()) {
    topic_name_ = topic_name_input.value();
    publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name_, rclcpp::QoS(10));
  }

  std_msgs::msg::String msg;
  msg.data = answer_input.value();
  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

bool PublishMathAnswerAction::canRunAtWaypoint(
  const std::string & waypoint_name,
  const std::string & required_waypoint_name) const
{
  return utils::normalizeToken(waypoint_name) == utils::normalizeToken(required_waypoint_name);
}

}  // namespace dog_behavior::bt_nodes
