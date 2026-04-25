#include "dog_behavior/bt_nodes/set_boxes_type_action.hpp"

#include <algorithm>
#include <utility>

namespace dog_behavior::bt_nodes
{

SetBoxesTypeAction::SetBoxesTypeAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
, subscription_initialized_(false)
, boxes_ready_once_(false)
{
}

BT::PortsList SetBoxesTypeAction::providedPorts()
{
  return {};
}

BT::NodeStatus SetBoxesTypeAction::tick()
{
  if (!config().blackboard) {
    return BT::NodeStatus::FAILURE;
  }

  if (boxes_ready_once_) {
    return BT::NodeStatus::SUCCESS;
  }

  if (!ensureSubscription()) {
    return BT::NodeStatus::FAILURE;
  }

  dog_interfaces::msg::Target3DArray::ConstSharedPtr boxes;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    boxes = latest_boxes_;
  }

  if (!boxes) {
    return BT::NodeStatus::FAILURE;
  }

  const auto boxes_type_list = sortToTwoRows(*boxes);
  config().blackboard->set("boxes_type_list", boxes_type_list);
  config().blackboard->set("boxes_ready", true);
  config().blackboard->set("boxes_capture_stamp", node_->now().nanoseconds());
  boxes_ready_once_ = true;
  return BT::NodeStatus::SUCCESS;
}

bool SetBoxesTypeAction::ensureSubscription()
{
  if (subscription_initialized_) {
    return true;
  }

  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  } catch (const std::exception &) {
    return false;
  }

  if (!node_) {
    return false;
  }

  subscription_ = node_->create_subscription<dog_interfaces::msg::Target3DArray>(
    "/target/box_result",
    rclcpp::QoS(10),
    [this](const dog_interfaces::msg::Target3DArray::ConstSharedPtr msg) {
      this->boxesCallback(msg);
    });

  subscription_initialized_ = true;
  return true;
}

void SetBoxesTypeAction::boxesCallback(const dog_interfaces::msg::Target3DArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_boxes_ = std::move(msg);
}

std::vector<std::string> SetBoxesTypeAction::sortToTwoRows(const dog_interfaces::msg::Target3DArray & msg) const
{
  auto sorted_targets = msg.targets;
  std::sort(
    sorted_targets.begin(),
    sorted_targets.end(),
    [](const dog_interfaces::msg::Target3D & a, const dog_interfaces::msg::Target3D & b) {
      return a.position.y < b.position.y;
    });

  const size_t first_row_count = std::min<size_t>(4, sorted_targets.size());
  std::vector<dog_interfaces::msg::Target3D> first_row(
    sorted_targets.begin(),
    sorted_targets.begin() + static_cast<std::ptrdiff_t>(first_row_count));
  std::vector<dog_interfaces::msg::Target3D> second_row(
    sorted_targets.begin() + static_cast<std::ptrdiff_t>(first_row_count),
    sorted_targets.end());

  std::sort(
    first_row.begin(),
    first_row.end(),
    [](const dog_interfaces::msg::Target3D & a, const dog_interfaces::msg::Target3D & b) {
      return a.position.x < b.position.x;
    });
  std::sort(
    second_row.begin(),
    second_row.end(),
    [](const dog_interfaces::msg::Target3D & a, const dog_interfaces::msg::Target3D & b) {
      return a.position.x < b.position.x;
    });

  std::vector<std::string> boxes_type_list;
  boxes_type_list.reserve(first_row.size() + second_row.size());

  for (const auto & target : first_row) {
    boxes_type_list.push_back(target.target_id);
  }
  for (const auto & target : second_row) {
    boxes_type_list.push_back(target.target_id);
  }

  return boxes_type_list;
}

}  // namespace dog_behavior::bt_nodes
