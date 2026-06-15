#include "dog_behavior/bt_nodes/set_boxes_type_action.hpp"

#include <algorithm>
#include <utility>

namespace dog_behavior::bt_nodes
{

SetBoxesTypeAction::SetBoxesTypeAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
, timeout_ms_(3000)
, subscription_initialized_(false)
, boxes_ready_once_(false)
{
}

BT::PortsList SetBoxesTypeAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("target_topic", "/target/target_3d", "target 3d array topic"),
    BT::InputPort<int>("timeout_ms", 3000, "target wait timeout milliseconds"),
  };
}

BT::NodeStatus SetBoxesTypeAction::onStart()
{
  if (!config().blackboard) {
    return BT::NodeStatus::FAILURE;
  }

  if (boxes_ready_once_) {
    return BT::NodeStatus::SUCCESS;
  }

  const auto target_topic_input = getInput<std::string>("target_topic");
  const auto timeout_input = getInput<int>("timeout_ms");
  if (!target_topic_input) {
    return BT::NodeStatus::FAILURE;
  }

  timeout_ms_ = timeout_input && timeout_input.value() > 0 ? timeout_input.value() : 3000;

  if (!ensureSubscription(target_topic_input.value())) {
    return BT::NodeStatus::FAILURE;
  }

  wait_start_time_ = node_->now();
  return tryCommitBoxes();
}

BT::NodeStatus SetBoxesTypeAction::onRunning()
{
  const auto status = tryCommitBoxes();
  if (status != BT::NodeStatus::RUNNING) {
    return status;
  }

  const auto elapsed_ms = (node_->now() - wait_start_time_).nanoseconds() / 1000000;
  if (elapsed_ms >= timeout_ms_) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SetBoxesTypeAction::onHalted()
{
}

BT::NodeStatus SetBoxesTypeAction::tryCommitBoxes()
{
  dog_interfaces::msg::Target3DArray::ConstSharedPtr boxes;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    boxes = latest_boxes_;
  }

  if (!boxes) {
    return BT::NodeStatus::RUNNING;
  }

  const auto boxes_type_list = sortToTwoRows(*boxes);
  config().blackboard->set("boxes_type_list", boxes_type_list);
  config().blackboard->set("boxes_ready", true);
  config().blackboard->set("boxes_capture_stamp", node_->now().nanoseconds());
  boxes_ready_once_ = true;
  return BT::NodeStatus::SUCCESS;
}

bool SetBoxesTypeAction::ensureSubscription(const std::string & target_topic)
{
  if (subscription_initialized_ && target_topic_ == target_topic) {
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

  target_topic_ = target_topic;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_boxes_.reset();
  }
  subscription_ = node_->create_subscription<dog_interfaces::msg::Target3DArray>(
    target_topic_,
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
