#include "dog_behavior/bt_nodes/wait_for_pose_condition.hpp"

#include <chrono>

namespace dog_behavior::bt_nodes
{

namespace
{
uint64_t nowMs()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}
}  // namespace

WaitForPoseCondition::WaitForPoseCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config), started_waiting_(false), first_wait_time_ms_(0)
{
}

BT::PortsList WaitForPoseCondition::providedPorts()
{
  return {
    BT::InputPort<bool>("has_pose"),
    BT::InputPort<int>("timeout_ms", 5000, "pose wait timeout milliseconds"),
  };
}

BT::NodeStatus WaitForPoseCondition::tick()
{
  const auto has_pose = getInput<bool>("has_pose");
  if (has_pose && has_pose.value()) {
    started_waiting_ = false;
    first_wait_time_ms_ = 0;
    return BT::NodeStatus::SUCCESS;
  }

  const auto timeout_ms = getInput<int>("timeout_ms");
  const uint64_t timeout_value = timeout_ms && timeout_ms.value() > 0 ?
    static_cast<uint64_t>(timeout_ms.value()) : 5000U;

  const uint64_t now = nowMs();
  if (!started_waiting_) {
    started_waiting_ = true;
    first_wait_time_ms_ = now;
    return BT::NodeStatus::FAILURE;
  }

  if (now - first_wait_time_ms_ >= timeout_value) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace dog_behavior::bt_nodes
