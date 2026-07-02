#include "dog_behavior/bt_nodes/auto_success_action.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>

namespace dog_behavior::bt_nodes
{

AutoSuccessAction::AutoSuccessAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList AutoSuccessAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("label", "auto_success", "test step label"),
    BT::OutputPort<std::string>("result_code_text"),
  };
}

BT::NodeStatus AutoSuccessAction::tick()
{
  const auto label_input = getInput<std::string>("label");
  const std::string label = label_input ? label_input.value() : "auto_success";
  setOutput("result_code_text", std::string("auto_success"));

  if (!config().blackboard) {
    return BT::NodeStatus::SUCCESS;
  }

  try {
    const auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
    if (node) {
      RCLCPP_INFO(
        node->get_logger(),
        "test_mode_auto_success label=%s result=auto_success",
        label.c_str());
    }
  } catch (const std::exception &) {
    // 单元测试或离线解析场景可以没有 ros_node，测试模式仍直接放行。
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace dog_behavior::bt_nodes
