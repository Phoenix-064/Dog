#include "dog_lifecycle/lifecycle_node.hpp"

#include <lifecycle_msgs/msg/transition.hpp>

#include <memory>

/// @brief dog_lifecycle ROS 2 节点进程入口。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_lifecycle::LifecycleNode>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}