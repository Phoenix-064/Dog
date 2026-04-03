#include "dog_lifecycle/lifecycle_node.hpp"

namespace dog_lifecycle
{

LifecycleNode::LifecycleNode()
: rclcpp::Node("dog_lifecycle")
{
  RCLCPP_INFO(get_logger(), "dog_lifecycle skeleton node initialized");
}

}  // namespace dog_lifecycle