#include "dog_behavior/behavior_node.hpp"

/// @brief dog_behavior ROS 2 节点进程入口。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_behavior::BehaviorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}