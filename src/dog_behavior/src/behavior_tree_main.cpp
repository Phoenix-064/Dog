#include "dog_behavior/behavior_tree_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_behavior::BehaviorTreeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}