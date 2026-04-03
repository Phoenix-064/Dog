#include "dog_lifecycle/lifecycle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_lifecycle::LifecycleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}