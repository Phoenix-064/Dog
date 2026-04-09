#include "dog_behavior/navigation_executor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_behavior::NavigationExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
