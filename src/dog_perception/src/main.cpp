#include "dog_perception/perception_node.hpp"

/// @brief Entry point for the dog_perception ROS 2 node process.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_perception::PerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}