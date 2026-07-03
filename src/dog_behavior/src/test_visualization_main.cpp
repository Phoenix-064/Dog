#include "dog_behavior/test_visualization_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_behavior::TestVisualizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
