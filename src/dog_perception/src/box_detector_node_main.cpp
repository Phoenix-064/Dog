#include "dog_perception/box_detector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_perception::BoxDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
