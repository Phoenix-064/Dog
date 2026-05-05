#include "dog_serial_bridge/serial_bridge_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_serial_bridge::SerialBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
