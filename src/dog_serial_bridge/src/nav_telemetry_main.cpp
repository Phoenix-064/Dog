#include "dog_serial_bridge/nav_telemetry_serial_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>();
  rclcpp::spin(node);
  node->SendShutdownArrivalFrame();
  rclcpp::shutdown();
  return 0;
}
