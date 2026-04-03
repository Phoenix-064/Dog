#include "dog_perception/perception_node.hpp"

#include <functional>

namespace dog_perception
{

PerceptionNode::PerceptionNode()
: rclcpp::Node("dog_perception")
{
  const auto lidar_topic = declare_parameter<std::string>("lidar_topic", "/livox/lidar");

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PerceptionNode::lidarCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "dog_perception skeleton node initialized, subscribed lidar_topic=%s",
    lidar_topic.c_str());
}

void PerceptionNode::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  (void)msg;
  RCLCPP_DEBUG(get_logger(), "Received lidar frame");
}

}  // namespace dog_perception