#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace dog_perception
{

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode();

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

}  // namespace dog_perception