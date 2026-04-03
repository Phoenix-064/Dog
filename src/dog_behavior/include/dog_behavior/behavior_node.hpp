#pragma once

#include <dog_interfaces/msg/target3_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dog_behavior
{

class BehaviorNode : public rclcpp::Node
{
public:
  BehaviorNode();

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<dog_interfaces::msg::Target3D>::SharedPtr target_pub_;
};

}  // namespace dog_behavior