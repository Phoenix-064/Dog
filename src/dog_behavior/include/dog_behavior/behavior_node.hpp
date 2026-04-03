#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dog_behavior
{

class BehaviorNode : public rclcpp::Node
{
public:
  BehaviorNode();
  explicit BehaviorNode(const rclcpp::NodeOptions & options);

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  bool isFinitePose(const geometry_msgs::msg::Pose & pose) const;
  bool hasValidQuaternionNorm(const geometry_msgs::msg::Pose & pose) const;

  std::string default_frame_id_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
};

}  // namespace dog_behavior