#include "dog_behavior/behavior_node.hpp"

#include <functional>

namespace dog_behavior
{

BehaviorNode::BehaviorNode()
: rclcpp::Node("dog_behavior")
{
  const auto target_topic = declare_parameter<std::string>("target_pick_topic", "/target/pick_tasks");
  const auto localization_topic = declare_parameter<std::string>("localization_topic", "/localization/dog");

  target_pub_ = create_publisher<dog_interfaces::msg::Target3D>(target_topic, 10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    localization_topic,
    10,
    std::bind(&BehaviorNode::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "dog_behavior skeleton node initialized, localization_topic=%s, target_pick_topic=%s",
    localization_topic.c_str(),
    target_topic.c_str());
}

void BehaviorNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  (void)msg;
  dog_interfaces::msg::Target3D target;
  target.target_id = "placeholder";
  target.confidence = 0.0F;
  target_pub_->publish(target);
}

}  // namespace dog_behavior