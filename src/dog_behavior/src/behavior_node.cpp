#include "dog_behavior/behavior_node.hpp"

#include <cmath>
#include <functional>

namespace dog_behavior
{

BehaviorNode::BehaviorNode()
: BehaviorNode(rclcpp::NodeOptions())
{
}

BehaviorNode::BehaviorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dog_behavior", options)
{
  const auto global_pose_topic = declare_parameter<std::string>("global_pose_topic", "/dog/global_pose");
  const auto localization_topic = declare_parameter<std::string>("localization_topic", "/localization/dog");
  default_frame_id_ = declare_parameter<std::string>("default_frame_id", "base_link");

  auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(20));
  pose_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  global_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    global_pose_topic,
    pose_qos);

  auto odom_qos = rclcpp::SensorDataQoS();
  odom_qos.keep_last(20);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    localization_topic,
    odom_qos,
    std::bind(&BehaviorNode::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "dog_behavior initialized, localization_topic=%s, global_pose_topic=%s, default_frame_id=%s",
    localization_topic.c_str(),
    global_pose_topic.c_str(),
    default_frame_id_.c_str());
}

void BehaviorNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = msg->header.frame_id;
  pose_msg.pose = msg->pose.pose;

  if (pose_msg.header.frame_id.empty()) {
    if (default_frame_id_.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Drop pose publish because input frame_id is empty and default_frame_id is not configured (stamp=%u.%u)",
        pose_msg.header.stamp.sec,
        pose_msg.header.stamp.nanosec);
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Input frame_id is empty at stamp=%u.%u, fallback to default_frame_id=%s",
      pose_msg.header.stamp.sec,
      pose_msg.header.stamp.nanosec,
      default_frame_id_.c_str());
    pose_msg.header.frame_id = default_frame_id_;
  }

  if (!isFinitePose(pose_msg.pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop pose publish because pose contains non-finite values (frame_id=%s)",
      pose_msg.header.frame_id.c_str());
    return;
  }

  if (!hasValidQuaternionNorm(pose_msg.pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop pose publish because quaternion norm is invalid (frame_id=%s)",
      pose_msg.header.frame_id.c_str());
    return;
  }

  global_pose_pub_->publish(pose_msg);
}

bool BehaviorNode::isFinitePose(const geometry_msgs::msg::Pose & pose) const
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool BehaviorNode::hasValidQuaternionNorm(const geometry_msgs::msg::Pose & pose) const
{
  const double norm =
    (pose.orientation.x * pose.orientation.x) +
    (pose.orientation.y * pose.orientation.y) +
    (pose.orientation.z * pose.orientation.z) +
    (pose.orientation.w * pose.orientation.w);

  constexpr double min_norm = 1e-6;
  constexpr double target_norm = 1.0;
  constexpr double tolerance = 0.1;
  return norm > min_norm && std::fabs(norm - target_norm) <= tolerance;
}

}  // namespace dog_behavior