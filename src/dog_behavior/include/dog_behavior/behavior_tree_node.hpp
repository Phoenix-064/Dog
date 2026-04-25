#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace dog_behavior
{

struct Waypoint
{
  std::string name;
  double x;
  double y;
  double z;
  double yaw;
};

class BehaviorTreeNode : public rclcpp::Node
{
public:
  BehaviorTreeNode();
  explicit BehaviorTreeNode(const rclcpp::NodeOptions & options);

  int TickCountForTest() const;
  std::string LastTickStatusForTest() const;
  std::string SystemModeForTest() const;
  std::string BehaviorNameForTest() const;
  bool HasLatestPoseForTest() const;
  bool IsTreeActiveForTest() const;
  size_t WaypointCountForTest() const;

private:
  void configureTree();
  void registerBuiltinNodes();
  void timerCallback();
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void executeTriggerCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void systemModeCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void recoveryContextCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void loadWaypoints(const std::string & file_path);

  std::string default_frame_id_;
  std::string execute_behavior_trigger_topic_;
  std::string recovery_context_topic_;
  std::string system_mode_topic_;
  std::string match_type_;
  std::string tree_xml_file_path_;
  int tick_period_ms_;

  mutable std::mutex state_mutex_;
  bool has_latest_pose_;
  bool tree_active_;
  bool ros_node_seeded_;
  int tick_count_;
  std::string system_mode_;
  std::string recovery_context_payload_;
  std::string behavior_name_;
  std::string last_tick_status_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  std::vector<Waypoint> waypoints_;

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execute_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_context_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace dog_behavior