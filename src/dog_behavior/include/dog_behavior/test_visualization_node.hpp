#pragma once

#include "dog_behavior/behavior_tree_node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace dog_behavior
{

class TestVisualizationNode : public rclcpp::Node
{
public:
  TestVisualizationNode();
  explicit TestVisualizationNode(const rclcpp::NodeOptions & options);

  size_t WaypointCountForTest() const;
  std::string NavStateForTest() const;
  bool HasGoalForTest() const;
  bool HasCurrentPoseForTest() const;

private:
  void loadWaypoints(const std::string & file_path);
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void stateCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void publishVisualization();
  nav_msgs::msg::Path buildRoutePath(const rclcpp::Time & stamp) const;
  visualization_msgs::msg::MarkerArray buildMarkers(const rclcpp::Time & stamp) const;

  std::string frame_id_;
  std::vector<Waypoint> waypoints_;

  mutable std::mutex state_mutex_;
  bool has_current_pose_;
  bool has_goal_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped current_goal_;
  std::string nav_state_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dog_behavior
