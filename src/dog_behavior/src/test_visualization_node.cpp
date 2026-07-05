#include "dog_behavior/test_visualization_node.hpp"

#include "dog_behavior/common/payload_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <exception>
#include <string>

namespace dog_behavior
{

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternionFromYawDegrees(const double yaw_deg)
{
  const double yaw_rad = degreesToRadians(yaw_deg);
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw_rad / 2.0);
  q.w = std::cos(yaw_rad / 2.0);
  return q;
}

std_msgs::msg::ColorRGBA makeColor(
  const float red,
  const float green,
  const float blue,
  const float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

std_msgs::msg::ColorRGBA stateColor(const std::string & state)
{
  if (state == "succeeded") {
    return makeColor(0.1F, 0.8F, 0.2F);
  }
  if (state == "running" || state == "forwarding_goal") {
    return makeColor(0.1F, 0.45F, 1.0F);
  }
  if (state == "timeout" || state == "failed") {
    return makeColor(1.0F, 0.15F, 0.1F);
  }
  return makeColor(0.95F, 0.75F, 0.15F);
}

visualization_msgs::msg::Marker baseMarker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns,
  const int id,
  const int type)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color = makeColor(1.0F, 1.0F, 1.0F);
  return marker;
}

}  // namespace

TestVisualizationNode::TestVisualizationNode()
: TestVisualizationNode(rclcpp::NodeOptions())
{
}

TestVisualizationNode::TestVisualizationNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dog_test_visualization", options)
, has_current_pose_(false)
, has_goal_(false)
, nav_state_("idle")
{
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  const auto waypoints_file = declare_parameter<std::string>("waypoints_file", "");
  const auto current_pose_topic = declare_parameter<std::string>("current_pose_topic", "/dog/global_pose");
  const auto goal_topic = declare_parameter<std::string>("goal_topic", "/behavior/nav_goal");
  const auto state_topic = declare_parameter<std::string>("state_topic", "/behavior/nav_exec_state");
  const auto route_topic = declare_parameter<std::string>(
    "route_topic",
    "/behavior/test_visualization/route");
  const auto marker_topic = declare_parameter<std::string>(
    "marker_topic",
    "/behavior/test_visualization/markers");
  const auto publish_period_ms = declare_parameter<int>("publish_period_ms", 500);

  if (!waypoints_file.empty()) {
    loadWaypoints(waypoints_file);
  }

  auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(20));
  pose_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic,
    pose_qos,
    std::bind(&TestVisualizationNode::currentPoseCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic,
    pose_qos,
    std::bind(&TestVisualizationNode::goalCallback, this, std::placeholders::_1));
  state_sub_ = create_subscription<std_msgs::msg::String>(
    state_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    std::bind(&TestVisualizationNode::stateCallback, this, std::placeholders::_1));

  auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  viz_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  viz_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  route_pub_ = create_publisher<nav_msgs::msg::Path>(route_topic, viz_qos);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, viz_qos);

  const int bounded_period_ms = publish_period_ms > 0 ? publish_period_ms : 500;
  timer_ = create_wall_timer(
    std::chrono::milliseconds(bounded_period_ms),
    std::bind(&TestVisualizationNode::publishVisualization, this));

  RCLCPP_INFO(
    get_logger(),
    "TestVisualizationNode initialized, frame_id=%s, waypoints=%zu, route_topic=%s, marker_topic=%s",
    frame_id_.c_str(),
    waypoints_.size(),
    route_topic.c_str(),
    marker_topic.c_str());
}

void TestVisualizationNode::loadWaypoints(const std::string & file_path)
{
  try {
    const auto config = YAML::LoadFile(file_path);
    const auto waypoints_node = config["waypoints"];
    if (!waypoints_node || !waypoints_node.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "Invalid waypoints file format: %s", file_path.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    waypoints_.clear();
    for (const auto & wp_node : waypoints_node) {
      Waypoint wp;
      wp.name = wp_node["name"].as<std::string>("");
      wp.x = wp_node["x"].as<double>(0.0);
      wp.y = wp_node["y"].as<double>(0.0);
      wp.z = wp_node["z"].as<double>(0.0);
      wp.yaw_deg = wp_node["yaw"].as<double>(0.0);
      waypoints_.push_back(wp);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load waypoints from %s: %s", file_path.c_str(), e.what());
  }
}

void TestVisualizationNode::currentPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!msg || !utils::isFinitePose(msg->pose) || !utils::hasValidQuaternionNorm(*msg)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop invalid current pose for test visualization");
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  current_pose_ = *msg;
  has_current_pose_ = true;
}

void TestVisualizationNode::goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!msg || !utils::isFinitePose(msg->pose) || !utils::hasValidQuaternionNorm(*msg)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop invalid goal pose for test visualization");
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  current_goal_ = *msg;
  has_goal_ = true;
}

void TestVisualizationNode::stateCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  nav_state_ = msg ? utils::normalizeToken(msg->data) : "idle";
  if (nav_state_.empty()) {
    nav_state_ = "idle";
  }
}

void TestVisualizationNode::publishVisualization()
{
  const auto stamp = now();
  route_pub_->publish(buildRoutePath(stamp));
  marker_pub_->publish(buildMarkers(stamp));
}

nav_msgs::msg::Path TestVisualizationNode::buildRoutePath(const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id_;
  path.header.stamp = stamp;

  for (const auto & wp : waypoints_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = wp.z;
    pose.pose.orientation = quaternionFromYawDegrees(wp.yaw_deg);
    path.poses.push_back(pose);
  }
  return path;
}

visualization_msgs::msg::MarkerArray TestVisualizationNode::buildMarkers(
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  visualization_msgs::msg::MarkerArray array;

  auto delete_all = baseMarker(
    frame_id_,
    stamp,
    "test_navigation",
    0,
    visualization_msgs::msg::Marker::SPHERE);
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(delete_all);

  auto route_line = baseMarker(
    frame_id_,
    stamp,
    "test_navigation_route",
    1,
    visualization_msgs::msg::Marker::LINE_STRIP);
  route_line.scale.x = 0.04;
  route_line.color = makeColor(0.2F, 0.9F, 0.9F, 0.85F);
  for (const auto & wp : waypoints_) {
    geometry_msgs::msg::Point point;
    point.x = wp.x;
    point.y = wp.y;
    point.z = wp.z + 0.03;
    route_line.points.push_back(point);
  }
  array.markers.push_back(route_line);

  int marker_id = 10;
  for (size_t index = 0; index < waypoints_.size(); ++index) {
    const auto & wp = waypoints_.at(index);
    auto sphere = baseMarker(
      frame_id_,
      stamp,
      "test_navigation_waypoints",
      marker_id++,
      visualization_msgs::msg::Marker::SPHERE);
    sphere.pose.position.x = wp.x;
    sphere.pose.position.y = wp.y;
    sphere.pose.position.z = wp.z + 0.08;
    sphere.scale.x = 0.18;
    sphere.scale.y = 0.18;
    sphere.scale.z = 0.18;
    sphere.color = makeColor(1.0F, 0.75F, 0.15F);
    array.markers.push_back(sphere);

    auto label = baseMarker(
      frame_id_,
      stamp,
      "test_navigation_labels",
      marker_id++,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    label.pose.position.x = wp.x;
    label.pose.position.y = wp.y;
    label.pose.position.z = wp.z + 0.45;
    label.scale.z = 0.22;
    label.color = makeColor(1.0F, 1.0F, 1.0F);
    label.text = wp.name.empty() ? ("wp_" + std::to_string(index + 1U)) : wp.name;
    array.markers.push_back(label);
  }

  if (has_goal_) {
    auto goal = baseMarker(
      current_goal_.header.frame_id.empty() ? frame_id_ : current_goal_.header.frame_id,
      stamp,
      "test_navigation_goal",
      marker_id++,
      visualization_msgs::msg::Marker::ARROW);
    goal.pose = current_goal_.pose;
    goal.scale.x = 0.55;
    goal.scale.y = 0.12;
    goal.scale.z = 0.12;
    goal.color = stateColor(nav_state_);
    array.markers.push_back(goal);
  }

  if (has_current_pose_) {
    auto current = baseMarker(
      current_pose_.header.frame_id.empty() ? frame_id_ : current_pose_.header.frame_id,
      stamp,
      "test_navigation_current_pose",
      marker_id++,
      visualization_msgs::msg::Marker::ARROW);
    current.pose = current_pose_.pose;
    current.scale.x = 0.45;
    current.scale.y = 0.09;
    current.scale.z = 0.09;
    current.color = makeColor(0.2F, 1.0F, 0.25F);
    array.markers.push_back(current);
  }

  auto status = baseMarker(
    frame_id_,
    stamp,
    "test_navigation_status",
    marker_id++,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  if (has_current_pose_) {
    status.pose.position = current_pose_.pose.position;
    status.pose.position.z += 0.9;
  } else if (!waypoints_.empty()) {
    status.pose.position.x = waypoints_.front().x;
    status.pose.position.y = waypoints_.front().y;
    status.pose.position.z = waypoints_.front().z + 0.9;
  } else {
    status.pose.position.z = 0.9;
  }
  status.scale.z = 0.28;
  status.color = stateColor(nav_state_);
  status.text = "nav_state: " + nav_state_;
  if (has_goal_) {
    status.text += "\ngoal_yaw_deg: " +
      std::to_string(radiansToDegrees(yawFromQuaternion(current_goal_.pose.orientation)));
  }
  array.markers.push_back(status);

  return array;
}

size_t TestVisualizationNode::WaypointCountForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return waypoints_.size();
}

std::string TestVisualizationNode::NavStateForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return nav_state_;
}

bool TestVisualizationNode::HasGoalForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return has_goal_;
}

bool TestVisualizationNode::HasCurrentPoseForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return has_current_pose_;
}

}  // namespace dog_behavior
