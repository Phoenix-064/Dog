#include "dog_behavior/test_visualization_node.hpp"

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace
{

bool waitUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::milliseconds timeout,
  const std::function<bool()> & condition)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return condition();
}

std::string writeWaypointFile()
{
  const std::string path = "/tmp/dog_test_visualization_waypoints.yaml";
  std::ofstream ofs(path);
  ofs << "waypoints:\n"
      << "  - name: alpha\n"
      << "    x: 1.0\n"
      << "    y: 2.0\n"
      << "    z: 0.0\n"
      << "    yaw: 0.0\n"
      << "  - name: beta\n"
      << "    x: 3.0\n"
      << "    y: 4.0\n"
      << "    z: 0.0\n"
      << "    yaw: 90.0\n";
  return path;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool markerTextContains(
  const visualization_msgs::msg::MarkerArray & markers,
  const std::string & text)
{
  for (const auto & marker : markers.markers) {
    if (marker.text.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

class TestVisualizationNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(TestVisualizationNodeTest, PublishesRouteMarkersGoalAndState)
{
  const std::string pose_topic = "/test/viz/current_pose";
  const std::string goal_topic = "/test/viz/nav_goal";
  const std::string state_topic = "/test/viz/nav_state";
  const std::string route_topic = "/test/viz/route";
  const std::string marker_topic = "/test/viz/markers";

  rclcpp::NodeOptions options;
  options.append_parameter_override("frame_id", "map");
  options.append_parameter_override("waypoints_file", writeWaypointFile());
  options.append_parameter_override("current_pose_topic", pose_topic);
  options.append_parameter_override("goal_topic", goal_topic);
  options.append_parameter_override("state_topic", state_topic);
  options.append_parameter_override("route_topic", route_topic);
  options.append_parameter_override("marker_topic", marker_topic);
  options.append_parameter_override("publish_period_ms", 50);

  auto viz_node = std::make_shared<dog_behavior::TestVisualizationNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("test_visualization_io");

  auto pose_pub = io_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    pose_topic,
    rclcpp::QoS(rclcpp::KeepLast(20)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  auto goal_pub = io_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    goal_topic,
    rclcpp::QoS(rclcpp::KeepLast(20)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  auto state_pub = io_node->create_publisher<std_msgs::msg::String>(
    state_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  viz_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  viz_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  nav_msgs::msg::Path::SharedPtr route_msg;
  visualization_msgs::msg::MarkerArray::SharedPtr marker_msg;
  auto route_sub = io_node->create_subscription<nav_msgs::msg::Path>(
    route_topic,
    viz_qos,
    [&route_msg](const nav_msgs::msg::Path::SharedPtr msg) { route_msg = msg; });
  auto marker_sub = io_node->create_subscription<visualization_msgs::msg::MarkerArray>(
    marker_topic,
    viz_qos,
    [&marker_msg](const visualization_msgs::msg::MarkerArray::SharedPtr msg) { marker_msg = msg; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(viz_node);
  executor.add_node(io_node);

  ASSERT_EQ(viz_node->WaypointCountForTest(), 2u);
  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&io_node, &pose_topic, &goal_topic, &state_topic]() {
      return io_node->count_subscribers(pose_topic) > 0u &&
             io_node->count_subscribers(goal_topic) > 0u &&
             io_node->count_subscribers(state_topic) > 0u;
    }));

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.25;
  pose.pose.orientation.w = 1.0;
  pose_pub->publish(pose);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 3.0;
  goal.pose.position.y = 4.0;
  goal.pose.orientation.w = 1.0;
  goal_pub->publish(goal);

  std_msgs::msg::String state;
  state.data = "running";
  state_pub->publish(state);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(2000),
    [&route_msg, &marker_msg]() {
      return route_msg && marker_msg;
    }));

  EXPECT_TRUE(viz_node->HasCurrentPoseForTest());
  EXPECT_TRUE(viz_node->HasGoalForTest());
  EXPECT_EQ(viz_node->NavStateForTest(), "running");
  ASSERT_EQ(route_msg->poses.size(), 2u);
  EXPECT_EQ(route_msg->header.frame_id, "map");
  EXPECT_NEAR(
    yawFromQuaternion(route_msg->poses.at(1).pose.orientation),
    dog_behavior::kPi / 2.0,
    1.0e-6);
  EXPECT_TRUE(markerTextContains(*marker_msg, "alpha"));
  EXPECT_TRUE(markerTextContains(*marker_msg, "beta"));
  EXPECT_TRUE(markerTextContains(*marker_msg, "nav_state: running"));

  executor.remove_node(io_node);
  executor.remove_node(viz_node);

  (void)route_sub;
  (void)marker_sub;
}

}  // namespace
