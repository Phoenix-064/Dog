#include "dog_behavior/behavior_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gtest/gtest.h>

namespace
{
void spinFor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

bool waitForTopicMatch(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const rclcpp::Node::SharedPtr & io_node,
  const std::string & odom_topic,
  const std::string & pose_topic,
  const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (io_node->count_subscribers(odom_topic) > 0u && io_node->count_publishers(pose_topic) > 0u) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

}  // namespace

class BehaviorNodeTest : public ::testing::Test
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

TEST_F(BehaviorNodeTest, NodeInitCreatesParametersAndPublisher)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", "/test/global_pose/init");
  options.append_parameter_override("localization_topic", "/test/localization/init");
  options.append_parameter_override("default_frame_id", "map");

  auto node = std::make_shared<dog_behavior::BehaviorNode>(options);

  EXPECT_STREQ(node->get_name(), "dog_behavior");
  EXPECT_TRUE(node->has_parameter("global_pose_topic"));
  EXPECT_TRUE(node->has_parameter("localization_topic"));
  EXPECT_TRUE(node->has_parameter("default_frame_id"));
  EXPECT_EQ(node->count_publishers("/test/global_pose/init"), 1u);
}

TEST_F(BehaviorNodeTest, ConvertsOdometryToPoseStampedAndPublishes)
{
  const std::string odom_topic = "/test/localization/convert";
  const std::string pose_topic = "/test/global_pose/convert";

  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", pose_topic);
  options.append_parameter_override("localization_topic", odom_topic);
  options.append_parameter_override("default_frame_id", "base_link");

  auto behavior_node = std::make_shared<dog_behavior::BehaviorNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("behavior_test_io_node");

  bool received = false;
  geometry_msgs::msg::PoseStamped captured;
  auto pose_sub = io_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic,
    rclcpp::QoS(rclcpp::KeepLast(20)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&received, &captured](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      received = true;
      captured = *msg;
    });

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS().keep_last(20));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(behavior_node);
  executor.add_node(io_node);

  spinFor(executor, std::chrono::milliseconds(100));
  ASSERT_TRUE(waitForTopicMatch(executor, io_node, odom_topic, pose_topic, std::chrono::milliseconds(800)));

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = behavior_node->now();
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = 1.2;
  odom.pose.pose.position.y = -0.8;
  odom.pose.pose.position.z = 0.15;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.7071067;
  odom.pose.pose.orientation.w = 0.7071067;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    odom_pub->publish(odom);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(received);
  EXPECT_EQ(captured.header.stamp, odom.header.stamp);
  EXPECT_EQ(captured.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(captured.pose.position.x, odom.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(captured.pose.position.y, odom.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(captured.pose.position.z, odom.pose.pose.position.z);
  EXPECT_DOUBLE_EQ(captured.pose.orientation.z, odom.pose.pose.orientation.z);
  EXPECT_DOUBLE_EQ(captured.pose.orientation.w, odom.pose.pose.orientation.w);

  executor.remove_node(io_node);
  executor.remove_node(behavior_node);
  (void)pose_sub;
}

TEST_F(BehaviorNodeTest, FallsBackToDefaultFrameWhenInputFrameEmpty)
{
  const std::string odom_topic = "/test/localization/fallback";
  const std::string pose_topic = "/test/global_pose/fallback";

  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", pose_topic);
  options.append_parameter_override("localization_topic", odom_topic);
  options.append_parameter_override("default_frame_id", "map_default");

  auto behavior_node = std::make_shared<dog_behavior::BehaviorNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("behavior_test_io_fallback");

  bool received = false;
  geometry_msgs::msg::PoseStamped captured;
  auto pose_sub = io_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic,
    rclcpp::QoS(rclcpp::KeepLast(20)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&received, &captured](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      received = true;
      captured = *msg;
    });

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS().keep_last(20));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(behavior_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitForTopicMatch(executor, io_node, odom_topic, pose_topic, std::chrono::milliseconds(800)));

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = behavior_node->now();
  odom.header.frame_id = "";
  odom.pose.pose.orientation.w = 1.0;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    odom_pub->publish(odom);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(received);
  EXPECT_EQ(captured.header.frame_id, "map_default");

  executor.remove_node(io_node);
  executor.remove_node(behavior_node);
  (void)pose_sub;
}

TEST_F(BehaviorNodeTest, DropsPoseWhenBothInputAndDefaultFrameEmpty)
{
  const std::string odom_topic = "/test/localization/drop";
  const std::string pose_topic = "/test/global_pose/drop";

  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", pose_topic);
  options.append_parameter_override("localization_topic", odom_topic);
  options.append_parameter_override("default_frame_id", "");

  auto behavior_node = std::make_shared<dog_behavior::BehaviorNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("behavior_test_io_drop");

  bool received = false;
  auto pose_sub = io_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic,
    rclcpp::QoS(rclcpp::KeepLast(20)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&received](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      (void)msg;
      received = true;
    });

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS().keep_last(20));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(behavior_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitForTopicMatch(executor, io_node, odom_topic, pose_topic, std::chrono::milliseconds(800)));

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = behavior_node->now();
  odom.header.frame_id = "";
  odom.pose.pose.orientation.w = 1.0;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
  while (std::chrono::steady_clock::now() < deadline) {
    odom_pub->publish(odom);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_FALSE(received);

  executor.remove_node(io_node);
  executor.remove_node(behavior_node);
  (void)pose_sub;
}