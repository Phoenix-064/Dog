#include "dog_behavior/behavior_tree_node.hpp"

#include <gtest/gtest.h>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
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

std::string readTextFile(const std::string & path)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return "";
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

class BehaviorTreeNodeTest : public ::testing::Test
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

TEST_F(BehaviorTreeNodeTest, NodeInitCreatesParametersAndPublisher)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", "/test/bt/global_pose/init");
  options.append_parameter_override("localization_topic", "/test/bt/localization/init");
  options.append_parameter_override("tree_xml_file_path", std::string(DOG_BEHAVIOR_TEST_BT_MAIN_XML_PATH));

  auto node = std::make_shared<dog_behavior::BehaviorTreeNode>(options);
  EXPECT_STREQ(node->get_name(), "dog_behavior_bt");
  EXPECT_TRUE(node->has_parameter("bt_tick_period_ms"));
  EXPECT_EQ(node->count_publishers("/test/bt/global_pose/init"), 1u);
}

TEST_F(BehaviorTreeNodeTest, TriggeredTickSucceedsAfterReceivingPose)
{
  const std::string odom_topic = "/test/bt/localization/success";
  const std::string pose_topic = "/test/bt/global_pose/success";
  const std::string trigger_topic = "/test/bt/execute_trigger/success";
  const std::string system_mode_topic = "/test/bt/system_mode/success";

  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", pose_topic);
  options.append_parameter_override("localization_topic", odom_topic);
  options.append_parameter_override("execute_behavior_trigger_topic", trigger_topic);
  options.append_parameter_override("system_mode_topic", system_mode_topic);
  options.append_parameter_override("tree_xml_file_path", std::string(DOG_BEHAVIOR_TEST_BT_MAIN_XML_PATH));
  options.append_parameter_override("bt_tick_period_ms", 50);

  auto bt_node = std::make_shared<dog_behavior::BehaviorTreeNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("bt_node_test_io_success");

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS().keep_last(20));
  auto trigger_pub = io_node->create_publisher<std_msgs::msg::String>(
    trigger_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  auto mode_pub = io_node->create_publisher<std_msgs::msg::String>(
    system_mode_topic,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bt_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&io_node, &odom_topic, &trigger_topic]() {
      return io_node->count_subscribers(odom_topic) > 0u && io_node->count_subscribers(trigger_topic) > 0u;
    }));

  std_msgs::msg::String mode_msg;
  mode_msg.data = "mode=normal";
  mode_pub->publish(mode_msg);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = bt_node->now();
  odom_msg.header.frame_id = "map";
  odom_msg.pose.pose.position.x = 1.0;
  odom_msg.pose.pose.position.y = 2.0;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.w = 1.0;
  odom_pub->publish(odom_msg);

  std_msgs::msg::String trigger_msg;
  trigger_msg.data = "grasp";
  trigger_pub->publish(trigger_msg);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(2000),
    [&bt_node]() {
      return bt_node->TickCountForTest() > 0;
    }));

  EXPECT_TRUE(bt_node->HasLatestPoseForTest());
  EXPECT_EQ(bt_node->BehaviorNameForTest(), "grasp");
  EXPECT_EQ(bt_node->LastTickStatusForTest(), "success");
  EXPECT_FALSE(bt_node->IsTreeActiveForTest());

  executor.remove_node(io_node);
  executor.remove_node(bt_node);
}

TEST_F(BehaviorTreeNodeTest, TriggeredTickFailsWhenSystemModeNotNormal)
{
  const std::string odom_topic = "/test/bt/localization/failure";
  const std::string trigger_topic = "/test/bt/execute_trigger/failure";
  const std::string system_mode_topic = "/test/bt/system_mode/failure";

  rclcpp::NodeOptions options;
  options.append_parameter_override("localization_topic", odom_topic);
  options.append_parameter_override("execute_behavior_trigger_topic", trigger_topic);
  options.append_parameter_override("system_mode_topic", system_mode_topic);
  options.append_parameter_override("tree_xml_file_path", std::string(DOG_BEHAVIOR_TEST_BT_MAIN_XML_PATH));
  options.append_parameter_override("bt_tick_period_ms", 50);

  auto bt_node = std::make_shared<dog_behavior::BehaviorTreeNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("bt_node_test_io_failure");

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS().keep_last(20));
  auto trigger_pub = io_node->create_publisher<std_msgs::msg::String>(
    trigger_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  auto mode_pub = io_node->create_publisher<std_msgs::msg::String>(
    system_mode_topic,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bt_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&io_node, &odom_topic, &trigger_topic]() {
      return io_node->count_subscribers(odom_topic) > 0u && io_node->count_subscribers(trigger_topic) > 0u;
    }));

  std_msgs::msg::String mode_msg;
  mode_msg.data = "mode=idle_spinning";
  mode_pub->publish(mode_msg);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = bt_node->now();
  odom_msg.header.frame_id = "map";
  odom_msg.pose.pose.orientation.w = 1.0;
  odom_pub->publish(odom_msg);

  std_msgs::msg::String trigger_msg;
  trigger_msg.data = "grasp";
  trigger_pub->publish(trigger_msg);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(2000),
    [&bt_node]() {
      return bt_node->TickCountForTest() > 0;
    }));

  EXPECT_EQ(bt_node->SystemModeForTest(), "idle_spinning");
  EXPECT_EQ(bt_node->LastTickStatusForTest(), "failure");
  EXPECT_FALSE(bt_node->IsTreeActiveForTest());

  executor.remove_node(io_node);
  executor.remove_node(bt_node);
}

TEST(BehaviorTreeXmlPlanCoverageTest, PlaceGoalsFollowForwardThenReverseOrder)
{
  const std::string xml = readTextFile(DOG_BEHAVIOR_PHASE3_BT_XML_PATH);
  ASSERT_FALSE(xml.empty());

  const auto first_p1 = xml.find("place_goal=\"{PlaceGoal1}\"");
  const auto first_p2 = xml.find("place_goal=\"{PlaceGoal2}\"", first_p1 + 1);
  const auto first_p3 = xml.find("place_goal=\"{PlaceGoal3}\"", first_p2 + 1);
  const auto first_p4 = xml.find("place_goal=\"{PlaceGoal4}\"", first_p3 + 1);

  ASSERT_NE(first_p1, std::string::npos);
  ASSERT_NE(first_p2, std::string::npos);
  ASSERT_NE(first_p3, std::string::npos);
  ASSERT_NE(first_p4, std::string::npos);
  EXPECT_LT(first_p1, first_p2);
  EXPECT_LT(first_p2, first_p3);
  EXPECT_LT(first_p3, first_p4);

  const auto second_p4 = xml.find("place_goal=\"{PlaceGoal4}\"", first_p4 + 1);
  const auto second_p3 = xml.find("place_goal=\"{PlaceGoal3}\"", second_p4 + 1);
  const auto second_p2 = xml.find("place_goal=\"{PlaceGoal2}\"", second_p3 + 1);
  const auto second_p1 = xml.find("place_goal=\"{PlaceGoal1}\"", second_p2 + 1);

  ASSERT_NE(second_p4, std::string::npos);
  ASSERT_NE(second_p3, std::string::npos);
  ASSERT_NE(second_p2, std::string::npos);
  ASSERT_NE(second_p1, std::string::npos);
  EXPECT_LT(second_p4, second_p3);
  EXPECT_LT(second_p3, second_p2);
  EXPECT_LT(second_p2, second_p1);
}

TEST(BehaviorTreeXmlPlanCoverageTest, MathNodePlacedAfterWaypoint3BeforeFirstPickup)
{
  const std::string xml = readTextFile(DOG_BEHAVIOR_PHASE3_BT_XML_PATH);
  ASSERT_FALSE(xml.empty());

  const auto waypoint3 = xml.find("goal=\"{WayPointGoal3}\"");
  const auto math_node = xml.find("<PublishMathAnswerAction", waypoint3 + 1);
  const auto first_pickup = xml.find("behavior_name=\"PickUpBoxes\"", math_node + 1);

  ASSERT_NE(waypoint3, std::string::npos);
  ASSERT_NE(math_node, std::string::npos);
  ASSERT_NE(first_pickup, std::string::npos);
  EXPECT_LT(waypoint3, math_node);
  EXPECT_LT(math_node, first_pickup);
}

}  // namespace
