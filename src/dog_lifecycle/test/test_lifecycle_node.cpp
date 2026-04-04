#include "dog_lifecycle/lifecycle_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <std_msgs/msg/string.hpp>
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

}  // namespace

class LifecycleNodeTest : public ::testing::Test
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

TEST_F(LifecycleNodeTest, NodeName)
{
  auto node = std::make_shared<dog_lifecycle::LifecycleNode>();
  EXPECT_STREQ(node->get_name(), "dog_lifecycle");
}

TEST_F(LifecycleNodeTest, ConsecutiveEmptyReachedThresholdOpensBreakerAndBlocksTask)
{
  const std::string feedback_topic = "/test/lifecycle/grasp_feedback";
  const std::string degrade_topic = "/test/lifecycle/degrade_command";

  rclcpp::NodeOptions options;
  options.append_parameter_override("grasp_feedback_topic", feedback_topic);
  options.append_parameter_override("degrade_command_topic", degrade_topic);
  options.append_parameter_override("degrade_timeout_ms", 500);
  options.append_parameter_override("empty_grasp_threshold", 2);
  options.append_parameter_override("breaker_reset_window_ms", 1000);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_test_io");
  auto feedback_pub = io_node->create_publisher<std_msgs::msg::String>(feedback_topic, 10);

  std::string last_degrade_command;
  auto degrade_sub = io_node->create_subscription<std_msgs::msg::String>(
    degrade_topic,
    10,
    [&last_degrade_command](const std_msgs::msg::String::ConstSharedPtr msg) {
      last_degrade_command = msg->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return io_node->count_subscribers(feedback_topic) > 0u && io_node->count_publishers(degrade_topic) > 0u;
  }));

  std_msgs::msg::String msg;
  msg.data = "task_a|empty";
  feedback_pub->publish(msg);
  spinFor(executor, std::chrono::milliseconds(30));

  EXPECT_FALSE(lifecycle_node->IsBreakerOpen());
  EXPECT_EQ(lifecycle_node->GetConsecutiveEmptyCount(), 1U);

  feedback_pub->publish(msg);
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return lifecycle_node->IsBreakerOpen() && !last_degrade_command.empty();
  }));

  EXPECT_TRUE(lifecycle_node->IsTaskBlocked("task_a"));
  EXPECT_NE(last_degrade_command.find("task_id=task_a"), std::string::npos);

  feedback_pub->publish(msg);
  spinFor(executor, std::chrono::milliseconds(30));
  EXPECT_EQ(lifecycle_node->GetConsecutiveEmptyCount(), 2U);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)degrade_sub;
}

TEST_F(LifecycleNodeTest, BreakerTriggerToDegradeLatencyIsBelowOneSecond)
{
  const std::string feedback_topic = "/test/lifecycle/grasp_feedback_latency";

  rclcpp::NodeOptions options;
  options.append_parameter_override("grasp_feedback_topic", feedback_topic);
  options.append_parameter_override("degrade_command_topic", "/test/lifecycle/degrade_command_latency");
  options.append_parameter_override("degrade_timeout_ms", 800);
  options.append_parameter_override("empty_grasp_threshold", 2);
  options.append_parameter_override("breaker_reset_window_ms", 1000);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_latency_io");
  auto feedback_pub = io_node->create_publisher<std_msgs::msg::String>(feedback_topic, 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return io_node->count_subscribers(feedback_topic) > 0u;
  }));

  std_msgs::msg::String msg;
  msg.data = "task_latency|empty";
  feedback_pub->publish(msg);
  spinFor(executor, std::chrono::milliseconds(20));
  feedback_pub->publish(msg);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return lifecycle_node->GetLastBreakerToDegradeLatencyMs() >= 0;
  }));

  EXPECT_LT(lifecycle_node->GetLastBreakerToDegradeLatencyMs(), 1000);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
}

TEST_F(LifecycleNodeTest, ResetWindowClearsCounterAndAvoidsFalseTrigger)
{
  const std::string feedback_topic = "/test/lifecycle/grasp_feedback_reset";
  const std::string degrade_topic = "/test/lifecycle/degrade_command_reset";

  rclcpp::NodeOptions options;
  options.append_parameter_override("grasp_feedback_topic", feedback_topic);
  options.append_parameter_override("degrade_command_topic", degrade_topic);
  options.append_parameter_override("degrade_timeout_ms", 800);
  options.append_parameter_override("empty_grasp_threshold", 2);
  options.append_parameter_override("breaker_reset_window_ms", 80);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_reset_io");
  auto feedback_pub = io_node->create_publisher<std_msgs::msg::String>(feedback_topic, 10);

  std::string degrade_command;
  auto degrade_sub = io_node->create_subscription<std_msgs::msg::String>(
    degrade_topic,
    10,
    [&degrade_command](const std_msgs::msg::String::ConstSharedPtr msg) {
      degrade_command = msg->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return io_node->count_subscribers(feedback_topic) > 0u;
  }));

  std_msgs::msg::String msg;
  msg.data = "task_reset|empty";
  feedback_pub->publish(msg);
  spinFor(executor, std::chrono::milliseconds(20));
  EXPECT_EQ(lifecycle_node->GetConsecutiveEmptyCount(), 1U);

  spinFor(executor, std::chrono::milliseconds(120));
  feedback_pub->publish(msg);
  spinFor(executor, std::chrono::milliseconds(40));

  EXPECT_EQ(lifecycle_node->GetConsecutiveEmptyCount(), 1U);
  EXPECT_FALSE(lifecycle_node->IsBreakerOpen());
  EXPECT_TRUE(degrade_command.empty());

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)degrade_sub;
}