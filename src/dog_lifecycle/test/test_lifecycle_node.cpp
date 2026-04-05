#include "dog_lifecycle/lifecycle_node.hpp"

#include <dog_interfaces/msg/target3_d.hpp>
#include <dog_interfaces/msg/target3_d_array.hpp>

#include <chrono>
#include <filesystem>
#include <functional>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

std::string extractPayloadValue(const std::string & payload, const std::string & key)
{
  const auto token = key + "=";
  const auto begin = payload.find(token);
  if (begin == std::string::npos) {
    return "";
  }
  const auto value_begin = begin + token.size();
  const auto value_end = payload.find(';', value_begin);
  if (value_end == std::string::npos) {
    return payload.substr(value_begin);
  }
  return payload.substr(value_begin, value_end - value_begin);
}

dog_interfaces::msg::Target3DArray makeValidFrame(const rclcpp::Node::SharedPtr & node)
{
  dog_interfaces::msg::Target3DArray frame;
  frame.header.stamp = node->now();
  frame.header.frame_id = "base_link";

  dog_interfaces::msg::Target3D target;
  target.header = frame.header;
  target.target_id = "target_1";
  target.position.x = 0.1;
  target.position.y = 0.2;
  target.position.z = 0.3;
  target.confidence = 0.9F;
  frame.targets.push_back(target);
  return frame;
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

TEST_F(LifecycleNodeTest, EstopSwitchesIdleSpinningModeAndRecoversToNormal)
{
  const std::string estop_topic = "/test/lifecycle/estop";
  const std::string system_mode_topic = "/test/lifecycle/system_mode";

  rclcpp::NodeOptions options;
  options.append_parameter_override("estop_topic", estop_topic);
  options.append_parameter_override("system_mode_topic", system_mode_topic);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_estop_io");
  auto estop_pub = io_node->create_publisher<std_msgs::msg::String>(
    estop_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  std::string latest_mode_payload;
  auto mode_sub = io_node->create_subscription<std_msgs::msg::String>(
    system_mode_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&latest_mode_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      latest_mode_payload = msg->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return io_node->count_subscribers(estop_topic) > 0u;
  }));

  std_msgs::msg::String estop_on;
  estop_on.data = "active=true";
  estop_pub->publish(estop_on);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(800), [&]() {
    return lifecycle_node->IsIdleSpinningForTest();
  }));
  EXPECT_NE(lifecycle_node->GetLastSystemModePayloadForTest().find("mode=idle_spinning"), std::string::npos);

  std_msgs::msg::String estop_off;
  estop_off.data = "active=false";
  estop_pub->publish(estop_off);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(800), [&]() {
    return !lifecycle_node->IsIdleSpinningForTest();
  }));
  EXPECT_NE(lifecycle_node->GetLastSystemModePayloadForTest().find("mode=normal"), std::string::npos);
  EXPECT_FALSE(latest_mode_payload.empty());

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)mode_sub;
}

TEST_F(LifecycleNodeTest, HeartbeatTimeoutTriggersLifecycleReconnect)
{
  const std::string valid_frame_topic = "/test/lifecycle/valid_frame_timeout";
  const std::string transition_topic = "/test/lifecycle/transition_timeout";

  rclcpp::NodeOptions options;
  options.append_parameter_override("valid_frame_topic", valid_frame_topic);
  options.append_parameter_override("lifecycle_transition_topic", transition_topic);
  options.append_parameter_override("heartbeat_timeout_ms", 60);
  options.append_parameter_override("heartbeat_check_period_ms", 10);
  options.append_parameter_override("reconnect_min_interval_ms", 500);
  options.append_parameter_override("max_restart_attempts", 3);
  options.append_parameter_override("restart_window_ms", 500);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_heartbeat_timeout_io");
  auto valid_frame_pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    valid_frame_topic,
    rclcpp::SensorDataQoS());

  std::vector<std::string> transitions;
  auto transition_sub = io_node->create_subscription<std_msgs::msg::String>(
    transition_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&transitions](const std_msgs::msg::String::ConstSharedPtr msg) {
      transitions.push_back(msg->data);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(600), [&]() {
    return io_node->count_subscribers(valid_frame_topic) > 0u;
  }));

  auto valid_frame = makeValidFrame(io_node);
  valid_frame_pub->publish(valid_frame);
  spinFor(executor, std::chrono::milliseconds(30));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&]() {
    return transitions.size() >= 2U;
  }));

  EXPECT_TRUE(lifecycle_node->IsReconnectPendingForTest());
  EXPECT_NE(transitions[0].find("from=active"), std::string::npos);
  EXPECT_NE(transitions[0].find("to=inactive"), std::string::npos);
  EXPECT_NE(transitions[1].find("from=inactive"), std::string::npos);
  EXPECT_NE(transitions[1].find("to=active"), std::string::npos);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)transition_sub;
}

TEST_F(LifecycleNodeTest, ValidFrameLatchMarksReconnectRecoveredWithinTwoSeconds)
{
  const std::string valid_frame_topic = "/test/lifecycle/valid_frame_recover";
  const std::string transition_topic = "/test/lifecycle/transition_recover";
  const std::string transition_status_topic = "/test/lifecycle/transition_status_recover";

  rclcpp::NodeOptions options;
  options.append_parameter_override("valid_frame_topic", valid_frame_topic);
  options.append_parameter_override("lifecycle_transition_topic", transition_topic);
  options.append_parameter_override("lifecycle_transition_status_topic", transition_status_topic);
  options.append_parameter_override("heartbeat_timeout_ms", 60);
  options.append_parameter_override("heartbeat_check_period_ms", 10);
  options.append_parameter_override("reconnect_min_interval_ms", 500);
  options.append_parameter_override("max_restart_attempts", 3);
  options.append_parameter_override("restart_window_ms", 500);
  options.append_parameter_override("valid_frame_recovery_consecutive", 1);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_heartbeat_recover_io");
  auto valid_frame_pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    valid_frame_topic,
    rclcpp::SensorDataQoS());
  auto transition_status_pub = io_node->create_publisher<std_msgs::msg::String>(
    transition_status_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  std::vector<std::string> transitions;

  auto transition_sub = io_node->create_subscription<std_msgs::msg::String>(
    transition_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&transitions](const std_msgs::msg::String::ConstSharedPtr msg) {
      transitions.push_back(msg->data);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(600), [&]() {
    return io_node->count_subscribers(valid_frame_topic) > 0u;
  }));
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(600), [&]() {
    return io_node->count_subscribers(transition_status_topic) > 0u;
  }));

  auto valid_frame = makeValidFrame(io_node);
  valid_frame_pub->publish(valid_frame);
  spinFor(executor, std::chrono::milliseconds(20));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&]() {
    return lifecycle_node->IsReconnectPendingForTest();
  }));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(600), [&]() {
    return transitions.size() >= 2U;
  }));

  const auto emitted_attempt = extractPayloadValue(transitions.back(), "attempt");
  ASSERT_FALSE(emitted_attempt.empty());

  auto valid_frame_recovery_1 = makeValidFrame(io_node);
  valid_frame_pub->publish(valid_frame_recovery_1);
  spinFor(executor, std::chrono::milliseconds(20));

  EXPECT_TRUE(lifecycle_node->IsReconnectPendingForTest());
  EXPECT_FALSE(lifecycle_node->IsReconnectTransitionCompletedForTest());

  std_msgs::msg::String inactive_ack;
  inactive_ack.data =
    "node=dog_perception;from=active;to=inactive;status=ok;attempt=" + emitted_attempt;
  transition_status_pub->publish(inactive_ack);
  spinFor(executor, std::chrono::milliseconds(20));

  std_msgs::msg::String active_ack;
  active_ack.data =
    "node=dog_perception;from=inactive;to=active;status=ok;attempt=" + emitted_attempt;
  transition_status_pub->publish(active_ack);
  spinFor(executor, std::chrono::milliseconds(20));

  EXPECT_TRUE(lifecycle_node->IsReconnectTransitionCompletedForTest());

  auto valid_frame_recovery_2 = makeValidFrame(io_node);
  valid_frame_pub->publish(valid_frame_recovery_2);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&]() {
    return !lifecycle_node->IsReconnectPendingForTest() &&
           lifecycle_node->GetLastReconnectRecoveryLatencyMsForTest() >= 0;
  }));

  EXPECT_LT(lifecycle_node->GetLastReconnectRecoveryLatencyMsForTest(), 2000);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)transition_sub;
}

TEST_F(LifecycleNodeTest, RestartLimitTriggersControlledDegradeAndAlarm)
{
  const std::string valid_frame_topic = "/test/lifecycle/valid_frame_degrade";
  const std::string transition_topic = "/test/lifecycle/transition_degrade";
  const std::string alarm_topic = "/test/lifecycle/alarm_degrade";
  const std::string system_mode_topic = "/test/lifecycle/system_mode_degrade";

  rclcpp::NodeOptions options;
  options.append_parameter_override("valid_frame_topic", valid_frame_topic);
  options.append_parameter_override("lifecycle_transition_topic", transition_topic);
  options.append_parameter_override("health_alarm_topic", alarm_topic);
  options.append_parameter_override("system_mode_topic", system_mode_topic);
  options.append_parameter_override("heartbeat_timeout_ms", 40);
  options.append_parameter_override("heartbeat_check_period_ms", 10);
  options.append_parameter_override("reconnect_min_interval_ms", 20);
  options.append_parameter_override("max_restart_attempts", 1);
  options.append_parameter_override("restart_window_ms", 500);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_heartbeat_degrade_io");
  auto valid_frame_pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    valid_frame_topic,
    rclcpp::SensorDataQoS());

  std::string alarm_payload;
  auto alarm_sub = io_node->create_subscription<std_msgs::msg::String>(
    alarm_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&alarm_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      alarm_payload = msg->data;
    });

  std::string mode_payload;
  auto mode_sub = io_node->create_subscription<std_msgs::msg::String>(
    system_mode_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&mode_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      mode_payload = msg->data;
    });

  auto transition_sub = io_node->create_subscription<std_msgs::msg::String>(
    transition_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [](const std_msgs::msg::String::ConstSharedPtr) {});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(600), [&]() {
    return io_node->count_subscribers(valid_frame_topic) > 0u;
  }));

  auto valid_frame = makeValidFrame(io_node);
  valid_frame_pub->publish(valid_frame);
  spinFor(executor, std::chrono::milliseconds(20));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1200), [&]() {
    return lifecycle_node->IsControlledDegradeModeForTest() && !alarm_payload.empty();
  }));

  EXPECT_NE(alarm_payload.find("type=heartbeat_restart_limit"), std::string::npos);
  EXPECT_NE(alarm_payload.find("last_reconnect_success_unix_ms="), std::string::npos);
  EXPECT_NE(mode_payload.find("mode=degraded"), std::string::npos);
  EXPECT_FALSE(lifecycle_node->IsReconnectPendingForTest());

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)alarm_sub;
  (void)mode_sub;
  (void)transition_sub;
}

TEST_F(LifecycleNodeTest, ReconnectPendingDoesNotAccumulateRestartAttempts)
{
  const std::string valid_frame_topic = "/test/lifecycle/valid_frame_pending_gate";
  const std::string transition_topic = "/test/lifecycle/transition_pending_gate";

  rclcpp::NodeOptions options;
  options.append_parameter_override("valid_frame_topic", valid_frame_topic);
  options.append_parameter_override("lifecycle_transition_topic", transition_topic);
  options.append_parameter_override("heartbeat_timeout_ms", 40);
  options.append_parameter_override("heartbeat_check_period_ms", 10);
  options.append_parameter_override("reconnect_min_interval_ms", 20);
  options.append_parameter_override("max_restart_attempts", 3);
  options.append_parameter_override("restart_window_ms", 500);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_heartbeat_pending_gate_io");
  auto valid_frame_pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    valid_frame_topic,
    rclcpp::SensorDataQoS());

  std::vector<std::string> transitions;
  auto transition_sub = io_node->create_subscription<std_msgs::msg::String>(
    transition_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&transitions](const std_msgs::msg::String::ConstSharedPtr msg) {
      transitions.push_back(msg->data);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(600), [&]() {
    return io_node->count_subscribers(valid_frame_topic) > 0u;
  }));

  auto valid_frame = makeValidFrame(io_node);
  valid_frame_pub->publish(valid_frame);
  spinFor(executor, std::chrono::milliseconds(20));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(800), [&]() {
    return lifecycle_node->IsReconnectPendingForTest();
  }));

  spinFor(executor, std::chrono::milliseconds(200));

  EXPECT_EQ(lifecycle_node->GetRestartAttemptsInWindowForTest(), 1U);
  EXPECT_EQ(transitions.size(), 2U);
  EXPECT_TRUE(lifecycle_node->IsReconnectPendingForTest());

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)transition_sub;
}

TEST_F(LifecycleNodeTest, AdaptivePendingTimeoutRespectsSmallRestartWindowAndCanDegrade)
{
  const std::string valid_frame_topic = "/test/lifecycle/valid_frame_small_window";
  const std::string transition_topic = "/test/lifecycle/transition_small_window";
  const std::string alarm_topic = "/test/lifecycle/alarm_small_window";

  rclcpp::NodeOptions options;
  options.append_parameter_override("valid_frame_topic", valid_frame_topic);
  options.append_parameter_override("lifecycle_transition_topic", transition_topic);
  options.append_parameter_override("health_alarm_topic", alarm_topic);
  options.append_parameter_override("heartbeat_timeout_ms", 40);
  options.append_parameter_override("heartbeat_check_period_ms", 10);
  options.append_parameter_override("reconnect_min_interval_ms", 20);
  options.append_parameter_override("max_restart_attempts", 1);
  options.append_parameter_override("restart_window_ms", 120);
  options.append_parameter_override("reconnect_pending_timeout_ms", 0);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_small_window_io");
  auto valid_frame_pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    valid_frame_topic,
    rclcpp::SensorDataQoS());

  std::string alarm_payload;
  auto alarm_sub = io_node->create_subscription<std_msgs::msg::String>(
    alarm_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&alarm_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      alarm_payload = msg->data;
    });

  auto transition_sub = io_node->create_subscription<std_msgs::msg::String>(
    transition_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [](const std_msgs::msg::String::ConstSharedPtr) {});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(600), [&]() {
    return io_node->count_subscribers(valid_frame_topic) > 0u;
  }));

  auto valid_frame = makeValidFrame(io_node);
  valid_frame_pub->publish(valid_frame);
  spinFor(executor, std::chrono::milliseconds(20));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1800), [&]() {
    return lifecycle_node->IsControlledDegradeModeForTest() && !alarm_payload.empty();
  }));

  EXPECT_NE(alarm_payload.find("type=heartbeat_restart_limit"), std::string::npos);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)alarm_sub;
  (void)transition_sub;
}

TEST_F(LifecycleNodeTest, EstopStillWorksDuringControlledDegrade)
{
  const std::string valid_frame_topic = "/test/lifecycle/valid_frame_degrade_estop";
  const std::string transition_topic = "/test/lifecycle/transition_degrade_estop";
  const std::string alarm_topic = "/test/lifecycle/alarm_degrade_estop";
  const std::string system_mode_topic = "/test/lifecycle/system_mode_degrade_estop";
  const std::string estop_topic = "/test/lifecycle/estop_degrade_estop";

  rclcpp::NodeOptions options;
  options.append_parameter_override("valid_frame_topic", valid_frame_topic);
  options.append_parameter_override("lifecycle_transition_topic", transition_topic);
  options.append_parameter_override("health_alarm_topic", alarm_topic);
  options.append_parameter_override("system_mode_topic", system_mode_topic);
  options.append_parameter_override("estop_topic", estop_topic);
  options.append_parameter_override("heartbeat_timeout_ms", 40);
  options.append_parameter_override("heartbeat_check_period_ms", 10);
  options.append_parameter_override("reconnect_min_interval_ms", 20);
  options.append_parameter_override("max_restart_attempts", 1);
  options.append_parameter_override("restart_window_ms", 500);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_heartbeat_degrade_estop_io");
  auto valid_frame_pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    valid_frame_topic,
    rclcpp::SensorDataQoS());
  auto estop_pub = io_node->create_publisher<std_msgs::msg::String>(
    estop_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  std::string alarm_payload;
  auto alarm_sub = io_node->create_subscription<std_msgs::msg::String>(
    alarm_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&alarm_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      alarm_payload = msg->data;
    });

  std::string mode_payload;
  auto mode_sub = io_node->create_subscription<std_msgs::msg::String>(
    system_mode_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&mode_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      mode_payload = msg->data;
    });

  auto transition_sub = io_node->create_subscription<std_msgs::msg::String>(
    transition_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [](const std_msgs::msg::String::ConstSharedPtr) {});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(600), [&]() {
    return io_node->count_subscribers(valid_frame_topic) > 0u &&
           io_node->count_subscribers(estop_topic) > 0u;
  }));

  auto valid_frame = makeValidFrame(io_node);
  valid_frame_pub->publish(valid_frame);
  spinFor(executor, std::chrono::milliseconds(20));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1200), [&]() {
    return lifecycle_node->IsControlledDegradeModeForTest() && !alarm_payload.empty();
  }));

  std_msgs::msg::String estop_on;
  estop_on.data = "active=true";
  estop_pub->publish(estop_on);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(800), [&]() {
    return lifecycle_node->IsIdleSpinningForTest();
  }));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(400), [&]() {
    return mode_payload.find("mode=idle_spinning") != std::string::npos;
  }));

  EXPECT_NE(mode_payload.find("mode=idle_spinning"), std::string::npos);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)alarm_sub;
  (void)mode_sub;
  (void)transition_sub;
}

TEST_F(LifecycleNodeTest, MaxRestartAttemptsBoundaryAllowsAttemptAtEquality)
{
  const std::string valid_frame_topic = "/test/lifecycle/valid_frame_boundary";
  const std::string transition_topic = "/test/lifecycle/transition_boundary";

  rclcpp::NodeOptions options;
  options.append_parameter_override("valid_frame_topic", valid_frame_topic);
  options.append_parameter_override("lifecycle_transition_topic", transition_topic);
  options.append_parameter_override("heartbeat_timeout_ms", 40);
  options.append_parameter_override("heartbeat_check_period_ms", 10);
  options.append_parameter_override("reconnect_min_interval_ms", 200);
  options.append_parameter_override("max_restart_attempts", 1);
  options.append_parameter_override("restart_window_ms", 1000);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_heartbeat_boundary_io");
  auto valid_frame_pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    valid_frame_topic,
    rclcpp::SensorDataQoS());

  std::vector<std::string> transitions;
  auto transition_sub = io_node->create_subscription<std_msgs::msg::String>(
    transition_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&transitions](const std_msgs::msg::String::ConstSharedPtr msg) {
      transitions.push_back(msg->data);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(600), [&]() {
    return io_node->count_subscribers(valid_frame_topic) > 0u;
  }));

  auto valid_frame = makeValidFrame(io_node);
  valid_frame_pub->publish(valid_frame);
  spinFor(executor, std::chrono::milliseconds(20));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&]() {
    return transitions.size() >= 2U;
  }));

  EXPECT_EQ(lifecycle_node->GetRestartAttemptsInWindowForTest(), 1U);
  EXPECT_FALSE(lifecycle_node->IsControlledDegradeModeForTest());

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)transition_sub;
}

TEST_F(LifecycleNodeTest, StartupPublishesRecoveredContextFromValidState)
{
  namespace fs = std::filesystem;

  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto test_dir = fs::temp_directory_path() / ("dog_lifecycle_recover_valid_" + std::to_string(now_ns));
  const auto state_file = test_dir / "state.yaml";
  const auto backup_file = test_dir / "state.bak.yaml";
  fs::create_directories(test_dir);

  {
    std::ofstream ofs(state_file);
    ofs << "task_phase: deliver_box\n";
    ofs << "timestamp_ms: 123456\n";
    ofs << "target_state: done\n";
    ofs << "version: 1\n";
  }

  const std::string recovery_topic = "/test/lifecycle/recovery_context_valid";

  rclcpp::NodeOptions options;
  options.append_parameter_override("persistence.state_file_path", state_file.string());
  options.append_parameter_override("persistence.backup_file_path", backup_file.string());
  options.append_parameter_override("recovery_context_topic", recovery_topic);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_recovery_valid_io");

  std::string last_recovery_payload;
  auto recovery_sub = io_node->create_subscription<std_msgs::msg::String>(
    recovery_topic,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    [&last_recovery_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      last_recovery_payload = msg->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(800), [&]() {
    return !last_recovery_payload.empty();
  }));

  EXPECT_NE(last_recovery_payload.find("mode=recovered"), std::string::npos);
  EXPECT_NE(last_recovery_payload.find("task_phase=deliver_box"), std::string::npos);
  EXPECT_NE(last_recovery_payload.find("target_state=done"), std::string::npos);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)recovery_sub;
  fs::remove_all(test_dir);
}

TEST_F(LifecycleNodeTest, StartupFallsBackToBackupWhenPrimaryIsCorrupted)
{
  namespace fs = std::filesystem;

  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto test_dir = fs::temp_directory_path() / ("dog_lifecycle_recover_backup_" + std::to_string(now_ns));
  const auto state_file = test_dir / "state.yaml";
  const auto backup_file = test_dir / "state.bak.yaml";
  fs::create_directories(test_dir);

  {
    std::ofstream ofs(state_file);
    ofs << "task_phase: [invalid\n";
  }
  {
    std::ofstream ofs(backup_file);
    ofs << "task_phase: phase_from_backup\n";
    ofs << "timestamp_ms: 223344\n";
    ofs << "target_state: done\n";
    ofs << "version: 1\n";
  }

  const std::string recovery_topic = "/test/lifecycle/recovery_context_backup";

  rclcpp::NodeOptions options;
  options.append_parameter_override("persistence.state_file_path", state_file.string());
  options.append_parameter_override("persistence.backup_file_path", backup_file.string());
  options.append_parameter_override("recovery_context_topic", recovery_topic);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_recovery_backup_io");

  std::string last_recovery_payload;
  auto recovery_sub = io_node->create_subscription<std_msgs::msg::String>(
    recovery_topic,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    [&last_recovery_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      last_recovery_payload = msg->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(800), [&]() {
    return !last_recovery_payload.empty();
  }));

  EXPECT_NE(last_recovery_payload.find("mode=recovered"), std::string::npos);
  EXPECT_NE(last_recovery_payload.find("task_phase=phase_from_backup"), std::string::npos);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)recovery_sub;
  fs::remove_all(test_dir);
}

TEST_F(LifecycleNodeTest, StartupPublishesColdStartWhenStateMissing)
{
  namespace fs = std::filesystem;

  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto test_dir = fs::temp_directory_path() / ("dog_lifecycle_recover_cold_" + std::to_string(now_ns));
  const auto state_file = test_dir / "state.yaml";
  const auto backup_file = test_dir / "state.bak.yaml";
  fs::create_directories(test_dir);

  const std::string recovery_topic = "/test/lifecycle/recovery_context_cold";

  rclcpp::NodeOptions options;
  options.append_parameter_override("persistence.state_file_path", state_file.string());
  options.append_parameter_override("persistence.backup_file_path", backup_file.string());
  options.append_parameter_override("recovery_context_topic", recovery_topic);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_recovery_cold_io");

  std::string last_recovery_payload;
  auto recovery_sub = io_node->create_subscription<std_msgs::msg::String>(
    recovery_topic,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    [&last_recovery_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      last_recovery_payload = msg->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(800), [&]() {
    return !last_recovery_payload.empty();
  }));

  EXPECT_NE(last_recovery_payload.find("mode=cold_start"), std::string::npos);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)recovery_sub;
  fs::remove_all(test_dir);
}

TEST_F(LifecycleNodeTest, StartupPublishesColdStartWhenPrimaryAndBackupBothCorrupted)
{
  namespace fs = std::filesystem;

  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto test_dir = fs::temp_directory_path() / ("dog_lifecycle_recover_both_corrupted_" + std::to_string(now_ns));
  const auto state_file = test_dir / "state.yaml";
  const auto backup_file = test_dir / "state.bak.yaml";
  fs::create_directories(test_dir);

  {
    std::ofstream ofs(state_file);
    ofs << "task_phase: [invalid\n";
  }
  {
    std::ofstream ofs(backup_file);
    ofs << "target_state: [invalid\n";
  }

  const std::string recovery_topic = "/test/lifecycle/recovery_context_both_corrupted";

  rclcpp::NodeOptions options;
  options.append_parameter_override("persistence.state_file_path", state_file.string());
  options.append_parameter_override("persistence.backup_file_path", backup_file.string());
  options.append_parameter_override("recovery_context_topic", recovery_topic);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_recovery_both_corrupted_io");

  std::string last_recovery_payload;
  auto recovery_sub = io_node->create_subscription<std_msgs::msg::String>(
    recovery_topic,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    [&last_recovery_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      last_recovery_payload = msg->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(800), [&]() {
    return !last_recovery_payload.empty();
  }));

  EXPECT_NE(last_recovery_payload.find("mode=cold_start"), std::string::npos);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)recovery_sub;
  fs::remove_all(test_dir);
}

TEST_F(LifecycleNodeTest, StartupRecoveryLatencyIsWithinTwoSeconds)
{
  namespace fs = std::filesystem;

  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto test_dir = fs::temp_directory_path() / ("dog_lifecycle_recover_latency_" + std::to_string(now_ns));
  const auto state_file = test_dir / "state.yaml";
  const auto backup_file = test_dir / "state.bak.yaml";
  fs::create_directories(test_dir);

  {
    std::ofstream ofs(state_file);
    ofs << "task_phase: latency_task\n";
    ofs << "timestamp_ms: 998877\n";
    ofs << "target_state: done\n";
    ofs << "version: 1\n";
  }

  const std::string recovery_topic = "/test/lifecycle/recovery_context_latency";

  rclcpp::NodeOptions options;
  options.append_parameter_override("persistence.state_file_path", state_file.string());
  options.append_parameter_override("persistence.backup_file_path", backup_file.string());
  options.append_parameter_override("recovery_context_topic", recovery_topic);

  auto lifecycle_node = std::make_shared<dog_lifecycle::LifecycleNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("lifecycle_recovery_latency_io");

  std::string last_recovery_payload;
  auto recovery_sub = io_node->create_subscription<std_msgs::msg::String>(
    recovery_topic,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    [&last_recovery_payload](const std_msgs::msg::String::ConstSharedPtr msg) {
      last_recovery_payload = msg->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node);
  executor.add_node(io_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(800), [&]() {
    return !last_recovery_payload.empty();
  }));

  const auto total_ms_value = extractPayloadValue(last_recovery_payload, "total_ms");
  ASSERT_FALSE(total_ms_value.empty());
  EXPECT_LT(std::stoll(total_ms_value), 2000);

  executor.remove_node(io_node);
  executor.remove_node(lifecycle_node);
  (void)recovery_sub;
  fs::remove_all(test_dir);
}