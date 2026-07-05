#include "dog_serial_bridge/nav_telemetry_serial_node.hpp"

#include <dog_interfaces/action/navigate_waypoint.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{

using NavigateWaypoint = dog_interfaces::action::NavigateWaypoint;

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
  executor.spin_some();
  return condition();
}

class FakeSerialConnection : public dog_serial_bridge::SerialConnection
{
public:
  explicit FakeSerialConnection(bool open_success = true)
  : open_success_(open_success)
  , open_(false)
  , fail_write_count_(0U)
  {
  }

  bool open(const dog_serial_bridge::SerialConfig & config, std::string & error) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    if (!open_success_) {
      open_ = false;
      error = "fake_open_failed";
      return false;
    }
    open_ = true;
    error.clear();
    cv_.notify_all();
    return true;
  }

  bool isOpen() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return open_;
  }

  void close() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = false;
    cv_.notify_all();
  }

  void clearReadBuffer() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    partial_data_.clear();
  }

  bool write(const std::string & data, std::string & error) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_) {
      error = "fake_not_open";
      return false;
    }
    if (fail_write_count_ > 0U) {
      --fail_write_count_;
      error = "fake_write_failed";
      return false;
    }
    writes_.push_back(data);
    error.clear();
    cv_.notify_all();
    return true;
  }

  ReadResult readLine(std::chrono::milliseconds timeout) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [&]() {
        return !open_ || !incoming_lines_.empty() || !partial_data_.empty();
      }))
    {
      return ReadResult{ReadStatus::kTimeout, "", "", ""};
    }
    if (!open_) {
      return ReadResult{ReadStatus::kClosed, "", "fake_closed", ""};
    }

    if (!partial_data_.empty()) {
      return ReadResult{ReadStatus::kTimeout, "", "", partial_data_};
    }

    auto line = incoming_lines_.front();
    incoming_lines_.pop_front();
    return ReadResult{ReadStatus::kLine, line, "", ""};
  }

  ByteReadResult readBytes(std::chrono::milliseconds timeout) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [&]() {
        return !open_ || !incoming_lines_.empty() || !partial_data_.empty();
      }))
    {
      return ByteReadResult{ReadStatus::kTimeout, "", ""};
    }
    if (!open_) {
      return ByteReadResult{ReadStatus::kClosed, "", "fake_closed"};
    }
    if (!partial_data_.empty()) {
      auto data = partial_data_;
      partial_data_.clear();
      return ByteReadResult{ReadStatus::kLine, data, ""};
    }

    auto data = incoming_lines_.front();
    incoming_lines_.pop_front();
    return ByteReadResult{ReadStatus::kLine, data, ""};
  }

  void enqueueIncomingLine(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    incoming_lines_.push_back(line);
    cv_.notify_all();
  }

  void enqueuePartialData(const std::string & data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    partial_data_ += data;
    cv_.notify_all();
  }

  bool waitForWriteCount(size_t expected_count, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&]() { return writes_.size() >= expected_count; });
  }

  std::vector<std::string> writes() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return writes_;
  }

  void failNextWrite()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_write_count_ = 1U;
  }

  void failNextWrites(size_t count)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_write_count_ = count;
  }

private:
  bool open_success_;
  bool open_;
  size_t fail_write_count_;
  dog_serial_bridge::SerialConfig config_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::string> writes_;
  std::deque<std::string> incoming_lines_;
  std::string partial_data_;
};

struct ActionCallState
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateWaypoint>;
  GoalHandle::SharedPtr goal_handle;
  GoalHandle::WrappedResult wrapped_result;
  bool result_ready{false};
  std::vector<std::string> feedback_states;
};

geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame_id,
  const double x,
  const double y,
  const double yaw_deg = 0.0)
{
  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
  const double yaw_rad = yaw_deg * kDegreesToRadians;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw_rad / 2.0);
  pose.pose.orientation.w = std::cos(yaw_rad / 2.0);
  return pose;
}

class NavTelemetrySerialNodeTest : public ::testing::Test
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

  rclcpp::NodeOptions makeOptions(
    const int ack_timeout_ms = 200,
    const bool send_shutdown_arrival = false,
    const int shutdown_arrival_repeat_count = 1,
    const bool continuous_send_enabled = false,
    const int continuous_send_period_ms = 100,
    const std::string & arrival_check_mode = "serial_reply") const
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("serial_port", "/tmp/fake_nav");
    options.append_parameter_override("ack_timeout_ms", ack_timeout_ms);
    options.append_parameter_override("reconnect_period_ms", 50);
    options.append_parameter_override("continuous_send_enabled", continuous_send_enabled);
    options.append_parameter_override("continuous_send_period_ms", continuous_send_period_ms);
    options.append_parameter_override("arrival_check_mode", arrival_check_mode);
    options.append_parameter_override("arrival_xy_tolerance_m", 0.15);
    options.append_parameter_override("arrival_yaw_tolerance_deg", 10.0);
    options.append_parameter_override("arrival_check_period_ms", 10);
    options.append_parameter_override("write_newline", true);
    options.append_parameter_override("send_shutdown_arrival_on_exit", send_shutdown_arrival);
    options.append_parameter_override("shutdown_arrival_repeat_count", shutdown_arrival_repeat_count);
    options.append_parameter_override("current_pose_topic", "/test/nav/current_pose");
    options.append_parameter_override("goal_pose_topic", "/test/nav/goal_pose");
    options.append_parameter_override("action_name", "/test/nav/execute");
    return options;
  }

  rclcpp_action::ClientGoalHandle<NavigateWaypoint>::SharedPtr sendGoal(
    rclcpp::executors::SingleThreadedExecutor & executor,
    const rclcpp_action::Client<NavigateWaypoint>::SharedPtr & client,
    const NavigateWaypoint::Goal & goal,
    ActionCallState & state)
  {
    rclcpp_action::Client<NavigateWaypoint>::SendGoalOptions options;
    options.goal_response_callback =
      [&state](ActionCallState::GoalHandle::SharedPtr goal_handle) {
        state.goal_handle = goal_handle;
      };
    options.feedback_callback =
      [&state](
      ActionCallState::GoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateWaypoint::Feedback> feedback) {
        if (feedback) {
          state.feedback_states.push_back(feedback->state);
        }
      };
    options.result_callback =
      [&state](const ActionCallState::GoalHandle::WrappedResult & result) {
        state.wrapped_result = result;
        state.result_ready = true;
      };

    auto future = client->async_send_goal(goal, options);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return future.get();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }
};

}  // namespace

TEST_F(NavTelemetrySerialNodeTest, NavigateGoalWritesFrameAndSucceedsOnArrival)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_success_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  auto current_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/current_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  geometry_msgs::msg::PoseStamped observed_goal;
  bool goal_observed = false;
  auto goal_sub = client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/test/nav/goal_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&goal_observed, &observed_goal](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      if (msg) {
        goal_observed = true;
        observed_goal = *msg;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  current_pub->publish(makePose("map", 1.23, 0.42, 45.0));
  executor.spin_some();

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0, 90.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  const auto writes = fake_serial->writes();
  ASSERT_FALSE(writes.empty());
  const auto & frame = writes.back();
  EXPECT_NE(frame.find("RCNAV;cur_valid=1;"), std::string::npos);
  EXPECT_NE(frame.find(";cur_x=123.000;cur_y=42.000;"), std::string::npos);
  EXPECT_NE(frame.find(";cur_yaw=-45.000;"), std::string::npos);
  EXPECT_NE(frame.find(";goal_valid=1;"), std::string::npos);
  EXPECT_NE(frame.find(";goal_x=300.000;goal_y=200.000;"), std::string::npos);
  EXPECT_NE(frame.find(";goal_yaw=-90.000"), std::string::npos);
  EXPECT_TRUE(frame.size() < 255U);
  EXPECT_EQ(frame.substr(frame.size() - 2U), "\r\n");

  fake_serial->enqueueIncomingLine("RCArrivalMX\n");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));

  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "arrival_ok");
  EXPECT_TRUE(goal_observed);
  EXPECT_EQ(observed_goal.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(observed_goal.pose.position.x, 3.0);

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, NavigateGoalSucceedsOnHostPoseArrivalWithoutSerialReply)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(
    makeOptions(500, false, 1, false, 100, "host_pose"), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_host_arrival_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");
  auto current_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/current_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  current_pub->publish(makePose("map", 2.0, 1.0, 90.0));
  for (int i = 0; i < 50; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 2.05, 1.05, 95.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));

  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_NE(state.wrapped_result.result->detail.find("host_arrival_ok"), std::string::npos);

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, NavigateGoalWaitsUntilHostYawWithinTolerance)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(
    makeOptions(80, false, 1, false, 100, "host_pose"), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_host_yaw_wait_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");
  auto current_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/current_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  current_pub->publish(makePose("map", 2.0, 1.0, 0.0));
  for (int i = 0; i < 50; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 2.0, 1.0, 30.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);

  EXPECT_FALSE(waitUntil(executor, std::chrono::milliseconds(250), [&state]() {
    return state.result_ready;
  }));

  current_pub->publish(makePose("map", 2.0, 1.0, 30.0));
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_NE(state.wrapped_result.result->detail.find("host_arrival_ok"), std::string::npos);

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, ContinuousTelemetrySendsLatestPoseAfterGoal)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(
    makeOptions(800, false, 1, true, 30), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_continuous_telemetry_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");
  auto current_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/current_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  current_pub->publish(makePose("map", 1.0, 2.0, 10.0));
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(200), [&]() {
    return fake_serial->writes().empty();
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 4.0, 90.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return fake_serial->writes().size() >= 3U;
  }));

  const auto initial_writes = fake_serial->writes();
  ASSERT_GE(initial_writes.size(), 3U);
  for (const auto & frame : initial_writes) {
    EXPECT_NE(frame.find("RCNAV;cur_valid=1;"), std::string::npos);
    EXPECT_NE(frame.find(";goal_valid=1;"), std::string::npos);
    EXPECT_NE(frame.find(";goal_x=300.000;goal_y=400.000;"), std::string::npos);
    EXPECT_NE(frame.find(";goal_yaw=-90.000"), std::string::npos);
  }

  current_pub->publish(makePose("map", 2.0, 2.5, 20.0));
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    const auto writes = fake_serial->writes();
    return std::any_of(writes.begin(), writes.end(), [](const std::string & frame) {
      return frame.find(";cur_x=200.000;cur_y=250.000;") != std::string::npos &&
             frame.find(";cur_yaw=-20.000;") != std::string::npos &&
             frame.find(";goal_x=300.000;goal_y=400.000;") != std::string::npos;
    });
  }));

  fake_serial->enqueueIncomingLine("RCArrivalMX\n");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, NavigateGoalSucceedsOnCarriageReturnArrival)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_cr_arrival_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  fake_serial->enqueueIncomingLine("RCArrivalMX\r");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));

  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "arrival_ok");

  ActionCallState second_state;
  auto second_goal_handle = sendGoal(executor, client, goal, second_state);
  ASSERT_NE(second_goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(2U, std::chrono::milliseconds(500)));
  EXPECT_FALSE(waitUntil(executor, std::chrono::milliseconds(250), [&second_state]() {
    return second_state.result_ready;
  }));
  fake_serial->enqueueIncomingLine("RCArrivalMX\r");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&second_state]() {
    return second_state.result_ready;
  }));
  ASSERT_TRUE(second_state.wrapped_result.result);
  EXPECT_TRUE(second_state.wrapped_result.result->accepted);
  EXPECT_EQ(second_state.wrapped_result.result->detail, "arrival_ok");

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, NavigateGoalSucceedsOnPartialArrivalWithoutDelimiter)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_partial_arrival_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  fake_serial->enqueuePartialData("RCArrivalMX");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));

  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "arrival_ok");

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, NavigateGoalSucceedsOnArrivalMixedWithDebugFrame)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_debug_arrival_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  std::string debug_frame(32, '\0');
  debug_frame.append("\x00\x00\x80\x7F", 4);
  fake_serial->enqueuePartialData(debug_frame + "RCArrivalMX" + debug_frame);
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));

  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "arrival_ok");

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}


TEST_F(NavTelemetrySerialNodeTest, SendShutdownArrivalFrameAfterGoal)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  const bool send_shutdown = true;
  const int shutdown_repeat_count = 3;
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(
    makeOptions(200, send_shutdown, shutdown_repeat_count), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("shutdown_arrival_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));
  fake_serial->enqueueIncomingLine("RCArrivalMX\n");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);

  auto current_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/current_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  geometry_msgs::msg::PoseStamped current_pose = makePose("camera_init", -0.26969, -0.08705, -30.0);
  current_pub->publish(current_pose);
  for (int i = 0; i < 50; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  const size_t writes_before_shutdown = fake_serial->writes().size();

  nav_node->SendShutdownArrivalFrame();
  executor.spin_some();

  const auto writes = fake_serial->writes();
  ASSERT_EQ(writes.size(), writes_before_shutdown + static_cast<size_t>(shutdown_repeat_count));
  for (size_t i = writes_before_shutdown; i < writes.size(); ++i) {
    const auto & shutdown_frame = writes.at(i);
    EXPECT_NE(shutdown_frame.find("RCNAV;cur_valid=1;"), std::string::npos);
    EXPECT_NE(shutdown_frame.find(";goal_x=-26.969;goal_y=-8.705;"), std::string::npos);
    EXPECT_NE(shutdown_frame.find(";cur_yaw=30.000;"), std::string::npos);
    EXPECT_NE(shutdown_frame.find(";goal_yaw=30.000"), std::string::npos);
    EXPECT_EQ(shutdown_frame.substr(shutdown_frame.size() - 2U), "\r\n");
  }

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, SendShutdownArrivalFrameNoCurrentPose)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  const bool send_shutdown = true;
  const int shutdown_repeat_count = 3;
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(
    makeOptions(200, send_shutdown, shutdown_repeat_count), fake_serial);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);

  nav_node->SendShutdownArrivalFrame();
  executor.spin_some();

  EXPECT_TRUE(fake_serial->writes().empty());

  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, SendShutdownArrivalFrameDisabledByDefault)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(
    makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("shutdown_disabled_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));
  fake_serial->enqueueIncomingLine("RCArrivalMX\n");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));

  const size_t writes_before = fake_serial->writes().size();

  nav_node->SendShutdownArrivalFrame();
  executor.spin_some();

  EXPECT_EQ(fake_serial->writes().size(), writes_before);

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, SendShutdownArrivalFrameIdempotent)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  const bool send_shutdown = true;
  const int shutdown_repeat_count = 3;
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(
    makeOptions(200, send_shutdown, shutdown_repeat_count), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("shutdown_idempotent_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));
  fake_serial->enqueueIncomingLine("RCArrivalMX\n");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));

  const size_t writes_before_shutdown = fake_serial->writes().size();

  auto current_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/current_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  geometry_msgs::msg::PoseStamped current_pose = makePose("camera_init", -0.26969, -0.08705);
  current_pub->publish(current_pose);
  for (int i = 0; i < 50; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  nav_node->SendShutdownArrivalFrame();
  nav_node->SendShutdownArrivalFrame();
  nav_node->SendShutdownArrivalFrame();
  executor.spin_some();

  EXPECT_EQ(
    fake_serial->writes().size(),
    writes_before_shutdown + static_cast<size_t>(shutdown_repeat_count));

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}


TEST_F(NavTelemetrySerialNodeTest, NavigateGoalWaitsPastAckTimeoutUntilArrival)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(80), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_waits_past_ack_timeout_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  EXPECT_FALSE(waitUntil(executor, std::chrono::milliseconds(250), [&state]() {
    return state.result_ready;
  }));

  fake_serial->enqueueIncomingLine("RCArrivalMX\n");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "arrival_ok");
  EXPECT_EQ(fake_serial->writes().size(), 1U);

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, NavigateGoalCancelsWhileWaitingForArrival)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(80), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_cancel_waiting_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));
  auto cancel_future = client->async_cancel_goal(goal_handle);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));
  executor.spin_until_future_complete(cancel_future, std::chrono::milliseconds(100));

  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_FALSE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "goal_canceled");
  EXPECT_EQ(fake_serial->writes().size(), 1U);

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, ShutdownStopSkipsActionResultWhileWaitingForArrival)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(
    makeOptions(80, true, 1), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_shutdown_waiting_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");
  auto current_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/current_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  current_pub->publish(makePose("camera_init", 0.25, 0.50, 15.0));
  for (int i = 0; i < 50; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  nav_node->SendShutdownArrivalFrame();
  ASSERT_TRUE(fake_serial->waitForWriteCount(2U, std::chrono::milliseconds(500)));

  EXPECT_FALSE(waitUntil(executor, std::chrono::milliseconds(250), [&state]() {
    return state.result_ready;
  }));

  const auto writes = fake_serial->writes();
  ASSERT_EQ(writes.size(), 2U);
  EXPECT_NE(writes.back().find(";cur_x=25.000;cur_y=50.000;"), std::string::npos);
  EXPECT_NE(writes.back().find(";goal_x=25.000;goal_y=50.000;"), std::string::npos);
  EXPECT_NE(writes.back().find(";goal_yaw=-15.000"), std::string::npos);

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, PartialNonArrivalDoesNotSucceed)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(80), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_partial_non_arrival_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  fake_serial->enqueuePartialData("debug:Arrival");
  EXPECT_FALSE(waitUntil(executor, std::chrono::milliseconds(250), [&state]() {
    return state.result_ready;
  }));

  fake_serial->enqueuePartialData("RCArrivalMX");
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "arrival_ok");

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}

TEST_F(NavTelemetrySerialNodeTest, NavigateGoalAbortsWhenSerialIsNotReady)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(false);
  auto nav_node = std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_action_not_ready_client");
  auto client = rclcpp_action::create_client<NavigateWaypoint>(client_node, "/test/nav/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nav_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  NavigateWaypoint::Goal goal;
  goal.target_pose = makePose("map", 3.0, 2.0);

  ActionCallState state;
  auto goal_handle = sendGoal(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1000), [&state]() {
    return state.result_ready;
  }));

  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_FALSE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "serial_not_ready");
  EXPECT_TRUE(fake_serial->writes().empty());

  executor.remove_node(client_node);
  executor.remove_node(nav_node);
}
