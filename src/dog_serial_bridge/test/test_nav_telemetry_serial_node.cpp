#include "dog_serial_bridge/nav_telemetry_serial_node.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
  executor.spin_some();
  return condition();
}

class FakeSerialConnection : public dog_serial_bridge::SerialConnection
{
public:
  explicit FakeSerialConnection(bool open_success = true)
  : open_success_(open_success)
  , open_(false)
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

  bool write(const std::string & data, std::string & error) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_) {
      error = "fake_not_open";
      return false;
    }
    writes_.push_back(data);
    error.clear();
    cv_.notify_all();
    return true;
  }

  ReadResult readLine(std::chrono::milliseconds) override
  {
    return ReadResult{ReadStatus::kTimeout, "", ""};
  }

  std::vector<std::string> writes() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return writes_;
  }

private:
  bool open_success_;
  bool open_;
  dog_serial_bridge::SerialConfig config_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::string> writes_;
};

geometry_msgs::msg::PoseStamped makePose(const std::string & frame_id, const double x, const double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
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

  rclcpp::NodeOptions makeOptions() const
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("serial_port", "/tmp/fake_nav");
    options.append_parameter_override("publish_period_ms", 20);
    options.append_parameter_override("reconnect_period_ms", 50);
    options.append_parameter_override("write_newline", true);
    options.append_parameter_override("current_pose_topic", "/test/nav/current_pose");
    options.append_parameter_override("goal_pose_topic", "/test/nav/goal_pose");
    return options;
  }
};

}  // namespace

TEST_F(NavTelemetrySerialNodeTest, WritesCurrentAndGoalPoseFrame)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto telemetry_node =
    std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_telemetry_test_client");

  auto current_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/current_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));
  auto goal_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/goal_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(telemetry_node);
  executor.add_node(client_node);

  current_pub->publish(makePose("map", 1.23, 0.42));
  goal_pub->publish(makePose("map", 3.0, 2.0));

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(500),
    [&fake_serial]() { return !fake_serial->writes().empty(); }));

  const auto writes = fake_serial->writes();
  ASSERT_FALSE(writes.empty());
  const auto & frame = writes.back();
  EXPECT_NE(frame.find("RCNAV;seq="), std::string::npos);
  EXPECT_NE(frame.find(";cur_valid=1;"), std::string::npos);
  EXPECT_NE(frame.find(";cur_frame=map;"), std::string::npos);
  EXPECT_NE(frame.find(";cur_x=1.230;cur_y=0.420;"), std::string::npos);
  EXPECT_NE(frame.find(";goal_valid=1;"), std::string::npos);
  EXPECT_NE(frame.find(";goal_frame=map;"), std::string::npos);
  EXPECT_NE(frame.find(";goal_x=3.000;goal_y=2.000;"), std::string::npos);
  EXPECT_EQ(frame.find("state="), std::string::npos);
  EXPECT_EQ(frame.find("goal_active="), std::string::npos);
  EXPECT_EQ(frame.back(), '\n');

  executor.remove_node(client_node);
  executor.remove_node(telemetry_node);
}

TEST_F(NavTelemetrySerialNodeTest, WritesCurrentPoseWhenGoalIsMissing)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto telemetry_node =
    std::make_shared<dog_serial_bridge::NavTelemetrySerialNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("nav_telemetry_missing_goal_client");

  auto current_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test/nav/current_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(telemetry_node);
  executor.add_node(client_node);

  current_pub->publish(makePose("map", 1.0, 2.0));

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(500),
    [&fake_serial]() { return !fake_serial->writes().empty(); }));

  const auto writes = fake_serial->writes();
  ASSERT_FALSE(writes.empty());
  const auto & frame = writes.back();
  EXPECT_NE(frame.find(";cur_valid=1;"), std::string::npos);
  EXPECT_NE(frame.find(";goal_valid=0;"), std::string::npos);
  EXPECT_NE(frame.find(";goal_frame=unknown;"), std::string::npos);
  EXPECT_EQ(frame.find("state="), std::string::npos);

  executor.remove_node(client_node);
  executor.remove_node(telemetry_node);
}
