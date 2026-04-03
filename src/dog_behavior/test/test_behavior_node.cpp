#include "dog_behavior/behavior_node.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <dog_interfaces/action/execute_behavior.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
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

class MockExecuteBehaviorServer : public rclcpp::Node
{
public:
  using ExecuteBehavior = dog_interfaces::action::ExecuteBehavior;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteBehavior>;

  enum class ResponseMode
  {
    kSucceed,
    kAbort,
    kNoFeedbackUntilCancel,
  };

  explicit MockExecuteBehaviorServer(
    const std::string & action_name,
    bool reject_goal,
    ResponseMode response_mode = ResponseMode::kSucceed)
  : Node("mock_execute_behavior_server")
  , reject_goal_(reject_goal)
  , response_mode_(response_mode)
  {
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<ExecuteBehavior>(
      this,
      action_name,
      std::bind(&MockExecuteBehaviorServer::handleGoal, this, _1, _2),
      std::bind(&MockExecuteBehaviorServer::handleCancel, this, _1),
      std::bind(&MockExecuteBehaviorServer::handleAccepted, this, _1));
  }

  int goalCount() const
  {
    return goal_count_.load();
  }

  bool feedbackPublished() const
  {
    return feedback_published_.load();
  }

  std::string lastBehaviorName() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_behavior_name_;
  }

  geometry_msgs::msg::PoseStamped lastTargetPose() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_target_pose_;
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteBehavior::Goal> goal)
  {
    goal_count_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_behavior_name_ = goal->behavior_name;
      last_target_pose_ = goal->target_pose;
    }
    if (reject_goal_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (response_mode_ == ResponseMode::kNoFeedbackUntilCancel) {
      std::thread(
        [goal_handle]() {
          while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
              auto result = std::make_shared<ExecuteBehavior::Result>();
              result->accepted = false;
              result->detail = "cancelled_due_to_timeout";
              goal_handle->canceled(result);
              return;
            }
            if (!goal_handle->is_active()) {
              return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
        })
        .detach();
      return;
    }

    std::thread(
      [this, goal_handle]() {
        auto feedback = std::make_shared<ExecuteBehavior::Feedback>();
        feedback->progress = 0.6F;
        feedback->state = "running";
        goal_handle->publish_feedback(feedback);
        feedback_published_.store(true);

        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        auto result = std::make_shared<ExecuteBehavior::Result>();
        if (response_mode_ == ResponseMode::kAbort) {
          result->accepted = false;
          result->detail = "aborted";
          goal_handle->abort(result);
          return;
        }

        result->accepted = true;
        result->detail = "ok";
        goal_handle->succeed(result);
      })
      .detach();
  }

  bool reject_goal_;
  ResponseMode response_mode_;
  std::atomic<int> goal_count_{0};
  std::atomic<bool> feedback_published_{false};
  mutable std::mutex data_mutex_;
  std::string last_behavior_name_;
  geometry_msgs::msg::PoseStamped last_target_pose_;
  rclcpp_action::Server<ExecuteBehavior>::SharedPtr action_server_;
};

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

TEST_F(BehaviorNodeTest, SendsActionGoalAndProcessesFeedbackAndResult)
{
  const std::string odom_topic = "/test/localization/action_success";
  const std::string pose_topic = "/test/global_pose/action_success";
  const std::string action_name = "/test/behavior/execute_success";
  const std::string trigger_topic = "/test/behavior/trigger_success";

  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", pose_topic);
  options.append_parameter_override("localization_topic", odom_topic);
  options.append_parameter_override("default_frame_id", "map");
  options.append_parameter_override("execute_behavior_action_name", action_name);
  options.append_parameter_override("execute_behavior_trigger_topic", trigger_topic);
  options.append_parameter_override("action_server_wait_timeout_sec", 2.0);
  options.append_parameter_override("feedback_timeout_sec", 1.0);

  auto behavior_node = std::make_shared<dog_behavior::BehaviorNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("behavior_test_io_action_success");
  auto action_server = std::make_shared<MockExecuteBehaviorServer>(
    action_name,
    false,
    MockExecuteBehaviorServer::ResponseMode::kSucceed);

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS().keep_last(20));
  auto trigger_pub = io_node->create_publisher<std_msgs::msg::String>(
    trigger_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(behavior_node);
  executor.add_node(io_node);
  executor.add_node(action_server);

  ASSERT_TRUE(waitForTopicMatch(executor, io_node, odom_topic, pose_topic, std::chrono::milliseconds(800)));
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1200), [&behavior_node]() {
    return behavior_node->getExecutionState() == "idle";
  }));

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = behavior_node->now();
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = 2.0;
  odom.pose.pose.position.y = -1.0;
  odom.pose.pose.orientation.w = 1.0;
  odom_pub->publish(odom);
  spinFor(executor, std::chrono::milliseconds(100));

  std_msgs::msg::String trigger;
  trigger.data = "pick";
  trigger_pub->publish(trigger);

  ASSERT_TRUE(waitUntil(executor, std::chrono::seconds(2), [&behavior_node]() {
    return behavior_node->getExecutionState() == "succeeded";
  }));

  EXPECT_EQ(action_server->goalCount(), 1);
  EXPECT_TRUE(action_server->feedbackPublished());
  EXPECT_EQ(action_server->lastBehaviorName(), "pick");
  EXPECT_EQ(action_server->lastTargetPose().header.frame_id, "map");

  executor.remove_node(action_server);
  executor.remove_node(io_node);
  executor.remove_node(behavior_node);
}

TEST_F(BehaviorNodeTest, HandlesRejectedActionGoal)
{
  const std::string odom_topic = "/test/localization/action_reject";
  const std::string pose_topic = "/test/global_pose/action_reject";
  const std::string action_name = "/test/behavior/execute_reject";
  const std::string trigger_topic = "/test/behavior/trigger_reject";

  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", pose_topic);
  options.append_parameter_override("localization_topic", odom_topic);
  options.append_parameter_override("default_frame_id", "map");
  options.append_parameter_override("execute_behavior_action_name", action_name);
  options.append_parameter_override("execute_behavior_trigger_topic", trigger_topic);
  options.append_parameter_override("action_server_wait_timeout_sec", 2.0);

  auto behavior_node = std::make_shared<dog_behavior::BehaviorNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("behavior_test_io_action_reject");
  auto action_server = std::make_shared<MockExecuteBehaviorServer>(
    action_name,
    true,
    MockExecuteBehaviorServer::ResponseMode::kSucceed);

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS().keep_last(20));
  auto trigger_pub = io_node->create_publisher<std_msgs::msg::String>(
    trigger_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(behavior_node);
  executor.add_node(io_node);
  executor.add_node(action_server);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1200), [&behavior_node]() {
    return behavior_node->getExecutionState() == "idle";
  }));

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = behavior_node->now();
  odom.header.frame_id = "map";
  odom.pose.pose.orientation.w = 1.0;
  odom_pub->publish(odom);
  spinFor(executor, std::chrono::milliseconds(100));

  std_msgs::msg::String trigger;
  trigger.data = "place";
  trigger_pub->publish(trigger);

  ASSERT_TRUE(waitUntil(executor, std::chrono::seconds(2), [&behavior_node]() {
    return behavior_node->getExecutionState() == "rejected";
  }));

  EXPECT_EQ(action_server->goalCount(), 1);

  executor.remove_node(action_server);
  executor.remove_node(io_node);
  executor.remove_node(behavior_node);
}

TEST_F(BehaviorNodeTest, HandlesActionServerUnavailableWithoutCrash)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", "/test/global_pose/no_server");
  options.append_parameter_override("localization_topic", "/test/localization/no_server");
  options.append_parameter_override("execute_behavior_action_name", "/test/behavior/execute_no_server");
  options.append_parameter_override("action_server_wait_timeout_sec", 0.2);

  auto behavior_node = std::make_shared<dog_behavior::BehaviorNode>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(behavior_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::seconds(2), [&behavior_node]() {
    return behavior_node->getExecutionState() == "server_unavailable";
  }));

  EXPECT_FALSE(behavior_node->triggerExecuteBehavior("walk"));
  EXPECT_EQ(behavior_node->getExecutionState(), "server_unavailable");

  executor.remove_node(behavior_node);
}

TEST_F(BehaviorNodeTest, KeepsTimeoutStateWhenWatchdogCancelsGoal)
{
  const std::string odom_topic = "/test/localization/action_timeout";
  const std::string pose_topic = "/test/global_pose/action_timeout";
  const std::string action_name = "/test/behavior/execute_timeout";
  const std::string trigger_topic = "/test/behavior/trigger_timeout";

  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", pose_topic);
  options.append_parameter_override("localization_topic", odom_topic);
  options.append_parameter_override("default_frame_id", "map");
  options.append_parameter_override("execute_behavior_action_name", action_name);
  options.append_parameter_override("execute_behavior_trigger_topic", trigger_topic);
  options.append_parameter_override("action_server_wait_timeout_sec", 2.0);
  options.append_parameter_override("feedback_timeout_sec", 0.05);

  auto behavior_node = std::make_shared<dog_behavior::BehaviorNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("behavior_test_io_action_timeout");
  auto action_server = std::make_shared<MockExecuteBehaviorServer>(
    action_name,
    false,
    MockExecuteBehaviorServer::ResponseMode::kNoFeedbackUntilCancel);

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS().keep_last(20));
  auto trigger_pub = io_node->create_publisher<std_msgs::msg::String>(
    trigger_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(behavior_node);
  executor.add_node(io_node);
  executor.add_node(action_server);

  ASSERT_TRUE(waitForTopicMatch(executor, io_node, odom_topic, pose_topic, std::chrono::milliseconds(800)));
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1200), [&behavior_node]() {
    return behavior_node->getExecutionState() == "idle";
  }));

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = behavior_node->now();
  odom.header.frame_id = "map";
  odom.pose.pose.orientation.w = 1.0;
  odom_pub->publish(odom);
  spinFor(executor, std::chrono::milliseconds(100));

  std_msgs::msg::String trigger;
  trigger.data = "watch";
  trigger_pub->publish(trigger);

  ASSERT_TRUE(waitUntil(executor, std::chrono::seconds(2), [&behavior_node]() {
    return behavior_node->getExecutionState() == "timeout";
  }));

  EXPECT_EQ(action_server->goalCount(), 1);

  executor.remove_node(action_server);
  executor.remove_node(io_node);
  executor.remove_node(behavior_node);
}

TEST_F(BehaviorNodeTest, HandlesAbortedResultAsFailed)
{
  const std::string odom_topic = "/test/localization/action_abort";
  const std::string pose_topic = "/test/global_pose/action_abort";
  const std::string action_name = "/test/behavior/execute_abort";
  const std::string trigger_topic = "/test/behavior/trigger_abort";

  rclcpp::NodeOptions options;
  options.append_parameter_override("global_pose_topic", pose_topic);
  options.append_parameter_override("localization_topic", odom_topic);
  options.append_parameter_override("default_frame_id", "map");
  options.append_parameter_override("execute_behavior_action_name", action_name);
  options.append_parameter_override("execute_behavior_trigger_topic", trigger_topic);
  options.append_parameter_override("action_server_wait_timeout_sec", 2.0);
  options.append_parameter_override("feedback_timeout_sec", 1.0);

  auto behavior_node = std::make_shared<dog_behavior::BehaviorNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("behavior_test_io_action_abort");
  auto action_server = std::make_shared<MockExecuteBehaviorServer>(
    action_name,
    false,
    MockExecuteBehaviorServer::ResponseMode::kAbort);

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS().keep_last(20));
  auto trigger_pub = io_node->create_publisher<std_msgs::msg::String>(
    trigger_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(behavior_node);
  executor.add_node(io_node);
  executor.add_node(action_server);

  ASSERT_TRUE(waitForTopicMatch(executor, io_node, odom_topic, pose_topic, std::chrono::milliseconds(800)));
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(1200), [&behavior_node]() {
    return behavior_node->getExecutionState() == "idle";
  }));

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = behavior_node->now();
  odom.header.frame_id = "map";
  odom.pose.pose.orientation.w = 1.0;
  odom_pub->publish(odom);
  spinFor(executor, std::chrono::milliseconds(100));

  std_msgs::msg::String trigger;
  trigger.data = "abort_case";
  trigger_pub->publish(trigger);

  ASSERT_TRUE(waitUntil(executor, std::chrono::seconds(2), [&behavior_node]() {
    return behavior_node->getExecutionState() == "failed";
  }));

  EXPECT_EQ(action_server->goalCount(), 1);

  executor.remove_node(action_server);
  executor.remove_node(io_node);
  executor.remove_node(behavior_node);
}