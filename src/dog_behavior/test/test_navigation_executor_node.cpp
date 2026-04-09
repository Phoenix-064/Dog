#include "dog_behavior/navigation_executor_node.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

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

class MockNav2Server : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  enum class ResponseMode
  {
    kSucceed,
    kNoFeedbackUntilCancel,
  };

  explicit MockNav2Server(
    const std::string & action_name,
    const ResponseMode response_mode = ResponseMode::kSucceed)
  : Node("mock_nav2_server")
  , response_mode_(response_mode)
  {
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name,
      std::bind(&MockNav2Server::handleGoal, this, _1, _2),
      std::bind(&MockNav2Server::handleCancel, this, _1),
      std::bind(&MockNav2Server::handleAccepted, this, _1));
  }

  int goalCount() const
  {
    return goal_count_.load();
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal>)
  {
    goal_count_.fetch_add(1);
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
              auto result = std::make_shared<NavigateToPose::Result>();
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
      [goal_handle]() {
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        feedback->distance_remaining = 0.1;
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        auto result = std::make_shared<NavigateToPose::Result>();
        goal_handle->succeed(result);
      })
      .detach();
  }

  std::atomic<int> goal_count_{0};
  ResponseMode response_mode_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
};

class NavigationExecutorNodeTest : public ::testing::Test
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

TEST_F(NavigationExecutorNodeTest, ForwardsGoalToNav2AndReturnsSucceeded)
{
  const std::string internal_action = "/test/behavior/navigate_execute";
  const std::string nav2_action = "/test/nav2/navigate_to_pose";

  auto mock_nav2_server = std::make_shared<MockNav2Server>(nav2_action);

  rclcpp::NodeOptions options;
  options.append_parameter_override("navigate_execute_action_name", internal_action);
  options.append_parameter_override("nav2_action_name", nav2_action);
  options.append_parameter_override("navigate_feedback_timeout_sec", 2.0);
  options.append_parameter_override("nav2_server_wait_timeout_sec", 2.0);

  auto executor_node = std::make_shared<dog_behavior::NavigationExecutorNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("nav_executor_test_client");

  auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node,
    internal_action);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mock_nav2_server);
  executor.add_node(executor_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&client]() {
      return client->wait_for_action_server(std::chrono::seconds(0));
    }));
  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&executor_node]() {
      return executor_node->getExecutionState() == "idle";
    }));

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = 1.0;
  goal.pose.pose.orientation.w = 1.0;

  auto goal_future = client->async_send_goal(goal);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(goal_future, std::chrono::milliseconds(1500)));

  auto goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(result_future, std::chrono::milliseconds(2000)));

  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_GE(mock_nav2_server->goalCount(), 1);

  executor.remove_node(client_node);
  executor.remove_node(executor_node);
  executor.remove_node(mock_nav2_server);
}

TEST_F(NavigationExecutorNodeTest, RejectsInvalidPoseGoal)
{
  const std::string internal_action = "/test/behavior/navigate_execute_reject";
  const std::string nav2_action = "/test/nav2/navigate_to_pose_reject";

  auto mock_nav2_server = std::make_shared<MockNav2Server>(nav2_action);

  rclcpp::NodeOptions options;
  options.append_parameter_override("navigate_execute_action_name", internal_action);
  options.append_parameter_override("nav2_action_name", nav2_action);
  options.append_parameter_override("navigate_feedback_timeout_sec", 2.0);
  options.append_parameter_override("nav2_server_wait_timeout_sec", 2.0);

  auto executor_node = std::make_shared<dog_behavior::NavigationExecutorNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("nav_executor_test_client_reject");

  auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node,
    internal_action);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mock_nav2_server);
  executor.add_node(executor_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&client]() {
      return client->wait_for_action_server(std::chrono::seconds(0));
    }));
  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&executor_node]() {
      return executor_node->getExecutionState() == "idle";
    }));

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  goal.pose.pose.orientation.w = 1.0;

  auto goal_future = client->async_send_goal(goal);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(goal_future, std::chrono::milliseconds(1500)));

  auto goal_handle = goal_future.get();
  EXPECT_FALSE(goal_handle);

  executor.remove_node(client_node);
  executor.remove_node(executor_node);
  executor.remove_node(mock_nav2_server);
}

TEST_F(NavigationExecutorNodeTest, CancelsGoalWhenSystemModeSwitchesToIdleSpinning)
{
  const std::string internal_action = "/test/behavior/navigate_execute_idle";
  const std::string nav2_action = "/test/nav2/navigate_to_pose_idle";
  const std::string system_mode_topic = "/test/lifecycle/system_mode_idle";

  auto mock_nav2_server = std::make_shared<MockNav2Server>(
    nav2_action,
    MockNav2Server::ResponseMode::kNoFeedbackUntilCancel);

  rclcpp::NodeOptions options;
  options.append_parameter_override("navigate_execute_action_name", internal_action);
  options.append_parameter_override("nav2_action_name", nav2_action);
  options.append_parameter_override("system_mode_topic", system_mode_topic);
  options.append_parameter_override("navigate_feedback_timeout_sec", 5.0);
  options.append_parameter_override("nav2_server_wait_timeout_sec", 2.0);

  auto executor_node = std::make_shared<dog_behavior::NavigationExecutorNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("nav_executor_test_client_idle");

  auto system_mode_pub = client_node->create_publisher<std_msgs::msg::String>(
    system_mode_topic,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal));

  auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node,
    internal_action);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mock_nav2_server);
  executor.add_node(executor_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&client]() {
      return client->wait_for_action_server(std::chrono::seconds(0));
    }));
  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&executor_node]() {
      return executor_node->getExecutionState() == "idle";
    }));

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = 1.0;
  goal.pose.pose.orientation.w = 1.0;

  auto goal_future = client->async_send_goal(goal);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(goal_future, std::chrono::milliseconds(1500)));

  auto goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&executor_node]() {
      return executor_node->getExecutionState() == "running";
    }));

  std_msgs::msg::String mode_msg;
  mode_msg.data = "mode=idle_spinning";

  const auto publish_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < publish_deadline) {
    system_mode_pub->publish(mode_msg);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(result_future, std::chrono::milliseconds(2000)));

  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1000),
    [&executor_node]() {
      return executor_node->getExecutionState() == "idle";
    }));

  executor.remove_node(client_node);
  executor.remove_node(executor_node);
  executor.remove_node(mock_nav2_server);
}

TEST_F(NavigationExecutorNodeTest, MarksTimeoutWhenNav2FeedbackIsMissing)
{
  const std::string internal_action = "/test/behavior/navigate_execute_timeout";
  const std::string nav2_action = "/test/nav2/navigate_to_pose_timeout";

  auto mock_nav2_server = std::make_shared<MockNav2Server>(
    nav2_action,
    MockNav2Server::ResponseMode::kNoFeedbackUntilCancel);

  rclcpp::NodeOptions options;
  options.append_parameter_override("navigate_execute_action_name", internal_action);
  options.append_parameter_override("nav2_action_name", nav2_action);
  options.append_parameter_override("navigate_feedback_timeout_sec", 0.2);
  options.append_parameter_override("nav2_server_wait_timeout_sec", 2.0);

  auto executor_node = std::make_shared<dog_behavior::NavigationExecutorNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("nav_executor_test_client_timeout");

  auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node,
    internal_action);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mock_nav2_server);
  executor.add_node(executor_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&client]() {
      return client->wait_for_action_server(std::chrono::seconds(0));
    }));
  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1500),
    [&executor_node]() {
      return executor_node->getExecutionState() == "idle";
    }));

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = 1.0;
  goal.pose.pose.orientation.w = 1.0;

  auto goal_future = client->async_send_goal(goal);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(goal_future, std::chrono::milliseconds(1500)));

  auto goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(result_future, std::chrono::milliseconds(2000)));

  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1000),
    [&executor_node]() {
      return executor_node->getExecutionState() == "timeout";
    }));

  executor.remove_node(client_node);
  executor.remove_node(executor_node);
  executor.remove_node(mock_nav2_server);
}

}  // namespace
