#include "dog_serial_bridge/serial_bridge_node.hpp"

#include <dog_interfaces/action/execute_behavior.hpp>
#include <dog_interfaces/action/place_boxes.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
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

using ExecuteBehavior = dog_interfaces::action::ExecuteBehavior;
using PlaceBoxes = dog_interfaces::action::PlaceBoxes;

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

class FakeSerialConnection : public dog_serial_bridge::SerialConnection
{
public:
  explicit FakeSerialConnection(bool open_success = true)
  : open_success_(open_success)
  , open_(false)
  , fail_next_write_(false)
  {
  }

  bool open(const dog_serial_bridge::SerialConfig & config, std::string & error) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    if (!open_success_) {
      error = "fake_open_failed";
      open_ = false;
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
    if (fail_next_write_) {
      fail_next_write_ = false;
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
    if (!cv_.wait_for(lock, timeout, [&]() { return !open_ || !incoming_lines_.empty(); })) {
      return ReadResult{ReadStatus::kTimeout, "", "", ""};
    }
    if (!open_) {
      return ReadResult{ReadStatus::kClosed, "", "fake_closed", ""};
    }

    auto line = incoming_lines_.front();
    incoming_lines_.pop_front();
    return ReadResult{ReadStatus::kLine, line, "", ""};
  }

  void enqueueIncomingLine(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    incoming_lines_.push_back(line);
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
    fail_next_write_ = true;
  }

private:
  bool open_success_;
  bool open_;
  bool fail_next_write_;
  dog_serial_bridge::SerialConfig config_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::string> writes_;
  std::deque<std::string> incoming_lines_;
};

template<typename ActionT>
struct ActionCallState
{
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  typename GoalHandle::SharedPtr goal_handle;
  typename GoalHandle::WrappedResult wrapped_result;
  bool result_ready{false};
  std::vector<std::string> feedback_states;
};

class SerialBridgeNodeTest : public ::testing::Test
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
    options.append_parameter_override("serial_port", "/tmp/fake");
    options.append_parameter_override("ack_timeout_ms", 120);
    options.append_parameter_override("feedback_period_ms", 40);
    options.append_parameter_override("write_newline", true);
    options.append_parameter_override("read_line_delimiter", "\\n");
    return options;
  }

  template<typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr sendGoal(
    rclcpp::executors::SingleThreadedExecutor & executor,
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal,
    ActionCallState<ActionT> & state)
  {
    typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
    options.goal_response_callback =
      [&state](typename ActionCallState<ActionT>::GoalHandle::SharedPtr goal_handle) {
        state.goal_handle = goal_handle;
      };
    options.feedback_callback =
      [&state](
      typename ActionCallState<ActionT>::GoalHandle::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        if (feedback) {
          state.feedback_states.push_back(feedback->state);
        }
      };
    options.result_callback =
      [&state](const typename ActionCallState<ActionT>::GoalHandle::WrappedResult & result) {
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

TEST_F(SerialBridgeNodeTest, ExecuteBehaviorSucceedsOnPickSuccess)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_execute_success_client");
  auto client = rclcpp_action::create_client<ExecuteBehavior>(client_node, "/behavior/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  ExecuteBehavior::Goal goal;
  goal.behavior_name = "PickUpBoxes";

  ActionCallState<ExecuteBehavior> state;
  auto goal_handle = sendGoal<ExecuteBehavior>(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return state.result_ready;
  }));

  EXPECT_EQ(state.wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "command_sent");
  EXPECT_EQ(fake_serial->writes().front(), "RCPickUpBoxes\r\n");

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

TEST_F(SerialBridgeNodeTest, ExecuteBehaviorIgnoresLegacyPickFailReply)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_execute_fail_client");
  auto client = rclcpp_action::create_client<ExecuteBehavior>(client_node, "/behavior/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  ExecuteBehavior::Goal goal;
  goal.behavior_name = "PickUpBoxes";

  ActionCallState<ExecuteBehavior> state;
  auto goal_handle = sendGoal<ExecuteBehavior>(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return state.result_ready;
  }));

  EXPECT_EQ(state.wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "command_sent");

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

TEST_F(SerialBridgeNodeTest, ExecuteBehaviorRejectsUnsupportedBehavior)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_execute_reject_client");
  auto client = rclcpp_action::create_client<ExecuteBehavior>(client_node, "/behavior/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  ExecuteBehavior::Goal goal;
  goal.behavior_name = "UnknownBehavior";

  ActionCallState<ExecuteBehavior> state;
  auto goal_handle = sendGoal<ExecuteBehavior>(executor, client, goal, state);
  EXPECT_EQ(goal_handle, nullptr);
  EXPECT_TRUE(fake_serial->writes().empty());

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

TEST_F(SerialBridgeNodeTest, ExecuteBehaviorAcceptsSequentialGoalsWithoutAck)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_execute_busy_client");
  auto client = rclcpp_action::create_client<ExecuteBehavior>(client_node, "/behavior/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  ExecuteBehavior::Goal goal;
  goal.behavior_name = "PickUpBoxes";

  ActionCallState<ExecuteBehavior> first_state;
  auto first_goal_handle = sendGoal<ExecuteBehavior>(executor, client, goal, first_state);
  ASSERT_NE(first_goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  ActionCallState<ExecuteBehavior> second_state;
  auto second_goal_handle = sendGoal<ExecuteBehavior>(executor, client, goal, second_state);
  ASSERT_NE(second_goal_handle, nullptr);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return first_state.result_ready && second_state.result_ready;
  }));

  EXPECT_EQ(fake_serial->writes().size(), 2U);

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

TEST_F(SerialBridgeNodeTest, ExecuteBehaviorSucceedsWithoutAck)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_execute_timeout_client");
  auto client = rclcpp_action::create_client<ExecuteBehavior>(client_node, "/behavior/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  ExecuteBehavior::Goal goal;
  goal.behavior_name = "PickUpBoxes";

  ActionCallState<ExecuteBehavior> state;
  auto goal_handle = sendGoal<ExecuteBehavior>(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return state.result_ready;
  }));

  EXPECT_EQ(state.wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "command_sent");

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

TEST_F(SerialBridgeNodeTest, ExecuteBehaviorAbortsWhenSerialIsNotReady)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(false);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_execute_not_ready_client");
  auto client = rclcpp_action::create_client<ExecuteBehavior>(client_node, "/behavior/execute");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  ExecuteBehavior::Goal goal;
  goal.behavior_name = "PickUpBoxes";

  ActionCallState<ExecuteBehavior> state;
  auto goal_handle = sendGoal<ExecuteBehavior>(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return state.result_ready;
  }));

  EXPECT_EQ(state.wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_EQ(state.wrapped_result.result->detail, "serial_not_ready");

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

TEST_F(SerialBridgeNodeTest, PlaceBoxesSucceedsAndIgnoresUnrelatedReplies)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_place_success_client");
  auto client = rclcpp_action::create_client<PlaceBoxes>(client_node, "/behavior/place_boxes");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  PlaceBoxes::Goal goal;
  goal.box_type = "food";
  goal.payload = "place=0,3,count=3";
  goal.step_counter = 1;

  ActionCallState<PlaceBoxes> state;
  auto goal_handle = sendGoal<PlaceBoxes>(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return state.result_ready;
  }));

  EXPECT_EQ(state.wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "command_sent");
  EXPECT_EQ(fake_serial->writes().front(), "RCplace=0,3,count=3\r\n");

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

TEST_F(SerialBridgeNodeTest, PlaceBoxesRejectsInvalidPayload)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_place_reject_client");
  auto client = rclcpp_action::create_client<PlaceBoxes>(client_node, "/behavior/place_boxes");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  PlaceBoxes::Goal goal;
  goal.payload = "place=0,3,count=abc";

  ActionCallState<PlaceBoxes> state;
  auto goal_handle = sendGoal<PlaceBoxes>(executor, client, goal, state);
  EXPECT_EQ(goal_handle, nullptr);
  EXPECT_TRUE(fake_serial->writes().empty());

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

TEST_F(SerialBridgeNodeTest, PlaceBoxesAcceptsSequentialGoalsWithoutAck)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_place_busy_client");
  auto client = rclcpp_action::create_client<PlaceBoxes>(client_node, "/behavior/place_boxes");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  PlaceBoxes::Goal goal;
  goal.payload = "place=0,3,count=3";

  ActionCallState<PlaceBoxes> first_state;
  auto first_goal_handle = sendGoal<PlaceBoxes>(executor, client, goal, first_state);
  ASSERT_NE(first_goal_handle, nullptr);
  ASSERT_TRUE(fake_serial->waitForWriteCount(1U, std::chrono::milliseconds(500)));

  ActionCallState<PlaceBoxes> second_state;
  auto second_goal_handle = sendGoal<PlaceBoxes>(executor, client, goal, second_state);
  ASSERT_NE(second_goal_handle, nullptr);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return first_state.result_ready && second_state.result_ready;
  }));

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

TEST_F(SerialBridgeNodeTest, PlaceBoxesSucceedsWithoutAck)
{
  auto fake_serial = std::make_shared<FakeSerialConnection>(true);
  auto bridge_node = std::make_shared<dog_serial_bridge::SerialBridgeNode>(makeOptions(), fake_serial);
  auto client_node = std::make_shared<rclcpp::Node>("serial_bridge_place_timeout_client");
  auto client = rclcpp_action::create_client<PlaceBoxes>(client_node, "/behavior/place_boxes");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge_node);
  executor.add_node(client_node);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return client->wait_for_action_server(std::chrono::seconds(0));
  }));

  PlaceBoxes::Goal goal;
  goal.payload = "place=0,3,count=3";

  ActionCallState<PlaceBoxes> state;
  auto goal_handle = sendGoal<PlaceBoxes>(executor, client, goal, state);
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&]() {
    return state.result_ready;
  }));

  EXPECT_EQ(state.wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(state.wrapped_result.result);
  EXPECT_TRUE(state.wrapped_result.result->accepted);
  EXPECT_EQ(state.wrapped_result.result->detail, "command_sent");

  executor.remove_node(client_node);
  executor.remove_node(bridge_node);
}

}  // namespace
