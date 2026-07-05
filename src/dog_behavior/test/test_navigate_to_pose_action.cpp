#include "dog_behavior/bt_nodes/navigate_to_pose_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <dog_interfaces/action/navigate_waypoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
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

class MockNavigateWaypointServer : public rclcpp::Node
{
public:
  using NavigateWaypoint = dog_interfaces::action::NavigateWaypoint;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateWaypoint>;

  explicit MockNavigateWaypointServer(
    const std::string & action_name,
    const bool complete_goal = true,
    const bool publish_feedback = true)
  : Node("mock_bt_navigate_waypoint_server")
  , complete_goal_(complete_goal)
  , publish_feedback_(publish_feedback)
  , cancel_count_(0)
  {
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<NavigateWaypoint>(
      this,
      action_name,
      std::bind(&MockNavigateWaypointServer::handleGoal, this, _1, _2),
      std::bind(&MockNavigateWaypointServer::handleCancel, this, _1),
      std::bind(&MockNavigateWaypointServer::handleAccepted, this, _1));
  }

  int cancelCount() const
  {
    return cancel_count_.load();
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateWaypoint::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    ++cancel_count_;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (!complete_goal_) {
      std::lock_guard<std::mutex> lock(active_mutex_);
      active_goals_.push_back(goal_handle);
      return;
    }

    std::thread(
      [goal_handle, publish_feedback = publish_feedback_]() {
        if (publish_feedback) {
          auto feedback = std::make_shared<NavigateWaypoint::Feedback>();
          feedback->progress = 0.5F;
          feedback->state = "waiting_arrival";
          goal_handle->publish_feedback(feedback);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        auto result = std::make_shared<NavigateWaypoint::Result>();
        result->accepted = true;
        result->detail = "arrival_ok";
        goal_handle->succeed(result);
      })
      .detach();
  }

  bool complete_goal_;
  bool publish_feedback_;
  std::atomic<int> cancel_count_;
  std::mutex active_mutex_;
  std::vector<std::shared_ptr<GoalHandle>> active_goals_;
  rclcpp_action::Server<NavigateWaypoint>::SharedPtr server_;
};

class NavigateWaypointActionNodeTest : public ::testing::Test
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

}  // namespace

TEST_F(NavigateWaypointActionNodeTest, ReturnsSuccessAfterSerialNavigationCompletes)
{
  const std::string action_name = "/test/bt/nav_execute";
  const std::string state_topic = "/test/bt/nav_exec_state";
  const std::string goal_topic = "/test/bt/nav_goal";

  auto server = std::make_shared<MockNavigateWaypointServer>(action_name);
  auto bt_node = std::make_shared<rclcpp::Node>("bt_navigate_waypoint_test_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(bt_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::NavigateWaypointAction>("NavigateWaypointAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.orientation.w = 1.0;
  blackboard->set("goal", goal);

  std::string last_state;
  auto state_sub = bt_node->create_subscription<std_msgs::msg::String>(
    state_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&last_state](const std_msgs::msg::String::ConstSharedPtr msg) {
      last_state = msg->data;
    });
  bool goal_received = false;
  geometry_msgs::msg::PoseStamped published_goal;
  auto goal_sub = bt_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    [&goal_received, &published_goal](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      if (msg) {
        goal_received = true;
        published_goal = *msg;
      }
    });

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <NavigateWaypointAction goal=\"{goal}\" action_name=\"/test/bt/nav_execute\" state_topic=\"/test/bt/nav_exec_state\" goal_topic=\"/test/bt/nav_goal\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    status = tree.tickRoot();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  const auto state_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < state_deadline && last_state != "succeeded") {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(last_state, "succeeded");
  ASSERT_TRUE(goal_received);
  EXPECT_EQ(published_goal.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(published_goal.pose.orientation.w, 1.0);

  executor.remove_node(bt_node);
  executor.remove_node(server);
  (void)state_sub;
  (void)goal_sub;
}

TEST_F(NavigateWaypointActionNodeTest, KeepsRunningWithoutDefaultFeedbackTimeout)
{
  const std::string action_name = "/test/bt/nav_execute_no_default_timeout";

  auto server = std::make_shared<MockNavigateWaypointServer>(action_name, false, false);
  auto bt_node = std::make_shared<rclcpp::Node>("bt_navigate_waypoint_no_timeout_test_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(bt_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::NavigateWaypointAction>("NavigateWaypointAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.orientation.w = 1.0;
  blackboard->set("goal", goal);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <NavigateWaypointAction goal=\"{goal}\" action_name=\"/test/bt/nav_execute_no_default_timeout\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    status = tree.tickRoot();
    ASSERT_NE(status, BT::NodeStatus::FAILURE);
    ASSERT_NE(status, BT::NodeStatus::SUCCESS);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(server->cancelCount(), 0);

  tree.haltTree();
  executor.spin_some();
  executor.remove_node(bt_node);
  executor.remove_node(server);
}

TEST_F(NavigateWaypointActionNodeTest, CancelsWhenFeedbackTimeoutIsExplicitlyPositive)
{
  const std::string action_name = "/test/bt/nav_execute_explicit_timeout";

  auto server = std::make_shared<MockNavigateWaypointServer>(action_name, false, false);
  auto bt_node = std::make_shared<rclcpp::Node>("bt_navigate_waypoint_explicit_timeout_test_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(bt_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::NavigateWaypointAction>("NavigateWaypointAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.orientation.w = 1.0;
  blackboard->set("goal", goal);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <NavigateWaypointAction goal=\"{goal}\" action_name=\"/test/bt/nav_execute_explicit_timeout\" feedback_timeout_sec=\"0.05\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    status = tree.tickRoot();
    if (status == BT::NodeStatus::FAILURE) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(500), [&server]() {
    return server->cancelCount() > 0;
  }));
  EXPECT_GT(server->cancelCount(), 0);

  executor.remove_node(bt_node);
  executor.remove_node(server);
}
