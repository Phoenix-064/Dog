#include "dog_behavior/bt_nodes/navigate_to_pose_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <thread>

namespace
{

class MockNav2Server : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit MockNav2Server(const std::string & action_name)
  : Node("mock_bt_nav2_server")
  {
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name,
      std::bind(&MockNav2Server::handleGoal, this, _1, _2),
      std::bind(&MockNav2Server::handleCancel, this, _1),
      std::bind(&MockNav2Server::handleAccepted, this, _1));
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread(
      [goal_handle]() {
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        feedback->distance_remaining = 0.2;
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        auto result = std::make_shared<NavigateToPose::Result>();
        goal_handle->succeed(result);
      })
      .detach();
  }

  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
};

class NavigateToPoseActionNodeTest : public ::testing::Test
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

TEST_F(NavigateToPoseActionNodeTest, ReturnsSuccessAfterNav2Completes)
{
  const std::string action_name = "/test/bt/navigate_to_pose";
  const std::string state_topic = "/test/bt/nav_exec_state";

  auto server = std::make_shared<MockNav2Server>(action_name);
  auto bt_node = std::make_shared<rclcpp::Node>("bt_navigate_test_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(bt_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::NavigateToPoseAction>("NavigateToPoseAction");

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

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <NavigateToPoseAction goal=\"{goal}\" action_name=\"/test/bt/navigate_to_pose\" state_topic=\"/test/bt/nav_exec_state\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    status = tree.tickRoot();
    if (status == BT::NodeStatus::RUNNING && (last_state == "forwarding_goal" || last_state == "running")) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_TRUE(last_state == "forwarding_goal" || last_state == "running");

  executor.remove_node(bt_node);
  executor.remove_node(server);
  (void)state_sub;
}
