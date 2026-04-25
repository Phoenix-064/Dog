#include "dog_behavior/bt_nodes/publish_math_answer_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <functional>
#include <memory>
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

class PublishMathAnswerActionNodeTest : public ::testing::Test
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

TEST_F(PublishMathAnswerActionNodeTest, PublishesAnswerAtRequiredWaypoint)
{
  auto bt_node = std::make_shared<rclcpp::Node>("bt_publish_math_node");
  auto io_node = std::make_shared<rclcpp::Node>("bt_publish_math_io");

  std::string received_answer;
  auto sub = io_node->create_subscription<std_msgs::msg::String>(
    "/test/math_answer",
    rclcpp::QoS(10),
    [&received_answer](const std_msgs::msg::String::ConstSharedPtr msg) {
      received_answer = msg ? msg->data : "";
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bt_node);
  executor.add_node(io_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::PublishMathAnswerAction>("PublishMathAnswerAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <PublishMathAnswerAction waypoint_name=\"WayPointGoal3\" required_waypoint_name=\"WayPointGoal3\" answer=\"13\" topic_name=\"/test/math_answer\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1000),
    [&received_answer]() {
      return !received_answer.empty();
    }));

  EXPECT_EQ(received_answer, "13");

  (void)sub;
  executor.remove_node(io_node);
  executor.remove_node(bt_node);
}

TEST_F(PublishMathAnswerActionNodeTest, FailsWhenWaypointDoesNotMatch)
{
  auto bt_node = std::make_shared<rclcpp::Node>("bt_publish_math_node_mismatch");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::PublishMathAnswerAction>("PublishMathAnswerAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <PublishMathAnswerAction waypoint_name=\"WayPointGoal2\" required_waypoint_name=\"WayPointGoal3\" answer=\"13\" topic_name=\"/test/math_answer_mismatch\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}
