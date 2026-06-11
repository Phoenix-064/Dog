#include "dog_behavior/bt_nodes/publish_math_answer_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <dog_interfaces/msg/target3_d_array.hpp>
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
    "    <PublishMathAnswerAction waypoint_name=\"WayPointGoal3\" required_waypoint_name=\"WayPointGoal3\" answer=\"13\" topic_name=\"/test/math_answer\" allow_fallback_answer=\"true\" ocr_wait_timeout_ms=\"0\"/>"
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

TEST_F(PublishMathAnswerActionNodeTest, ComputesAnswerFromDigitResultExpression)
{
  auto bt_node = std::make_shared<rclcpp::Node>("bt_publish_math_node_ocr");
  auto io_node = std::make_shared<rclcpp::Node>("bt_publish_math_io_ocr");

  std::string received_answer;
  auto answer_sub = io_node->create_subscription<std_msgs::msg::String>(
    "/test/math_answer_ocr",
    rclcpp::QoS(10),
    [&received_answer](const std_msgs::msg::String::ConstSharedPtr msg) {
      received_answer = msg ? msg->data : "";
    });
  auto digit_pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    "/test/digit_result_ocr",
    rclcpp::SensorDataQoS());

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
    "    <PublishMathAnswerAction waypoint_name=\"WayPointGoal3\" required_waypoint_name=\"WayPointGoal3\" topic_name=\"/test/math_answer_ocr\" digit_result_topic=\"/test/digit_result_ocr\" ocr_wait_timeout_ms=\"50\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1000),
    [&digit_pub]() {
      return digit_pub->get_subscription_count() > 0U;
    }));

  dog_interfaces::msg::Target3DArray ocr_message;
  dog_interfaces::msg::Target3D target;
  target.target_id = "type=math_expr;expr=1+2*3;raw=1+2x3;reason=ok";
  target.confidence = 0.95F;
  ocr_message.targets.push_back(target);
  digit_pub->publish(ocr_message);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(100),
    []() {return true;}));

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1000),
    [&received_answer]() {
      return !received_answer.empty();
    }));

  EXPECT_EQ(received_answer, "7");

  (void)answer_sub;
  executor.remove_node(io_node);
  executor.remove_node(bt_node);
}

TEST_F(PublishMathAnswerActionNodeTest, RejectsInvalidOcrExpressionWithoutPublishing)
{
  auto bt_node = std::make_shared<rclcpp::Node>("bt_publish_math_node_invalid_ocr");
  auto io_node = std::make_shared<rclcpp::Node>("bt_publish_math_io_invalid_ocr");

  std::string received_answer;
  auto answer_sub = io_node->create_subscription<std_msgs::msg::String>(
    "/test/math_answer_invalid_ocr",
    rclcpp::QoS(10),
    [&received_answer](const std_msgs::msg::String::ConstSharedPtr msg) {
      received_answer = msg ? msg->data : "";
    });
  auto digit_pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    "/test/digit_result_invalid_ocr",
    rclcpp::SensorDataQoS());

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
    "    <PublishMathAnswerAction waypoint_name=\"WayPointGoal3\" required_waypoint_name=\"WayPointGoal3\" topic_name=\"/test/math_answer_invalid_ocr\" digit_result_topic=\"/test/digit_result_invalid_ocr\" ocr_wait_timeout_ms=\"50\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1000),
    [&digit_pub]() {
      return digit_pub->get_subscription_count() > 0U;
    }));

  dog_interfaces::msg::Target3DArray ocr_message;
  dog_interfaces::msg::Target3D target;
  target.target_id = "type=math_expr;expr=8/3;raw=8/3;reason=ok";
  target.confidence = 0.95F;
  ocr_message.targets.push_back(target);
  digit_pub->publish(ocr_message);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(100),
    []() {return true;}));

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(received_answer.empty());

  (void)answer_sub;
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
