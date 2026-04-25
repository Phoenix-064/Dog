#include "dog_behavior/bt_nodes/set_boxes_type_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <dog_interfaces/msg/target3_d.hpp>
#include <dog_interfaces/msg/target3_d_array.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
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
  return condition();
}

dog_interfaces::msg::Target3D makeTarget(const std::string & id, const double x, const double y)
{
  dog_interfaces::msg::Target3D target;
  target.target_id = id;
  target.position.x = x;
  target.position.y = y;
  target.position.z = 0.0;
  target.confidence = 1.0F;
  return target;
}

class SetBoxesTypeActionNodeTest : public ::testing::Test
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

TEST_F(SetBoxesTypeActionNodeTest, SortsBoxesAndCachesResult)
{
  auto bt_node = std::make_shared<rclcpp::Node>("bt_set_boxes_type_node");
  auto io_node = std::make_shared<rclcpp::Node>("bt_set_boxes_type_io");

  auto pub = io_node->create_publisher<dog_interfaces::msg::Target3DArray>(
    "/target/box_result",
    rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bt_node);
  executor.add_node(io_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::SetBoxesTypeAction>("SetBoxesTypeAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <SetBoxesTypeAction/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1000),
    [&io_node]() {
      return io_node->count_subscribers("/target/box_result") > 0u;
    }));

  dog_interfaces::msg::Target3DArray boxes;
  boxes.targets = {
    makeTarget("G", 4.0, 3.0),
    makeTarget("D", 3.0, 1.0),
    makeTarget("B", 1.0, 0.0),
    makeTarget("F", 1.0, 2.0),
    makeTarget("A", 2.0, 0.0),
    makeTarget("H", 3.0, 3.0),
    makeTarget("E", 2.0, 2.0),
    makeTarget("C", 4.0, 1.0),
  };
  pub->publish(boxes);

  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(1000),
    [&tree]() {
      return tree.tickRoot() == BT::NodeStatus::SUCCESS;
    }));

  EXPECT_EQ(
    blackboard->get<std::vector<std::string>>("boxes_type_list"),
    (std::vector<std::string>{"B", "A", "D", "C", "F", "E", "H", "G"}));
  EXPECT_TRUE(blackboard->get<bool>("boxes_ready"));
  EXPECT_GT(blackboard->get<int64_t>("boxes_capture_stamp"), 0);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  executor.remove_node(io_node);
  executor.remove_node(bt_node);
}
