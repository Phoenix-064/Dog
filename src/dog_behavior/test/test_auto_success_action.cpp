#include "dog_behavior/bt_nodes/auto_success_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace
{

class AutoSuccessActionNodeTest : public ::testing::Test
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

TEST_F(AutoSuccessActionNodeTest, ReturnsSuccessAndWritesResultCode)
{
  auto node = std::make_shared<rclcpp::Node>("bt_auto_success_test_node");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::AutoSuccessAction>("AutoSuccessAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(node));
  blackboard->set("result_code_text", std::string(""));

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <AutoSuccessAction label=\"pickup_test\" result_code_text=\"{result_code_text}\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto tree = factory.createTreeFromText(xml, blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<std::string>("result_code_text"), "auto_success");
}
