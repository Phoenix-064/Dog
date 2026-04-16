#include "dog_behavior/bt_nodes/execute_behavior_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <dog_interfaces/action/execute_behavior.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <memory>
#include <thread>

namespace
{

class MockExecuteBehaviorServer : public rclcpp::Node
{
public:
  using ExecuteBehavior = dog_interfaces::action::ExecuteBehavior;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteBehavior>;

  explicit MockExecuteBehaviorServer(const std::string & action_name)
  : Node("mock_bt_execute_server")
  {
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<ExecuteBehavior>(
      this,
      action_name,
      std::bind(&MockExecuteBehaviorServer::handleGoal, this, _1, _2),
      std::bind(&MockExecuteBehaviorServer::handleCancel, this, _1),
      std::bind(&MockExecuteBehaviorServer::handleAccepted, this, _1));
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteBehavior::Goal>)
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
        auto feedback = std::make_shared<ExecuteBehavior::Feedback>();
        feedback->progress = 0.5F;
        feedback->state = "running";
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        auto result = std::make_shared<ExecuteBehavior::Result>();
        result->accepted = true;
        result->detail = "ok";
        goal_handle->succeed(result);
      })
      .detach();
  }

  rclcpp_action::Server<ExecuteBehavior>::SharedPtr server_;
};

class ExecuteBehaviorActionNodeTest : public ::testing::Test
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

TEST_F(ExecuteBehaviorActionNodeTest, ReturnsSuccessAfterActionCompletes)
{
  const std::string action_name = "/test/bt/execute_behavior";
  auto server = std::make_shared<MockExecuteBehaviorServer>(action_name);
  auto bt_node = std::make_shared<rclcpp::Node>("bt_execute_test_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(bt_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::ExecuteBehaviorAction>("ExecuteBehaviorAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));
  blackboard->set("behavior_name", std::string("grasp"));

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "map";
  target_pose.pose.orientation.w = 1.0;
  blackboard->set("target_pose", target_pose);

  const std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <ExecuteBehaviorAction behavior_name=\"{behavior_name}\" target_pose=\"{target_pose}\" action_name=\"/test/bt/execute_behavior\"/>"
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

  executor.remove_node(bt_node);
  executor.remove_node(server);
}
