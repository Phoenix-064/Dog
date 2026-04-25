#include "dog_behavior/bt_nodes/execute_place_boxes_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <dog_interfaces/action/place_boxes.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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

class MockPlaceBoxesServer : public rclcpp::Node
{
public:
  using PlaceBoxes = dog_interfaces::action::PlaceBoxes;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlaceBoxes>;

  MockPlaceBoxesServer(const std::string & action_name, const bool result_accepted)
  : Node("mock_place_boxes_server")
  , result_accepted_(result_accepted)
  {
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<PlaceBoxes>(
      this,
      action_name,
      std::bind(&MockPlaceBoxesServer::handleGoal, this, _1, _2),
      std::bind(&MockPlaceBoxesServer::handleCancel, this, _1),
      std::bind(&MockPlaceBoxesServer::handleAccepted, this, _1));
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PlaceBoxes::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const bool accepted = result_accepted_;
    std::thread(
      [goal_handle, accepted]() {
        auto feedback = std::make_shared<PlaceBoxes::Feedback>();
        feedback->progress = 0.5F;
        feedback->state = "running";
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        auto result = std::make_shared<PlaceBoxes::Result>();
        result->accepted = accepted;
        result->detail = accepted ? "ok" : "rejected";
        goal_handle->succeed(result);
      })
      .detach();
  }

  bool result_accepted_;
  rclcpp_action::Server<PlaceBoxes>::SharedPtr server_;
};

class ExecutePlaceBoxesActionNodeTest : public ::testing::Test
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

const char * kTreeXml =
  "<root main_tree_to_execute=\"Main\">"
  "  <BehaviorTree ID=\"Main\">"
  "    <ExecutePlaceBoxesAction box_type=\"{box_type}\" payload=\"{payload}\" step_counter=\"{step_counter}\" has_target=\"{has_target}\" count_after_success=\"{count_after_success}\" match_type=\"{match_type}\" local_indices=\"{local_indices}\" action_name=\"{action_name}\" result_code_text=\"{result_code_text}\"/>"
  "  </BehaviorTree>"
  "</root>";

}  // namespace

TEST_F(ExecutePlaceBoxesActionNodeTest, CommitsTypeCounterWhenActionSucceeds)
{
  const std::string action_name = "/test/bt/place_boxes/success";
  auto server = std::make_shared<MockPlaceBoxesServer>(action_name, true);
  auto bt_node = std::make_shared<rclcpp::Node>("bt_execute_place_success");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(bt_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::ExecutePlaceBoxesAction>("ExecutePlaceBoxesAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));
  blackboard->set("food_box_count", 1);
  blackboard->set("tool_box_count", 0);
  blackboard->set("instrument_box_count", 0);
  blackboard->set("medical_box_count", 0);
  blackboard->set("box_type", std::string("food"));
  blackboard->set("payload", std::string("place=0,1,count=3"));
  blackboard->set("step_counter", 2);
  blackboard->set("has_target", true);
  blackboard->set("count_after_success", 3);
  blackboard->set("match_type", std::string("left"));
  blackboard->set("local_indices", std::vector<int>{0, 1});
  blackboard->set("action_name", action_name);

  auto tree = factory.createTreeFromText(kTreeXml, blackboard);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(2000),
    [&]() {
      status = tree.tickRoot();
      return status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE;
    }));

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<int>("food_box_count"), 3);
  EXPECT_EQ(blackboard->get<std::string>("result_code_text"), "success");

  executor.remove_node(bt_node);
  executor.remove_node(server);
}

TEST_F(ExecutePlaceBoxesActionNodeTest, KeepsTypeCounterWhenResultNotAccepted)
{
  const std::string action_name = "/test/bt/place_boxes/not_accepted";
  auto server = std::make_shared<MockPlaceBoxesServer>(action_name, false);
  auto bt_node = std::make_shared<rclcpp::Node>("bt_execute_place_not_accepted");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(bt_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::ExecutePlaceBoxesAction>("ExecutePlaceBoxesAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));
  blackboard->set("food_box_count", 0);
  blackboard->set("tool_box_count", 5);
  blackboard->set("instrument_box_count", 0);
  blackboard->set("medical_box_count", 0);
  blackboard->set("box_type", std::string("tool"));
  blackboard->set("payload", std::string("place=2,count=6"));
  blackboard->set("step_counter", 4);
  blackboard->set("has_target", true);
  blackboard->set("count_after_success", 6);
  blackboard->set("match_type", std::string("right"));
  blackboard->set("local_indices", std::vector<int>{2});
  blackboard->set("action_name", action_name);

  auto tree = factory.createTreeFromText(kTreeXml, blackboard);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  ASSERT_TRUE(waitUntil(
    executor,
    std::chrono::milliseconds(2000),
    [&]() {
      status = tree.tickRoot();
      return status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE;
    }));

  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(blackboard->get<int>("tool_box_count"), 5);
  EXPECT_EQ(blackboard->get<std::string>("result_code_text"), "succeeded_not_accepted");

  executor.remove_node(bt_node);
  executor.remove_node(server);
}

TEST_F(ExecutePlaceBoxesActionNodeTest, SkipsActionWhenNoTarget)
{
  auto bt_node = std::make_shared<rclcpp::Node>("bt_execute_place_skip");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<dog_behavior::bt_nodes::ExecutePlaceBoxesAction>("ExecutePlaceBoxesAction");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", std::static_pointer_cast<rclcpp::Node>(bt_node));
  blackboard->set("food_box_count", 2);
  blackboard->set("tool_box_count", 0);
  blackboard->set("instrument_box_count", 0);
  blackboard->set("medical_box_count", 0);
  blackboard->set("box_type", std::string("food"));
  blackboard->set("payload", std::string(""));
  blackboard->set("step_counter", 1);
  blackboard->set("has_target", false);
  blackboard->set("count_after_success", 2);
  blackboard->set("match_type", std::string("left"));
  blackboard->set("local_indices", std::vector<int>{});
  blackboard->set("action_name", std::string("/test/bt/place_boxes/skip"));

  auto tree = factory.createTreeFromText(kTreeXml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<int>("food_box_count"), 2);
  EXPECT_EQ(blackboard->get<std::string>("result_code_text"), "skipped_no_target");
}
