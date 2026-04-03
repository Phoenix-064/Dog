#include "dog_behavior/behavior_node.hpp"

#include <gtest/gtest.h>

TEST(BehaviorNodeTest, NodeName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<dog_behavior::BehaviorNode>();
  EXPECT_STREQ(node->get_name(), "dog_behavior");
  rclcpp::shutdown();
}