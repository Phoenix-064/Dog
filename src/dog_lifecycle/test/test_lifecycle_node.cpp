#include "dog_lifecycle/lifecycle_node.hpp"

#include <gtest/gtest.h>

TEST(LifecycleNodeTest, NodeName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<dog_lifecycle::LifecycleNode>();
  EXPECT_STREQ(node->get_name(), "dog_lifecycle");
  rclcpp::shutdown();
}