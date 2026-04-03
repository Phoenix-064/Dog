#include "dog_perception/perception_node.hpp"

#include <gtest/gtest.h>

TEST(PerceptionNodeTest, NodeName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<dog_perception::PerceptionNode>();
  EXPECT_STREQ(node->get_name(), "dog_perception");
  rclcpp::shutdown();
}