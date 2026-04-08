#include "dog_behavior/behavior_tree.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

class BehaviorTreeTest : public ::testing::Test
{
};

TEST_F(BehaviorTreeTest, ExecutesActionLeafThenPlaceholderLeaf)
{
  std::vector<std::string> visit_order;
  int action_call_count = 0;
  int placeholder_call_count = 0;

  dog_behavior::BehaviorTree tree(
    [&visit_order, &action_call_count](const std::string & behavior_name) {
      ++action_call_count;
      visit_order.push_back("action:" + behavior_name);
      return true;
    },
    [&visit_order, &placeholder_call_count]() {
      ++placeholder_call_count;
      visit_order.push_back("placeholder");
    },
    DOG_BEHAVIOR_TEST_BT_XML_PATH);

  EXPECT_TRUE(tree.execute("pick"));
  EXPECT_EQ(action_call_count, 1);
  EXPECT_EQ(placeholder_call_count, 1);
  ASSERT_EQ(visit_order.size(), 2u);
  EXPECT_EQ(visit_order[0], "action:pick");
  EXPECT_EQ(visit_order[1], "placeholder");
}

TEST_F(BehaviorTreeTest, SkipsPlaceholderLeafWhenActionLeafFails)
{
  std::vector<std::string> visit_order;
  int action_call_count = 0;
  int placeholder_call_count = 0;

  dog_behavior::BehaviorTree tree(
    [&visit_order, &action_call_count](const std::string & behavior_name) {
      ++action_call_count;
      visit_order.push_back("action:" + behavior_name);
      return false;
    },
    [&visit_order, &placeholder_call_count]() {
      ++placeholder_call_count;
      visit_order.push_back("placeholder");
    },
    DOG_BEHAVIOR_TEST_BT_XML_PATH);

  EXPECT_FALSE(tree.execute("place"));
  EXPECT_EQ(action_call_count, 1);
  EXPECT_EQ(placeholder_call_count, 0);
  ASSERT_EQ(visit_order.size(), 1u);
  EXPECT_EQ(visit_order[0], "action:place");
}

TEST_F(BehaviorTreeTest, PropagatesEmptyBehaviorNameToActionLeaf)
{
  std::string captured_behavior_name;
  int placeholder_call_count = 0;

  dog_behavior::BehaviorTree tree(
    [&captured_behavior_name](const std::string & behavior_name) {
      captured_behavior_name = behavior_name;
      return true;
    },
    [&placeholder_call_count]() {
      ++placeholder_call_count;
    },
    DOG_BEHAVIOR_TEST_BT_XML_PATH);

  EXPECT_TRUE(tree.execute(""));
  EXPECT_EQ(captured_behavior_name, "");
  EXPECT_EQ(placeholder_call_count, 1);
}

}  // namespace
