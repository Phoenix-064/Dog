#include "dog_behavior/common/payload_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>

#include <limits>

namespace
{

using dog_behavior::utils::hasValidQuaternionNorm;
using dog_behavior::utils::isCompletedState;
using dog_behavior::utils::isFinitePose;
using dog_behavior::utils::normalizeToken;
using dog_behavior::utils::parseKeyValuePayload;

TEST(PayloadUtilsTest, NormalizeTokenRemovesWhitespaceAndLowercases)
{
  EXPECT_EQ(normalizeToken("  IDLE_ SpInNing \t\n"), "idle_spinning");
}

TEST(PayloadUtilsTest, ParseKeyValuePayloadMatchesNormalizedKey)
{
  const std::string payload = "mode=Recovered; task_phase=abc";
  EXPECT_EQ(parseKeyValuePayload(payload, " MODE "), "Recovered");
}

TEST(PayloadUtilsTest, ParseKeyValuePayloadDecodesPercentEncodedValue)
{
  const std::string payload = "target_state=done%20ok%2f1";
  EXPECT_EQ(parseKeyValuePayload(payload, "target_state"), "done ok/1");
}

TEST(PayloadUtilsTest, ParseKeyValuePayloadReturnsEmptyWhenKeyMissing)
{
  const std::string payload = "mode=recovered;task_phase=abc";
  EXPECT_EQ(parseKeyValuePayload(payload, "target_state"), "");
}

TEST(PayloadUtilsTest, ParseKeyValuePayloadKeepsInvalidPercentLiteral)
{
  const std::string payload = "value=abc%2Xdef";
  EXPECT_EQ(parseKeyValuePayload(payload, "value"), "abc%2Xdef");
}

TEST(PayloadUtilsTest, CompletedStateRecognizesKnownTerminalStates)
{
  EXPECT_TRUE(isCompletedState("done"));
  EXPECT_TRUE(isCompletedState("completed"));
  EXPECT_TRUE(isCompletedState("succeeded"));
  EXPECT_TRUE(isCompletedState("success"));
  EXPECT_TRUE(isCompletedState("finished"));
  EXPECT_FALSE(isCompletedState("running"));
}

TEST(PayloadUtilsTest, IsFinitePoseRejectsNonFiniteValues)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  EXPECT_TRUE(isFinitePose(pose));
  pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(isFinitePose(pose));
}

TEST(PayloadUtilsTest, IsFinitePoseStampedUsesInnerPose)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(isFinitePose(pose));

  pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isFinitePose(pose));
}

TEST(PayloadUtilsTest, HasValidQuaternionNormChecksTolerance)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  EXPECT_TRUE(hasValidQuaternionNorm(pose));

  pose.orientation.w = 0.0;
  EXPECT_FALSE(hasValidQuaternionNorm(pose));

  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 2.0;
  EXPECT_FALSE(hasValidQuaternionNorm(pose));
}

TEST(PayloadUtilsTest, HasValidQuaternionNormStampedUsesInnerPose)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(hasValidQuaternionNorm(pose));

  pose.pose.orientation.w = 0.5;
  EXPECT_FALSE(hasValidQuaternionNorm(pose));
}

}  // namespace
