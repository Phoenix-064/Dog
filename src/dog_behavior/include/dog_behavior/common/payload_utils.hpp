#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace dog_behavior::utils
{

std::string normalizeToken(const std::string & value);
std::string parseKeyValuePayload(const std::string & payload, const std::string & key);
bool isCompletedState(const std::string & target_state);
bool isFinitePose(const geometry_msgs::msg::Pose & pose);
bool isFinitePose(const geometry_msgs::msg::PoseStamped & pose);
bool hasValidQuaternionNorm(const geometry_msgs::msg::Pose & pose);
bool hasValidQuaternionNorm(const geometry_msgs::msg::PoseStamped & pose);

}  // namespace dog_behavior::utils