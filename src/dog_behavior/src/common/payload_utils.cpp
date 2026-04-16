#include "dog_behavior/common/payload_utils.hpp"

#include <cctype>
#include <cmath>

namespace dog_behavior::utils
{

namespace
{

int hexDigitValue(const char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return 10 + (c - 'a');
  }
  if (c >= 'A' && c <= 'F') {
    return 10 + (c - 'A');
  }
  return -1;
}

std::string percentDecode(const std::string & value)
{
  std::string decoded;
  decoded.reserve(value.size());

  size_t index = 0;
  while (index < value.size()) {
    if (value[index] == '%' && (index + 2) < value.size()) {
      const auto high = hexDigitValue(value[index + 1]);
      const auto low = hexDigitValue(value[index + 2]);
      if (high >= 0 && low >= 0) {
        const auto byte_value = static_cast<unsigned char>((high << 4) | low);
        decoded.push_back(static_cast<char>(byte_value));
        index += 3;
        continue;
      }
    }

    decoded.push_back(value[index]);
    ++index;
  }

  return decoded;
}

}  // namespace

std::string normalizeToken(const std::string & value)
{
  std::string normalized;
  normalized.reserve(value.size());
  for (const auto ch : value) {
    if (std::isspace(static_cast<unsigned char>(ch))) {
      continue;
    }
    normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }
  return normalized;
}

std::string parseKeyValuePayload(const std::string & payload, const std::string & key)
{
  static const std::string delimiter = ";";
  const auto normalized_key = normalizeToken(key);

  size_t start = 0;
  while (start <= payload.size()) {
    const auto end = payload.find(delimiter, start);
    const auto token = payload.substr(start, end == std::string::npos ? std::string::npos : end - start);
    const auto equal_pos = token.find('=');
    if (equal_pos != std::string::npos) {
      const auto token_key = normalizeToken(token.substr(0, equal_pos));
      if (token_key == normalized_key) {
        return percentDecode(token.substr(equal_pos + 1));
      }
    }

    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }

  return "";
}

bool isCompletedState(const std::string & target_state)
{
  return target_state == "done" || target_state == "completed" || target_state == "succeeded" ||
         target_state == "success" || target_state == "finished";
}

bool isFinitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool isFinitePose(const geometry_msgs::msg::PoseStamped & pose)
{
  return isFinitePose(pose.pose);
}

bool hasValidQuaternionNorm(const geometry_msgs::msg::Pose & pose)
{
  const double norm =
    (pose.orientation.x * pose.orientation.x) +
    (pose.orientation.y * pose.orientation.y) +
    (pose.orientation.z * pose.orientation.z) +
    (pose.orientation.w * pose.orientation.w);

  constexpr double min_norm = 1e-6;
  constexpr double target_norm = 1.0;
  constexpr double tolerance = 0.1;
  return norm > min_norm && std::fabs(norm - target_norm) <= tolerance;
}

bool hasValidQuaternionNorm(const geometry_msgs::msg::PoseStamped & pose)
{
  return hasValidQuaternionNorm(pose.pose);
}

}  // namespace dog_behavior::utils