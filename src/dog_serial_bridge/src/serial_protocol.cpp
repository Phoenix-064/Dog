#include "dog_serial_bridge/serial_protocol.hpp"

#include <sstream>
#include <vector>

namespace dog_serial_bridge
{

namespace
{

std::vector<std::string> split(const std::string & input, const char delimiter)
{
  std::vector<std::string> tokens;
  std::stringstream stream(input);
  std::string token;
  while (std::getline(stream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

bool isInteger(const std::string & text)
{
  if (text.empty()) {
    return false;
  }

  size_t start = 0;
  if (text[0] == '-' || text[0] == '+') {
    if (text.size() == 1U) {
      return false;
    }
    start = 1;
  }

  for (size_t index = start; index < text.size(); ++index) {
    if (text[index] < '0' || text[index] > '9') {
      return false;
    }
  }

  return true;
}

}  // namespace

std::string buildPickupCommand()
{
  return kPickupCommand;
}

std::string buildPlaceCommand(const std::string & payload)
{
  return "RC" + payload;
}

bool parsePlacePayload(const std::string & payload, PlacePayload & parsed, std::string & error_detail)
{
  static constexpr char kPrefix[] = "place=";
  static constexpr char kCountMarker[] = ",count=";

  if (payload.rfind(kPrefix, 0) != 0U) {
    error_detail = "payload_missing_place";
    return false;
  }

  const auto count_marker_pos = payload.find(kCountMarker);
  if (count_marker_pos == std::string::npos) {
    error_detail = "payload_missing_count";
    return false;
  }

  const auto place_value = payload.substr(sizeof(kPrefix) - 1U, count_marker_pos - (sizeof(kPrefix) - 1U));
  const auto count_value = payload.substr(count_marker_pos + (sizeof(kCountMarker) - 1U));
  if (place_value.empty()) {
    error_detail = "payload_missing_place";
    return false;
  }
  if (count_value.empty()) {
    error_detail = "payload_missing_count";
    return false;
  }

  const auto indices = split(place_value, ',');
  if (indices.empty()) {
    error_detail = "payload_missing_place";
    return false;
  }
  for (const auto & index : indices) {
    if (!isInteger(index)) {
      error_detail = "payload_invalid_place";
      return false;
    }
  }

  if (!isInteger(count_value)) {
    error_detail = "payload_invalid_count";
    return false;
  }

  parsed.place = place_value;
  parsed.count = std::stoi(count_value);
  error_detail.clear();
  return true;
}

ReplyType classifyReply(const std::string & line)
{
  if (line == kPickSuccessReply) {
    return ReplyType::kPickSuccess;
  }
  if (line == kPickFailReply) {
    return ReplyType::kPickFail;
  }
  if (line == kOkReply) {
    return ReplyType::kOk;
  }
  return ReplyType::kUnknown;
}

}  // namespace dog_serial_bridge
