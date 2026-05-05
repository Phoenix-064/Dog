#pragma once

#include <cstdint>
#include <string>

namespace dog_serial_bridge
{

inline constexpr char kSupportedBehaviorName[] = "PickUpBoxes";
inline constexpr char kPickupCommand[] = "RCPickUpBoxes";
inline constexpr char kPickSuccessReply[] = "RCPickSuccess";
inline constexpr char kPickFailReply[] = "RCPickFail";
inline constexpr char kOkReply[] = "RCOK";

enum class ReplyType
{
  kUnknown,
  kPickSuccess,
  kPickFail,
  kOk,
};

struct PlacePayload
{
  std::string place;
  int count{0};
};

std::string buildPickupCommand();
std::string buildPlaceCommand(const std::string & payload);
bool parsePlacePayload(const std::string & payload, PlacePayload & parsed, std::string & error_detail);
ReplyType classifyReply(const std::string & line);
std::string buildPickupFeedback(uint64_t sequence, bool success);

}  // namespace dog_serial_bridge
