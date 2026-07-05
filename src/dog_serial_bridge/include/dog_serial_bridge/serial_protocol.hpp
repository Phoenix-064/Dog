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
inline constexpr char kArrivalReply[] = "RCArrivalMX";
inline constexpr uint8_t kDebugFrameTail[] = {0x00, 0x00, 0x80, 0x7F};

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
bool containsArrivalReply(const std::string & bytes);
size_t eraseCompleteDebugFrames(std::string & bytes);

}  // namespace dog_serial_bridge
