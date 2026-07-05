#include "dog_serial_bridge/serial_protocol.hpp"

#include <gtest/gtest.h>

namespace
{

using dog_serial_bridge::PlacePayload;
using dog_serial_bridge::ReplyType;

TEST(SerialProtocolTest, BuildsPickupCommand)
{
  EXPECT_EQ(dog_serial_bridge::buildPickupCommand(), "RCPickUpBoxes");
}

TEST(SerialProtocolTest, BuildsPlaceCommand)
{
  EXPECT_EQ(
    dog_serial_bridge::buildPlaceCommand("place=0,3,count=3"),
    "RCplace=0,3,count=3");
}

TEST(SerialProtocolTest, ParsePlacePayloadRejectsMissingPlace)
{
  PlacePayload payload;
  std::string error_detail;
  EXPECT_FALSE(dog_serial_bridge::parsePlacePayload("count=3", payload, error_detail));
  EXPECT_EQ(error_detail, "payload_missing_place");
}

TEST(SerialProtocolTest, ParsePlacePayloadRejectsMissingCount)
{
  PlacePayload payload;
  std::string error_detail;
  EXPECT_FALSE(dog_serial_bridge::parsePlacePayload("place=0,3", payload, error_detail));
  EXPECT_EQ(error_detail, "payload_missing_count");
}

TEST(SerialProtocolTest, ParsePlacePayloadRejectsNonIntegerCount)
{
  PlacePayload payload;
  std::string error_detail;
  EXPECT_FALSE(dog_serial_bridge::parsePlacePayload("place=0,3,count=abc", payload, error_detail));
  EXPECT_EQ(error_detail, "payload_invalid_count");
}

TEST(SerialProtocolTest, ParsePlacePayloadAcceptsExpectedFormat)
{
  PlacePayload payload;
  std::string error_detail;
  ASSERT_TRUE(dog_serial_bridge::parsePlacePayload("place=0,3,count=3", payload, error_detail));
  EXPECT_EQ(payload.place, "0,3");
  EXPECT_EQ(payload.count, 3);
  EXPECT_TRUE(error_detail.empty());
}

TEST(SerialProtocolTest, ClassifiesReplies)
{
  EXPECT_EQ(dog_serial_bridge::classifyReply("RCPickSuccess"), ReplyType::kPickSuccess);
  EXPECT_EQ(dog_serial_bridge::classifyReply("RCPickFail"), ReplyType::kPickFail);
  EXPECT_EQ(dog_serial_bridge::classifyReply("RCOK"), ReplyType::kOk);
  EXPECT_EQ(dog_serial_bridge::classifyReply("RCOther"), ReplyType::kUnknown);
}

}  // namespace
