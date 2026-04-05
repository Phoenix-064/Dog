#include "dog_perception/box_detector.hpp"

#include <sensor_msgs/msg/image.hpp>

#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

sensor_msgs::msg::Image makeImage(const rclcpp::Time & stamp, uint8_t value = 128U)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = "camera_optical_frame";
  image.width = 64;
  image.height = 64;
  image.encoding = "rgb8";
  image.step = image.width * 3U;
  image.data.resize(static_cast<size_t>(image.step * image.height), value);
  return image;
}

sensor_msgs::msg::Image makeInvalidImage(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = "camera_optical_frame";
  image.width = 0;
  image.height = 0;
  image.encoding = "rgb8";
  image.step = 0;
  return image;
}

dog_perception::BoxDetector::Params makeParams(const std::string & model_path)
{
  return dog_perception::BoxDetector::Params{
    model_path,
    0.35,
    0.45,
    8,
    {"blue", "red", "green", "yellow"}};
}

}  // namespace

class BoxDetectorNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(BoxDetectorNodeTest, NodeName)
{
  const auto logger = rclcpp::get_logger("box_detector_test_logger");
  dog_perception::BoxDetector detector(makeParams("/tmp/not_found_boxes_model.pt"), logger);

  auto image = std::make_shared<sensor_msgs::msg::Image>(makeImage(rclcpp::Clock().now()));
  const auto result = detector.detect(image);
  ASSERT_FALSE(result.targets.empty());
  EXPECT_EQ(result.targets.front().target_id, "no_box");
}

TEST_F(BoxDetectorNodeTest, ThrowsWhenClassCountIsNotFour)
{
  const auto logger = rclcpp::get_logger("box_detector_test_logger_invalid_class");
  EXPECT_THROW(
    {
      dog_perception::BoxDetector detector(
        dog_perception::BoxDetector::Params{
          "/tmp/not_found_boxes_model.pt",
          0.35,
          0.45,
          8,
          {"a", "b", "c"}},
        logger);
      (void)detector;
    },
    std::runtime_error);
}

TEST_F(BoxDetectorNodeTest, PublishesNoBoxWhenModelUnavailable)
{
  const auto logger = rclcpp::get_logger("box_detector_test_logger_no_model");
  dog_perception::BoxDetector detector(makeParams("/tmp/not_found_boxes_model.pt"), logger);

  auto image = std::make_shared<sensor_msgs::msg::Image>(makeImage(rclcpp::Clock().now()));
  const auto result = detector.detect(image);
  ASSERT_FALSE(result.targets.empty());
  EXPECT_EQ(result.targets.front().target_id, "no_box");
}

TEST_F(BoxDetectorNodeTest, PublishesNoBoxOnInvalidImage)
{
  const auto logger = rclcpp::get_logger("box_detector_test_logger_invalid_image");
  dog_perception::BoxDetector detector(makeParams("/tmp/not_found_boxes_model.pt"), logger);

  auto image = std::make_shared<sensor_msgs::msg::Image>(makeInvalidImage(rclcpp::Clock().now()));
  const auto result = detector.detect(image);
  ASSERT_FALSE(result.targets.empty());
  EXPECT_EQ(result.targets.front().target_id, "no_box");
}
