#include "dog_perception/digit_recognizer.hpp"

#include <gtest/gtest.h>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <memory>
#include <string>

namespace
{

sensor_msgs::msg::Image::ConstSharedPtr makeImage(uint8_t value, uint32_t width = 32, uint32_t height = 32)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = width;
  image->height = height;
  image->encoding = "rgb8";
  image->step = width * 3U;
  image->data.resize(static_cast<size_t>(image->step * image->height), value);
  return image;
}

dog_perception::DigitRecognizerParams defaultParams()
{
  return dog_perception::DigitRecognizerParams{
    0,
    0,
    32,
    32,
    0.10,
    245.0,
    0.40,
    "yolo11n.pt"};
}

}  // namespace

TEST(DigitRecognizerFactoryTest, KnownTypeHeuristicCanBeCreated)
{
  auto recognizer = dog_perception::DigitRecognizerFactory::create(
    "heuristic",
    defaultParams(),
    rclcpp::get_logger("digit_factory_test"));

  ASSERT_NE(recognizer, nullptr);
  const auto result = recognizer->infer(dog_perception::ImageView{makeImage(127U)});
  EXPECT_FALSE(result.reason.empty());
}

TEST(DigitRecognizerFactoryTest, KnownTypeMeanIntensityCanBeCreated)
{
  auto recognizer = dog_perception::DigitRecognizerFactory::create(
    "mean_intensity",
    defaultParams(),
    rclcpp::get_logger("digit_factory_test"));

  ASSERT_NE(recognizer, nullptr);
  const auto result = recognizer->infer(dog_perception::ImageView{makeImage(210U)});
  EXPECT_TRUE(result.has_feature);
  EXPECT_GE(result.confidence, 0.10F);
}

TEST(DigitRecognizerFactoryTest, UnknownTypeFallsBackToHeuristic)
{
  EXPECT_NO_THROW({
    auto recognizer = dog_perception::DigitRecognizerFactory::create(
      "not_exist",
      defaultParams(),
      rclcpp::get_logger("digit_factory_test"));
    ASSERT_NE(recognizer, nullptr);
    const auto result = recognizer->infer(dog_perception::ImageView{makeImage(80U)});
    EXPECT_FALSE(result.reason.empty());
  });
}

TEST(DigitRecognizerFactoryTest, DuplicateRegistrationIsRejected)
{
  const bool first = dog_perception::registerDigitRecognizer(
    "duplicate_test",
    [](const dog_perception::DigitRecognizerParams &, const rclcpp::Logger &) {
      return dog_perception::DigitRecognizerFactory::create(
        "heuristic",
        defaultParams(),
        rclcpp::get_logger("digit_factory_test"));
    });

  const bool second = dog_perception::registerDigitRecognizer(
    "duplicate_test",
    [](const dog_perception::DigitRecognizerParams &, const rclcpp::Logger &) {
      return dog_perception::DigitRecognizerFactory::create(
        "mean_intensity",
        defaultParams(),
        rclcpp::get_logger("digit_factory_test"));
    });

  EXPECT_TRUE(first);
  EXPECT_FALSE(second);
}

TEST(DigitRecognizerFactoryTest, NullCreatorResultThrows)
{
  const bool registered = dog_perception::registerDigitRecognizer(
    "null_creator",
    [](const dog_perception::DigitRecognizerParams &, const rclcpp::Logger &) {
      return std::unique_ptr<dog_perception::IDigitRecognizer>{};
    });
  ASSERT_TRUE(registered);

  EXPECT_THROW(
    {
      (void)dog_perception::DigitRecognizerFactory::create(
        "null_creator",
        defaultParams(),
        rclcpp::get_logger("digit_factory_test"));
    },
    std::runtime_error);
}

TEST(DigitRecognizerFactoryTest, KnownTypeOpencvDnnYoloCanBeCreated)
{
  auto params = defaultParams();
  params.yolo_model_path = "/tmp/not_found_yolo_model.onnx";

  auto recognizer = dog_perception::DigitRecognizerFactory::create(
    "opencv_dnn_yolo",
    params,
    rclcpp::get_logger("digit_factory_test"));

  ASSERT_NE(recognizer, nullptr);
  const auto result = recognizer->infer(dog_perception::ImageView{makeImage(180U)});
  EXPECT_FALSE(result.has_feature);
  EXPECT_EQ(result.reason, "model_unavailable");
}
