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

sensor_msgs::msg::Image::ConstSharedPtr makePatternImage(uint32_t width = 32, uint32_t height = 32)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = width;
  image->height = height;
  image->encoding = "rgb8";
  image->step = width * 3U;
  image->data.resize(static_cast<size_t>(image->step * image->height), 0U);

  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      const size_t index = static_cast<size_t>(y * image->step + x * 3U);
      image->data[index] = static_cast<uint8_t>((x * 5U + y * 11U) % 255U);
      image->data[index + 1U] = static_cast<uint8_t>((x * 3U + y * 7U) % 255U);
      image->data[index + 2U] = static_cast<uint8_t>((x * 13U + y * 2U) % 255U);
    }
  }
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
  const auto results = recognizer->infer(dog_perception::ImageView{makePatternImage()});
  if (!results.empty()) {
    EXPECT_FALSE(results.front().reason.empty());
  }
}

TEST(DigitRecognizerFactoryTest, KnownTypeMeanIntensityCanBeCreated)
{
  auto recognizer = dog_perception::DigitRecognizerFactory::create(
    "mean_intensity",
    defaultParams(),
    rclcpp::get_logger("digit_factory_test"));

  ASSERT_NE(recognizer, nullptr);
  const auto results = recognizer->infer(dog_perception::ImageView{makeImage(210U)});
  ASSERT_FALSE(results.empty());
  EXPECT_TRUE(results.front().has_feature);
  EXPECT_GE(results.front().confidence, 0.10F);
}

TEST(DigitRecognizerFactoryTest, UnknownTypeFallsBackToHeuristic)
{
  EXPECT_NO_THROW({
    auto recognizer = dog_perception::DigitRecognizerFactory::create(
      "not_exist",
      defaultParams(),
      rclcpp::get_logger("digit_factory_test"));
    ASSERT_NE(recognizer, nullptr);
    const auto results = recognizer->infer(dog_perception::ImageView{makePatternImage()});
    if (!results.empty()) {
      EXPECT_FALSE(results.front().reason.empty());
    }
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
  const auto results = recognizer->infer(dog_perception::ImageView{makeImage(180U)});
  EXPECT_TRUE(results.empty());
}

TEST(DigitRecognizerFactoryTest, ToDigitTarget3DConvertsMultipleResults)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  image->header.frame_id = "camera_optical_frame";
  image->header.stamp.sec = 123;
  image->header.stamp.nanosec = 456;

  geometry_msgs::msg::Point point_a;
  point_a.x = 10.0;
  point_a.y = 20.0;
  point_a.z = 0.0;

  geometry_msgs::msg::Point point_b;
  point_b.x = 30.0;
  point_b.y = 40.0;
  point_b.z = 0.0;

  const dog_perception::DigitRecognitionResultArrary results{
    dog_perception::DigitRecognitionResult{true, 3, 0.92F, point_a, "ok"},
    dog_perception::DigitRecognitionResult{true, 8, 0.81F, point_b, "ok"}};

  const auto target_array = dog_perception::toDigitTarget3D(image, "base_link", results);
  ASSERT_EQ(target_array.targets.size(), 2U);
  EXPECT_EQ(target_array.targets[0].target_id, "digit_3");
  EXPECT_FLOAT_EQ(target_array.targets[0].confidence, 0.92F);
  EXPECT_DOUBLE_EQ(target_array.targets[0].position.x, 10.0);
  EXPECT_DOUBLE_EQ(target_array.targets[0].position.y, 20.0);
  EXPECT_EQ(target_array.targets[1].target_id, "digit_8");
  EXPECT_FLOAT_EQ(target_array.targets[1].confidence, 0.81F);
  EXPECT_DOUBLE_EQ(target_array.targets[1].position.x, 30.0);
  EXPECT_DOUBLE_EQ(target_array.targets[1].position.y, 40.0);
}

TEST(DigitRecognizerFactoryTest, ToDigitTarget3DReturnsNoFeatureWhenInputEmpty)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  image->header.stamp.sec = 1;
  image->header.stamp.nanosec = 2;

  const auto target_array = dog_perception::toDigitTarget3D(image, "base_link", {});
  ASSERT_EQ(target_array.targets.size(), 1U);
  EXPECT_EQ(target_array.targets.front().target_id, "no_feature");
  EXPECT_FLOAT_EQ(target_array.targets.front().confidence, 0.0F);
}
