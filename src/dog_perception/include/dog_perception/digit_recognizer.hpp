#pragma once

#include <dog_interfaces/msg/target3_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace dog_perception
{

struct ImageView
{
  sensor_msgs::msg::Image::ConstSharedPtr image;
};

struct DigitRecognizerParams
{
  int roi_x;
  int roi_y;
  int roi_width;
  int roi_height;
  double min_confidence;
  double glare_brightness_threshold;
  double glare_ratio_threshold;
};

struct DigitRecognitionResult
{
  bool has_feature;
  int label;
  float confidence;
  std::string reason;
};

class IDigitRecognizer
{
public:
  virtual ~IDigitRecognizer() = default;

  virtual DigitRecognitionResult infer(const ImageView & view) = 0;
};

class DigitRecognizerFactory
{
public:
  static std::unique_ptr<IDigitRecognizer> create(
    const std::string & recognizer_type,
    const DigitRecognizerParams & params,
    const rclcpp::Logger & logger);
};

dog_interfaces::msg::Target3D toDigitTarget3D(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const std::string & output_frame_id,
  const DigitRecognitionResult & result);

}  // namespace dog_perception
