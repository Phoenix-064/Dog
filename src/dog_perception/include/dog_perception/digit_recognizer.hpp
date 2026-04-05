#pragma once

#include <dog_interfaces/msg/target3_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <functional>

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
  /// @brief Virtual destructor for polymorphic digit recognizers.
  virtual ~IDigitRecognizer() = default;

  /// @brief Run digit inference on provided image view.
  /// @param view Input image view.
  /// @return Digit recognition result.
  virtual DigitRecognitionResult infer(const ImageView & view) = 0;
};

using DigitRecognizerCreator = std::function<std::unique_ptr<IDigitRecognizer>(
    const DigitRecognizerParams &, const rclcpp::Logger &)>;

/// @brief Register a named digit recognizer creator.
/// @param recognizer_type Type key used by the factory.
/// @param creator Factory callback creating recognizer instance.
/// @return True when registration succeeds.
bool registerDigitRecognizer(
  const std::string & recognizer_type,
  DigitRecognizerCreator creator);

/// @brief Register built-in heuristic digit recognizer implementation.
/// @return True when registration succeeds.
bool registerHeuristicDigitRecognizer();
/// @brief Register built-in mean-intensity digit recognizer implementation.
/// @return True when registration succeeds.
bool registerMeanIntensityDigitRecognizer();

class DigitRecognizerFactory
{
public:
  /// @brief Create digit recognizer by type with fallback behavior.
  /// @param recognizer_type Requested recognizer type.
  /// @param params Recognizer runtime parameters.
  /// @param logger Logger for diagnostics.
  /// @return Digit recognizer instance.
  static std::unique_ptr<IDigitRecognizer> create(
    const std::string & recognizer_type,
    const DigitRecognizerParams & params,
    const rclcpp::Logger & logger);
};

/// @brief Convert digit recognition result to Target3D message form.
/// @param image_msg Source image message.
/// @param output_frame_id Output frame id.
/// @param result Digit recognition result.
/// @return Target3D representation for publication.
dog_interfaces::msg::Target3D toDigitTarget3D(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const std::string & output_frame_id,
  const DigitRecognitionResult & result);

}  // namespace dog_perception
