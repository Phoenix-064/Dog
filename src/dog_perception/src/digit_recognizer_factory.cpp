#include "dog_perception/digit_recognizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace dog_perception
{

namespace
{

class HeuristicDigitRecognizer final : public IDigitRecognizer
{
public:
  HeuristicDigitRecognizer(const DigitRecognizerParams & params, const rclcpp::Logger & logger)
  : params_(params), logger_(logger)
  {
  }

  DigitRecognitionResult infer(const ImageView & view) override
  {
    if (!view.image) {
      return DigitRecognitionResult{false, -1, 0.0F, "null_image"};
    }

    const auto & image = *view.image;
    if (image.width == 0U || image.height == 0U || image.data.empty()) {
      return DigitRecognitionResult{false, -1, 0.0F, "invalid_image"};
    }

    const int image_width = static_cast<int>(image.width);
    const int image_height = static_cast<int>(image.height);
    const int roi_x = std::clamp(params_.roi_x, 0, image_width - 1);
    const int roi_y = std::clamp(params_.roi_y, 0, image_height - 1);
    const int roi_w = std::max(1, std::min(params_.roi_width, image_width - roi_x));
    const int roi_h = std::max(1, std::min(params_.roi_height, image_height - roi_y));

    const uint32_t channels = image.step / image.width;
    if (channels == 0U) {
      return DigitRecognitionResult{false, -1, 0.0F, "invalid_step"};
    }

    double mean = 0.0;
    double m2 = 0.0;
    size_t sample_count = 0;
    size_t bright_count = 0;

    for (int y = roi_y; y < roi_y + roi_h; ++y) {
      const size_t row_base = static_cast<size_t>(y) * image.step;
      for (int x = roi_x; x < roi_x + roi_w; ++x) {
        const size_t index = row_base + static_cast<size_t>(x) * channels;
        if (index >= image.data.size()) {
          return DigitRecognitionResult{false, -1, 0.0F, "out_of_range"};
        }

        const double intensity = static_cast<double>(image.data[index]);
        ++sample_count;
        const double delta = intensity - mean;
        mean += delta / static_cast<double>(sample_count);
        const double delta2 = intensity - mean;
        m2 += delta * delta2;

        if (intensity >= params_.glare_brightness_threshold) {
          ++bright_count;
        }
      }
    }

    if (sample_count == 0U) {
      return DigitRecognitionResult{false, -1, 0.0F, "empty_roi"};
    }

    const double variance = m2 / static_cast<double>(sample_count);
    const double glare_ratio = static_cast<double>(bright_count) / static_cast<double>(sample_count);
    if (glare_ratio >= params_.glare_ratio_threshold) {
      return DigitRecognitionResult{false, -1, 0.0F, "glare_filtered"};
    }

    double confidence = std::clamp((variance / 2500.0) * (1.0 - glare_ratio), 0.0, 1.0);
    if (confidence < params_.min_confidence) {
      return DigitRecognitionResult{false, -1, static_cast<float>(confidence), "low_confidence"};
    }

    const int label = static_cast<int>(std::lround(mean)) % 10;
    return DigitRecognitionResult{true, label, static_cast<float>(confidence), "ok"};
  }

private:
  DigitRecognizerParams params_;
  rclcpp::Logger logger_;
};

}  // namespace

std::unique_ptr<IDigitRecognizer> DigitRecognizerFactory::create(
  const std::string & recognizer_type,
  const DigitRecognizerParams & params,
  const rclcpp::Logger & logger)
{
  if (recognizer_type.empty() || recognizer_type == "heuristic") {
    return std::make_unique<HeuristicDigitRecognizer>(params, logger);
  }

  throw std::runtime_error("Unsupported digit recognizer type: " + recognizer_type);
}

dog_interfaces::msg::Target3D toDigitTarget3D(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const std::string & output_frame_id,
  const DigitRecognitionResult & result)
{
  dog_interfaces::msg::Target3D message;
  if (image_msg) {
    message.header.stamp = image_msg->header.stamp;
  }
  message.header.frame_id = output_frame_id;
  message.target_id = result.has_feature ? ("digit_" + std::to_string(result.label)) : "no_feature";
  message.confidence = result.confidence;
  return message;
}

}  // namespace dog_perception
