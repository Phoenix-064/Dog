#include "dog_perception/digit_recognizer.hpp"

#include <algorithm>
#include <cstdint>

namespace dog_perception
{

namespace
{

class MeanIntensityDigitRecognizer final : public IDigitRecognizer
{
public:
  MeanIntensityDigitRecognizer(const DigitRecognizerParams & params, const rclcpp::Logger & logger)
  : params_(params), logger_(logger)
  {
  }

  DigitRecognitionResult infer(const ImageView & view) override
  {
    if (!view.image || view.image->width == 0U || view.image->height == 0U || view.image->data.empty()) {
      return DigitRecognitionResult{false, -1, 0.0F, "invalid_image"};
    }

    const auto & image = *view.image;
    const int image_width = static_cast<int>(image.width);
    const int image_height = static_cast<int>(image.height);
    const int roi_x = std::clamp(params_.roi_x, 0, image_width - 1);
    const int roi_y = std::clamp(params_.roi_y, 0, image_height - 1);
    const int roi_w = std::max(1, std::min(params_.roi_width, image_width - roi_x));
    const int roi_h = std::max(1, std::min(params_.roi_height, image_height - roi_y));

    if (image.step < image.width || (image.step % image.width) != 0U) {
      return DigitRecognitionResult{false, -1, 0.0F, "invalid_step"};
    }

    const size_t expected_size = static_cast<size_t>(image.step) * static_cast<size_t>(image.height);
    if (expected_size > image.data.size()) {
      return DigitRecognitionResult{false, -1, 0.0F, "invalid_layout"};
    }

    const uint32_t channels = image.step / image.width;
    if (channels == 0U) {
      return DigitRecognitionResult{false, -1, 0.0F, "invalid_step"};
    }

    double sum = 0.0;
    size_t count = 0;
    for (int y = roi_y; y < roi_y + roi_h; ++y) {
      const size_t row_base = static_cast<size_t>(y) * image.step;
      for (int x = roi_x; x < roi_x + roi_w; ++x) {
        const size_t index = row_base + static_cast<size_t>(x) * channels;
        if (index >= image.data.size()) {
          return DigitRecognitionResult{false, -1, 0.0F, "out_of_range"};
        }
        sum += static_cast<double>(image.data[index]);
        ++count;
      }
    }

    if (count == 0U) {
      return DigitRecognitionResult{false, -1, 0.0F, "empty_roi"};
    }

    const double mean = sum / static_cast<double>(count);
    const float confidence = static_cast<float>(std::clamp(mean / 255.0, 0.0, 1.0));
    if (confidence < params_.min_confidence) {
      return DigitRecognitionResult{false, -1, confidence, "low_confidence"};
    }

    const int label = static_cast<int>(mean) % 10;
    return DigitRecognitionResult{true, label, confidence, "ok"};
  }

private:
  DigitRecognizerParams params_;
  rclcpp::Logger logger_;
};

}  // namespace

bool registerMeanIntensityDigitRecognizer()
{
  return registerDigitRecognizer(
    "mean_intensity",
    [](const DigitRecognizerParams & params, const rclcpp::Logger & logger) {
      return std::make_unique<MeanIntensityDigitRecognizer>(params, logger);
    });
}

}  // namespace dog_perception
