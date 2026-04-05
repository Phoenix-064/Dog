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
  /// @brief 构造均值强度数字识别器。
  /// @param params 识别参数。
  /// @param logger 日志器。
  MeanIntensityDigitRecognizer(const DigitRecognizerParams & params, const rclcpp::Logger & logger)
  : params_(params), logger_(logger)
  {
  }

  /// @brief 对输入图像执行均值强度数字识别。
  /// @param view 输入图像视图。
  /// @return 数字识别结果数组。
  DigitRecognitionResultArrary infer(const ImageView & view) override
  {
    constexpr float kHighConfidenceThreshold = 0.8F;
    if (!view.image || view.image->width == 0U || view.image->height == 0U || view.image->data.empty()) {
      return {};
    }

    const auto & image = *view.image;
    const int image_width = static_cast<int>(image.width);
    const int image_height = static_cast<int>(image.height);
    const int roi_x = std::clamp(params_.roi_x, 0, image_width - 1);
    const int roi_y = std::clamp(params_.roi_y, 0, image_height - 1);
    const int roi_w = std::max(1, std::min(params_.roi_width, image_width - roi_x));
    const int roi_h = std::max(1, std::min(params_.roi_height, image_height - roi_y));

    if (image.step < image.width || (image.step % image.width) != 0U) {
      return {};
    }

    const size_t expected_size = static_cast<size_t>(image.step) * static_cast<size_t>(image.height);
    if (expected_size > image.data.size()) {
      return {};
    }

    const uint32_t channels = image.step / image.width;
    if (channels == 0U) {
      return {};
    }

    double sum = 0.0;
    size_t count = 0;
    for (int y = roi_y; y < roi_y + roi_h; ++y) {
      const size_t row_base = static_cast<size_t>(y) * image.step;
      for (int x = roi_x; x < roi_x + roi_w; ++x) {
        const size_t index = row_base + static_cast<size_t>(x) * channels;
        if (index >= image.data.size()) {
          return {};
        }
        sum += static_cast<double>(image.data[index]);
        ++count;
      }
    }

    if (count == 0U) {
      return {};
    }

    const double mean = sum / static_cast<double>(count);
    const float confidence = static_cast<float>(std::clamp(mean / 255.0, 0.0, 1.0));
    const float confidence_threshold =
      static_cast<float>(std::max(params_.min_confidence, static_cast<double>(kHighConfidenceThreshold)));
    if (confidence < confidence_threshold) {
      return {};
    }

    const int label = static_cast<int>(mean) % 10;
    geometry_msgs::msg::Point center;
    center.x = static_cast<double>(roi_x + roi_w / 2);
    center.y = static_cast<double>(roi_y + roi_h / 2);
    center.z = 0.0;

    return DigitRecognitionResultArrary{DigitRecognitionResult{true, label, confidence, center, "ok"}};
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
