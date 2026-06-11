#include "dog_perception/digit_recognizer.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/text.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace dog_perception
{

namespace
{

constexpr const char * kMathWhitelist = "0123456789+-*/xX()=:";

bool consumeUtf8At(const std::string & value, size_t & index, const char * token)
{
  const std::string expected(token);
  if (value.compare(index, expected.size(), expected) != 0) {
    return false;
  }
  index += expected.size();
  return true;
}

bool isAllowedExpressionChar(const char ch)
{
  return std::isdigit(static_cast<unsigned char>(ch)) || ch == '+' || ch == '-' || ch == '*' ||
         ch == '/' || ch == '(' || ch == ')';
}

std::string makeMathPayload(
  const std::string & expression,
  const std::string & raw_text,
  const std::string & reason)
{
  std::ostringstream stream;
  stream << "type=math_expr;expr=" << expression << ";raw=" << raw_text << ";reason=" << reason;
  return stream.str();
}

cv::Mat toCvMat(const sensor_msgs::msg::Image & image)
{
  const size_t expected_size = static_cast<size_t>(image.step) * static_cast<size_t>(image.height);
  if (image.width == 0U || image.height == 0U || image.data.empty() || expected_size > image.data.size()) {
    return cv::Mat();
  }

  if (image.encoding == sensor_msgs::image_encodings::MONO8) {
    return cv::Mat(
      static_cast<int>(image.height),
      static_cast<int>(image.width),
      CV_8UC1,
      const_cast<unsigned char *>(image.data.data()),
      static_cast<size_t>(image.step));
  }

  if (image.encoding == sensor_msgs::image_encodings::BGR8 ||
    image.encoding == sensor_msgs::image_encodings::RGB8)
  {
    return cv::Mat(
      static_cast<int>(image.height),
      static_cast<int>(image.width),
      CV_8UC3,
      const_cast<unsigned char *>(image.data.data()),
      static_cast<size_t>(image.step));
  }

  return cv::Mat();
}

cv::Mat preprocessForOcr(const cv::Mat & roi)
{
  cv::Mat gray;
  if (roi.channels() == 1) {
    gray = roi.clone();
  } else {
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
  }

  cv::Mat enlarged;
  cv::resize(gray, enlarged, cv::Size(), 3.0, 3.0, cv::INTER_CUBIC);

  cv::Mat equalized;
  auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
  clahe->apply(enlarged, equalized);

  cv::Mat binary;
  cv::threshold(equalized, binary, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

  const double mean_intensity = cv::mean(binary)[0];
  if (mean_intensity < 127.0) {
    cv::bitwise_not(binary, binary);
  }

  return binary;
}

float averageConfidence(const std::vector<float> & confidences)
{
  if (confidences.empty()) {
    return 1.0F;
  }

  double sum = 0.0;
  for (const auto confidence : confidences) {
    sum += static_cast<double>(confidence);
  }
  return static_cast<float>(std::clamp(sum / static_cast<double>(confidences.size()) / 100.0, 0.0, 1.0));
}

class MathOcrDigitRecognizer final : public IDigitRecognizer
{
public:
  MathOcrDigitRecognizer(const DigitRecognizerParams & params, const rclcpp::Logger & logger)
  : params_(params), logger_(logger)
  {
  }

  DigitRecognitionResultArrary infer(const ImageView & view) override
  {
    if (!view.image) {
      return {};
    }

    const auto & image = *view.image;
    cv::Mat frame = toCvMat(image);
    if (frame.empty()) {
      return {};
    }

    if (image.encoding == sensor_msgs::image_encodings::RGB8 && frame.channels() == 3) {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    const int image_width = frame.cols;
    const int image_height = frame.rows;
    const int roi_x = std::clamp(params_.roi_x, 0, image_width - 1);
    const int roi_y = std::clamp(params_.roi_y, 0, image_height - 1);
    const int roi_w = std::max(1, std::min(params_.roi_width, image_width - roi_x));
    const int roi_h = std::max(1, std::min(params_.roi_height, image_height - roi_y));
    const cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
    if (roi.empty()) {
      return {};
    }

    if (!ensureOcrCreated()) {
      return {};
    }

    cv::Mat ocr_input = preprocessForOcr(frame(roi));
    std::string raw_text;
    std::vector<cv::Rect> component_rects;
    std::vector<std::string> component_texts;
    std::vector<float> component_confidences;

    try {
      ocr_->run(
        ocr_input,
        raw_text,
        &component_rects,
        &component_texts,
        &component_confidences,
        cv::text::OCR_LEVEL_TEXTLINE);
    } catch (const cv::Exception & exception) {
      RCLCPP_WARN(logger_, "Math OCR failed: %s", exception.what());
      return {};
    }

    const std::string expression = normalizeMathExpressionForOcr(raw_text);
    if (expression.empty()) {
      return {};
    }

    const float confidence = averageConfidence(component_confidences);
    if (confidence < static_cast<float>(params_.min_confidence)) {
      return {};
    }

    geometry_msgs::msg::Point center;
    center.x = static_cast<double>(roi_x) + static_cast<double>(roi_w) * 0.5;
    center.y = static_cast<double>(roi_y) + static_cast<double>(roi_h) * 0.5;
    center.z = 0.0;

    return DigitRecognitionResultArrary{
      DigitRecognitionResult{
        true,
        -1,
        confidence,
        center,
        "ok",
        makeMathPayload(expression, raw_text, "ok")}};
  }

private:
  bool ensureOcrCreated()
  {
    if (ocr_) {
      return true;
    }

    try {
      ocr_ = cv::text::OCRTesseract::create(
        nullptr,
        "eng",
        kMathWhitelist,
        cv::text::OEM_DEFAULT,
        cv::text::PSM_SINGLE_LINE);
      return static_cast<bool>(ocr_);
    } catch (const cv::Exception & exception) {
      RCLCPP_ERROR(logger_, "Failed to initialize OpenCV OCRTesseract: %s", exception.what());
      return false;
    }
  }

  DigitRecognizerParams params_;
  rclcpp::Logger logger_;
  cv::Ptr<cv::text::OCRTesseract> ocr_;
};

}  // namespace

std::string normalizeMathExpressionForOcr(const std::string & raw_text)
{
  std::string normalized;
  normalized.reserve(raw_text.size());

  size_t index = 0;
  while (index < raw_text.size()) {
    if (consumeUtf8At(raw_text, index, u8"×") || consumeUtf8At(raw_text, index, u8"✕")) {
      normalized.push_back('*');
      continue;
    }
    if (consumeUtf8At(raw_text, index, u8"÷")) {
      normalized.push_back('/');
      continue;
    }
    if (consumeUtf8At(raw_text, index, u8"−") || consumeUtf8At(raw_text, index, u8"–") ||
      consumeUtf8At(raw_text, index, u8"—"))
    {
      normalized.push_back('-');
      continue;
    }

    const unsigned char byte = static_cast<unsigned char>(raw_text[index]);
    const char ch = static_cast<char>(byte);
    if (std::isspace(byte)) {
      ++index;
      continue;
    }
    if (ch == '=') {
      break;
    }
    if (ch == 'x' || ch == 'X') {
      normalized.push_back('*');
    } else if (ch == ':') {
      normalized.push_back('/');
    } else if (ch == 'O' || ch == 'o') {
      normalized.push_back('0');
    } else if (ch == 'I' || ch == 'l') {
      normalized.push_back('1');
    } else if (isAllowedExpressionChar(ch)) {
      normalized.push_back(ch);
    } else {
      return "";
    }
    ++index;
  }

  const auto has_digit = std::any_of(
    normalized.begin(),
    normalized.end(),
    [](const char ch) {return std::isdigit(static_cast<unsigned char>(ch));});
  return has_digit ? normalized : "";
}

bool registerMathOcrDigitRecognizer()
{
  return registerDigitRecognizer(
    "math_ocr",
    [](const DigitRecognizerParams & params, const rclcpp::Logger & logger) {
      return std::make_unique<MathOcrDigitRecognizer>(params, logger);
    });
}

}  // namespace dog_perception
