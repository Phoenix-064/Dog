#include "dog_perception/digit_recognizer.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace dog_perception
{

namespace
{

class OpencvDnnYoloDigitRecognizer final : public IDigitRecognizer
{
public:
  OpencvDnnYoloDigitRecognizer(const DigitRecognizerParams & params, const rclcpp::Logger & logger)
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

    if (!ensureModelLoaded()) {
      return DigitRecognitionResult{false, -1, 0.0F, "model_unavailable"};
    }

    cv::Mat frame = toCvMat(image);
    if (frame.empty()) {
      return DigitRecognitionResult{false, -1, 0.0F, "unsupported_encoding_or_layout"};
    }

    const int image_width = frame.cols;
    const int image_height = frame.rows;
    const int roi_x = std::clamp(params_.roi_x, 0, image_width - 1);
    const int roi_y = std::clamp(params_.roi_y, 0, image_height - 1);
    const int roi_w = std::max(1, std::min(params_.roi_width, image_width - roi_x));
    const int roi_h = std::max(1, std::min(params_.roi_height, image_height - roi_y));

    const cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
    if (roi.empty()) {
      return DigitRecognitionResult{false, -1, 0.0F, "empty_roi"};
    }

    cv::Mat roi_image = frame(roi).clone();
    if (roi_image.channels() == 1) {
      cv::cvtColor(roi_image, roi_image, cv::COLOR_GRAY2BGR);
    }

    const bool swap_rb = image.encoding == sensor_msgs::image_encodings::RGB8;
    cv::Mat blob = cv::dnn::blobFromImage(
      roi_image,
      1.0 / 255.0,
      cv::Size(640, 640),
      cv::Scalar(),
      swap_rb,
      false,
      CV_32F);

    try {
      net_.setInput(blob);
      std::vector<cv::Mat> outputs;
      net_.forward(outputs, net_.getUnconnectedOutLayersNames());

      if (outputs.empty()) {
        return DigitRecognitionResult{false, -1, 0.0F, "empty_output"};
      }

      int best_label = -1;
      float best_score = 0.0F;
      parseOutputs(outputs, best_label, best_score);

      if (best_label < 0 || best_score < static_cast<float>(params_.min_confidence)) {
        return DigitRecognitionResult{false, -1, best_score, "no_detection"};
      }

      return DigitRecognitionResult{true, best_label % 10, best_score, "ok"};
    } catch (const cv::Exception & exception) {
      RCLCPP_ERROR(logger_, "OpenCV DNN infer failed: %s", exception.what());
      return DigitRecognitionResult{false, -1, 0.0F, "dnn_infer_failed"};
    }
  }

private:
  static cv::Mat toCvMat(const sensor_msgs::msg::Image & image)
  {
    const size_t expected_size = static_cast<size_t>(image.step) * static_cast<size_t>(image.height);
    if (expected_size > image.data.size()) {
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

  bool ensureModelLoaded()
  {
    if (model_load_attempted_) {
      return model_loaded_;
    }

    model_load_attempted_ = true;
    if (params_.yolo_model_path.empty()) {
      RCLCPP_ERROR(logger_, "digit_yolo_model_path is empty");
      return false;
    }

    try {
      net_ = cv::dnn::readNet(params_.yolo_model_path);
      if (net_.empty()) {
        RCLCPP_ERROR(logger_, "OpenCV DNN failed to load YOLO model: %s", params_.yolo_model_path.c_str());
        return false;
      }
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      model_loaded_ = true;
      RCLCPP_INFO(logger_, "Loaded YOLO model via OpenCV DNN: %s", params_.yolo_model_path.c_str());
      return true;
    } catch (const cv::Exception & exception) {
      RCLCPP_ERROR(
        logger_,
        "Failed to load YOLO model via OpenCV DNN (%s): %s",
        params_.yolo_model_path.c_str(),
        exception.what());
      return false;
    }
  }

  static void parseOutputs(const std::vector<cv::Mat> & outputs, int & best_label, float & best_score)
  {
    for (const auto & output : outputs) {
      cv::Mat rows_view;
      if (output.dims == 3 && output.size[0] == 1) {
        rows_view = output.reshape(1, output.size[1]);
      } else if (output.dims == 2) {
        rows_view = output;
      } else {
        continue;
      }

      if (rows_view.cols < 6) {
        continue;
      }

      for (int row = 0; row < rows_view.rows; ++row) {
        const float * data = rows_view.ptr<float>(row);
        float score = 0.0F;
        int label = -1;

        if (rows_view.cols > 6) {
          const float objectness = data[4];
          const auto class_begin = data + 5;
          const auto class_end = data + rows_view.cols;
          const auto best_class_it = std::max_element(class_begin, class_end);
          if (best_class_it == class_end) {
            continue;
          }
          score = objectness * (*best_class_it);
          label = static_cast<int>(best_class_it - class_begin);
        } else {
          score = data[4];
          label = static_cast<int>(data[5]);
        }

        if (std::isfinite(score) && score > best_score) {
          best_score = score;
          best_label = label;
        }
      }
    }
  }

  DigitRecognizerParams params_;
  rclcpp::Logger logger_;
  cv::dnn::Net net_;
  bool model_load_attempted_{false};
  bool model_loaded_{false};
};

}  // namespace

bool registerOpencvDnnYoloDigitRecognizer()
{
  return registerDigitRecognizer(
    "opencv_dnn_yolo",
    [](const DigitRecognizerParams & params, const rclcpp::Logger & logger) {
      return std::make_unique<OpencvDnnYoloDigitRecognizer>(params, logger);
    });
}

}  // namespace dog_perception
