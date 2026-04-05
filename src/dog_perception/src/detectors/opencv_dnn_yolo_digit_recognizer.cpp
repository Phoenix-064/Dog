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

/// @brief 基于 OpenCV DNN 的 YOLO 数字识别器实现。
///
/// 该实现负责：
/// 1) 按需加载 YOLO 模型；
/// 2) 对输入图像进行 ROI 裁剪与预处理；
/// 3) 执行前向推理并解析输出，返回置信度最高的数字结果。
class OpencvDnnYoloDigitRecognizer final : public IDigitRecognizer
{
public:
  /// @brief 构造识别器并保存运行参数与日志对象。
  OpencvDnnYoloDigitRecognizer(const DigitRecognizerParams & params, const rclcpp::Logger & logger)
  : params_(params), logger_(logger)
  {
  }

  /// @brief 对单帧图像执行数字识别。
  /// @param view 输入图像视图。
  /// @return 识别结果数组（高置信候选集合）。
  DigitRecognitionResultArrary infer(const ImageView & view) override
  {
    constexpr float kHighConfidenceThreshold = 0.8F;
    // 输入空指针保护。
    if (!view.image) {
      return {};
    }

    const auto & image = *view.image;
    // 基本图像有效性校验。
    if (image.width == 0U || image.height == 0U || image.data.empty()) {
      return {};
    }

    // 懒加载模型，首次失败后将不再重复尝试。
    if (!ensureModelLoaded()) {
      return {};
    }

    cv::Mat frame = toCvMat(image);
    if (frame.empty()) {
      return {};
    }

    const int image_width = frame.cols;
    const int image_height = frame.rows;
    const int roi_x = std::clamp(params_.roi_x, 0, image_width - 1);
    const int roi_y = std::clamp(params_.roi_y, 0, image_height - 1);
    const int roi_w = std::max(1, std::min(params_.roi_width, image_width - roi_x));
    const int roi_h = std::max(1, std::min(params_.roi_height, image_height - roi_y));

    // ROI 统一限制在图像边界内，避免越界访问。
    const cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
    if (roi.empty()) {
      return {};
    }

    cv::Mat roi_image = frame(roi).clone();
    if (roi_image.channels() == 1) {
      cv::cvtColor(roi_image, roi_image, cv::COLOR_GRAY2BGR);
    }

    // ROS RGB8 需要交换 R/B 通道，BGR8 则保持原样。
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
        return {};
      }

      DigitRecognitionResultArrary results;
      parseOutputs(
        outputs,
        roi,
        static_cast<float>(std::max(params_.min_confidence, static_cast<double>(kHighConfidenceThreshold))),
        results);
      return results;
    } catch (const cv::Exception & exception) {
      RCLCPP_ERROR(logger_, "OpenCV DNN infer failed: %s", exception.what());
      return {};
    }
  }

private:
  /// @brief 将 ROS Image 映射为 OpenCV Mat（零拷贝视图）。
  /// @note 仅支持 MONO8、BGR8、RGB8；若步长/布局异常则返回空 Mat。
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

  /// @brief 确保模型已加载。
  /// @return 模型可用时返回 true，否则返回 false。
  /// @note 该函数只在首次调用时尝试加载，后续直接复用结果。
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

  /// @brief 将候选中心点映射到完整图像坐标系。
  static geometry_msgs::msg::Point mapCenterPoint(
    const cv::Rect & roi,
    const float center_x,
    const float center_y)
  {
    geometry_msgs::msg::Point point;
    point.x = (center_x >= 0.0F && center_x <= 1.0F) ?
      static_cast<double>(roi.x) + static_cast<double>(center_x) * static_cast<double>(roi.width) :
      static_cast<double>(roi.x) + static_cast<double>(center_x);
    point.y = (center_y >= 0.0F && center_y <= 1.0F) ?
      static_cast<double>(roi.y) + static_cast<double>(center_y) * static_cast<double>(roi.height) :
      static_cast<double>(roi.y) + static_cast<double>(center_y);
    point.z = 0.0;
    return point;
  }

  /// @brief 解析 YOLO 输出并收集高置信候选。
  ///
  /// 兼容两类常见输出格式：
  /// 1) [x, y, w, h, obj, cls1, cls2, ...]；
  /// 2) [x, y, w, h, score, label]。
  static void parseOutputs(
    const std::vector<cv::Mat> & outputs,
    const cv::Rect & roi,
    const float confidence_threshold,
    DigitRecognitionResultArrary & results)
  {
    for (const auto & output : outputs) {
      cv::Mat rows_view;
      // 将 3D 输出拍平为按候选框逐行遍历的 2D 视图。
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

        // YOLO 风格输出：类别分数与 objectness 相乘得到最终置信度。
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
          if (rows_view.cols != 6) {
            continue;
          }
          score = data[4];
          label = static_cast<int>(data[5]);
        }

        if (!std::isfinite(score) || label < 0) {
          continue;
        }

        if (score < confidence_threshold) {
          continue;
        }

        const geometry_msgs::msg::Point point = mapCenterPoint(roi, data[0], data[1]);
        results.push_back(
          DigitRecognitionResult{true, label % 10, score, point, "ok"});
      }
    }
  }

  /// 识别器参数（ROI、阈值、模型路径等）。
  DigitRecognizerParams params_;
  /// ROS 日志句柄。
  rclcpp::Logger logger_;
  /// OpenCV DNN 网络对象。
  cv::dnn::Net net_;
  /// 是否已经执行过模型加载尝试。
  bool model_load_attempted_{false};
  /// 模型是否加载成功。
  bool model_loaded_{false};
};

}  // namespace

bool registerOpencvDnnYoloDigitRecognizer()
{
  // 将该实现注册到工厂，供配置项 "opencv_dnn_yolo" 动态创建。
  return registerDigitRecognizer(
    "opencv_dnn_yolo",
    [](const DigitRecognizerParams & params, const rclcpp::Logger & logger) {
      return std::make_unique<OpencvDnnYoloDigitRecognizer>(params, logger);
    });
}

}  // namespace dog_perception
