#include "dog_perception/box_detector_node.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dog_perception
{

namespace
{

constexpr int kExpectedClassCount = 4;
constexpr int kYoloInputSize = 640;

std::string resolveDefaultModelPath()
{
  namespace fs = std::filesystem;

#ifdef DOG_WORKSPACE_ROOT
  const fs::path pinned_path = fs::path(DOG_WORKSPACE_ROOT) / "boxes_detector.pt";
  if (fs::exists(pinned_path) && fs::is_regular_file(pinned_path)) {
    return pinned_path.string();
  }
#endif

  fs::path current = fs::current_path();

  for (int depth = 0; depth < 6; ++depth) {
    const fs::path candidate = current / "boxes_detector.pt";
    if (fs::exists(candidate) && fs::is_regular_file(candidate)) {
      return candidate.string();
    }
    if (!current.has_parent_path()) {
      break;
    }
    current = current.parent_path();
  }

  return "boxes_detector.pt";
}

cv::Mat toCvMat(const sensor_msgs::msg::Image & image)
{
  const size_t expected_size =
    static_cast<size_t>(image.step) * static_cast<size_t>(image.height);
  if (image.width == 0U || image.height == 0U || expected_size > image.data.size()) {
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

  if (
    image.encoding == sensor_msgs::image_encodings::BGR8 ||
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

}  // namespace

BoxDetectorNode::BoxDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dog_box_detector", options),
  confidence_threshold_(0.35),
  nms_threshold_(0.45),
  max_boxes_(8),
  class_names_({"type_0", "type_1", "type_2", "type_3"}),
  model_load_attempted_(false),
  model_loaded_(false)
{
  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
  box_result_topic_ =
    declare_parameter<std::string>("box_result_topic", "/target/box_result");
  model_path_ =
    declare_parameter<std::string>("box_yolo_model_path", resolveDefaultModelPath());
  confidence_threshold_ = declare_parameter<double>("box_confidence_threshold", 0.35);
  nms_threshold_ = declare_parameter<double>("box_nms_threshold", 0.45);
  max_boxes_ = declare_parameter<int>("box_max_detections", 8);
  class_names_ = declare_parameter<std::vector<std::string>>(
    "box_class_names",
    std::vector<std::string>{"type_0", "type_1", "type_2", "type_3"});

  if (confidence_threshold_ <= 0.0 || confidence_threshold_ > 1.0) {
    throw std::runtime_error("box_confidence_threshold must be in (0, 1]");
  }
  if (nms_threshold_ <= 0.0 || nms_threshold_ > 1.0) {
    throw std::runtime_error("box_nms_threshold must be in (0, 1]");
  }
  if (max_boxes_ <= 0 || max_boxes_ > 8) {
    throw std::runtime_error("box_max_detections must be in [1, 8]");
  }
  if (class_names_.size() != static_cast<size_t>(kExpectedClassCount)) {
    throw std::runtime_error("box_class_names must provide exactly 4 class labels");
  }
  for (const auto & class_name : class_names_) {
    if (class_name.empty()) {
      throw std::runtime_error("box_class_names cannot contain empty values");
    }
  }

  result_pub_ =
    create_publisher<dog_interfaces::msg::Target3D>(box_result_topic_, rclcpp::SensorDataQoS());
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&BoxDetectorNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "dog_box_detector initialized, image_topic=%s, box_result_topic=%s, model_path=%s, max_boxes=%d",
    image_topic_.c_str(),
    box_result_topic_.c_str(),
    model_path_.c_str(),
    max_boxes_);
}

bool BoxDetectorNode::ensureModelLoaded()
{
  if (model_load_attempted_) {
    return model_loaded_;
  }

  model_load_attempted_ = true;
  if (model_path_.empty()) {
    RCLCPP_ERROR(get_logger(), "box_yolo_model_path is empty");
    return false;
  }

  try {
    net_ = cv::dnn::readNet(model_path_);
    if (net_.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to load YOLO model from %s", model_path_.c_str());
      return false;
    }
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    model_loaded_ = true;
    RCLCPP_INFO(get_logger(), "Loaded YOLO model: %s", model_path_.c_str());
    return true;
  } catch (const cv::Exception & exception) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to load YOLO model from %s: %s",
      model_path_.c_str(),
      exception.what());
    return false;
  }
}

std::vector<BoxDetectorNode::Detection> BoxDetectorNode::parseDetections(
  const std::vector<cv::Mat> & outputs,
  int frame_width,
  int frame_height) const
{
  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;

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
      int class_id = -1;

      if (rows_view.cols > 6) {
        const float objectness = data[4];
        const auto class_begin = data + 5;
        const auto class_end = data + rows_view.cols;
        const auto best_class_it = std::max_element(class_begin, class_end);
        if (best_class_it == class_end) {
          continue;
        }
        score = objectness * (*best_class_it);
        class_id = static_cast<int>(best_class_it - class_begin);
      } else {
        score = data[4];
        class_id = static_cast<int>(data[5]);
      }

      if (!std::isfinite(score) || score < static_cast<float>(confidence_threshold_)) {
        continue;
      }
      if (class_id < 0 || class_id >= static_cast<int>(class_names_.size())) {
        continue;
      }

      float cx = data[0];
      float cy = data[1];
      float width = data[2];
      float height = data[3];

      if (width <= 0.0F || height <= 0.0F) {
        continue;
      }

      const bool normalized =
        cx <= 1.5F && cy <= 1.5F && width <= 1.5F && height <= 1.5F;
      if (normalized) {
        cx *= static_cast<float>(frame_width);
        cy *= static_cast<float>(frame_height);
        width *= static_cast<float>(frame_width);
        height *= static_cast<float>(frame_height);
      }

      int left = static_cast<int>(cx - width * 0.5F);
      int top = static_cast<int>(cy - height * 0.5F);
      int box_width = static_cast<int>(width);
      int box_height = static_cast<int>(height);

      left = std::clamp(left, 0, std::max(0, frame_width - 1));
      top = std::clamp(top, 0, std::max(0, frame_height - 1));
      box_width = std::clamp(box_width, 1, std::max(1, frame_width - left));
      box_height = std::clamp(box_height, 1, std::max(1, frame_height - top));

      boxes.emplace_back(left, top, box_width, box_height);
      confidences.push_back(score);
      class_ids.push_back(class_id);
    }
  }

  std::vector<int> kept_indices;
  cv::dnn::NMSBoxes(
    boxes,
    confidences,
    static_cast<float>(confidence_threshold_),
    static_cast<float>(nms_threshold_),
    kept_indices);

  std::vector<Detection> detections;
  detections.reserve(kept_indices.size());
  for (const int index : kept_indices) {
    const cv::Rect & box = boxes.at(static_cast<size_t>(index));
    detections.push_back(Detection{
      class_ids.at(static_cast<size_t>(index)),
      confidences.at(static_cast<size_t>(index)),
      box.x,
      box.y,
      box.width,
      box.height});
  }

  std::sort(
    detections.begin(),
    detections.end(),
    [](const Detection & lhs, const Detection & rhs) {
      return lhs.confidence > rhs.confidence;
    });

  if (detections.size() > static_cast<size_t>(max_boxes_)) {
    detections.resize(static_cast<size_t>(max_boxes_));
  }

  return detections;
}

void BoxDetectorNode::publishNoBox(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const std::string & reason)
{
  dog_interfaces::msg::Target3D message;
  if (image_msg) {
    message.header.stamp = image_msg->header.stamp;
    message.header.frame_id = image_msg->header.frame_id;
  }
  message.target_id = "no_box";
  message.position.x = 0.0;
  message.position.y = 0.0;
  message.position.z = 0.0;
  message.confidence = 0.0F;
  result_pub_->publish(message);

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "No box detected: %s",
    reason.c_str());
}

void BoxDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  if (!image_msg) {
    publishNoBox(image_msg, "null_image");
    return;
  }

  cv::Mat frame = toCvMat(*image_msg);
  if (frame.empty()) {
    publishNoBox(image_msg, "invalid_image_or_encoding");
    return;
  }

  if (!ensureModelLoaded()) {
    publishNoBox(image_msg, "model_unavailable");
    return;
  }

  if (frame.channels() == 1) {
    cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
  }

  const bool swap_rb = image_msg->encoding == sensor_msgs::image_encodings::RGB8;
  cv::Mat blob = cv::dnn::blobFromImage(
    frame,
    1.0 / 255.0,
    cv::Size(kYoloInputSize, kYoloInputSize),
    cv::Scalar(),
    swap_rb,
    false,
    CV_32F);

  std::vector<cv::Mat> outputs;
  try {
    net_.setInput(blob);
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());
  } catch (const cv::Exception & exception) {
    RCLCPP_ERROR(get_logger(), "OpenCV DNN infer failed: %s", exception.what());
    publishNoBox(image_msg, "dnn_infer_failed");
    return;
  }

  if (outputs.empty()) {
    publishNoBox(image_msg, "empty_output");
    return;
  }

  auto detections = parseDetections(outputs, frame.cols, frame.rows);
  if (detections.empty()) {
    publishNoBox(image_msg, "no_detection");
    return;
  }

  const float width_f = static_cast<float>(std::max(1, frame.cols));
  const float height_f = static_cast<float>(std::max(1, frame.rows));
  const float area_f = width_f * height_f;

  int index = 0;
  for (const auto & detection : detections) {
    dog_interfaces::msg::Target3D message;
    message.header = image_msg->header;
    message.target_id = class_names_[static_cast<size_t>(detection.class_id)] +
      "#" + std::to_string(index);
    message.position.x = static_cast<double>(detection.x + detection.width / 2.0F) /
      static_cast<double>(width_f);
    message.position.y = static_cast<double>(detection.y + detection.height / 2.0F) /
      static_cast<double>(height_f);
    message.position.z = static_cast<double>(detection.width * detection.height) /
      static_cast<double>(area_f);
    message.confidence = detection.confidence;
    result_pub_->publish(message);
    ++index;
  }
}

}  // namespace dog_perception
