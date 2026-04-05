#include "dog_perception/camera_publisher_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace dog_perception
{

namespace
{

class OpenCvCameraFrameSource final : public ICameraFrameSource
{
public:
  bool open(const int device_id) override
  {
    return capture_.open(device_id);
  }

  bool isOpened() const override
  {
    return capture_.isOpened();
  }

  bool read(cv::Mat & frame) override
  {
    return capture_.read(frame);
  }

private:
  cv::VideoCapture capture_;
};

}  // namespace

CameraPublisherNode::CameraPublisherNode(
  const rclcpp::NodeOptions & options,
  std::shared_ptr<ICameraFrameSource> frame_source)
: rclcpp::Node("dog_camera_publisher", options)
, camera_device_id_(0)
, publish_period_ms_(33)
, frame_source_(std::move(frame_source))
{
  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
  camera_frame_id_ = declare_parameter<std::string>("camera_frame_id", "camera_optical_frame");
  camera_device_id_ = declare_parameter<int>("camera_device_id", 0);
  publish_period_ms_ = declare_parameter<int>("publish_period_ms", 33);

  if (image_topic_.empty()) {
    throw std::invalid_argument("Parameter 'image_topic' must be non-empty");
  }
  if (camera_frame_id_.empty()) {
    throw std::invalid_argument("Parameter 'camera_frame_id' must be non-empty");
  }
  if (publish_period_ms_ <= 0) {
    throw std::invalid_argument("Parameter 'publish_period_ms' must be > 0");
  }

  if (!frame_source_) {
    frame_source_ = std::make_shared<OpenCvCameraFrameSource>();
  }

  if (!frame_source_->open(camera_device_id_) || !frame_source_->isOpened()) {
    throw std::runtime_error(
            "Failed to open camera device id: " + std::to_string(camera_device_id_));
  }

  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_),
    std::bind(&CameraPublisherNode::captureAndPublish, this));

  RCLCPP_INFO(
    get_logger(),
    "dog_camera_publisher initialized, image_topic=%s, frame_id=%s, camera_device_id=%d, publish_period_ms=%d",
    image_topic_.c_str(),
    camera_frame_id_.c_str(),
    camera_device_id_,
    publish_period_ms_);
}

void CameraPublisherNode::captureAndPublish()
{
  cv::Mat frame;
  if (!frame_source_->read(frame) || frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Camera capture returned empty frame");
    return;
  }

  cv::Mat output;
  std::string encoding;
  if (frame.channels() == 1) {
    output = frame;
    encoding = "mono8";
  } else if (frame.channels() == 3) {
    output = frame;
    encoding = "bgr8";
  } else if (frame.channels() == 4) {
    cv::cvtColor(frame, output, cv::COLOR_BGRA2BGR);
    encoding = "bgr8";
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Unsupported channel count: %d",
      frame.channels());
    return;
  }

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, output).toImageMsg();
  msg->header.stamp = now();
  msg->header.frame_id = camera_frame_id_;
  image_pub_->publish(*msg);
}

}  // namespace dog_perception
