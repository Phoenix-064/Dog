#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace cv
{
class Mat;
}

namespace dog_perception
{

/// @brief 相机采集抽象，便于节点在测试中注入假实现。
class ICameraFrameSource
{
public:
  virtual ~ICameraFrameSource() = default;
  virtual bool open(int device_id) = 0;
  virtual bool isOpened() const = 0;
  virtual bool read(cv::Mat & frame) = 0;
};

/// @brief 周期性采集相机画面并发布 sensor_msgs/Image。
class CameraPublisherNode : public rclcpp::Node
{
public:
  explicit CameraPublisherNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::shared_ptr<ICameraFrameSource> frame_source = nullptr);

private:
  void captureAndPublish();

  std::string image_topic_;
  std::string camera_frame_id_;
  int camera_device_id_;
  int publish_period_ms_;

  std::shared_ptr<ICameraFrameSource> frame_source_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace dog_perception
