#include "dog_perception/camera_publisher_node.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

namespace
{

class FakeCameraFrameSource final : public dog_perception::ICameraFrameSource
{
public:
  explicit FakeCameraFrameSource(const bool open_result)
  : open_result_(open_result), opened_(false), read_count_(0)
  {
  }

  bool open(const int /*device_id*/) override
  {
    opened_ = open_result_;
    return open_result_;
  }

  bool isOpened() const override
  {
    return opened_;
  }

  bool read(cv::Mat & frame) override
  {
    if (!opened_) {
      return false;
    }

    frame = cv::Mat(24, 32, CV_8UC3, cv::Scalar(10, 20, 30));
    ++read_count_;
    return true;
  }

  int readCount() const
  {
    return read_count_;
  }

private:
  bool open_result_;
  bool opened_;
  int read_count_;
};

bool waitUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::milliseconds timeout,
  const std::function<bool()> & condition)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return condition();
}

}  // namespace

class CameraPublisherNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(CameraPublisherNodeTest, PublishesImageOnParameterizedTopic)
{
  const std::string image_topic = "/test/camera/image";
  const std::string frame_id = "test_camera_optical_frame";

  rclcpp::NodeOptions options;
  options.append_parameter_override("image_topic", image_topic);
  options.append_parameter_override("camera_frame_id", frame_id);
  options.append_parameter_override("camera_device_id", 99);
  options.append_parameter_override("publish_period_ms", 10);

  auto fake_source = std::make_shared<FakeCameraFrameSource>(true);
  auto camera_node = std::make_shared<dog_perception::CameraPublisherNode>(options, fake_source);
  auto probe_node = std::make_shared<rclcpp::Node>("camera_probe_node");

  sensor_msgs::msg::Image::SharedPtr received;
  auto sub = probe_node->create_subscription<sensor_msgs::msg::Image>(
    image_topic,
    rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::Image::SharedPtr msg) {
      received = msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(camera_node);
  executor.add_node(probe_node);

  const bool got_message = waitUntil(executor, std::chrono::milliseconds(800), [&]() {
      return static_cast<bool>(received);
    });

  executor.remove_node(camera_node);
  executor.remove_node(probe_node);

  ASSERT_TRUE(got_message);
  ASSERT_NE(received, nullptr);
  EXPECT_EQ(received->header.frame_id, frame_id);
  EXPECT_EQ(received->encoding, "bgr8");
  EXPECT_EQ(received->width, 32U);
  EXPECT_EQ(received->height, 24U);
  EXPECT_GT(fake_source->readCount(), 0);
  (void)sub;
}

TEST_F(CameraPublisherNodeTest, ThrowsWhenImageTopicIsEmpty)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("image_topic", "");

  auto fake_source = std::make_shared<FakeCameraFrameSource>(true);
  EXPECT_THROW(
    std::make_shared<dog_perception::CameraPublisherNode>(options, fake_source),
    std::invalid_argument);
}
