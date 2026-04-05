#include "dog_perception/box_detector_node.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <filesystem>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>

namespace
{

sensor_msgs::msg::Image makeImage(const rclcpp::Time & stamp, uint8_t value = 128U)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = "camera_optical_frame";
  image.width = 64;
  image.height = 64;
  image.encoding = "rgb8";
  image.step = image.width * 3U;
  image.data.resize(static_cast<size_t>(image.step * image.height), value);
  return image;
}

sensor_msgs::msg::Image makeInvalidImage(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = "camera_optical_frame";
  image.width = 0;
  image.height = 0;
  image.encoding = "rgb8";
  image.step = 0;
  return image;
}

void spinFor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

std::shared_ptr<dog_perception::BoxDetectorNode> createNode(
  const std::string & image_topic,
  const std::string & result_topic,
  const std::string & model_path)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("image_topic", image_topic);
  options.append_parameter_override("box_result_topic", result_topic);
  options.append_parameter_override("box_yolo_model_path", model_path);
  options.append_parameter_override("box_max_detections", 8);
  options.append_parameter_override(
    "box_class_names",
    std::vector<std::string>{"blue", "red", "green", "yellow"});
  return std::make_shared<dog_perception::BoxDetectorNode>(options);
}

}  // namespace

class BoxDetectorNodeTest : public ::testing::Test
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

TEST_F(BoxDetectorNodeTest, NodeName)
{
  auto node = createNode(
    "/test/box_detector/image/name",
    "/test/box_detector/result/name",
    "/tmp/not_found_boxes_model.pt");
  EXPECT_STREQ(node->get_name(), "dog_box_detector");
}

TEST_F(BoxDetectorNodeTest, ThrowsWhenClassCountIsNotFour)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("box_class_names", std::vector<std::string>{"a", "b", "c"});
  EXPECT_THROW(
    {
      auto node = std::make_shared<dog_perception::BoxDetectorNode>(options);
      (void)node;
    },
    std::runtime_error);
}

TEST_F(BoxDetectorNodeTest, PublishesNoBoxWhenModelUnavailable)
{
  const std::string image_topic = "/test/box_detector/image/model_missing";
  const std::string result_topic = "/test/box_detector/result/model_missing";

  auto box_node = createNode(image_topic, result_topic, "/tmp/not_found_boxes_model.pt");
  auto io_node = std::make_shared<rclcpp::Node>("box_detector_test_io_model_missing");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());

  bool received_no_box = false;
  auto result_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    result_topic,
    rclcpp::SensorDataQoS(),
    [&received_no_box](const dog_interfaces::msg::Target3DArray::ConstSharedPtr message) {
      if (!message->targets.empty() && message->targets.front().target_id == "no_box") {
        received_no_box = true;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(box_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(60));

  image_pub->publish(makeImage(io_node->now()));
  spinFor(executor, std::chrono::milliseconds(220));

  EXPECT_TRUE(received_no_box);

  executor.remove_node(io_node);
  executor.remove_node(box_node);
  (void)result_sub;
}

TEST_F(BoxDetectorNodeTest, PublishesNoBoxOnInvalidImage)
{
  const std::string image_topic = "/test/box_detector/image/invalid";
  const std::string result_topic = "/test/box_detector/result/invalid";

  auto box_node = createNode(image_topic, result_topic, "/tmp/not_found_boxes_model.pt");
  auto io_node = std::make_shared<rclcpp::Node>("box_detector_test_io_invalid");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());

  int no_box_count = 0;
  auto result_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    result_topic,
    rclcpp::SensorDataQoS(),
    [&no_box_count](const dog_interfaces::msg::Target3DArray::ConstSharedPtr message) {
      if (!message->targets.empty() && message->targets.front().target_id == "no_box") {
        ++no_box_count;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(box_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(60));

  image_pub->publish(makeInvalidImage(io_node->now()));
  spinFor(executor, std::chrono::milliseconds(220));

  EXPECT_GE(no_box_count, 1);

  executor.remove_node(io_node);
  executor.remove_node(box_node);
  (void)result_sub;
}
