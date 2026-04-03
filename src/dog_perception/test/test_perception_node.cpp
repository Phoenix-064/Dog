#include "dog_perception/perception_node.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>

namespace
{

std::string createTempExtrinsicsYaml()
{
  const auto temp_yaml_path =
    std::filesystem::temp_directory_path() /
    ("dog_perception_extrinsics_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");

  std::ofstream stream(temp_yaml_path);
  stream << "frame_id: base_link\n";
  stream << "child_frame_id: camera_optical_frame\n";
  stream << "x: 0.11\n";
  stream << "y: -0.02\n";
  stream << "z: 0.33\n";
  stream << "qx: 0.0\n";
  stream << "qy: 0.0\n";
  stream << "qz: 0.0\n";
  stream << "qw: 1.0\n";
  stream.close();

  return temp_yaml_path.string();
}

}  // namespace

class PerceptionNodeTest : public ::testing::Test
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

TEST_F(PerceptionNodeTest, NodeName)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  rclcpp::NodeOptions options;
  options.append_parameter_override("extrinsics_yaml_path", yaml_path);
  options.append_parameter_override("lidar_topic", "/livox/lidar");

  auto node = std::make_shared<dog_perception::PerceptionNode>(options);
  EXPECT_STREQ(node->get_name(), "dog_perception");

  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, LoadsConfigAndPublishesStaticTf)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  rclcpp::NodeOptions options;
  options.append_parameter_override("extrinsics_yaml_path", yaml_path);
  options.append_parameter_override("lidar_topic", "/livox/lidar");

  auto perception_node = std::make_shared<dog_perception::PerceptionNode>(options);
  auto listener_node = std::make_shared<rclcpp::Node>("tf_static_listener");

  bool received = false;
  auto qos = rclcpp::QoS(10).transient_local().reliable();
  auto tf_sub = listener_node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static",
    qos,
    [&received](const tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
      for (const auto & transform : message->transforms) {
        if (transform.header.frame_id == "base_link" &&
          transform.child_frame_id == "camera_optical_frame")
        {
          received = true;
          return;
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(listener_node);

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
  }

  EXPECT_TRUE(received);

  executor.remove_node(listener_node);
  executor.remove_node(perception_node);
  (void)tf_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, ThrowsOnMissingExtrinsicsFile)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("extrinsics_yaml_path", "/tmp/does_not_exist_camera_extrinsics.yaml");
  options.append_parameter_override("lidar_topic", "/livox/lidar");

  EXPECT_THROW(
    {
      auto node = std::make_shared<dog_perception::PerceptionNode>(options);
      (void)node;
    },
    std::runtime_error);
}