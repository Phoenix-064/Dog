#include "dog_perception/perception_node.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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

sensor_msgs::msg::Image makeImage(const rclcpp::Time & stamp, uint32_t width = 640, uint32_t height = 480)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = "camera_optical_frame";
  image.width = width;
  image.height = height;
  image.encoding = "rgb8";
  image.step = width * 3U;
  image.data.resize(static_cast<size_t>(image.step * image.height), 1U);
  return image;
}

sensor_msgs::msg::Image makePatternImage(const rclcpp::Time & stamp, uint32_t width = 64, uint32_t height = 64)
{
  auto image = makeImage(stamp, width, height);
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      const size_t index = static_cast<size_t>(y * image.step + x * 3U);
      image.data[index] = static_cast<uint8_t>((x * 7U + y * 13U) % 255U);
    }
  }
  return image;
}

sensor_msgs::msg::Image makeBrightImage(const rclcpp::Time & stamp, uint32_t width = 64, uint32_t height = 64)
{
  auto image = makeImage(stamp, width, height);
  std::fill(image.data.begin(), image.data.end(), 255U);
  return image;
}

sensor_msgs::msg::Image makeUniformImage(const rclcpp::Time & stamp, uint8_t value)
{
  auto image = makeImage(stamp, 64, 64);
  std::fill(image.data.begin(), image.data.end(), value);
  return image;
}

sensor_msgs::msg::PointCloud2 makePointCloud(const rclcpp::Time & stamp, uint32_t width = 32)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = "base_link";
  cloud.width = width;
  cloud.height = 1U;
  cloud.point_step = 16U;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(static_cast<size_t>(cloud.row_step), 0U);
  return cloud;
}

sensor_msgs::msg::PointCloud2 makePointCloudWithXyz(
  const rclcpp::Time & stamp,
  const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = "base_link";
  cloud.height = 1U;
  cloud.width = static_cast<uint32_t>(points.size());

  cloud.fields.resize(3);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0U;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1U;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4U;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1U;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8U;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1U;

  cloud.point_step = 12U;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_dense = false;
  cloud.data.resize(static_cast<size_t>(cloud.row_step), 0U);

  for (size_t i = 0; i < points.size(); ++i) {
    const size_t offset = i * static_cast<size_t>(cloud.point_step);
    std::memcpy(&cloud.data[offset + 0U], &points[i][0], sizeof(float));
    std::memcpy(&cloud.data[offset + 4U], &points[i][1], sizeof(float));
    std::memcpy(&cloud.data[offset + 8U], &points[i][2], sizeof(float));
  }

  return cloud;
}

std::shared_ptr<dog_perception::PerceptionNode> createPerceptionNode(
  const std::string & yaml_path,
  const std::string & image_topic,
  const std::string & pointcloud_topic,
  const std::string & target_topic,
  const std::string & digit_topic = "/test/digit/default",
  int frame_cache_size = 8,
  const std::string & qos_reliability = "best_effort",
  int stale_timeout_ms = 50,
  int dropout_timeout_ms = 150,
  const std::string & lifecycle_mode_topic = "/test/lifecycle/system_mode/default",
  int digit_temporal_confirm_count = 2,
  const std::string & digit_recognizer_type = "heuristic",
  const std::string & solver_type = "mock_minimal")
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("extrinsics_yaml_path", yaml_path);
  options.append_parameter_override("image_topic", image_topic);
  options.append_parameter_override("pointcloud_topic", pointcloud_topic);
  options.append_parameter_override("target3d_topic", target_topic);
  options.append_parameter_override("digit_result_topic", digit_topic);
  options.append_parameter_override("sync_queue_size", 10);
  options.append_parameter_override("sync_slop_ms", 30);
  options.append_parameter_override("frame_cache_size", frame_cache_size);
  options.append_parameter_override("stale_frame_timeout_ms", stale_timeout_ms);
  options.append_parameter_override("single_side_dropout_timeout_ms", dropout_timeout_ms);
  options.append_parameter_override("extrapolation_watchdog_ms", 20);
  options.append_parameter_override("extrapolation_min_interval_ms", 30);
  options.append_parameter_override("lifecycle_mode_topic", lifecycle_mode_topic);
  options.append_parameter_override("digit_temporal_window", 5);
  options.append_parameter_override("digit_temporal_confirm_count", digit_temporal_confirm_count);
  options.append_parameter_override("digit_roi_width", 64);
  options.append_parameter_override("digit_roi_height", 64);
  options.append_parameter_override("digit_min_confidence", 0.30);
  options.append_parameter_override("digit_glare_brightness_threshold", 245.0);
  options.append_parameter_override("digit_glare_ratio_threshold", 0.35);
  options.append_parameter_override("qos_reliability", qos_reliability);
  options.append_parameter_override("digit_recognizer_type", digit_recognizer_type);
  options.append_parameter_override("solver_type", solver_type);
  options.append_parameter_override("box_yolo_model_path", "/tmp/not_found_boxes_model.pt");
  options.append_parameter_override(
    "box_class_names",
    std::vector<std::string>{"type_0", "type_1", "type_2", "type_3"});
  return std::make_shared<dog_perception::PerceptionNode>(options);
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

  auto node = createPerceptionNode(
    yaml_path,
    "/test/image/name",
    "/test/cloud/name",
    "/test/target/name");
  EXPECT_STREQ(node->get_name(), "dog_perception");

  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, LoadsConfigAndPublishesStaticTf)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  auto perception_node = createPerceptionNode(
    yaml_path,
    "/test/image/tf",
    "/test/cloud/tf",
    "/test/target/tf");
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
  options.append_parameter_override("image_topic", "/test/image/missing");
  options.append_parameter_override("pointcloud_topic", "/test/cloud/missing");
  options.append_parameter_override("target3d_topic", "/test/target/missing");

  EXPECT_THROW(
    {
      auto node = std::make_shared<dog_perception::PerceptionNode>(options);
      (void)node;
    },
    std::runtime_error);
}

TEST_F(PerceptionNodeTest, SynchronizedPipelinePublishesTarget3D)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/sync_success";
  const std::string cloud_topic = "/test/cloud/sync_success";
  const std::string target_topic = "/test/target/sync_success";

  auto perception_node = createPerceptionNode(yaml_path, image_topic, cloud_topic, target_topic);
  auto io_node = std::make_shared<rclcpp::Node>("io_sync_success");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  bool target_received = false;
  auto target_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    target_topic,
    rclcpp::SensorDataQoS(),
    [&target_received](const dog_interfaces::msg::Target3DArray::ConstSharedPtr message) {
      if (
        !message->targets.empty() &&
        message->targets.front().target_id.find("synced_target|box:") == 0U)
      {
        target_received = true;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp = io_node->now();
  image_pub->publish(makeImage(stamp));
  cloud_pub->publish(makePointCloud(stamp));

  spinFor(executor, std::chrono::milliseconds(300));

  EXPECT_TRUE(target_received);
  EXPECT_GE(perception_node->getSolvedFrameCount(), 1U);
  EXPECT_GE(perception_node->getLatencySampleCount(), 1U);
  EXPECT_LT(perception_node->getLatencyP95Ms(), 50.0);
  EXPECT_LT(perception_node->getEndToEndLatencyP95Ms(), 50.0);

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  (void)target_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, MinimalPnpSolverPublishesFiniteTarget3D)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/minimal_pnp";
  const std::string cloud_topic = "/test/cloud/minimal_pnp";
  const std::string target_topic = "/test/target/minimal_pnp";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    "/test/digit/minimal_pnp",
    8,
    "best_effort",
    50,
    150,
    "/test/lifecycle/system_mode/minimal_pnp",
    2,
    "heuristic",
    "minimal_pnp");
  auto io_node = std::make_shared<rclcpp::Node>("io_minimal_pnp");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  bool target_received = false;
  dog_interfaces::msg::Target3D received_msg;
  auto target_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    target_topic,
    rclcpp::SensorDataQoS(),
    [&target_received, &received_msg](const dog_interfaces::msg::Target3DArray::ConstSharedPtr message) {
      if (message->targets.empty()) {
        return;
      }
      target_received = true;
      received_msg = message->targets.front();
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp = io_node->now();
  image_pub->publish(makeImage(stamp));
  cloud_pub->publish(makePointCloudWithXyz(stamp, {
      {1.0F, 2.0F, 3.0F},
      {2.0F, 0.0F, 4.0F},
      {0.0F, 3.0F, 5.0F},
      {3.0F, 1.0F, 6.0F}}));

  spinFor(executor, std::chrono::milliseconds(250));

  ASSERT_TRUE(target_received);
  EXPECT_EQ(received_msg.target_id, "synced_target|box:no_box");
  EXPECT_EQ(received_msg.header.frame_id, "base_link");
  EXPECT_TRUE(std::isfinite(received_msg.position.x));
  EXPECT_TRUE(std::isfinite(received_msg.position.y));
  EXPECT_TRUE(std::isfinite(received_msg.position.z));
  EXPECT_NEAR(received_msg.position.x, 1.5, 1e-3);
  EXPECT_NEAR(received_msg.position.y, 1.5, 1e-3);
  EXPECT_NEAR(received_msg.position.z, 4.5, 1e-3);
  EXPECT_GE(perception_node->getSolvedFrameCount(), 1U);
  EXPECT_LT(perception_node->getEndToEndLatencyP95Ms(), 50.0);

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  (void)target_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, MinimalPnpSolverSkipsPointCloudWithoutXyzFields)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/minimal_pnp_missing_xyz";
  const std::string cloud_topic = "/test/cloud/minimal_pnp_missing_xyz";
  const std::string target_topic = "/test/target/minimal_pnp_missing_xyz";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    "/test/digit/minimal_pnp_missing_xyz",
    8,
    "best_effort",
    50,
    150,
    "/test/lifecycle/system_mode/minimal_pnp_missing_xyz",
    2,
    "heuristic",
    "minimal_pnp");
  auto io_node = std::make_shared<rclcpp::Node>("io_minimal_pnp_missing_xyz");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  bool target_received = false;
  auto target_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    target_topic,
    rclcpp::SensorDataQoS(),
    [&target_received](const dog_interfaces::msg::Target3DArray::ConstSharedPtr) { target_received = true; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp = io_node->now();
  image_pub->publish(makeImage(stamp));
  cloud_pub->publish(makePointCloud(stamp));

  spinFor(executor, std::chrono::milliseconds(250));

  EXPECT_FALSE(target_received);
  EXPECT_EQ(perception_node->getSolvedFrameCount(), 0U);

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  (void)target_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, MinimalPnpSolverSkipsAllNonFinitePoints)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/minimal_pnp_non_finite";
  const std::string cloud_topic = "/test/cloud/minimal_pnp_non_finite";
  const std::string target_topic = "/test/target/minimal_pnp_non_finite";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    "/test/digit/minimal_pnp_non_finite",
    8,
    "best_effort",
    50,
    150,
    "/test/lifecycle/system_mode/minimal_pnp_non_finite",
    2,
    "heuristic",
    "minimal_pnp");
  auto io_node = std::make_shared<rclcpp::Node>("io_minimal_pnp_non_finite");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  bool target_received = false;
  auto target_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    target_topic,
    rclcpp::SensorDataQoS(),
    [&target_received](const dog_interfaces::msg::Target3DArray::ConstSharedPtr) { target_received = true; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp = io_node->now();
  image_pub->publish(makeImage(stamp));
  cloud_pub->publish(makePointCloudWithXyz(stamp, {
      {std::numeric_limits<float>::quiet_NaN(), 1.0F, 2.0F},
      {1.0F, std::numeric_limits<float>::infinity(), 2.0F},
      {1.0F, 2.0F, -std::numeric_limits<float>::infinity()}}));

  spinFor(executor, std::chrono::milliseconds(250));

  EXPECT_FALSE(target_received);
  EXPECT_EQ(perception_node->getSolvedFrameCount(), 0U);

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  (void)target_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, SingleSideDropoutTriggersExtrapolationAndThenRecovers)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/dropout_extrapolation";
  const std::string cloud_topic = "/test/cloud/dropout_extrapolation";
  const std::string target_topic = "/test/target/dropout_extrapolation";
  const std::string mode_topic = "/test/lifecycle/system_mode/dropout_extrapolation";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    "/test/digit/dropout_extrapolation",
    8,
    "best_effort",
    50,
    80,
    mode_topic,
    2,
    "heuristic");
  auto io_node = std::make_shared<rclcpp::Node>("io_dropout_extrapolation");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub = io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());
  auto mode_pub = io_node->create_publisher<std_msgs::msg::String>(
    mode_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  size_t extrapolated_count = 0U;
  auto target_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    target_topic,
    rclcpp::SensorDataQoS(),
    [&extrapolated_count](const dog_interfaces::msg::Target3DArray::ConstSharedPtr msg) {
      if (!msg->targets.empty() && msg->targets.front().target_id == "extrapolated_target") {
        ++extrapolated_count;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(120));

  std_msgs::msg::String normal_mode;
  normal_mode.data = "mode=normal;reason=test_setup";
  mode_pub->publish(normal_mode);

  const auto stamp1 = io_node->now();
  image_pub->publish(makeImage(stamp1));
  cloud_pub->publish(makePointCloud(stamp1));
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp2 = io_node->now();
  image_pub->publish(makeImage(stamp2));
  cloud_pub->publish(makePointCloud(stamp2));
  spinFor(executor, std::chrono::milliseconds(80));

  const auto dropout_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(700);
  while (std::chrono::steady_clock::now() < dropout_deadline) {
    image_pub->publish(makeImage(io_node->now()));
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  EXPECT_GE(extrapolated_count, 1U);
  EXPECT_GE(perception_node->getExtrapolationTriggerCount(), 1U);

  const auto recover_stamp = io_node->now();
  image_pub->publish(makeImage(recover_stamp));
  cloud_pub->publish(makePointCloud(recover_stamp));

  ASSERT_TRUE(waitUntil(executor, std::chrono::milliseconds(700), [&]() {
    return perception_node->getExtrapolationRecoveryCount() >= 1U;
  }));

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  (void)target_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, DigitRecognitionPublishesSemanticResult)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/digit_ok";
  const std::string cloud_topic = "/test/cloud/digit_ok";
  const std::string target_topic = "/test/target/digit_ok";
  const std::string digit_topic = "/test/digit/digit_ok";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    digit_topic);
  auto io_node = std::make_shared<rclcpp::Node>("io_digit_ok");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  std::string received_target_id;
  auto digit_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    digit_topic,
    rclcpp::SensorDataQoS(),
    [&received_target_id](const dog_interfaces::msg::Target3DArray::ConstSharedPtr message) {
      if (!message->targets.empty()) {
        received_target_id = message->targets.front().target_id;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp1 = io_node->now();
  image_pub->publish(makePatternImage(stamp1));
  cloud_pub->publish(makePointCloud(stamp1));
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp2 = io_node->now();
  image_pub->publish(makePatternImage(stamp2));
  cloud_pub->publish(makePointCloud(stamp2));
  spinFor(executor, std::chrono::milliseconds(200));

  EXPECT_NE(received_target_id, "");
  EXPECT_NE(received_target_id, "no_feature");
  EXPECT_GE(perception_node->getDigitLatencySampleCount(), 1U);
  EXPECT_LT(perception_node->getDigitLatencyP95Ms(), 50.0);

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  (void)digit_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, LowConfidenceDigitIsDowngradedToNoFeature)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/low_conf";
  const std::string cloud_topic = "/test/cloud/low_conf";
  const std::string target_topic = "/test/target/low_conf";
  const std::string digit_topic = "/test/digit/low_conf";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    digit_topic,
    8,
    "best_effort",
    50,
    1);
  auto io_node = std::make_shared<rclcpp::Node>("io_low_conf");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  std::string received_target_id;
  auto digit_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    digit_topic,
    rclcpp::SensorDataQoS(),
    [&received_target_id](const dog_interfaces::msg::Target3DArray::ConstSharedPtr message) {
      if (!message->targets.empty()) {
        received_target_id = message->targets.front().target_id;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp = io_node->now();
  image_pub->publish(makeUniformImage(stamp, 20U));
  cloud_pub->publish(makePointCloud(stamp));
  spinFor(executor, std::chrono::milliseconds(160));

  EXPECT_EQ(received_target_id, "no_feature");

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  (void)digit_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, GlareFrameIsSuppressedToNoFeature)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/glare";
  const std::string cloud_topic = "/test/cloud/glare";
  const std::string target_topic = "/test/target/glare";
  const std::string digit_topic = "/test/digit/glare";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    digit_topic,
    8,
    "best_effort",
    50,
    1);
  auto io_node = std::make_shared<rclcpp::Node>("io_glare");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  std::string received_target_id;
  auto digit_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    digit_topic,
    rclcpp::SensorDataQoS(),
    [&received_target_id](const dog_interfaces::msg::Target3DArray::ConstSharedPtr message) {
      if (!message->targets.empty()) {
        received_target_id = message->targets.front().target_id;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp = io_node->now();
  image_pub->publish(makeBrightImage(stamp));
  cloud_pub->publish(makePointCloud(stamp));
  spinFor(executor, std::chrono::milliseconds(160));

  EXPECT_EQ(received_target_id, "no_feature");

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  (void)digit_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, DetectorTypeSwitchKeepsOutputContract)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/detector_switch";
  const std::string cloud_topic = "/test/cloud/detector_switch";
  const std::string target_topic = "/test/target/detector_switch";
  const std::string digit_topic = "/test/digit/detector_switch";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    digit_topic,
    8,
    "best_effort",
    50,
    1,
    "mean_intensity");
  auto io_node = std::make_shared<rclcpp::Node>("io_detector_switch");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  bool received = false;
  std::string received_target_id;
  std::string received_frame_id;
  float received_confidence = -1.0F;
  auto digit_sub = io_node->create_subscription<dog_interfaces::msg::Target3DArray>(
    digit_topic,
    rclcpp::SensorDataQoS(),
    [&received, &received_target_id, &received_frame_id,
      &received_confidence](const dog_interfaces::msg::Target3DArray::ConstSharedPtr message) {
      if (message->targets.empty()) {
        return;
      }
      const auto & target = message->targets.front();
      received = true;
      received_target_id = target.target_id;
      received_frame_id = target.header.frame_id;
      received_confidence = target.confidence;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp = io_node->now();
  image_pub->publish(makePatternImage(stamp));
  cloud_pub->publish(makePointCloud(stamp));
  spinFor(executor, std::chrono::milliseconds(200));

  EXPECT_TRUE(received);
  EXPECT_TRUE(std::regex_match(received_target_id, std::regex("^(digit_[0-9]|no_feature)$")));
  EXPECT_EQ(received_frame_id, "base_link");
  EXPECT_GE(received_confidence, 0.0F);
  EXPECT_LE(received_confidence, 1.0F);

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  (void)digit_sub;
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, ReliableQosWithoutPublisherMismatchRemainsCompatible)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  auto perception_node = createPerceptionNode(
    yaml_path,
    "/test/image/qos",
    "/test/cloud/qos",
    "/test/target/qos",
    "/test/digit/qos",
    8,
    "reliable");

  EXPECT_TRUE(perception_node->isQosCompatible());
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, CircularBufferOverwritesOldFramesAtCapacity)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/cache";
  const std::string cloud_topic = "/test/cloud/cache";
  const std::string target_topic = "/test/target/cache";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    "/test/digit/cache",
    2);
  auto io_node = std::make_shared<rclcpp::Node>("io_cache");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  for (int i = 0; i < 4; ++i) {
    const auto stamp = io_node->now();
    image_pub->publish(makeImage(stamp));
    cloud_pub->publish(makePointCloud(stamp));
    spinFor(executor, std::chrono::milliseconds(60));
  }

  EXPECT_EQ(perception_node->getFrameCacheSize(), 2U);

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, InvalidInputTriggersFailurePath)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/invalid";
  const std::string cloud_topic = "/test/cloud/invalid";
  const std::string target_topic = "/test/target/invalid";

  auto perception_node = createPerceptionNode(yaml_path, image_topic, cloud_topic, target_topic);
  auto io_node = std::make_shared<rclcpp::Node>("io_invalid");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stamp = io_node->now();
  image_pub->publish(makeImage(stamp, 0U, 0U));
  cloud_pub->publish(makePointCloud(stamp, 0U));
  spinFor(executor, std::chrono::milliseconds(150));

  EXPECT_GE(perception_node->getSolveFailureCount(), 1U);

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  std::filesystem::remove(yaml_path);
}

TEST_F(PerceptionNodeTest, StaleFramesAreDropped)
{
  const auto yaml_path = createTempExtrinsicsYaml();

  const std::string image_topic = "/test/image/stale";
  const std::string cloud_topic = "/test/cloud/stale";
  const std::string target_topic = "/test/target/stale";

  auto perception_node = createPerceptionNode(
    yaml_path,
    image_topic,
    cloud_topic,
    target_topic,
    "/test/digit/stale",
    8,
    "best_effort",
    5);
  auto io_node = std::make_shared<rclcpp::Node>("io_stale");

  auto image_pub = io_node->create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  auto cloud_pub =
    io_node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node);
  executor.add_node(io_node);
  spinFor(executor, std::chrono::milliseconds(80));

  const auto stale_stamp = io_node->now() - rclcpp::Duration::from_seconds(0.2);
  image_pub->publish(makeImage(stale_stamp));
  cloud_pub->publish(makePointCloud(stale_stamp));
  spinFor(executor, std::chrono::milliseconds(120));

  EXPECT_GE(perception_node->getDroppedFrameCount(), 1U);

  executor.remove_node(io_node);
  executor.remove_node(perception_node);
  std::filesystem::remove(yaml_path);
}