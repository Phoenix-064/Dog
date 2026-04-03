#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <dog_interfaces/msg/target3_d.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "dog_perception/target_3d_solver.hpp"

#include <boost/circular_buffer.hpp>

#include <memory>
#include <string>

namespace dog_perception
{

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  size_t getFrameCacheSize() const;
  size_t getDroppedFrameCount() const;
  size_t getSolvedFrameCount() const;
  size_t getSolveFailureCount() const;
  size_t getLatencySampleCount() const;
  bool isQosCompatible() const;
  double getLatencyP95Ms() const;
  double getEndToEndLatencyP95Ms() const;

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::PointCloud2>;

  struct CameraExtrinsics
  {
    std::string frame_id;
    std::string child_frame_id;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
  };

  static std::string getDefaultExtrinsicsYamlPath();
  CameraExtrinsics loadExtrinsicsFromYaml(const std::string & yaml_path) const;
  void initializeStaticTransform();
  void setupSynchronizedPipeline();
  void synchronizedCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg);
  bool evaluateRuntimeQosCompatibility();
  bool shouldDropAsStale(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg) const;

  struct FrameState
  {
    rclcpp::Time stamp;
    double latency_ms;
    bool published;
  };

  std::string image_topic_;
  std::string pointcloud_topic_;
  std::string target3d_topic_;
  int sync_queue_size_;
  int sync_slop_ms_;
  int stale_frame_timeout_ms_;
  int max_future_skew_ms_;
  int frame_cache_size_;
  std::string qos_reliability_;
  std::string solver_type_;
  std::string extrinsics_yaml_path_;
  CameraExtrinsics camera_extrinsics_;
  bool qos_compatible_;
  size_t dropped_frame_count_;
  size_t solved_frame_count_;
  size_t solve_failure_count_;

  boost::circular_buffer<FrameState> frame_history_;
  boost::circular_buffer<double> latency_samples_ms_;
  boost::circular_buffer<double> end_to_end_latency_samples_ms_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::unique_ptr<ITarget3DSolver> solver_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_subscriber_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;

  rclcpp::Publisher<dog_interfaces::msg::Target3D>::SharedPtr target3d_pub_;
};

}  // namespace dog_perception