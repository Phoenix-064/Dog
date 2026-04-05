#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <dog_interfaces/msg/target3_d.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "dog_perception/target_3d_solver.hpp"
#include "dog_perception/digit_recognizer.hpp"

#include <boost/circular_buffer.hpp>

#include <memory>
#include <string>

namespace dog_perception
{

class PerceptionNode : public rclcpp::Node
{
public:
  /// @brief Construct perception node and initialize synchronized perception pipeline.
  /// @param options ROS node options.
  explicit PerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Get current cached frame-history size.
  /// @return Number of cached frame records.
  size_t getFrameCacheSize() const;
  /// @brief Get number of dropped synchronized frames.
  /// @return Dropped frame count.
  size_t getDroppedFrameCount() const;
  /// @brief Get number of successfully solved target frames.
  /// @return Solved frame count.
  size_t getSolvedFrameCount() const;
  /// @brief Get number of solver failures.
  /// @return Solve failure count.
  size_t getSolveFailureCount() const;
  /// @brief Get number of synchronized-latency samples.
  /// @return Latency sample count.
  size_t getLatencySampleCount() const;
  /// @brief Get number of digit-recognition latency samples.
  /// @return Digit latency sample count.
  size_t getDigitLatencySampleCount() const;
  /// @brief Get number of extrapolation trigger events.
  /// @return Extrapolation trigger count.
  size_t getExtrapolationTriggerCount() const;
  /// @brief Get number of recoveries from extrapolation to synchronized solving.
  /// @return Extrapolation recovery count.
  size_t getExtrapolationRecoveryCount() const;
  /// @brief Get number of idle-spinning mode enter events.
  /// @return Idle-spinning trigger count.
  size_t getIdleSpinningTriggerCount() const;
  /// @brief Report runtime QoS compatibility state.
  /// @return True when runtime publishers match configured QoS reliability.
  bool isQosCompatible() const;
  /// @brief Report whether lifecycle mode is idle-spinning/degraded.
  /// @return True when node is in idle-spinning mode.
  bool isIdleSpinningMode() const;
  /// @brief Compute p95 solver execution latency in milliseconds.
  /// @return P95 latency.
  double getLatencyP95Ms() const;
  /// @brief Compute p95 end-to-end latency from source timestamp to publish time.
  /// @return P95 end-to-end latency.
  double getEndToEndLatencyP95Ms() const;
  /// @brief Compute p95 digit-recognition latency in milliseconds.
  /// @return P95 digit latency.
  double getDigitLatencyP95Ms() const;

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

  /// @brief Resolve default extrinsics YAML path from package share directory.
  /// @return Default extrinsics YAML absolute path.
  static std::string getDefaultExtrinsicsYamlPath();
  /// @brief Load and validate camera extrinsics from YAML.
  /// @param yaml_path Path to extrinsics YAML file.
  /// @return Parsed camera extrinsics.
  CameraExtrinsics loadExtrinsicsFromYaml(const std::string & yaml_path) const;
  /// @brief Publish static camera transform after extrinsics are loaded.
  void initializeStaticTransform();
  /// @brief Configure synchronized image-pointcloud processing pipeline.
  void setupSynchronizedPipeline();
  /// @brief Process synchronized image-pointcloud pair.
  /// @param image_msg Synchronized image message.
  /// @param pointcloud_msg Synchronized pointcloud message.
  void synchronizedCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg);
  /// @brief Track latest image timestamp and receive time.
  /// @param image_msg Incoming image message.
  void imageStampCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  /// @brief Track latest pointcloud timestamp and receive time.
  /// @param pointcloud_msg Incoming pointcloud message.
  void pointcloudStampCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg);
  /// @brief Handle lifecycle mode changes from lifecycle node.
  /// @param msg Lifecycle mode payload.
  void lifecycleModeCallback(const std_msgs::msg::String::ConstSharedPtr & msg);
  /// @brief Periodic watchdog for dropout extrapolation and idle-spinning publish.
  void watchdogCallback();
  /// @brief Determine whether extrapolation should trigger for current stream state.
  /// @param current_time Current node time.
  /// @param reason Output trigger reason token.
  /// @return True when extrapolation should be published.
  bool shouldTriggerExtrapolation(const rclcpp::Time & current_time, std::string & reason) const;
  /// @brief Publish extrapolated target from recent pose history.
  /// @param current_time Publish timestamp.
  /// @param reason Trigger reason token.
  /// @return True when extrapolated target is published.
  bool publishExtrapolatedTarget(const rclcpp::Time & current_time, const std::string & reason);
  /// @brief Publish placeholder target pose in idle-spinning mode.
  /// @param current_time Publish timestamp.
  void publishIdleSpinningPose(const rclcpp::Time & current_time);
  /// @brief Run digit recognizer and publish digit result target.
  /// @param image_msg Input image message.
  void processDigitRecognition(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  /// @brief Validate runtime endpoint QoS reliability compatibility.
  /// @return True when runtime publishers match expected reliability.
  bool evaluateRuntimeQosCompatibility();
  /// @brief Check whether synchronized frame is stale or too far in future.
  /// @param image_msg Synchronized image message.
  /// @param pointcloud_msg Synchronized pointcloud message.
  /// @return True when frame should be dropped as stale.
  bool shouldDropAsStale(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg) const;

  struct FrameState
  {
    rclcpp::Time stamp;
    double latency_ms;
    bool published;
  };

  struct PoseState
  {
    rclcpp::Time stamp;
    dog_interfaces::msg::Target3D target;
  };

  std::string image_topic_;
  std::string pointcloud_topic_;
  std::string target3d_topic_;
  std::string digit_result_topic_;
  int sync_queue_size_;
  int sync_slop_ms_;
  int stale_frame_timeout_ms_;
  int max_future_skew_ms_;
  int frame_cache_size_;
  int single_side_dropout_timeout_ms_;
  int extrapolation_watchdog_ms_;
  int extrapolation_max_window_ms_;
  int extrapolation_min_interval_ms_;
  int idle_spinning_publish_ms_;
  std::string qos_reliability_;
  std::string solver_type_;
  std::string digit_recognizer_type_;
  std::string lifecycle_mode_topic_;
  std::string extrinsics_yaml_path_;
  int digit_roi_x_;
  int digit_roi_y_;
  int digit_roi_width_;
  int digit_roi_height_;
  double digit_min_confidence_;
  double digit_glare_brightness_threshold_;
  double digit_glare_ratio_threshold_;
  int digit_temporal_window_;
  int digit_temporal_confirm_count_;
  CameraExtrinsics camera_extrinsics_;
  bool qos_compatible_;
  bool idle_spinning_mode_;
  bool extrapolation_active_;
  bool has_last_image_stamp_;
  bool has_last_pointcloud_stamp_;
  bool has_last_extrapolation_pub_time_;
  bool has_last_idle_publish_time_;
  bool has_mode_enter_time_;
  size_t dropped_frame_count_;
  size_t solved_frame_count_;
  size_t solve_failure_count_;
  size_t extrapolation_trigger_count_;
  size_t extrapolation_recovery_count_;
  size_t idle_spinning_trigger_count_;

  rclcpp::Time last_image_stamp_;
  rclcpp::Time last_pointcloud_stamp_;
  rclcpp::Time last_image_receive_time_;
  rclcpp::Time last_pointcloud_receive_time_;
  rclcpp::Time last_extrapolation_pub_time_;
  rclcpp::Time last_idle_publish_time_;
  rclcpp::Time mode_enter_time_;

  boost::circular_buffer<FrameState> frame_history_;
  boost::circular_buffer<PoseState> pose_history_;
  boost::circular_buffer<double> latency_samples_ms_;
  boost::circular_buffer<double> end_to_end_latency_samples_ms_;
  boost::circular_buffer<int> digit_label_history_;
  boost::circular_buffer<double> digit_latency_samples_ms_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::unique_ptr<ITarget3DSolver> solver_;
  std::unique_ptr<IDigitRecognizer> digit_recognizer_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_subscriber_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_stamp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_stamp_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lifecycle_mode_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Publisher<dog_interfaces::msg::Target3D>::SharedPtr target3d_pub_;
  rclcpp::Publisher<dog_interfaces::msg::Target3D>::SharedPtr digit_result_pub_;
};

}  // namespace dog_perception