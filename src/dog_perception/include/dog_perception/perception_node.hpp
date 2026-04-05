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
  /// @brief 构造感知节点并初始化同步感知流水线。
  /// @param options ROS 节点选项。
  explicit PerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief 获取当前缓存的帧历史大小。
  /// @return 缓存帧记录数量。
  size_t getFrameCacheSize() const;
  /// @brief 获取被丢弃的同步帧数量。
  /// @return 丢帧计数。
  size_t getDroppedFrameCount() const;
  /// @brief 获取成功求解目标帧数量。
  /// @return 成功求解帧计数。
  size_t getSolvedFrameCount() const;
  /// @brief 获取求解器失败次数。
  /// @return 求解失败计数。
  size_t getSolveFailureCount() const;
  /// @brief 获取同步链路时延采样数量。
  /// @return 时延样本计数。
  size_t getLatencySampleCount() const;
  /// @brief 获取数字识别时延采样数量。
  /// @return 数字时延样本计数。
  size_t getDigitLatencySampleCount() const;
  /// @brief 获取外推触发事件数量。
  /// @return 外推触发计数。
  size_t getExtrapolationTriggerCount() const;
  /// @brief 获取从外推恢复到同步求解的次数。
  /// @return 外推恢复计数。
  size_t getExtrapolationRecoveryCount() const;
  /// @brief 获取进入空转模式的事件数量。
  /// @return 空转触发计数。
  size_t getIdleSpinningTriggerCount() const;
  /// @brief 报告运行时 QoS 兼容状态。
  /// @return 当运行时发布端匹配配置的 QoS 可靠性时返回 true。
  bool isQosCompatible() const;
  /// @brief 报告生命周期模式是否为空转/降级。
  /// @return 节点处于空转模式时返回 true。
  bool isIdleSpinningMode() const;
  /// @brief 计算求解执行时延的 p95（毫秒）。
  /// @return P95 时延。
  double getLatencyP95Ms() const;
  /// @brief 计算从源时间戳到发布时间的端到端 p95 时延。
  /// @return 端到端 P95 时延。
  double getEndToEndLatencyP95Ms() const;
  /// @brief 计算数字识别时延的 p95（毫秒）。
  /// @return 数字识别 P95 时延。
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

  /// @brief 从包共享目录解析默认外参 YAML 路径。
  /// @return 默认外参 YAML 绝对路径。
  static std::string getDefaultExtrinsicsYamlPath();
  /// @brief 从 YAML 加载并校验相机外参。
  /// @param yaml_path 外参 YAML 文件路径。
  /// @return 解析后的相机外参。
  CameraExtrinsics loadExtrinsicsFromYaml(const std::string & yaml_path) const;
  /// @brief 外参加载后发布静态相机变换。
  void initializeStaticTransform();
  /// @brief 配置图像-点云同步处理流水线。
  void setupSynchronizedPipeline();
  /// @brief 处理同步后的图像与点云对。
  /// @param image_msg 同步图像消息。
  /// @param pointcloud_msg 同步点云消息。
  void synchronizedCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg);
  /// @brief 跟踪最新图像时间戳与接收时间。
  /// @param image_msg 输入图像消息。
  void imageStampCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  /// @brief 跟踪最新点云时间戳与接收时间。
  /// @param pointcloud_msg 输入点云消息。
  void pointcloudStampCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg);
  /// @brief 处理来自生命周期节点的模式变化。
  /// @param msg 生命周期模式负载。
  void lifecycleModeCallback(const std_msgs::msg::String::ConstSharedPtr & msg);
  /// @brief 用于掉流外推和空转发布的周期性看门狗。
  void watchdogCallback();
  /// @brief 判断当前流状态下是否应触发外推。
  /// @param current_time 当前节点时间。
  /// @param reason 输出触发原因标记。
  /// @return 应发布外推结果时返回 true。
  bool shouldTriggerExtrapolation(const rclcpp::Time & current_time, std::string & reason) const;
  /// @brief 基于近期位姿历史发布外推目标。
  /// @param current_time 发布时间戳。
  /// @param reason 触发原因标记。
  /// @return 外推目标发布成功时返回 true。
  bool publishExtrapolatedTarget(const rclcpp::Time & current_time, const std::string & reason);
  /// @brief 在空转模式发布占位目标位姿。
  /// @param current_time 发布时间戳。
  void publishIdleSpinningPose(const rclcpp::Time & current_time);
  /// @brief 运行数字识别器并发布数字结果目标。
  /// @param image_msg 输入图像消息。
  void processDigitRecognition(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  /// @brief 校验运行时端点 QoS 可靠性兼容性。
  /// @return 当运行时发布端匹配期望可靠性时返回 true。
  bool evaluateRuntimeQosCompatibility();
  /// @brief 检查同步帧是否过旧或未来时间偏差过大。
  /// @param image_msg 同步图像消息。
  /// @param pointcloud_msg 同步点云消息。
  /// @return 应按陈旧帧丢弃时返回 true。
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
  std::string digit_yolo_model_path_;
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