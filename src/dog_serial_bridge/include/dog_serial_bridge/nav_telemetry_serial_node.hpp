#pragma once

#include "dog_serial_bridge/serial_connection.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace dog_serial_bridge
{

class NavTelemetrySerialNode : public rclcpp::Node
{
public:
  struct PoseCache
  {
    bool valid{false};
    geometry_msgs::msg::PoseStamped pose;
  };

  explicit NavTelemetrySerialNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::shared_ptr<SerialConnection> serial_connection = nullptr);
  ~NavTelemetrySerialNode() override;

private:
  void declareParameters();
  void loadParameters();
  char decodeDelimiter(const std::string & text) const;
  void initializeSerial();
  void closeSerial();
  bool isSerialReady() const;
  void markSerialNotReady(const std::string & detail);
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void publishTimerCallback();
  bool maybeReconnect();
  std::string buildFrame(const rclcpp::Time & stamp, const PoseCache & current, const PoseCache & goal);
  std::string appendConfiguredNewline(const std::string & frame) const;

  std::shared_ptr<SerialConnection> serial_connection_;
  mutable std::mutex serial_mutex_;
  bool serial_ready_;
  std::string serial_error_;
  SerialConfig serial_config_;
  bool write_newline_;
  int publish_period_ms_;
  int reconnect_period_ms_;
  rclcpp::Time next_reconnect_time_;

  mutable std::mutex pose_mutex_;
  PoseCache current_pose_;
  PoseCache goal_pose_;
  uint64_t sequence_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace dog_serial_bridge
