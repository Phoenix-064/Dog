#pragma once

#include "dog_serial_bridge/serial_connection.hpp"

#include <dog_interfaces/action/navigate_waypoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace dog_serial_bridge
{

class NavTelemetrySerialNode : public rclcpp::Node
{
public:
  using NavigateWaypoint = dog_interfaces::action::NavigateWaypoint;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateWaypoint>;

  struct PoseCache
  {
    bool valid{false};
    geometry_msgs::msg::PoseStamped pose;
  };

  explicit NavTelemetrySerialNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::shared_ptr<SerialConnection> serial_connection = nullptr);
  ~NavTelemetrySerialNode() override;

  void SendShutdownArrivalFrame();

private:
  void declareParameters();
  void loadParameters();
  char decodeDelimiter(const std::string & text) const;
  void initializeSerial();
  void closeSerial();
  bool isSerialReady() const;
  void markSerialNotReady(const std::string & detail);
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  bool maybeReconnect();
  std::string buildFrame(const rclcpp::Time & stamp, const PoseCache & current, const PoseCache & goal, const std::string & event = "");
  std::string appendConfiguredNewline(const std::string & frame) const;
  void continuousSendTimerCallback();
  void updateLastGoalPose(const PoseCache & goal);
  bool getContinuousTelemetrySnapshot(PoseCache & current, PoseCache & goal) const;
  bool writeFrame(const std::string & frame, const char * log_prefix, std::string & detail);
  bool isValidGoal(const geometry_msgs::msg::PoseStamped & goal) const;
  void publishGoal(const geometry_msgs::msg::PoseStamped & goal);
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateWaypoint::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void executeGoal(const std::shared_ptr<GoalHandle> goal_handle);
  bool reserveGoalSlot();
  void releaseGoalSlot();
  bool waitForArrival(
    const std::chrono::steady_clock::time_point & deadline,
    const PoseCache & goal,
    std::string & detail);
  bool hasHostPoseArrived(const PoseCache & goal, std::string & detail) const;
  bool sendCurrentPoseStopFrames(const std::string & event, const char * log_prefix, int repeat_count);

  std::shared_ptr<SerialConnection> serial_connection_;
  mutable std::mutex serial_mutex_;
  std::mutex write_mutex_;
  bool serial_ready_;
  std::string serial_error_;
  SerialConfig serial_config_;
  bool write_newline_;
  int ack_timeout_ms_;
  int reconnect_period_ms_;
  bool continuous_send_enabled_;
  int continuous_send_period_ms_;
  std::string arrival_check_mode_;
  double arrival_xy_tolerance_m_;
  double arrival_yaw_tolerance_deg_;
  int arrival_check_period_ms_;
  rclcpp::Time next_reconnect_time_;

  mutable std::mutex pose_mutex_;
  PoseCache current_pose_;
  PoseCache last_goal_pose_;

  std::mutex goal_mutex_;
  bool goal_reserved_;
  bool send_shutdown_arrival_on_exit_;
  int shutdown_arrival_repeat_count_;
  int timeout_stop_repeat_count_;
  int timeout_stop_interval_ms_;
  bool shutdown_arrival_sent_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::TimerBase::SharedPtr continuous_send_timer_;
  rclcpp_action::Server<NavigateWaypoint>::SharedPtr action_server_;
};

}  // namespace dog_serial_bridge
