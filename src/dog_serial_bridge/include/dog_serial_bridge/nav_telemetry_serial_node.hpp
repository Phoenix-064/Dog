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
  std::string buildFrame(const rclcpp::Time & stamp, const PoseCache & current, const PoseCache & goal);
  std::string appendConfiguredNewline(const std::string & frame) const;
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
  bool waitForArrival(const std::chrono::steady_clock::time_point & deadline, std::string & detail);

  std::shared_ptr<SerialConnection> serial_connection_;
  mutable std::mutex serial_mutex_;
  bool serial_ready_;
  std::string serial_error_;
  SerialConfig serial_config_;
  bool write_newline_;
  int ack_timeout_ms_;
  int reconnect_period_ms_;
  rclcpp::Time next_reconnect_time_;

  mutable std::mutex pose_mutex_;
  PoseCache current_pose_;
  uint64_t sequence_;

  std::mutex goal_mutex_;
  bool goal_reserved_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp_action::Server<NavigateWaypoint>::SharedPtr action_server_;
};

}  // namespace dog_serial_bridge
