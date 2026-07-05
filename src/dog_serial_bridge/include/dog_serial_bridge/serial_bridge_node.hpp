#pragma once

#include "dog_serial_bridge/serial_connection.hpp"

#include <dog_interfaces/action/execute_behavior.hpp>
#include <dog_interfaces/action/place_boxes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace dog_serial_bridge
{

class SerialBridgeNode : public rclcpp::Node
{
public:
  using ExecuteBehavior = dog_interfaces::action::ExecuteBehavior;
  using PlaceBoxes = dog_interfaces::action::PlaceBoxes;
  using ExecuteGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteBehavior>;
  using PlaceGoalHandle = rclcpp_action::ServerGoalHandle<PlaceBoxes>;

  explicit SerialBridgeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::shared_ptr<SerialConnection> serial_connection = nullptr);
  ~SerialBridgeNode() override;

private:
  enum class TransactionKind
  {
    kNone,
    kExecuteBehavior,
    kPlaceBoxes,
  };

  enum class TransactionOutcome
  {
    kPending,
    kPickSuccess,
    kPickFail,
    kOk,
    kCanceled,
    kAborted,
  };

  struct ActiveTransaction
  {
    TransactionKind kind{TransactionKind::kNone};
    TransactionOutcome outcome{TransactionOutcome::kPending};
    std::string detail;
    std::string feedback_state;
    uint64_t pickup_sequence{0U};
    std::shared_ptr<ExecuteGoalHandle> execute_goal_handle;
    std::shared_ptr<PlaceGoalHandle> place_goal_handle;
    std::chrono::steady_clock::time_point deadline{};
  };

  struct TransactionWaitResult
  {
    TransactionOutcome outcome{TransactionOutcome::kAborted};
    std::string detail;
    uint64_t pickup_sequence{0U};
  };

  void declareParameters();
  void loadParameters();
  char decodeDelimiter(const std::string & text) const;
  void initializeSerial();
  void startReaderThread();
  void stopReaderThread();
  void readerLoop();
  void dispatchSerialLine(const std::string & line);
  void failActiveTransaction(const std::string & detail);
  std::string appendConfiguredNewline(const std::string & frame) const;
  bool isSerialReady() const;
  void markSerialNotReady(const std::string & detail);

  rclcpp_action::GoalResponse handleExecuteGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteBehavior::Goal> goal);
  rclcpp_action::GoalResponse handlePlaceGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlaceBoxes::Goal> goal);
  rclcpp_action::CancelResponse handleExecuteCancel(const std::shared_ptr<ExecuteGoalHandle> goal_handle);
  rclcpp_action::CancelResponse handlePlaceCancel(const std::shared_ptr<PlaceGoalHandle> goal_handle);
  void handleExecuteAccepted(const std::shared_ptr<ExecuteGoalHandle> goal_handle);
  void handlePlaceAccepted(const std::shared_ptr<PlaceGoalHandle> goal_handle);
  void executePickupGoal(const std::shared_ptr<ExecuteGoalHandle> goal_handle);
  void executePlaceGoal(const std::shared_ptr<PlaceGoalHandle> goal_handle, std::string payload);

  bool reserveTransactionSlot();
  void releaseReservation();
  bool beginPickupTransaction(const std::shared_ptr<ExecuteGoalHandle> & goal_handle, uint64_t pickup_sequence);
  bool beginPlaceTransaction(const std::shared_ptr<PlaceGoalHandle> & goal_handle);
  TransactionWaitResult waitForTransaction(TransactionKind kind);
  void clearActiveTransactionLocked();
  bool matchesActiveGoalLocked(const std::shared_ptr<ExecuteGoalHandle> & goal_handle) const;
  bool matchesActiveGoalLocked(const std::shared_ptr<PlaceGoalHandle> & goal_handle) const;
  void feedbackTimerCallback();

  std::shared_ptr<SerialConnection> serial_connection_;
  mutable std::mutex serial_mutex_;
  bool serial_ready_;
  std::string serial_error_;
  SerialConfig serial_config_;
  bool write_newline_;
  int ack_timeout_ms_;
  int feedback_period_ms_;

  rclcpp_action::Server<ExecuteBehavior>::SharedPtr execute_server_;
  rclcpp_action::Server<PlaceBoxes>::SharedPtr place_server_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  std::atomic<bool> stop_reader_;
  std::thread reader_thread_;

  mutable std::mutex transaction_mutex_;
  std::condition_variable transaction_cv_;
  ActiveTransaction active_transaction_;
  bool transaction_reserved_;
  std::atomic<uint64_t> next_pickup_sequence_;
};

}  // namespace dog_serial_bridge
