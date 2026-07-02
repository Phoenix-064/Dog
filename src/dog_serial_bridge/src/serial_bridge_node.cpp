#include "dog_serial_bridge/serial_bridge_node.hpp"

#include "dog_serial_bridge/serial_protocol.hpp"
#include "dog_serial_bridge/system_serial_connection.hpp"

#include <functional>
#include <utility>

namespace dog_serial_bridge
{

namespace
{

std::string trimLine(std::string line)
{
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.pop_back();
  }
  return line;
}

std::string escapeSerialPayload(const std::string & payload)
{
  std::string escaped;
  escaped.reserve(payload.size());
  for (const char value : payload) {
    switch (value) {
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped.push_back(value);
        break;
    }
  }
  return escaped;
}

}  // namespace

SerialBridgeNode::SerialBridgeNode(
  const rclcpp::NodeOptions & options,
  std::shared_ptr<SerialConnection> serial_connection)
: rclcpp::Node("dog_serial_bridge", options)
, serial_connection_(serial_connection ? std::move(serial_connection) : std::make_shared<SystemSerialConnection>())
, serial_ready_(false)
, write_newline_(true)
, ack_timeout_ms_(1500)
, feedback_period_ms_(200)
, stop_reader_(false)
, transaction_reserved_(false)
, next_pickup_sequence_(1U)
{
  declareParameters();
  loadParameters();

  grasp_feedback_pub_ = create_publisher<std_msgs::msg::String>(
    "/behavior/grasp_feedback",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable));

  using namespace std::placeholders;
  execute_server_ = rclcpp_action::create_server<ExecuteBehavior>(
    this,
    "/behavior/execute",
    std::bind(&SerialBridgeNode::handleExecuteGoal, this, _1, _2),
    std::bind(&SerialBridgeNode::handleExecuteCancel, this, _1),
    std::bind(&SerialBridgeNode::handleExecuteAccepted, this, _1));
  place_server_ = rclcpp_action::create_server<PlaceBoxes>(
    this,
    "/behavior/place_boxes",
    std::bind(&SerialBridgeNode::handlePlaceGoal, this, _1, _2),
    std::bind(&SerialBridgeNode::handlePlaceCancel, this, _1),
    std::bind(&SerialBridgeNode::handlePlaceAccepted, this, _1));

  feedback_timer_ = create_wall_timer(
    std::chrono::milliseconds(feedback_period_ms_),
    std::bind(&SerialBridgeNode::feedbackTimerCallback, this));

  initializeSerial();
}

SerialBridgeNode::~SerialBridgeNode()
{
  stopReaderThread();
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_connection_) {
      serial_connection_->close();
    }
    serial_ready_ = false;
  }
}

void SerialBridgeNode::declareParameters()
{
  declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  declare_parameter<int>("baud_rate", 115200);
  declare_parameter<int>("ack_timeout_ms", 1500);
  declare_parameter<int>("feedback_period_ms", 200);
  declare_parameter<bool>("write_newline", true);
  declare_parameter<std::string>("read_line_delimiter", "\\n");
}

void SerialBridgeNode::loadParameters()
{
  serial_config_.port = get_parameter("serial_port").as_string();
  serial_config_.baud_rate = get_parameter("baud_rate").as_int();
  ack_timeout_ms_ = get_parameter("ack_timeout_ms").as_int();
  feedback_period_ms_ = get_parameter("feedback_period_ms").as_int();
  write_newline_ = get_parameter("write_newline").as_bool();
  serial_config_.line_delimiter = decodeDelimiter(get_parameter("read_line_delimiter").as_string());
}

char SerialBridgeNode::decodeDelimiter(const std::string & text) const
{
  if (text == "\\n") {
    return '\n';
  }
  if (text == "\\r") {
    return '\r';
  }
  if (text == "\\t") {
    return '\t';
  }
  if (text.empty()) {
    return '\n';
  }
  return text.front();
}

void SerialBridgeNode::initializeSerial()
{
  std::string error;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_ready_ = serial_connection_ && serial_connection_->open(serial_config_, error);
    serial_error_ = serial_ready_ ? "" : (error.empty() ? "serial_open_failed" : error);
  }

  if (!serial_ready_) {
    RCLCPP_ERROR(
      get_logger(),
      "serial_open_failed port=%s baud=%d detail=%s",
      serial_config_.port.c_str(),
      serial_config_.baud_rate,
      serial_error_.c_str());
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "serial_ready port=%s baud=%d delimiter=%d",
    serial_config_.port.c_str(),
    serial_config_.baud_rate,
    static_cast<int>(serial_config_.line_delimiter));
  startReaderThread();
}

void SerialBridgeNode::startReaderThread()
{
  stop_reader_.store(false);
  reader_thread_ = std::thread([this]() { readerLoop(); });
}

void SerialBridgeNode::stopReaderThread()
{
  stop_reader_.store(true);
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_connection_) {
      serial_connection_->close();
    }
  }
  transaction_cv_.notify_all();
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

void SerialBridgeNode::readerLoop()
{
  while (!stop_reader_.load()) {
    SerialConnection::ReadResult result;
    std::shared_ptr<SerialConnection> serial_connection;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (!serial_ready_ || !serial_connection_) {
        break;
      }
      serial_connection = serial_connection_;
    }
    result = serial_connection->readLine(std::chrono::milliseconds(100));

    if (result.status == SerialConnection::ReadStatus::kTimeout) {
      continue;
    }
    if (result.status == SerialConnection::ReadStatus::kLine) {
      const auto line = trimLine(std::move(result.line));
      RCLCPP_INFO(
        get_logger(),
        "serial_rx port=%s data=%s",
        serial_config_.port.c_str(),
        escapeSerialPayload(line).c_str());
      dispatchSerialLine(line);
      continue;
    }

    if (!stop_reader_.load()) {
      const std::string detail =
        result.error.empty() ? "serial_read_error" : "serial_read_error:" + result.error;
      RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
      markSerialNotReady(detail);
      failActiveTransaction(detail);
    }
    break;
  }
}

void SerialBridgeNode::dispatchSerialLine(const std::string & line)
{
  if (line.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (active_transaction_.kind == TransactionKind::kNone) {
    RCLCPP_WARN(get_logger(), "serial_line_without_active_transaction line=%s", line.c_str());
    return;
  }

  const auto reply = classifyReply(line);
  if (active_transaction_.kind == TransactionKind::kExecuteBehavior) {
    if (reply == ReplyType::kPickSuccess) {
      active_transaction_.outcome = TransactionOutcome::kPickSuccess;
      active_transaction_.detail = "pick_success";
      transaction_cv_.notify_all();
      return;
    }
    if (reply == ReplyType::kPickFail) {
      active_transaction_.outcome = TransactionOutcome::kPickFail;
      active_transaction_.detail = "pick_fail";
      transaction_cv_.notify_all();
      return;
    }
  } else if (active_transaction_.kind == TransactionKind::kPlaceBoxes) {
    if (reply == ReplyType::kOk) {
      active_transaction_.outcome = TransactionOutcome::kOk;
      active_transaction_.detail = "place_ok";
      transaction_cv_.notify_all();
      return;
    }
  }

  RCLCPP_WARN(get_logger(), "serial_line_unmatched kind=%d line=%s", static_cast<int>(active_transaction_.kind), line.c_str());
}

void SerialBridgeNode::failActiveTransaction(const std::string & detail)
{
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (active_transaction_.kind == TransactionKind::kNone) {
    return;
  }
  active_transaction_.outcome = TransactionOutcome::kAborted;
  active_transaction_.detail = detail;
  transaction_cv_.notify_all();
}

std::string SerialBridgeNode::appendConfiguredNewline(const std::string & frame) const
{
  if (!write_newline_) {
    return frame;
  }
  return frame + '\n';
}

bool SerialBridgeNode::isSerialReady() const
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return serial_ready_;
}

void SerialBridgeNode::markSerialNotReady(const std::string & detail)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  serial_ready_ = false;
  serial_error_ = detail;
  if (serial_connection_) {
    serial_connection_->close();
  }
}

rclcpp_action::GoalResponse SerialBridgeNode::handleExecuteGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteBehavior::Goal> goal)
{
  (void)uuid;
  if (!goal || goal->behavior_name != kSupportedBehaviorName) {
    RCLCPP_WARN(get_logger(), "execute_goal_rejected behavior=%s", goal ? goal->behavior_name.c_str() : "");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!reserveTransactionSlot()) {
    RCLCPP_WARN(get_logger(), "execute_goal_rejected_busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse SerialBridgeNode::handlePlaceGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PlaceBoxes::Goal> goal)
{
  (void)uuid;
  if (!goal) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  PlacePayload parsed_payload;
  std::string error_detail;
  if (!parsePlacePayload(goal->payload, parsed_payload, error_detail)) {
    RCLCPP_WARN(
      get_logger(),
      "place_goal_rejected payload=%s detail=%s",
      goal->payload.c_str(),
      error_detail.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!reserveTransactionSlot()) {
    RCLCPP_WARN(get_logger(), "place_goal_rejected_busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SerialBridgeNode::handleExecuteCancel(const std::shared_ptr<ExecuteGoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (matchesActiveGoalLocked(goal_handle)) {
    active_transaction_.outcome = TransactionOutcome::kCanceled;
    active_transaction_.detail = "goal_canceled";
    transaction_cv_.notify_all();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse SerialBridgeNode::handlePlaceCancel(const std::shared_ptr<PlaceGoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (matchesActiveGoalLocked(goal_handle)) {
    active_transaction_.outcome = TransactionOutcome::kCanceled;
    active_transaction_.detail = "goal_canceled";
    transaction_cv_.notify_all();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SerialBridgeNode::handleExecuteAccepted(const std::shared_ptr<ExecuteGoalHandle> goal_handle)
{
  auto self = std::static_pointer_cast<SerialBridgeNode>(shared_from_this());
  std::thread([self, goal_handle]() { self->executePickupGoal(goal_handle); }).detach();
}

void SerialBridgeNode::handlePlaceAccepted(const std::shared_ptr<PlaceGoalHandle> goal_handle)
{
  const auto payload = goal_handle->get_goal()->payload;
  auto self = std::static_pointer_cast<SerialBridgeNode>(shared_from_this());
  std::thread([self, goal_handle, payload]() { self->executePlaceGoal(goal_handle, payload); }).detach();
}

void SerialBridgeNode::executePickupGoal(const std::shared_ptr<ExecuteGoalHandle> goal_handle)
{
  if (!isSerialReady()) {
    releaseReservation();
    auto result = std::make_shared<ExecuteBehavior::Result>();
    result->accepted = false;
    result->detail = "serial_not_ready";
    goal_handle->abort(result);
    return;
  }

  const auto pickup_sequence = next_pickup_sequence_.fetch_add(1U);
  if (!beginPickupTransaction(goal_handle, pickup_sequence)) {
    auto result = std::make_shared<ExecuteBehavior::Result>();
    result->accepted = false;
    result->detail = "transaction_begin_failed";
    goal_handle->abort(result);
    return;
  }

  std::string error;
  const auto frame = appendConfiguredNewline(buildPickupCommand());
  RCLCPP_INFO(
    get_logger(),
    "serial_tx port=%s data=%s",
    serial_config_.port.c_str(),
    escapeSerialPayload(frame).c_str());
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_connection_ || !serial_ready_ ||
      !serial_connection_->write(frame, error))
    {
      const auto detail = error.empty() ? "serial_write_error" : "serial_write_error:" + error;
      serial_ready_ = false;
      serial_error_ = detail;
      if (serial_connection_) {
        serial_connection_->close();
      }
      failActiveTransaction(detail);
    }
  }

  const auto wait_result = waitForTransaction(TransactionKind::kExecuteBehavior);
  if (wait_result.outcome == TransactionOutcome::kPickSuccess) {
    auto feedback = std_msgs::msg::String();
    feedback.data = buildPickupFeedback(wait_result.pickup_sequence, true);
    grasp_feedback_pub_->publish(feedback);

    auto result = std::make_shared<ExecuteBehavior::Result>();
    result->accepted = true;
    result->detail = "pick_success";
    goal_handle->succeed(result);
    return;
  }

  if (wait_result.outcome == TransactionOutcome::kPickFail) {
    auto feedback = std_msgs::msg::String();
    feedback.data = buildPickupFeedback(wait_result.pickup_sequence, false);
    grasp_feedback_pub_->publish(feedback);

    auto result = std::make_shared<ExecuteBehavior::Result>();
    result->accepted = false;
    result->detail = "pick_fail";
    goal_handle->succeed(result);
    return;
  }

  if (wait_result.outcome == TransactionOutcome::kCanceled) {
    auto result = std::make_shared<ExecuteBehavior::Result>();
    result->accepted = false;
    result->detail = wait_result.detail;
    goal_handle->canceled(result);
    return;
  }

  auto result = std::make_shared<ExecuteBehavior::Result>();
  result->accepted = false;
  result->detail = wait_result.detail;
  goal_handle->abort(result);
}

void SerialBridgeNode::executePlaceGoal(
  const std::shared_ptr<PlaceGoalHandle> goal_handle,
  std::string payload)
{
  if (!isSerialReady()) {
    releaseReservation();
    auto result = std::make_shared<PlaceBoxes::Result>();
    result->accepted = false;
    result->detail = "serial_not_ready";
    goal_handle->abort(result);
    return;
  }

  if (!beginPlaceTransaction(goal_handle)) {
    auto result = std::make_shared<PlaceBoxes::Result>();
    result->accepted = false;
    result->detail = "transaction_begin_failed";
    goal_handle->abort(result);
    return;
  }

  std::string error;
  const auto frame = appendConfiguredNewline(buildPlaceCommand(payload));
  RCLCPP_INFO(
    get_logger(),
    "serial_tx port=%s data=%s",
    serial_config_.port.c_str(),
    escapeSerialPayload(frame).c_str());
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_connection_ || !serial_ready_ ||
      !serial_connection_->write(frame, error))
    {
      const auto detail = error.empty() ? "serial_write_error" : "serial_write_error:" + error;
      serial_ready_ = false;
      serial_error_ = detail;
      if (serial_connection_) {
        serial_connection_->close();
      }
      failActiveTransaction(detail);
    }
  }

  const auto wait_result = waitForTransaction(TransactionKind::kPlaceBoxes);
  if (wait_result.outcome == TransactionOutcome::kOk) {
    auto result = std::make_shared<PlaceBoxes::Result>();
    result->accepted = true;
    result->detail = "place_ok";
    goal_handle->succeed(result);
    return;
  }

  if (wait_result.outcome == TransactionOutcome::kCanceled) {
    auto result = std::make_shared<PlaceBoxes::Result>();
    result->accepted = false;
    result->detail = wait_result.detail;
    goal_handle->canceled(result);
    return;
  }

  auto result = std::make_shared<PlaceBoxes::Result>();
  result->accepted = false;
  result->detail = wait_result.detail;
  goal_handle->abort(result);
}

bool SerialBridgeNode::reserveTransactionSlot()
{
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (transaction_reserved_ || active_transaction_.kind != TransactionKind::kNone) {
    return false;
  }
  transaction_reserved_ = true;
  return true;
}

void SerialBridgeNode::releaseReservation()
{
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  transaction_reserved_ = false;
}

bool SerialBridgeNode::beginPickupTransaction(
  const std::shared_ptr<ExecuteGoalHandle> & goal_handle,
  const uint64_t pickup_sequence)
{
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (!transaction_reserved_ || active_transaction_.kind != TransactionKind::kNone) {
    transaction_reserved_ = false;
    return false;
  }

  active_transaction_.kind = TransactionKind::kExecuteBehavior;
  active_transaction_.outcome = TransactionOutcome::kPending;
  active_transaction_.detail.clear();
  active_transaction_.feedback_state = "waiting_pick_result";
  active_transaction_.pickup_sequence = pickup_sequence;
  active_transaction_.execute_goal_handle = goal_handle;
  active_transaction_.place_goal_handle.reset();
  active_transaction_.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ack_timeout_ms_);
  transaction_reserved_ = false;
  return true;
}

bool SerialBridgeNode::beginPlaceTransaction(const std::shared_ptr<PlaceGoalHandle> & goal_handle)
{
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (!transaction_reserved_ || active_transaction_.kind != TransactionKind::kNone) {
    transaction_reserved_ = false;
    return false;
  }

  active_transaction_.kind = TransactionKind::kPlaceBoxes;
  active_transaction_.outcome = TransactionOutcome::kPending;
  active_transaction_.detail.clear();
  active_transaction_.feedback_state = "waiting_place_ack";
  active_transaction_.pickup_sequence = 0U;
  active_transaction_.execute_goal_handle.reset();
  active_transaction_.place_goal_handle = goal_handle;
  active_transaction_.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ack_timeout_ms_);
  transaction_reserved_ = false;
  return true;
}

SerialBridgeNode::TransactionWaitResult SerialBridgeNode::waitForTransaction(const TransactionKind kind)
{
  std::unique_lock<std::mutex> lock(transaction_mutex_);
  if (!transaction_cv_.wait_until(
      lock,
      active_transaction_.deadline,
      [&]() {
        return active_transaction_.kind != kind ||
               active_transaction_.outcome != TransactionOutcome::kPending ||
               stop_reader_.load();
      }))
  {
    TransactionWaitResult result;
    result.outcome = TransactionOutcome::kAborted;
    result.detail = "ack_timeout";
    result.pickup_sequence = active_transaction_.pickup_sequence;
    clearActiveTransactionLocked();
    return result;
  }

  TransactionWaitResult result;
  result.pickup_sequence = active_transaction_.pickup_sequence;
  if (active_transaction_.kind != kind) {
    result.outcome = TransactionOutcome::kAborted;
    result.detail = "transaction_interrupted";
    return result;
  }

  result.outcome = active_transaction_.outcome;
  result.detail = active_transaction_.detail.empty() ? "serial_aborted" : active_transaction_.detail;
  clearActiveTransactionLocked();
  return result;
}

void SerialBridgeNode::clearActiveTransactionLocked()
{
  active_transaction_ = ActiveTransaction{};
}

bool SerialBridgeNode::matchesActiveGoalLocked(const std::shared_ptr<ExecuteGoalHandle> & goal_handle) const
{
  return active_transaction_.kind == TransactionKind::kExecuteBehavior &&
         active_transaction_.execute_goal_handle == goal_handle;
}

bool SerialBridgeNode::matchesActiveGoalLocked(const std::shared_ptr<PlaceGoalHandle> & goal_handle) const
{
  return active_transaction_.kind == TransactionKind::kPlaceBoxes &&
         active_transaction_.place_goal_handle == goal_handle;
}

void SerialBridgeNode::feedbackTimerCallback()
{
  std::shared_ptr<ExecuteGoalHandle> execute_goal_handle;
  std::shared_ptr<PlaceGoalHandle> place_goal_handle;
  std::string feedback_state;

  {
    std::lock_guard<std::mutex> lock(transaction_mutex_);
    if (active_transaction_.kind == TransactionKind::kExecuteBehavior &&
      active_transaction_.execute_goal_handle &&
      active_transaction_.outcome == TransactionOutcome::kPending)
    {
      execute_goal_handle = active_transaction_.execute_goal_handle;
      feedback_state = active_transaction_.feedback_state;
    } else if (
      active_transaction_.kind == TransactionKind::kPlaceBoxes &&
      active_transaction_.place_goal_handle &&
      active_transaction_.outcome == TransactionOutcome::kPending)
    {
      place_goal_handle = active_transaction_.place_goal_handle;
      feedback_state = active_transaction_.feedback_state;
    }
  }

  if (execute_goal_handle) {
    auto feedback = std::make_shared<ExecuteBehavior::Feedback>();
    feedback->progress = 0.5F;
    feedback->state = feedback_state;
    execute_goal_handle->publish_feedback(feedback);
  } else if (place_goal_handle) {
    auto feedback = std::make_shared<PlaceBoxes::Feedback>();
    feedback->progress = 0.5F;
    feedback->state = feedback_state;
    place_goal_handle->publish_feedback(feedback);
  }
}

}  // namespace dog_serial_bridge
