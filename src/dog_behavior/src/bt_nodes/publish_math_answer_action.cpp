#include "dog_behavior/bt_nodes/publish_math_answer_action.hpp"

#include "dog_behavior/common/payload_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <thread>

namespace dog_behavior::bt_nodes
{

namespace
{

class ExpressionParser
{
public:
  explicit ExpressionParser(const std::string & expression)
  : expression_(expression), index_(0)
  {
  }

  bool parse(int64_t & result)
  {
    try {
      result = parseExpression();
      skipSpaces();
      return index_ == expression_.size();
    } catch (const std::exception &) {
      return false;
    }
  }

private:
  int64_t parseExpression()
  {
    int64_t value = parseTerm();
    while (true) {
      skipSpaces();
      if (match('+')) {
        value += parseTerm();
      } else if (match('-')) {
        value -= parseTerm();
      } else {
        return value;
      }
    }
  }

  int64_t parseTerm()
  {
    int64_t value = parseFactor();
    while (true) {
      skipSpaces();
      if (match('*')) {
        value *= parseFactor();
      } else if (match('/')) {
        const int64_t divisor = parseFactor();
        if (divisor == 0 || (value % divisor) != 0) {
          throw std::runtime_error("invalid division");
        }
        value /= divisor;
      } else {
        return value;
      }
    }
  }

  int64_t parseFactor()
  {
    skipSpaces();
    if (match('(')) {
      const int64_t value = parseExpression();
      if (!match(')')) {
        throw std::runtime_error("missing closing parenthesis");
      }
      return value;
    }

    if (match('-')) {
      return -parseFactor();
    }

    return parseNumber();
  }

  int64_t parseNumber()
  {
    skipSpaces();
    if (index_ >= expression_.size() ||
      !std::isdigit(static_cast<unsigned char>(expression_[index_])))
    {
      throw std::runtime_error("expected number");
    }

    int64_t value = 0;
    while (index_ < expression_.size() &&
      std::isdigit(static_cast<unsigned char>(expression_[index_])))
    {
      const int digit = expression_[index_] - '0';
      if (value > (std::numeric_limits<int64_t>::max() - digit) / 10) {
        throw std::runtime_error("integer overflow");
      }
      value = value * 10 + digit;
      ++index_;
    }
    return value;
  }

  bool match(const char expected)
  {
    skipSpaces();
    if (index_ < expression_.size() && expression_[index_] == expected) {
      ++index_;
      return true;
    }
    return false;
  }

  void skipSpaces()
  {
    while (index_ < expression_.size() &&
      std::isspace(static_cast<unsigned char>(expression_[index_])))
    {
      ++index_;
    }
  }

  const std::string & expression_;
  size_t index_;
};

bool parseBoolInput(const std::string & value)
{
  const auto normalized = utils::normalizeToken(value);
  return normalized == "true" || normalized == "1" || normalized == "yes";
}

}  // namespace

PublishMathAnswerAction::PublishMathAnswerAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PublishMathAnswerAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "waypoint_name",
      std::string("WayPointGoal3"),
      "current waypoint name"),
    BT::InputPort<std::string>(
      "required_waypoint_name",
      std::string("WayPointGoal3"),
      "required waypoint name"),
    BT::InputPort<std::string>("answer", std::string("42"), "fallback answer"),
    BT::InputPort<std::string>("topic_name", std::string("/math_answer"), "answer topic"),
    BT::InputPort<std::string>(
      "digit_result_topic",
      std::string("/target/digit_result"),
      "OCR expression topic"),
    BT::InputPort<int>("ocr_wait_timeout_ms", 500, "wait timeout for OCR expression"),
    BT::InputPort<std::string>(
      "allow_fallback_answer",
      std::string("false"),
      "allow configured answer when OCR is missing"),
  };
}

BT::NodeStatus PublishMathAnswerAction::tick()
{
  if (!config().blackboard) {
    return BT::NodeStatus::FAILURE;
  }

  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  } catch (const std::exception &) {
    return BT::NodeStatus::FAILURE;
  }

  if (!node_) {
    return BT::NodeStatus::FAILURE;
  }

  const auto waypoint_name_input = getInput<std::string>("waypoint_name");
  const auto required_waypoint_name_input = getInput<std::string>("required_waypoint_name");
  const auto answer_input = getInput<std::string>("answer");
  const auto topic_name_input = getInput<std::string>("topic_name");
  const auto digit_result_topic_input = getInput<std::string>("digit_result_topic");
  const auto ocr_wait_timeout_input = getInput<int>("ocr_wait_timeout_ms");
  const auto allow_fallback_input = getInput<std::string>("allow_fallback_answer");
  if (!waypoint_name_input || !required_waypoint_name_input || !answer_input || !topic_name_input ||
    !digit_result_topic_input || !ocr_wait_timeout_input || !allow_fallback_input)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (!canRunAtWaypoint(waypoint_name_input.value(), required_waypoint_name_input.value())) {
    return BT::NodeStatus::FAILURE;
  }

  if (!publisher_ || topic_name_ != topic_name_input.value()) {
    topic_name_ = topic_name_input.value();
    publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name_, rclcpp::QoS(10));
  }

  std::string answer = answer_input.value();
  std::string expression;
  if (ensureOcrSubscription(digit_result_topic_input.value())) {
    expression = waitForExpression(ocr_wait_timeout_input.value());
  } else if (!parseBoolInput(allow_fallback_input.value())) {
    return BT::NodeStatus::FAILURE;
  }

  if (!expression.empty()) {
    int64_t computed_answer = 0;
    if (!evaluateExpression(expression, computed_answer)) {
      return BT::NodeStatus::FAILURE;
    }
    answer = std::to_string(computed_answer);
  } else if (!parseBoolInput(allow_fallback_input.value())) {
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::String msg;
  msg.data = answer;
  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

bool PublishMathAnswerAction::canRunAtWaypoint(
  const std::string & waypoint_name,
  const std::string & required_waypoint_name) const
{
  return utils::normalizeToken(waypoint_name) == utils::normalizeToken(required_waypoint_name);
}

bool PublishMathAnswerAction::ensureOcrSubscription(const std::string & digit_result_topic)
{
  if (digit_result_topic.empty()) {
    return false;
  }

  if (digit_result_sub_ && digit_result_topic_ == digit_result_topic) {
    return true;
  }

  if (!ocr_listener_node_) {
    ocr_listener_node_ = std::make_shared<rclcpp::Node>("publish_math_answer_ocr_listener");
    ocr_executor_.add_node(ocr_listener_node_);
  }

  {
    std::lock_guard<std::mutex> lock(expression_mutex_);
    latest_expression_.clear();
  }
  digit_result_topic_ = digit_result_topic;
  digit_result_sub_ = ocr_listener_node_->create_subscription<dog_interfaces::msg::Target3DArray>(
    digit_result_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PublishMathAnswerAction::digitResultCallback, this, std::placeholders::_1));
  return static_cast<bool>(digit_result_sub_);
}

void PublishMathAnswerAction::digitResultCallback(
  const dog_interfaces::msg::Target3DArray::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  for (const auto & target : msg->targets) {
    const auto type = utils::parseKeyValuePayload(target.target_id, "type");
    if (utils::normalizeToken(type) != "math_expr") {
      continue;
    }

    const auto expression = utils::parseKeyValuePayload(target.target_id, "expr");
    if (expression.empty()) {
      continue;
    }

    std::lock_guard<std::mutex> lock(expression_mutex_);
    latest_expression_ = expression;
    return;
  }
}

std::string PublishMathAnswerAction::waitForExpression(const int timeout_ms)
{
  const auto timeout = std::chrono::milliseconds(std::max(0, timeout_ms));
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() <= deadline) {
    ocr_executor_.spin_some();
    {
      std::lock_guard<std::mutex> lock(expression_mutex_);
      if (!latest_expression_.empty()) {
        return latest_expression_;
      }
    }

    if (timeout_ms <= 0) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  return "";
}

bool PublishMathAnswerAction::evaluateExpression(const std::string & expression, int64_t & result) const
{
  if (expression.empty()) {
    return false;
  }

  for (const auto ch : expression) {
    if (!std::isdigit(static_cast<unsigned char>(ch)) && ch != '+' && ch != '-' && ch != '*' &&
      ch != '/' && ch != '(' && ch != ')' && !std::isspace(static_cast<unsigned char>(ch)))
    {
      return false;
    }
  }

  ExpressionParser parser(expression);
  return parser.parse(result);
}

}  // namespace dog_behavior::bt_nodes
