#include "dog_behavior/behavior_tree_node.hpp"

#include "dog_behavior/bt_nodes/check_system_mode.hpp"
#include "dog_behavior/bt_nodes/execute_behavior_action.hpp"
#include "dog_behavior/bt_nodes/navigate_to_pose_action.hpp"
#include "dog_behavior/bt_nodes/select_waypoint_action.hpp"
#include "dog_behavior/bt_nodes/wait_for_pose_condition.hpp"
#include "dog_behavior/common/payload_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <exception>
#include <utility>

namespace dog_behavior
{

namespace
{

std::string getBehaviorTreeXmlPath()
{
  const auto package_share = ament_index_cpp::get_package_share_directory("dog_behavior");
  return package_share + "/config/behavior_tree.xml";
}

std::string nodeStatusToString(const BT::NodeStatus status)
{
  switch (status) {
    case BT::NodeStatus::IDLE:
      return "idle";
    case BT::NodeStatus::RUNNING:
      return "running";
    case BT::NodeStatus::SUCCESS:
      return "success";
    case BT::NodeStatus::FAILURE:
      return "failure";
    default:
      return "unknown";
  }
}

}  // namespace

BehaviorTreeNode::BehaviorTreeNode()
: BehaviorTreeNode(rclcpp::NodeOptions())
{
}

BehaviorTreeNode::BehaviorTreeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dog_behavior_bt", options)
, tick_period_ms_(100)
, has_latest_pose_(false)
, tree_active_(false)
, ros_node_seeded_(false)
, tick_count_(0)
, system_mode_("normal")
, last_tick_status_("idle")
{
  const auto global_pose_topic = declare_parameter<std::string>("global_pose_topic", "/dog/global_pose");
  const auto localization_topic = declare_parameter<std::string>("localization_topic", "/localization/dog");
  default_frame_id_ = declare_parameter<std::string>("default_frame_id", "base_link");
  execute_behavior_trigger_topic_ = declare_parameter<std::string>(
    "execute_behavior_trigger_topic",
    "/behavior/execute_trigger");
  recovery_context_topic_ = declare_parameter<std::string>(
    "recovery_context_topic",
    "/lifecycle/recovery_context");
  system_mode_topic_ = declare_parameter<std::string>("system_mode_topic", "/lifecycle/system_mode");
  tree_xml_file_path_ = declare_parameter<std::string>("tree_xml_file_path", getBehaviorTreeXmlPath());
  tick_period_ms_ = declare_parameter<int>("bt_tick_period_ms", 100);
  const auto waypoints_file = declare_parameter<std::string>("waypoints_file", "");

  if (tick_period_ms_ <= 0) {
    RCLCPP_WARN(get_logger(), "Invalid bt_tick_period_ms=%d, fallback to 100", tick_period_ms_);
    tick_period_ms_ = 100;
  }

  if (!waypoints_file.empty()) {
    loadWaypoints(waypoints_file);
  }

  auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(20));
  pose_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  global_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(global_pose_topic, pose_qos);

  auto odom_qos = rclcpp::SensorDataQoS();
  odom_qos.keep_last(20);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    localization_topic,
    odom_qos,
    std::bind(&BehaviorTreeNode::odomCallback, this, std::placeholders::_1));

  execute_trigger_sub_ = create_subscription<std_msgs::msg::String>(
    execute_behavior_trigger_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable),
    std::bind(&BehaviorTreeNode::executeTriggerCallback, this, std::placeholders::_1));

  recovery_context_sub_ = create_subscription<std_msgs::msg::String>(
    recovery_context_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&BehaviorTreeNode::recoveryContextCallback, this, std::placeholders::_1));

  system_mode_sub_ = create_subscription<std_msgs::msg::String>(
    system_mode_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&BehaviorTreeNode::systemModeCallback, this, std::placeholders::_1));

  blackboard_ = BT::Blackboard::create();
  blackboard_->set("system_mode", system_mode_);
  blackboard_->set("recovery_context", std::string(""));
  blackboard_->set("behavior_name", std::string(""));
  blackboard_->set("has_current_pose", false);
  blackboard_->set("waypoints", waypoints_);
  blackboard_->set("waypoint_index", 0);

  registerBuiltinNodes();
  configureTree();

  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(tick_period_ms_),
    std::bind(&BehaviorTreeNode::timerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "BehaviorTreeNode initialized, localization_topic=%s, global_pose_topic=%s, trigger_topic=%s, tick_period_ms=%d, waypoints=%zu",
    localization_topic.c_str(),
    global_pose_topic.c_str(),
    execute_behavior_trigger_topic_.c_str(),
    tick_period_ms_,
    waypoints_.size());
}

void BehaviorTreeNode::registerBuiltinNodes()
{
  factory_.registerNodeType<bt_nodes::CheckSystemMode>("CheckSystemMode");
  factory_.registerNodeType<bt_nodes::WaitForPoseCondition>("WaitForPose");
  factory_.registerNodeType<bt_nodes::SelectWaypointAction>("SelectWaypoint");
  factory_.registerNodeType<bt_nodes::ExecuteBehaviorAction>("ExecuteBehaviorAction");
  factory_.registerNodeType<bt_nodes::NavigateToPoseAction>("NavigateToPoseAction");
}

void BehaviorTreeNode::configureTree()
{
  try {
    tree_ = factory_.createTreeFromFile(tree_xml_file_path_, blackboard_);
    RCLCPP_INFO(get_logger(), "Loaded behavior tree xml: %s", tree_xml_file_path_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to load behavior tree xml=%s, error=%s",
      tree_xml_file_path_.c_str(),
      e.what());
    throw;
  }
}

void BehaviorTreeNode::timerCallback()
{
  if (!ros_node_seeded_) {
    blackboard_->set("ros_node", std::static_pointer_cast<rclcpp::Node>(shared_from_this()));
    ros_node_seeded_ = true;
  }

  bool should_tick = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    blackboard_->set("system_mode", system_mode_);
    blackboard_->set("recovery_context", recovery_context_payload_);
    blackboard_->set("behavior_name", behavior_name_);
    blackboard_->set("has_current_pose", has_latest_pose_);
    if (has_latest_pose_) {
      blackboard_->set("current_pose", latest_pose_);
    }

    should_tick = tree_active_;
  }

  if (!should_tick) {
    return;
  }

  const auto status = tree_.tickRoot();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++tick_count_;
    last_tick_status_ = nodeStatusToString(status);
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      tree_active_ = false;
    }
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "BT tick completed status=%s",
    nodeStatusToString(status).c_str());
}

void BehaviorTreeNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = msg->header.frame_id;
  pose_msg.pose = msg->pose.pose;

  if (pose_msg.header.frame_id.empty()) {
    if (default_frame_id_.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Drop pose publish because input frame_id is empty and default_frame_id is not configured");
      return;
    }
    pose_msg.header.frame_id = default_frame_id_;
  }

  if (!utils::isFinitePose(pose_msg.pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop pose publish because pose contains non-finite values");
    return;
  }

  if (!utils::hasValidQuaternionNorm(pose_msg.pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Drop pose publish because quaternion norm is invalid");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_pose_ = pose_msg;
    has_latest_pose_ = true;
  }

  global_pose_pub_->publish(pose_msg);
}

void BehaviorTreeNode::executeTriggerCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  behavior_name_ = msg ? msg->data : "";
  tree_active_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Received behavior trigger: behavior_name=%s",
    behavior_name_.c_str());
}

void BehaviorTreeNode::systemModeCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto mode = utils::normalizeToken(utils::parseKeyValuePayload(msg->data, "mode"));
  if (mode.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  system_mode_ = mode;
}

void BehaviorTreeNode::recoveryContextCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  recovery_context_payload_ = msg->data;
}

void BehaviorTreeNode::loadWaypoints(const std::string & file_path)
{
  try {
    const auto config = YAML::LoadFile(file_path);
    const auto waypoints_node = config["waypoints"];
    if (!waypoints_node || !waypoints_node.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "Invalid waypoints file format: %s", file_path.c_str());
      return;
    }

    for (const auto & wp_node : waypoints_node) {
      Waypoint wp;
      wp.name = wp_node["name"].as<std::string>("");
      wp.x = wp_node["x"].as<double>(0.0);
      wp.y = wp_node["y"].as<double>(0.0);
      wp.z = wp_node["z"].as<double>(0.0);
      wp.yaw = wp_node["yaw"].as<double>(0.0);
      waypoints_.push_back(wp);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(), file_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load waypoints from %s: %s", file_path.c_str(), e.what());
  }
}

int BehaviorTreeNode::TickCountForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return tick_count_;
}

std::string BehaviorTreeNode::LastTickStatusForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_tick_status_;
}

std::string BehaviorTreeNode::SystemModeForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return system_mode_;
}

std::string BehaviorTreeNode::BehaviorNameForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return behavior_name_;
}

bool BehaviorTreeNode::HasLatestPoseForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return has_latest_pose_;
}

bool BehaviorTreeNode::IsTreeActiveForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return tree_active_;
}

size_t BehaviorTreeNode::WaypointCountForTest() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return waypoints_.size();
}

}  // namespace dog_behavior