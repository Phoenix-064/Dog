#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <dog_behavior/behavior_tree.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <dog_interfaces/action/execute_behavior.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <unordered_set>
#include <vector>

namespace dog_behavior
{

struct Waypoint {
  std::string name;
  double x;
  double y;
  double z;
  double yaw;
};

class BehaviorNode : public rclcpp::Node
{
public:
  /// @brief 使用默认选项构造行为节点。
  BehaviorNode();
  /// @brief 使用显式 ROS 节点选项构造行为节点。
  /// @param options 用于节点初始化的 ROS 节点选项。
  explicit BehaviorNode(const rclcpp::NodeOptions & options);

  /// @brief 使用最新缓存位姿触发 ExecuteBehavior 动作目标。
  /// @param behavior_name 转发给动作服务端的行为标识。
  /// @return 当目标请求被接受并可发送时返回 true。
  bool triggerExecuteBehavior(const std::string & behavior_name);
  /// @brief 使用最新缓存位姿触发内部导航执行动作目标。
  /// @param behavior_name 触发导航行为时携带的语义标识。
  /// @return 当导航目标请求被接受并可发送时返回 true。
  bool triggerNavigateGoal(const std::string & behavior_name);
  /// @brief 以字符串标记获取当前执行状态。
  /// @return 小写的执行状态文本。
  std::string getExecutionState() const;
  /// @brief 测试辅助函数，检查恢复后任务阶段是否被阻塞。
  /// @param task_phase 需要查询的任务阶段标记。
  /// @return 若已跟踪恢复完成阶段则返回 true。
  bool IsTaskPhaseRecoveredForTest(const std::string & task_phase) const;
  /// @brief 测试辅助函数，报告空转模式是否激活。
  /// @return 当行为执行受空转模式限制时返回 true。
  bool IsIdleSpinningForTest() const;

private:
  using ExecuteBehavior = dog_interfaces::action::ExecuteBehavior;
  using ExecuteBehaviorGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteBehavior>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  enum class ExecutionState
  {
    kIdle,
    kWaitingServer,
    kServerUnavailable,
    kSendingGoal,
    kRunning,
    kSucceeded,
    kFailed,
    kRejected,
    kTimeout,
  };

  /// @brief 将里程计更新转换为全局位姿发布并缓存最新位姿。
  /// @param msg 输入的里程计消息。
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  /// @brief 处理行为触发消息并转发到动作分发流程。
  /// @param msg 输入的触发负载。
  void executeTriggerCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief 更新由恢复上下文推导出的执行过滤条件。
  /// @param msg 恢复上下文负载。
  void recoveryContextCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief 响应生命周期系统模式切换。
  /// @param msg 系统模式负载。
  void systemModeCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief 周期性探测动作服务端就绪状态。
  void actionServerWaitTimerCallback();
  /// @brief 当动作反馈超时失效时取消活动目标。
  void feedbackWatchdogTimerCallback();
  /// @brief 处理动作目标的接受或拒绝响应。
  /// @param goal_handle 动作客户端返回的目标句柄。
  void goalResponseCallback(ExecuteBehaviorGoalHandle::SharedPtr goal_handle);
  /// @brief 跟踪动作反馈并刷新看门狗时间戳。
  /// @param goal_handle 与反馈关联的目标句柄。
  /// @param feedback 动作反馈负载。
  void feedbackCallback(
    ExecuteBehaviorGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const ExecuteBehavior::Feedback> feedback);
  /// @brief 处理动作终态结果并更新执行状态。
  /// @param result 封装后的动作结果。
  void resultCallback(const ExecuteBehaviorGoalHandle::WrappedResult & result);
  /// @brief 处理导航动作目标的接受或拒绝响应。
  /// @param goal_handle 导航动作客户端返回的目标句柄。
  void navigateGoalResponseCallback(NavigateGoalHandle::SharedPtr goal_handle);
  /// @brief 跟踪导航动作反馈并刷新看门狗时间戳。
  /// @param goal_handle 与反馈关联的目标句柄。
  /// @param feedback 导航动作反馈负载。
  void navigateFeedbackCallback(
    NavigateGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  /// @brief 处理导航动作终态结果并更新执行状态。
  /// @param result 导航动作封装结果。
  void navigateResultCallback(const NavigateGoalHandle::WrappedResult & result);
  /// @brief 检查当前内部状态是否允许发送新目标。
  /// @return 当动作服务端就绪且无在途目标时返回 true。
  bool canSendGoalLocked() const;
  /// @brief 将内部执行状态枚举转换为稳定的文本表示。
  /// @param state 执行状态枚举值。
  /// @return 小写状态标记。
  std::string executionStateToString(ExecutionState state) const;

  /// @brief 从 YAML 文件加载预设航点列表。
  /// @param file_path 航点 YAML 文件的绝对路径。
  void loadWaypoints(const std::string & file_path);

  /// @brief 将航点转换为 PoseStamped 消息。
  /// @param wp 待转换的航点。
  /// @return 包含位置和朝向的 PoseStamped。
  geometry_msgs::msg::PoseStamped waypointToPoseStamped(const Waypoint & wp) const;

  std::string default_frame_id_;
  std::string execute_behavior_action_name_;
  std::string navigate_execute_action_name_;
  std::string execute_behavior_trigger_topic_;
  std::string recovery_context_topic_;
  std::string system_mode_topic_;
  double action_server_wait_timeout_sec_;
  double feedback_timeout_sec_;

  mutable std::mutex state_mutex_;
  ExecutionState execution_state_;
  bool action_server_ready_;
  bool action_goal_pending_;
  bool action_goal_active_;
  bool navigate_goal_pending_;
  bool navigate_goal_active_;
  bool navigate_server_ready_;
  bool cancel_due_to_idle_spinning_;
  bool idle_spinning_mode_active_;
  bool has_latest_pose_;
  std::unordered_set<std::string> recovered_completed_task_phases_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  rclcpp::Time action_server_wait_start_time_;
  rclcpp::Time navigate_server_wait_start_time_;
  rclcpp::Time last_feedback_time_;
  rclcpp::Time navigate_last_feedback_time_;

  std::string match_type_;
  std::vector<Waypoint> waypoints_;
  size_t current_waypoint_index_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execute_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_context_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
  rclcpp::TimerBase::SharedPtr action_server_wait_timer_;
  rclcpp::TimerBase::SharedPtr navigate_server_wait_timer_;
  rclcpp::TimerBase::SharedPtr feedback_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr navigate_feedback_watchdog_timer_;
  rclcpp_action::Client<ExecuteBehavior>::SharedPtr execute_behavior_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
  ExecuteBehaviorGoalHandle::SharedPtr active_goal_handle_;
  NavigateGoalHandle::SharedPtr active_navigate_goal_handle_;
  BehaviorTree behavior_tree_;
};

}  // namespace dog_behavior