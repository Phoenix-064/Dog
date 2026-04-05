#pragma once

#include "dog_lifecycle/state_store.hpp"

#include <dog_interfaces/msg/target3_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

namespace dog_lifecycle
{

class LifecycleNode : public rclcpp::Node
{
public:
  /// @brief 使用默认节点选项构造生命周期节点。
  LifecycleNode();
  /// @brief 使用显式 ROS 节点选项构造生命周期节点。
  /// @param options ROS 节点选项。
  explicit LifecycleNode(const rclcpp::NodeOptions & options);
  /// @brief 关闭钩子，用于清理持久化恢复状态。
  ~LifecycleNode() override;

  /// @brief 持久化生命周期迁移信息，供下次启动恢复。
  /// @param task_phase 迁移任务阶段。
  /// @param target_state 迁移目标状态。
  /// @return 状态持久化成功时返回 true。
  bool PersistTransition(const std::string & task_phase, const std::string & target_state);
  /// @brief 删除已持久化的生命周期状态文件。
  /// @return 持久化状态已清理或本就不存在时返回 true。
  bool ClearPersistentState();

  /// @brief 为确定性单元测试注入原始抓取反馈。
  /// @param raw_feedback 模拟反馈负载。
  void InjectGraspFeedbackForTest(const std::string & raw_feedback);
  /// @brief 用于测试的熔断器开启状态查询。
  /// @return 熔断器当前开启时返回 true。
  bool IsBreakerOpen() const;
  /// @brief 用于测试的任务阻塞状态查询。
  /// @param task_id 任务标识。
  /// @return 当任务被熔断器状态阻塞时返回 true。
  bool IsTaskBlocked(const std::string & task_id) const;
  /// @brief 用于测试的连续空抓取计数查询。
  /// @return 当前连续空抓取计数。
  size_t GetConsecutiveEmptyCount() const;
  /// @brief 用于测试的降级待确认状态查询。
  /// @return 正在等待降级确认时返回 true。
  bool IsDegradePending() const;
  /// @brief 用于测试的熔断到降级时延指标查询。
  /// @return 最近一次测得时延（毫秒），不可用时为 -1。
  int64_t GetLastBreakerToDegradeLatencyMs() const;
  /// @brief 用于测试，暴露最近发布的恢复上下文负载。
  /// @return 恢复上下文负载字符串。
  std::string GetLastRecoveryContextForTest() const;
  /// @brief 用于测试的空转状态查询。
  /// @return 空转模式激活时返回 true。
  bool IsIdleSpinningForTest() const;
  /// @brief 用于测试，暴露最近系统模式负载。
  /// @return 最近发布的系统模式负载。
  std::string GetLastSystemModePayloadForTest() const;
  /// @brief 用于测试的重连待完成状态查询。
  /// @return 重连流程进行中时返回 true。
  bool IsReconnectPendingForTest() const;
  /// @brief 用于测试的重连迁移完成状态查询。
  /// @return 当重连迁移两个阶段均确认时返回 true。
  bool IsReconnectTransitionCompletedForTest() const;
  /// @brief 用于测试的受控降级模式状态查询。
  /// @return 节点进入受控降级模式时返回 true。
  bool IsControlledDegradeModeForTest() const;
  /// @brief 用于测试的重连恢复时延指标查询。
  /// @return 最近一次重连恢复时延（毫秒）。
  int64_t GetLastReconnectRecoveryLatencyMsForTest() const;
  /// @brief 用于测试的窗口内重启尝试次数查询。
  /// @return 滑动重启窗口中的重试次数。
  size_t GetRestartAttemptsInWindowForTest() const;

private:
  enum class GraspFeedbackType
  {
    kSuccess,
    kEmpty,
    kException,
    kUnknown,
  };

  struct GraspFeedbackEvent
  {
    std::string task_id;
    GraspFeedbackType type{GraspFeedbackType::kUnknown};
    rclcpp::Time stamp;
  };

  struct DegradeAckEvent
  {
    std::string task_id;
    std::string status;
    uint64_t request_id{0U};
    bool has_request_id{false};
  };

  /// @brief 消费抓取反馈流并驱动熔断/降级逻辑。
  /// @param msg 抓取反馈负载消息。
  void graspFeedbackCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief 消费降级确认消息。
  /// @param msg 降级确认负载。
  void degradeAckCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief 消费急停模式更新。
  /// @param msg 急停负载。
  void estopCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief 跟踪有效感知帧，用于心跳恢复判定。
  /// @param msg 感知目标帧。
  void validFrameCallback(const dog_interfaces::msg::Target3D::ConstSharedPtr msg);
  /// @brief 消费重连流程中的迁移状态确认。
  /// @param msg 迁移状态负载。
  void lifecycleTransitionStatusCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  /// @brief 校验输入目标帧是否满足健康数据条件。
  /// @param msg 目标帧消息。
  /// @return 帧内容有效时返回 true。
  bool isValidFrameMessage(const dog_interfaces::msg::Target3D::ConstSharedPtr & msg) const;
  /// @brief 将原始抓取反馈负载解析为结构化事件。
  /// @param payload 原始反馈负载。
  /// @param stamp 接收时间戳。
  /// @return 解析后的抓取反馈事件。
  GraspFeedbackEvent parseGraspFeedback(const std::string & payload, const rclcpp::Time & stamp) const;
  /// @brief 解析降级确认负载。
  /// @param payload 原始确认负载。
  /// @return 解析后的降级确认事件。
  DegradeAckEvent parseDegradeAck(const std::string & payload) const;
  /// @brief 将急停负载解析为布尔激活状态。
  /// @param payload 原始急停负载。
  /// @return 解析后的激活标记，歧义时返回 nullopt。
  std::optional<bool> parseEstopActive(const std::string & payload) const;
  /// @brief 将已解析抓取反馈应用到熔断/降级状态机。
  /// @param event 解析后的抓取反馈事件。
  void processGraspFeedback(const GraspFeedbackEvent & event);
  /// @brief 周期性心跳守护回调。
  void heartbeatTimerCallback();
  /// @brief 当近期无空抓取事件时重置熔断器计数。
  /// @param now 当前节点时间。
  void resetBreakerIfWindowElapsed(const rclcpp::Time & now);
  /// @brief 发布降级命令并启动降级超时跟踪。
  /// @param task_id 被阻塞的任务标识。
  /// @param reason 触发降级的原因。
  void publishDegradeCommand(const std::string & task_id, const char * reason);
  /// @brief 发布生命周期迁移命令负载。
  /// @param from_state 迁移源状态。
  /// @param to_state 迁移目标状态。
  /// @param reason 迁移原因。
  /// @param attempt 重连尝试序号。
  void publishLifecycleTransition(
    const char * from_state,
    const char * to_state,
    const char * reason,
    uint32_t attempt);
  /// @brief 发布与心跳相关的健康告警负载。
  /// @param reason 告警原因。
  /// @param attempts 重启尝试次数。
  /// @param since_last_valid_frame_ms 距上次有效帧的毫秒数。
  void publishHealthAlarm(const char * reason, uint32_t attempts, int64_t since_last_valid_frame_ms);
  /// @brief 加载持久化状态后发布启动恢复上下文。
  /// @param load_result 持久化状态加载结果。
  /// @param load_cost_ms 加载阶段时延（毫秒）。
  /// @param map_cost_ms 映射阶段时延（毫秒）。
  /// @param total_cost_ms 恢复发布总时延（毫秒）。
  void publishRecoveryContext(
    const StateStoreLoadResult & load_result,
    int64_t load_cost_ms,
    int64_t map_cost_ms,
    int64_t total_cost_ms);
  /// @brief 处理用于请求跟踪的降级超时到期。
  /// @param request_id 与超时定时器关联的请求 id。
  void degradeTimeoutCallback(uint64_t request_id);
  /// @brief 发布生命周期系统模式负载。
  /// @param mode 目标模式标记。
  /// @param reason 模式切换原因。
  void publishSystemMode(const std::string & mode, const char * reason);
  /// @brief 从分号分隔的键值对负载中提取指定键值。
  /// @param payload 输入负载。
  /// @param key 要查询的键名。
  /// @return 规范化后的值，或空字符串。
  static std::string parseKeyValuePayload(const std::string & payload, const std::string & key);
  /// @brief 将反馈类型枚举转换为稳定的小写文本。
  /// @param type 反馈枚举值。
  /// @return 反馈类型字符串。
  static const char * feedbackTypeToString(GraspFeedbackType type);
  /// @brief 通过去空白并转小写规范化标记。
  /// @param value 原始标记。
  /// @return 规范化后的标记。
  static std::string normalizeToken(const std::string & value);
  /// @brief 在持锁状态下检查任务当前是否被阻塞。
  /// @param task_id 任务标识。
  /// @return 被熔断逻辑阻塞时返回 true。
  bool isTaskBlockedLocked(const std::string & task_id) const;

  std::unique_ptr<IStateStore> state_store_;
  uint32_t supported_state_version_{1U};

  int64_t empty_grasp_threshold_{2};
  int64_t degrade_timeout_ms_{1000};
  int64_t breaker_reset_window_ms_{3000};
  int64_t heartbeat_timeout_ms_{2000};
  int64_t heartbeat_check_period_ms_{100};
  int64_t reconnect_min_interval_ms_{500};
  int64_t reconnect_pending_timeout_ms_{0};
  int64_t max_restart_attempts_{3};
  int64_t restart_window_ms_{10000};
  int64_t valid_frame_recovery_consecutive_{2};

  std::string grasp_feedback_topic_;
  std::string degrade_command_topic_;
  std::string degrade_ack_topic_;
  std::string recovery_context_topic_;
  std::string estop_topic_;
  std::string system_mode_topic_;
  std::string valid_frame_topic_;
  std::string lifecycle_transition_topic_;
  std::string lifecycle_transition_status_topic_;
  std::string health_alarm_topic_;
  std::string monitored_node_name_;

  mutable std::mutex breaker_mutex_;
  size_t consecutive_empty_count_{0U};
  std::string current_empty_task_id_;
  bool breaker_open_{false};
  std::string blocked_task_id_;
  bool has_last_empty_time_{false};
  rclcpp::Time last_empty_time_;
  bool degrade_pending_{false};
  uint64_t next_degrade_request_id_{0U};
  uint64_t pending_degrade_request_id_{0U};
  bool has_breaker_trigger_time_{false};
  rclcpp::Time breaker_trigger_time_;
  int64_t last_breaker_to_degrade_latency_ms_{-1};
  int64_t feedback_dedup_window_ms_{10};
  bool has_last_feedback_stamp_{false};
  rclcpp::Time last_feedback_stamp_;
  std::string last_feedback_task_id_;
  GraspFeedbackType last_feedback_type_{GraspFeedbackType::kUnknown};
  bool has_last_valid_frame_time_{false};
  rclcpp::Time last_valid_frame_time_;
  bool reconnect_pending_{false};
  bool has_last_reconnect_trigger_time_{false};
  rclcpp::Time last_reconnect_trigger_time_;
  rclcpp::Time last_reconnect_start_time_;
  int64_t last_reconnect_recovery_latency_ms_{-1};
  size_t reconnect_valid_frame_consecutive_{0U};
  bool reconnect_transition_inactive_confirmed_{false};
  bool reconnect_transition_active_confirmed_{false};
  uint32_t reconnect_transition_attempt_{0U};
  bool has_last_transition_verified_time_{false};
  rclcpp::Time last_transition_verified_time_;
  bool has_last_reconnect_recovery_time_{false};
  rclcpp::Time last_reconnect_recovery_time_;
  bool controlled_degrade_mode_{false};
  std::deque<rclcpp::Time> restart_attempt_timestamps_;

  mutable std::mutex recovery_context_mutex_;
  std::string last_recovery_context_payload_;
  bool idle_spinning_active_{false};
  std::string last_system_mode_payload_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grasp_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr degrade_ack_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr estop_sub_;
  rclcpp::Subscription<dog_interfaces::msg::Target3D>::SharedPtr valid_frame_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lifecycle_transition_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr degrade_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_context_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_transition_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_alarm_pub_;
  rclcpp::TimerBase::SharedPtr degrade_timeout_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace dog_lifecycle