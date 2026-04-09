#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>

#include <functional>
#include <string>

namespace dog_behavior
{

class BehaviorTree
{
public:
  using ActionCallback = std::function<bool(const std::string & behavior_name)>;
  using NavigateActionCallback = std::function<bool(const std::string & behavior_name)>;
  using PlaceholderCallback = std::function<void()>;

  explicit BehaviorTree(
    ActionCallback action_callback,
    NavigateActionCallback navigate_action_callback,
    PlaceholderCallback placeholder_callback,
    std::string tree_xml_file_path);

  /// @brief 执行行为树的固定触发路径。
  /// @param behavior_name 触发的行为名称。
  /// @return 当所有叶子都成功执行时返回 true。
  bool execute(const std::string & behavior_name);

private:
  BT::BehaviorTreeFactory factory_;
  std::string tree_xml_file_path_;
  ActionCallback action_callback_;
  NavigateActionCallback navigate_action_callback_;
  PlaceholderCallback placeholder_callback_;
};

}  // namespace dog_behavior
