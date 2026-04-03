#pragma once

#include "dog_lifecycle/state_store.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace dog_lifecycle
{

class LifecycleNode : public rclcpp::Node
{
public:
  LifecycleNode();
  ~LifecycleNode() override;

  bool PersistTransition(const std::string & task_phase, const std::string & target_state);
  bool ClearPersistentState();

private:
  std::unique_ptr<IStateStore> state_store_;
  uint32_t supported_state_version_{1U};
};

}  // namespace dog_lifecycle