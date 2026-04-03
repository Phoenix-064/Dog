#pragma once

#include <rclcpp/rclcpp.hpp>

namespace dog_lifecycle
{

class LifecycleNode : public rclcpp::Node
{
public:
  LifecycleNode();
};

}  // namespace dog_lifecycle