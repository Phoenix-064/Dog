#pragma once

#include <array>
#include <string>
#include <vector>

namespace dog_behavior::bt_nodes
{

enum class BoxType
{
  Food,
  Tool,
  Instrument,
  Medical,
  Unknown
};

struct PlaceSelection
{
  std::array<int, 4> group_indices{0, 1, 2, 3};
  std::vector<std::string> selected_boxes;
  std::vector<int> local_indices;
  bool has_target{false};
};

}  // namespace dog_behavior::bt_nodes
