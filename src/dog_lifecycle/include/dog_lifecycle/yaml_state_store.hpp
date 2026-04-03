#pragma once

#include "dog_lifecycle/state_store.hpp"

#include <chrono>
#include <mutex>
#include <string>

namespace dog_lifecycle
{

struct StateStoreConfig
{
  std::string state_file_path;
  std::string backup_file_path;
  size_t max_write_bytes{16 * 1024};
  std::chrono::milliseconds min_write_interval{std::chrono::milliseconds(100)};
  uint32_t supported_version{1U};
};

class YamlStateStore : public IStateStore
{
public:
  explicit YamlStateStore(StateStoreConfig config);

  StateStoreResult Save(const RecoverableState & state) override;
  StateStoreLoadResult Load() override;
  StateStoreResult Clear() override;

private:
  StateStoreResult ValidateState(const RecoverableState & state) const;
  StateStoreResult WriteAtomically(const std::string & content);
  StateStoreLoadResult ParseStateFile(const std::string & file_path) const;
  std::string Serialize(const RecoverableState & state) const;
  static int64_t NowMs();

  StateStoreConfig config_;
  mutable std::mutex mutex_;
  std::chrono::steady_clock::time_point last_write_tp_{};
  bool has_last_write_{false};
};

}  // namespace dog_lifecycle
