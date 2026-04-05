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
  /// @brief Construct YAML-backed state store with runtime configuration.
  /// @param config State-store configuration.
  explicit YamlStateStore(StateStoreConfig config);

  /// @brief Persist state to primary file and backup atomically.
  /// @param state Recoverable state payload.
  /// @return Save operation result.
  StateStoreResult Save(const RecoverableState & state) override;
  /// @brief Load state from primary file with backup fallback.
  /// @return Load result with optional recoverable state.
  StateStoreLoadResult Load() override;
  /// @brief Remove primary and backup state files.
  /// @return Clear operation result.
  StateStoreResult Clear() override;

private:
  /// @brief Validate recoverable state fields before persistence.
  /// @param state Recoverable state payload.
  /// @return Validation result.
  StateStoreResult ValidateState(const RecoverableState & state) const;
  /// @brief Write serialized state atomically through temp-file rename.
  /// @param content Serialized YAML content.
  /// @return Write operation result.
  StateStoreResult WriteAtomically(const std::string & content);
  /// @brief Parse and validate a state file from disk.
  /// @param file_path State file path.
  /// @return Parsed load result.
  StateStoreLoadResult ParseStateFile(const std::string & file_path) const;
  /// @brief Serialize recoverable state into YAML text.
  /// @param state Recoverable state payload.
  /// @return Serialized YAML string.
  std::string Serialize(const RecoverableState & state) const;
  /// @brief Get current wall-clock time in milliseconds.
  /// @return Current epoch milliseconds.
  static int64_t NowMs();

  StateStoreConfig config_;
  mutable std::mutex mutex_;
  std::chrono::steady_clock::time_point last_write_tp_{};
  bool has_last_write_{false};
};

}  // namespace dog_lifecycle
