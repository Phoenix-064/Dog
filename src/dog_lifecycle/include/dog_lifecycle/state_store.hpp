#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace dog_lifecycle
{

struct RecoverableState
{
  std::string task_phase;
  int64_t timestamp_ms{0};
  std::string target_state;
  uint32_t version{1U};
};

enum class StateStoreError
{
  None,
  FileNotFound,
  ParseError,
  ValidationError,
  VersionMismatch,
  IOError,
  RateLimited,
  PayloadTooLarge
};

struct StateStoreResult
{
  bool ok{false};
  StateStoreError error{StateStoreError::IOError};
  std::string message;

  /// @brief Create a successful state-store result.
  /// @param message Optional informational message.
  /// @return Success result instance.
  static StateStoreResult Success(const std::string & message = "")
  {
    return StateStoreResult{true, StateStoreError::None, message};
  }

  /// @brief Create a failed state-store result.
  /// @param error_code Specific error code.
  /// @param message Diagnostic error details.
  /// @return Failure result instance.
  static StateStoreResult Failure(StateStoreError error_code, const std::string & message)
  {
    return StateStoreResult{false, error_code, message};
  }
};

struct StateStoreLoadResult
{
  StateStoreResult result;
  std::optional<RecoverableState> state;
};

class IStateStore
{
public:
  /// @brief Virtual destructor for polymorphic state-store implementations.
  virtual ~IStateStore() = default;

  /// @brief Persist recoverable lifecycle state.
  /// @param state Recoverable state payload.
  /// @return Save operation result.
  virtual StateStoreResult Save(const RecoverableState & state) = 0;
  /// @brief Load previously persisted recoverable state.
  /// @return Load result with optional state payload.
  virtual StateStoreLoadResult Load() = 0;
  /// @brief Clear persisted state files.
  /// @return Clear operation result.
  virtual StateStoreResult Clear() = 0;
};

}  // namespace dog_lifecycle
