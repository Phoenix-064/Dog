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

  static StateStoreResult Success(const std::string & message = "")
  {
    return StateStoreResult{true, StateStoreError::None, message};
  }

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
  virtual ~IStateStore() = default;

  virtual StateStoreResult Save(const RecoverableState & state) = 0;
  virtual StateStoreLoadResult Load() = 0;
  virtual StateStoreResult Clear() = 0;
};

}  // namespace dog_lifecycle
