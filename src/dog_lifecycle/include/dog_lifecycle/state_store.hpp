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

  /// @brief 创建成功的状态存储结果。
  /// @param message 可选的信息提示。
  /// @return 成功结果实例。
  static StateStoreResult Success(const std::string & message = "")
  {
    return StateStoreResult{true, StateStoreError::None, message};
  }

  /// @brief 创建失败的状态存储结果。
  /// @param error_code 具体错误码。
  /// @param message 诊断错误详情。
  /// @return 失败结果实例。
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
  /// @brief 多态状态存储实现的虚析构函数。
  virtual ~IStateStore() = default;

  /// @brief 持久化可恢复生命周期状态。
  /// @param state 可恢复状态负载。
  /// @return 保存操作结果。
  virtual StateStoreResult Save(const RecoverableState & state) = 0;
  /// @brief 加载先前持久化的可恢复状态。
  /// @return 包含可选状态负载的加载结果。
  virtual StateStoreLoadResult Load() = 0;
  /// @brief 清理持久化状态文件。
  /// @return 清理操作结果。
  virtual StateStoreResult Clear() = 0;
};

}  // namespace dog_lifecycle
