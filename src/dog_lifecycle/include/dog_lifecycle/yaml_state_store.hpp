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
  /// @brief 使用运行时配置构造基于 YAML 的状态存储。
  /// @param config 状态存储配置。
  explicit YamlStateStore(StateStoreConfig config);

  /// @brief 将状态原子化持久化到主文件和备份文件。
  /// @param state 可恢复状态负载。
  /// @return 保存操作结果。
  StateStoreResult Save(const RecoverableState & state) override;
  /// @brief 从主文件加载状态，必要时回退到备份文件。
  /// @return 包含可选可恢复状态的加载结果。
  StateStoreLoadResult Load() override;
  /// @brief 删除主状态文件和备份状态文件。
  /// @return 清理操作结果。
  StateStoreResult Clear() override;

private:
  /// @brief 在持久化前校验可恢复状态字段。
  /// @param state 可恢复状态负载。
  /// @return 校验结果。
  StateStoreResult ValidateState(const RecoverableState & state) const;
  /// @brief 通过临时文件重命名原子写入序列化状态。
  /// @param content 序列化后的 YAML 内容。
  /// @return 写入操作结果。
  StateStoreResult WriteAtomically(const std::string & content);
  /// @brief 从磁盘解析并校验状态文件。
  /// @param file_path 状态文件路径。
  /// @return 解析后的加载结果。
  StateStoreLoadResult ParseStateFile(const std::string & file_path) const;
  /// @brief 将可恢复状态序列化为 YAML 文本。
  /// @param state 可恢复状态负载。
  /// @return 序列化后的 YAML 字符串。
  std::string Serialize(const RecoverableState & state) const;
  /// @brief 获取当前墙钟时间（毫秒）。
  /// @return 当前 epoch 毫秒值。
  static int64_t NowMs();

  StateStoreConfig config_;
  mutable std::mutex mutex_;
  std::chrono::steady_clock::time_point last_write_tp_{};
  bool has_last_write_{false};
};

}  // namespace dog_lifecycle
