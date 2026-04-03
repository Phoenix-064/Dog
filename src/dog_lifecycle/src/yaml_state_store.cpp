#include "dog_lifecycle/yaml_state_store.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace dog_lifecycle
{

namespace
{

constexpr const char * kTaskPhaseKey = "task_phase";
constexpr const char * kTimestampKey = "timestamp_ms";
constexpr const char * kTargetStateKey = "target_state";
constexpr const char * kVersionKey = "version";

struct FileLockGuard
{
  int fd{-1};

  ~FileLockGuard()
  {
    if (fd >= 0) {
      (void)::flock(fd, LOCK_UN);
      (void)::close(fd);
    }
  }
};

StateStoreResult AcquireLock(const std::string & state_file_path, const int lock_op, FileLockGuard & guard)
{
  if (state_file_path.empty()) {
    return StateStoreResult::Failure(StateStoreError::ValidationError, "state_file_path is empty");
  }

  const fs::path state_path(state_file_path);
  std::error_code ec;
  if (!state_path.parent_path().empty()) {
    fs::create_directories(state_path.parent_path(), ec);
    if (ec) {
      return StateStoreResult::Failure(StateStoreError::IOError, "cannot create lock directory: " + ec.message());
    }
  }

  const fs::path lock_path = fs::path(state_file_path).string() + ".lock";
  guard.fd = ::open(lock_path.c_str(), O_RDWR | O_CREAT, 0644);
  if (guard.fd < 0) {
    return StateStoreResult::Failure(StateStoreError::IOError, "cannot open lock file");
  }

  if (::flock(guard.fd, lock_op) != 0) {
    return StateStoreResult::Failure(StateStoreError::IOError, "cannot acquire file lock");
  }

  return StateStoreResult::Success();
}

}  // namespace

YamlStateStore::YamlStateStore(StateStoreConfig config)
: config_(std::move(config))
{
}

StateStoreResult YamlStateStore::Save(const RecoverableState & state)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  FileLockGuard lock_guard;
  const auto lock_result = AcquireLock(config_.state_file_path, LOCK_EX, lock_guard);
  if (!lock_result.ok) {
    return lock_result;
  }

  const auto validation = ValidateState(state);
  if (!validation.ok) {
    return validation;
  }

  const auto now_tp = std::chrono::steady_clock::now();
  if (has_last_write_) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - last_write_tp_);
    if (elapsed < config_.min_write_interval) {
      return StateStoreResult::Failure(
        StateStoreError::RateLimited,
        "save request rejected because write interval is too short");
    }
  }

  const std::string content = Serialize(state);
  if (content.size() > config_.max_write_bytes) {
    return StateStoreResult::Failure(
      StateStoreError::PayloadTooLarge,
      "serialized state exceeds max_write_bytes");
  }

  const auto write_result = WriteAtomically(content);
  if (!write_result.ok) {
    return write_result;
  }

  last_write_tp_ = now_tp;
  has_last_write_ = true;
  if (!write_result.message.empty()) {
    return write_result;
  }
  return StateStoreResult::Success("state persisted successfully");
}

StateStoreLoadResult YamlStateStore::Load()
{
  std::scoped_lock<std::mutex> lock(mutex_);

  FileLockGuard lock_guard;
  const auto lock_result = AcquireLock(config_.state_file_path, LOCK_SH, lock_guard);
  if (!lock_result.ok) {
    return StateStoreLoadResult{lock_result, std::nullopt};
  }

  if (!fs::exists(config_.state_file_path)) {
    if (!config_.backup_file_path.empty() && fs::exists(config_.backup_file_path)) {
      auto backup_only = ParseStateFile(config_.backup_file_path);
      if (backup_only.result.ok && backup_only.state.has_value()) {
        const auto repair_result = WriteAtomically(Serialize(*backup_only.state));
        if (!repair_result.ok) {
          backup_only.result.message += "; backup loaded but primary repair failed: " + repair_result.message;
        }
        return backup_only;
      }
      return backup_only;
    }
    return StateStoreLoadResult{
      StateStoreResult::Failure(StateStoreError::FileNotFound, "state file does not exist"),
      std::nullopt};
  }

  auto primary = ParseStateFile(config_.state_file_path);
  if (primary.result.ok) {
    return primary;
  }

  if (!config_.backup_file_path.empty() && fs::exists(config_.backup_file_path)) {
    auto backup = ParseStateFile(config_.backup_file_path);
    if (backup.result.ok) {
      if (backup.state.has_value()) {
        const auto repair_result = WriteAtomically(Serialize(*backup.state));
        if (!repair_result.ok) {
          backup.result.message += "; backup loaded but primary repair failed: " + repair_result.message;
        }
      }
      return backup;
    }
    return StateStoreLoadResult{
      StateStoreResult::Failure(
        StateStoreError::ParseError,
        "state file and backup file are both invalid"),
      std::nullopt};
  }

  return primary;
}

StateStoreResult YamlStateStore::Clear()
{
  std::scoped_lock<std::mutex> lock(mutex_);

  FileLockGuard lock_guard;
  const auto lock_result = AcquireLock(config_.state_file_path, LOCK_EX, lock_guard);
  if (!lock_result.ok) {
    return lock_result;
  }

  std::error_code ec;
  if (!config_.state_file_path.empty()) {
    fs::remove(config_.state_file_path, ec);
    if (ec) {
      return StateStoreResult::Failure(
        StateStoreError::IOError,
        "failed to remove state file: " + ec.message());
    }
  }

  if (!config_.backup_file_path.empty()) {
    fs::remove(config_.backup_file_path, ec);
    if (ec) {
      return StateStoreResult::Failure(
        StateStoreError::IOError,
        "failed to remove backup file: " + ec.message());
    }
  }

  return StateStoreResult::Success("state files cleared");
}

StateStoreResult YamlStateStore::ValidateState(const RecoverableState & state) const
{
  if (state.task_phase.empty() || state.target_state.empty()) {
    return StateStoreResult::Failure(
      StateStoreError::ValidationError,
      "task_phase and target_state are required");
  }

  if (state.timestamp_ms <= 0) {
    return StateStoreResult::Failure(
      StateStoreError::ValidationError,
      "timestamp_ms must be positive");
  }

  if (state.version != config_.supported_version) {
    return StateStoreResult::Failure(
      StateStoreError::VersionMismatch,
      "state version mismatch");
  }

  return StateStoreResult::Success();
}

StateStoreResult YamlStateStore::WriteAtomically(const std::string & content)
{
  if (config_.state_file_path.empty()) {
    return StateStoreResult::Failure(StateStoreError::ValidationError, "state_file_path is empty");
  }

  const fs::path state_path(config_.state_file_path);
  const fs::path tmp_path = state_path.string() + ".tmp";
  const auto cleanup_tmp = [&tmp_path]() {
    std::error_code remove_ec;
    fs::remove(tmp_path, remove_ec);
  };

  std::error_code ec;
  if (!state_path.parent_path().empty()) {
    fs::create_directories(state_path.parent_path(), ec);
    if (ec) {
      return StateStoreResult::Failure(
        StateStoreError::IOError,
        "failed to create state directory: " + ec.message());
    }
  }

  {
    std::ofstream ofs(tmp_path, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
      cleanup_tmp();
      return StateStoreResult::Failure(StateStoreError::IOError, "cannot open temp file for writing");
    }
    ofs << content;
    ofs.flush();
    if (!ofs.good()) {
      cleanup_tmp();
      return StateStoreResult::Failure(StateStoreError::IOError, "failed to flush temp file stream");
    }
  }

  const int fd = ::open(tmp_path.c_str(), O_RDONLY);
  if (fd < 0) {
    cleanup_tmp();
    return StateStoreResult::Failure(StateStoreError::IOError, "cannot open temp file for fsync");
  }
  const int sync_rc = ::fsync(fd);
  ::close(fd);
  if (sync_rc != 0) {
    cleanup_tmp();
    return StateStoreResult::Failure(StateStoreError::IOError, "fsync failed on temp file");
  }

  fs::rename(tmp_path, state_path, ec);
  if (ec) {
    cleanup_tmp();
    return StateStoreResult::Failure(
      StateStoreError::IOError,
      "atomic rename failed: " + ec.message());
  }

  const fs::path parent_path = state_path.parent_path();
  if (!parent_path.empty()) {
    const int dir_fd = ::open(parent_path.c_str(), O_RDONLY | O_DIRECTORY);
    if (dir_fd < 0) {
      return StateStoreResult::Failure(StateStoreError::IOError, "cannot open state directory for fsync");
    }
    const int dir_sync_rc = ::fsync(dir_fd);
    ::close(dir_fd);
    if (dir_sync_rc != 0) {
      return StateStoreResult::Failure(StateStoreError::IOError, "fsync failed on state directory");
    }
  }

  if (!config_.backup_file_path.empty()) {
    const fs::path backup_path(config_.backup_file_path);
    if (!backup_path.parent_path().empty()) {
      fs::create_directories(backup_path.parent_path(), ec);
      if (ec) {
        return StateStoreResult::Success("state persisted but backup directory create failed: " + ec.message());
      }
    }
    fs::copy_file(state_path, config_.backup_file_path, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      return StateStoreResult::Success("state persisted but backup update failed: " + ec.message());
    }
  }

  return StateStoreResult::Success("state persisted successfully");
}

StateStoreLoadResult YamlStateStore::ParseStateFile(const std::string & file_path) const
{
  YAML::Node node;
  try {
    node = YAML::LoadFile(file_path);
  } catch (const YAML::Exception & ex) {
    return StateStoreLoadResult{
      StateStoreResult::Failure(StateStoreError::ParseError, std::string("yaml parse error: ") + ex.what()),
      std::nullopt};
  }

  if (!node[kTaskPhaseKey] || !node[kTimestampKey] || !node[kTargetStateKey] || !node[kVersionKey]) {
    return StateStoreLoadResult{
      StateStoreResult::Failure(StateStoreError::ValidationError, "missing required fields in state file"),
      std::nullopt};
  }

  RecoverableState state;
  try {
    state.task_phase = node[kTaskPhaseKey].as<std::string>();
    state.timestamp_ms = node[kTimestampKey].as<int64_t>();
    state.target_state = node[kTargetStateKey].as<std::string>();
    state.version = node[kVersionKey].as<uint32_t>();
  } catch (const YAML::Exception & ex) {
    return StateStoreLoadResult{
      StateStoreResult::Failure(
        StateStoreError::ValidationError,
        std::string("invalid field type in state file: ") + ex.what()),
      std::nullopt};
  }

  const auto validation = ValidateState(state);
  if (!validation.ok) {
    return StateStoreLoadResult{validation, std::nullopt};
  }

  return StateStoreLoadResult{StateStoreResult::Success("state loaded"), state};
}

std::string YamlStateStore::Serialize(const RecoverableState & state) const
{
  YAML::Node root;
  root[kTaskPhaseKey] = state.task_phase;
  root[kTimestampKey] = state.timestamp_ms;
  root[kTargetStateKey] = state.target_state;
  root[kVersionKey] = state.version;

  std::ostringstream oss;
  oss << root;
  return oss.str();
}

int64_t YamlStateStore::NowMs()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

}  // namespace dog_lifecycle
