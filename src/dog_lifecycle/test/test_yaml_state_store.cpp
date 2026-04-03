#include "dog_lifecycle/yaml_state_store.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace
{

class YamlStateStoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    temp_root_ = fs::temp_directory_path() / ("dog_lifecycle_state_store_" + std::to_string(stamp));
    fs::create_directories(temp_root_);

    file_path_ = temp_root_ / "state.yaml";
    backup_path_ = temp_root_ / "state.bak.yaml";

    dog_lifecycle::StateStoreConfig config;
    config.state_file_path = file_path_.string();
    config.backup_file_path = backup_path_.string();
    config.max_write_bytes = 8 * 1024;
    config.min_write_interval = std::chrono::milliseconds(0);
    config.supported_version = 1U;

    store_ = std::make_unique<dog_lifecycle::YamlStateStore>(config);
  }

  void TearDown() override
  {
    store_.reset();
    std::error_code ec;
    fs::remove_all(temp_root_, ec);
  }

  static dog_lifecycle::RecoverableState MakeState(
    const std::string & phase,
    const std::string & target,
    const uint32_t version = 1U)
  {
    dog_lifecycle::RecoverableState state;
    state.task_phase = phase;
    state.target_state = target;
    state.version = version;
    state.timestamp_ms = 1700000000000LL;
    return state;
  }

  fs::path temp_root_;
  fs::path file_path_;
  fs::path backup_path_;
  std::unique_ptr<dog_lifecycle::YamlStateStore> store_;
};

TEST_F(YamlStateStoreTest, SaveAndLoadSuccess)
{
  auto save_result = store_->Save(MakeState("tracking", "locked"));
  ASSERT_TRUE(save_result.ok) << save_result.message;

  const auto load = store_->Load();
  ASSERT_TRUE(load.result.ok) << load.result.message;
  ASSERT_TRUE(load.state.has_value());
  EXPECT_EQ(load.state->task_phase, "tracking");
  EXPECT_EQ(load.state->target_state, "locked");
  EXPECT_EQ(load.state->version, 1U);
}

TEST_F(YamlStateStoreTest, OverwriteWriteTakesLatestValue)
{
  ASSERT_TRUE(store_->Save(MakeState("phase-a", "state-a")).ok);
  ASSERT_TRUE(store_->Save(MakeState("phase-b", "state-b")).ok);

  const auto load = store_->Load();
  ASSERT_TRUE(load.result.ok);
  ASSERT_TRUE(load.state.has_value());
  EXPECT_EQ(load.state->task_phase, "phase-b");
  EXPECT_EQ(load.state->target_state, "state-b");
}

TEST_F(YamlStateStoreTest, CorruptedPrimaryFallsBackToBackup)
{
  ASSERT_TRUE(store_->Save(MakeState("good", "backup-source")).ok);
  std::ofstream corrupted(file_path_, std::ios::trunc);
  corrupted << "task_phase: [broken";
  corrupted.close();

  const auto load = store_->Load();
  ASSERT_TRUE(load.result.ok) << load.result.message;
  ASSERT_TRUE(load.state.has_value());
  EXPECT_EQ(load.state->task_phase, "good");
  EXPECT_EQ(load.state->target_state, "backup-source");
}

TEST_F(YamlStateStoreTest, VersionMismatchReturnsExplicitError)
{
  const auto save_result = store_->Save(MakeState("phase", "target", 2U));
  ASSERT_FALSE(save_result.ok);
  EXPECT_EQ(save_result.error, dog_lifecycle::StateStoreError::VersionMismatch);
}

TEST_F(YamlStateStoreTest, ConcurrentSaveIsProtected)
{
  std::vector<std::thread> workers;
  workers.reserve(8);

  for (int i = 0; i < 8; ++i) {
    workers.emplace_back([this, i]() {
      const auto result = store_->Save(
        MakeState("phase-" + std::to_string(i), "target-" + std::to_string(i)));
      EXPECT_TRUE(result.ok);
    });
  }

  for (auto & worker : workers) {
    worker.join();
  }

  const auto load = store_->Load();
  ASSERT_TRUE(load.result.ok);
  ASSERT_TRUE(load.state.has_value());
  EXPECT_EQ(load.state->target_state.rfind("target-", 0), 0U);
  EXPECT_EQ(load.state->task_phase.rfind("phase-", 0), 0U);
}

TEST_F(YamlStateStoreTest, MissingPrimaryFallsBackToBackup)
{
  ASSERT_TRUE(store_->Save(MakeState("recover", "from-backup")).ok);
  std::error_code ec;
  fs::remove(file_path_, ec);
  ASSERT_FALSE(ec);
  ASSERT_FALSE(fs::exists(file_path_));
  ASSERT_TRUE(fs::exists(backup_path_));

  const auto load = store_->Load();
  ASSERT_TRUE(load.result.ok) << load.result.message;
  ASSERT_TRUE(load.state.has_value());
  EXPECT_EQ(load.state->task_phase, "recover");
  EXPECT_EQ(load.state->target_state, "from-backup");
  EXPECT_TRUE(fs::exists(file_path_));
}

TEST_F(YamlStateStoreTest, BackupFailureDoesNotFailPrimaryWrite)
{
  dog_lifecycle::StateStoreConfig config;
  config.state_file_path = file_path_.string();
  config.backup_file_path = temp_root_.string();
  config.max_write_bytes = 8 * 1024;
  config.min_write_interval = std::chrono::milliseconds(0);
  config.supported_version = 1U;
  store_ = std::make_unique<dog_lifecycle::YamlStateStore>(config);

  const auto save_result = store_->Save(MakeState("backup", "write-failure"));
  ASSERT_TRUE(save_result.ok) << save_result.message;
  EXPECT_NE(save_result.message.find("backup"), std::string::npos);

  const auto load = store_->Load();
  ASSERT_TRUE(load.result.ok) << load.result.message;
  ASSERT_TRUE(load.state.has_value());
  EXPECT_EQ(load.state->task_phase, "backup");
  EXPECT_EQ(load.state->target_state, "write-failure");
}

TEST_F(YamlStateStoreTest, ClearRemovesStateFiles)
{
  ASSERT_TRUE(store_->Save(MakeState("clear", "me")).ok);
  ASSERT_TRUE(fs::exists(file_path_));

  const auto clear_result = store_->Clear();
  ASSERT_TRUE(clear_result.ok) << clear_result.message;
  EXPECT_FALSE(fs::exists(file_path_));
  EXPECT_FALSE(fs::exists(backup_path_));
}

TEST_F(YamlStateStoreTest, FlushLatencyP95WithinBudget)
{
  std::vector<double> write_cost_ms;
  write_cost_ms.reserve(100);

  for (int i = 0; i < 100; ++i) {
    const auto start = std::chrono::steady_clock::now();
    const auto result = store_->Save(MakeState("bench", "state-" + std::to_string(i)));
    const auto end = std::chrono::steady_clock::now();
    ASSERT_TRUE(result.ok) << result.message;
    write_cost_ms.push_back(
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count());
  }

  std::sort(write_cost_ms.begin(), write_cost_ms.end());
  const size_t p95_index = static_cast<size_t>(write_cost_ms.size() * 0.95) - 1U;
  const double p95_ms = write_cost_ms.at(p95_index);
  EXPECT_LT(p95_ms, 100.0);
}

}  // namespace
