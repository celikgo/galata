// SPDX-License-Identifier: Apache-2.0
// Input limits apply before model parsing, including shared and cached files.
#include "galata/modeling/model.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#ifndef _WIN32
#include <sys/stat.h>
#endif

namespace {
namespace fs = std::filesystem;
using namespace galata::pipeline;

class ModelInputLimits : public ::testing::Test {
 protected:
  fs::path root;
  unsigned run_sequence = 0;

  void SetUp() override {
    static std::atomic<unsigned> sequence{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-model-input-" + std::to_string(tick) + "-"
              + std::to_string(sequence.fetch_add(1)));
    ASSERT_TRUE(fs::create_directory(root));
  }

  void TearDown() override {
    std::error_code error;
    fs::remove_all(root, error);
  }

  void put(const std::string& name, const std::string& bytes) {
    std::ofstream file(root / name, std::ios::binary | std::ios::trunc);
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    ASSERT_TRUE(file.good());
  }

  RunResult run(const std::string& yaml, const Registry& registry) {
    return run_pipeline(parse_pipeline(yaml),
                        registry,
                        root.string(),
                        (root / ("output-" + std::to_string(run_sequence++))).string(),
                        nullptr,
                        RunOptions{.write_manifest = false});
  }

  static Capability reader(const std::string& id, std::size_t limit = 0) {
    Capability capability;
    capability.id = id;
    capability.produces = "input_fixture";
    capability.input_keys = {"path"};
    capability.input_file_keys = {"path"};
    if (limit != 0)
      capability.input_file_byte_limits.emplace("path", limit);
    capability.run = [](const StageContext& context) {
      Artifact artifact;
      artifact.kind = "input_fixture";
      artifact.summary = context.read_input(context.input->string_at("path"));
      return artifact;
    };
    return capability;
  }
};

TEST_F(ModelInputLimits, ChunkedReaderAcceptsTheBoundaryAndRejectsTheFirstExcessByte) {
  for (const std::size_t size : {0U, 1U, 8191U, 8192U, 8193U, 65537U}) {
    SCOPED_TRACE(size);
    std::string bytes(size, 'x');
    if (!bytes.empty())
      bytes[size / 2] = '\0';
    put("input.bin", bytes);
    const auto path = (root / "input.bin").string();
    EXPECT_EQ(read_file_bytes(path, size), bytes);
    EXPECT_EQ(read_file_bytes(path), bytes);
    if (size != 0) {
      EXPECT_THROW((void)read_file_bytes(path, size - 1), std::runtime_error);
    }
  }
}

TEST_F(ModelInputLimits, FailedBoundedReadDoesNotCachePartialBytes) {
  put("model.yaml", std::string(65, 'x'));
  RunFiles files((root / "output").string());
  EXPECT_THROW((void)files.read_input((root / "model.yaml").string(), 64), std::runtime_error);
  EXPECT_TRUE(files.inputs().empty());
  put("model.yaml", "complete replacement");
  EXPECT_EQ(files.read_input((root / "model.yaml").string(), 64), "complete replacement");
}

TEST_F(ModelInputLimits, AStricterLimitAppliesToCachedAndExplicitlyRecordedSnapshots) {
  put("model.yaml", "123456789");
  const auto path = (root / "model.yaml").string();
  RunFiles files((root / "output").string());
  EXPECT_EQ(files.read_input(path), "123456789");
  // The source now fits, but a cached nine-byte snapshot must still fail eight.
  put("model.yaml", "new");
  EXPECT_THROW((void)files.read_input(path, 8), std::runtime_error);
  EXPECT_EQ(files.read_input(path, 9), "123456789");

  RunFiles recorded((root / "recorded-output").string());
  recorded.record_input(path, "recorded input bytes");
  EXPECT_THROW((void)recorded.read_input(path, 8), std::runtime_error);
  EXPECT_EQ(recorded.read_input(path, 20), "recorded input bytes");
}

TEST_F(ModelInputLimits, StageContextEnforcesTheLimitWithAndWithoutRunFiles) {
  put("model.yaml", "12345678");
  StageContext direct;
  direct.base_directory = root.string();
  EXPECT_EQ(direct.read_input("model.yaml", 8), "12345678");
  EXPECT_THROW((void)direct.read_input("model.yaml", 7), std::runtime_error);
  direct.files = std::make_shared<RunFiles>((root / "output").string());
  EXPECT_EQ(direct.read_input("model.yaml", 8), "12345678");
  EXPECT_THROW((void)direct.read_input("model.yaml", 7), std::runtime_error);
}

TEST_F(ModelInputLimits, RefusesNonRegularInputsWithoutWaitingForData) {
  EXPECT_THROW((void)read_file_bytes(root.string(), 64), std::runtime_error);
#ifndef _WIN32
  const auto pipe = root / "input-pipe";
  ASSERT_EQ(::mkfifo(pipe.c_str(), 0600), 0);
  EXPECT_THROW((void)read_file_bytes(pipe.string(), 64), std::runtime_error);
#endif
}

TEST_F(ModelInputLimits, BoundedRolesPrecedeLegacyReadsOfTheSameFile) {
  put("model.yaml", std::string(65, 'x'));
  Registry registry;
  registry.add(reader("fixture.legacy"));
  registry.add(reader("fixture.bounded", 64));
  try {
    (void)run(R"(
version: 1
stages:
  - {id: legacy, capability: fixture.legacy, input: {path: model.yaml}}
  - {id: bounded, capability: fixture.bounded, input: {path: model.yaml}}
)",
              registry);
    FAIL() << "oversized model reached execution";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("stage 'bounded'"), std::string::npos) << message;
    EXPECT_NE(message.find("byte limit 64"), std::string::npos) << message;
    EXPECT_EQ(message.find("cached"), std::string::npos) << message;
  }
}

TEST_F(ModelInputLimits, TheStrictestBoundedRolePrecedesLargerAliases) {
  put("model.yaml", std::string(65, 'x'));
  Registry registry;
  registry.add(reader("fixture.large", 1024));
  registry.add(reader("fixture.small", 64));
  try {
    (void)run(R"(
version: 1
stages:
  - {id: larger, capability: fixture.large, input: {path: model.yaml}}
  - {id: smaller, capability: fixture.small, input: {path: ./model.yaml}}
)",
              registry);
    FAIL() << "oversized model reached execution";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("stage 'smaller'"), std::string::npos) << message;
    EXPECT_NE(message.find("byte limit 64"), std::string::npos) << message;
    EXPECT_EQ(message.find("cached"), std::string::npos) << message;
  }
}

TEST_F(ModelInputLimits, ValidSharedRolesConsumeTheSamePreflightSnapshot) {
  put("model.yaml", "original input");
  Registry registry;
  auto legacy = reader("fixture.legacy");
  const auto read = legacy.run;
  legacy.run = [this, read](const StageContext& context) {
    put("model.yaml", "a later replacement exceeds the original input cap");
    return read(context);
  };
  registry.add(std::move(legacy));
  registry.add(reader("fixture.bounded", 16));
  const auto result = run(R"(
version: 1
stages:
  - {id: legacy, capability: fixture.legacy, input: {path: model.yaml}}
  - {id: bounded, capability: fixture.bounded, input: {path: model.yaml}}
)",
                          registry);
  ASSERT_EQ(result.stages.size(), 2U);
  EXPECT_EQ(result.stages[0].artifact.summary, "original input");
  EXPECT_EQ(result.stages[1].artifact.summary, "original input");
}

TEST_F(ModelInputLimits, RegistryRejectsLimitsWithoutAPositiveDeclaredInputFileRole) {
  Registry registry;
  auto zero = reader("fixture.zero");
  zero.input_file_byte_limits["path"] = 0;
  EXPECT_THROW(registry.add(std::move(zero)), std::invalid_argument);

  auto unknown = reader("fixture.unknown");
  unknown.input_file_byte_limits["other"] = 64;
  EXPECT_THROW(registry.add(std::move(unknown)), std::invalid_argument);

  auto ordinary = reader("fixture.ordinary");
  ordinary.input_file_keys.clear();
  ordinary.input_file_byte_limits["path"] = 64;
  EXPECT_THROW(registry.add(std::move(ordinary)), std::invalid_argument);
  EXPECT_NO_THROW(registry.add(reader("fixture.valid", 64)));
}

TEST_F(ModelInputLimits, BuiltinModelCompilationRejectsOversizeBeforeParsing) {
  put("model.yaml", std::string(galata::modeling::kMaxSourceBytes + 1U, ' '));
  try {
    (void)run(
        "version: 1\nstages: [{id: compile, capability: model.compile, "
        "input: {path: model.yaml}}]\n",
        builtin_registry());
    FAIL() << "oversized model reached the parser";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("stage 'compile'"), std::string::npos) << message;
    EXPECT_NE(message.find("byte limit " + std::to_string(galata::modeling::kMaxSourceBytes)),
              std::string::npos)
        << message;
  }
}
}  // namespace
