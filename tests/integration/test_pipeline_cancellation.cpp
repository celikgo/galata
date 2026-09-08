// SPDX-License-Identifier: Apache-2.0
// Acceptance of the cancellation contract in ADR-0012 and the public API.
// Authored from the proposed callback contract before reading its implementation.
// The constant-rate synthetic fixture tests control flow, not aircraft validity.
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {
namespace fs = std::filesystem;
using namespace galata::pipeline;

class PipelineCancellation : public ::testing::Test {
 protected:
  fs::path root;
  fs::path output;

  void SetUp() override {
    static std::atomic<unsigned> sequence{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-pipeline-cancel-" + std::to_string(tick) + "-"
              + std::to_string(sequence.fetch_add(1)));
    ASSERT_TRUE(fs::create_directory(root));
    output = root / "output";
  }

  void TearDown() override {
    std::error_code error;
    fs::remove_all(root, error);
  }

  void put(const fs::path& path, const std::string& bytes) {
    std::ofstream file(path, std::ios::binary);
    file << bytes;
    ASSERT_TRUE(file.good());
  }

  void expect_no_completed_manifest() const {
    if (!fs::exists(output))
      return;
    for (const auto& entry : fs::recursive_directory_iterator(output)) {
      const auto filename = entry.path().filename().string();
      EXPECT_FALSE(filename.starts_with("run-") && entry.path().extension() == ".json")
          << entry.path();
    }
  }

  RunResult run(const std::string& source,
                const RunOptions& options,
                const Registry& registry = builtin_registry(),
                const ProgressCallback& progress = nullptr) {
    return run_pipeline(
        parse_pipeline(source), registry, root.string(), output.string(), progress, options);
  }

  static std::string one_stage() {
    return "version: 1\nstages:\n"
           "  - id: work\n    capability: test.work\n    input: {}\n";
  }

  std::string model_study() {
    put(root / "model.yaml",
        "schema: galata.model.v1\n"
        "profile: continuous-scalar.v1\n"
        "blocks:\n"
        "  - id: rate\n    kind: constant\n    value: 1\n"
        "    output: {dimension: [1,0,-1,0,0,0,0,0], frame: none}\n"
        "  - id: state\n    kind: integrator\n    initial_value: 0\n"
        "    output: {dimension: [1,0,0,0,0,0,0,0], frame: none}\n"
        "  - id: scope\n    kind: output\n"
        "    output: {dimension: [1,0,0,0,0,0,0,0], frame: none}\n"
        "connections:\n"
        "  - {source: rate, target: state, input: 0}\n"
        "  - {source: state, target: scope, input: 0}\n");
    return "version: 1\nstages:\n"
           "  - id: model\n    capability: model.compile\n    input: {path: model.yaml}\n"
           "  - id: response\n    capability: sim.model\n"
           "    input: {model: {from: model}, step_s: 0.01, steps: 1000, "
           "sample_stride: 100, csv_path: response.csv, evidence_path: evidence.json}\n";
  }
};

TEST_F(PipelineCancellation, CancellationBeforeExecutionRunsNoCapability) {
  int executions = 0;
  Registry registry;
  registry.add(Capability{"test.work",
                          "Test callback containment",
                          "test_value",
                          Capability::State::ImplementedUnvalidated,
                          [&](const StageContext&) {
                            ++executions;
                            return Artifact{};
                          }});
  RunOptions options;
  options.cancelled = [] { return true; };
  EXPECT_THROW((void)run(one_stage(), options, registry), std::exception);
  EXPECT_EQ(executions, 0);
  expect_no_completed_manifest();
}

TEST_F(PipelineCancellation, StageReceivesTheSubmittedCancellationCallback) {
  bool request_cancel = false;
  bool observed = false;
  Registry registry;
  registry.add(Capability{"test.work",
                          "Test callback forwarding",
                          "test_value",
                          Capability::State::ImplementedUnvalidated,
                          [&](const StageContext& context) {
                            request_cancel = true;
                            observed = context.cancelled && context.cancelled();
                            return Artifact{};
                          }});
  RunOptions options;
  options.cancelled = [&] { return request_cancel; };
  EXPECT_THROW((void)run(one_stage(), options, registry), std::exception);
  EXPECT_TRUE(observed);
  expect_no_completed_manifest();
}

TEST_F(PipelineCancellation, CancellationAfterFinalStagePreventsManifestCommit) {
  bool request_cancel = false;
  bool completed_stage = false;
  Registry registry;
  registry.add(Capability{"test.work",
                          "Test final commit cancellation",
                          "test_value",
                          Capability::State::ImplementedUnvalidated,
                          [](const StageContext&) { return Artifact{}; }});
  RunOptions options;
  options.cancelled = [&] { return request_cancel; };
  const auto progress =
      [&](const std::string&, const std::string&, bool finished, const std::string&) {
        if (finished) {
          completed_stage = true;
          request_cancel = true;
        }
      };
  EXPECT_THROW((void)run(one_stage(), options, registry, progress), std::exception);
  EXPECT_TRUE(completed_stage);
  expect_no_completed_manifest();
}

TEST_F(PipelineCancellation, ModelSimulationPollsDuringIntegration) {
  unsigned polls = 0;
  bool response_completed = false;
  RunOptions options;
  // The public contract fixes polling per integration step. This threshold is
  // well past stage setup and below the fixture's 1000 integration steps;
  // it does not assert an implementation-specific exact callback count.
  options.cancelled = [&] { return ++polls >= 100; };
  const auto progress =
      [&](const std::string& stage, const std::string&, bool finished, const std::string&) {
        if (stage == "response" && finished)
          response_completed = true;
      };
  EXPECT_THROW((void)run(model_study(), options, builtin_registry(), progress), std::exception);
  EXPECT_GE(polls, 100U);
  EXPECT_FALSE(response_completed);
  EXPECT_FALSE(fs::exists(output / "response.csv"));
  EXPECT_FALSE(fs::exists(output / "evidence.json"));
  expect_no_completed_manifest();
}

TEST_F(PipelineCancellation, CallbackFailureDoesNotBecomeSuccessfulExecution) {
  unsigned executions = 0;
  Registry registry;
  registry.add(Capability{"test.work",
                          "Test callback exception containment",
                          "test_value",
                          Capability::State::ImplementedUnvalidated,
                          [&](const StageContext&) {
                            ++executions;
                            return Artifact{};
                          }});
  RunOptions options;
  options.cancelled = []() -> bool { throw std::runtime_error("cancellation callback failed"); };
  EXPECT_THROW((void)run(one_stage(), options, registry), std::exception);
  EXPECT_EQ(executions, 0U);
  expect_no_completed_manifest();
}

TEST_F(PipelineCancellation, InactiveCancellationPreservesExactTrajectory) {
  const auto study = model_study();
  const auto plain = run(study, {});
  ASSERT_FALSE(plain.manifest_path.empty());
  const auto expected = read_file_bytes((output / "response.csv").string());
  output = root / "with-callback";
  unsigned polls = 0;
  RunOptions options;
  options.cancelled = [&] {
    ++polls;
    return false;
  };
  const auto checked = run(study, options);
  EXPECT_FALSE(checked.manifest_path.empty());
  EXPECT_GE(polls, 1000U);
  EXPECT_EQ(read_file_bytes((output / "response.csv").string()), expected);
}

}  // namespace
