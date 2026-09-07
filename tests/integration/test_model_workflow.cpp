// SPDX-License-Identifier: Apache-2.0
// Independent model.compile -> sim.model contract; implementation was not read.
// Analytic reference: xdot=1-x, x0=0, and classical RK4 R(z), as predeclared in
// docs/architecture/MODEL_CONFORMANCE.md (MC04/MC12, budgets B01/B03).
// Completing this synthetic model does not assess aircraft validity or accuracy.
#include "galata/modeling/model.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using namespace galata::pipeline;

class ModelWorkflow : public ::testing::Test {
 protected:
  fs::path root;
  fs::path output;
  std::string raw_model;

  void SetUp() override {
    static std::atomic<unsigned> sequence{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-model-workflow-" + std::to_string(tick) + "-"
              + std::to_string(sequence.fetch_add(1)));
    ASSERT_TRUE(fs::create_directory(root));
    output = root / "output";
    raw_model = read_file_bytes(
        (fs::path(GALATA_EXAMPLES_DIR) / "continuous-feedback/model.yaml").string());
    put(root / "model.yaml", raw_model);
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

  static std::string study(const std::string& response_input =
                               "{model: {from: model}, step_s: 0.125, steps: 8, sample_stride: 3, "
                               "csv_path: response.csv, evidence_path: evidence.json}") {
    return "version: 1\nstages:\n"
           "  - id: model\n    capability: model.compile\n    input: {path: model.yaml}\n"
           "  - id: response\n    capability: sim.model\n    input: "
           + response_input + "\n";
  }

  RunResult run(const std::string& document,
                RunOptions options = {},
                const Registry& registry = builtin_registry()) {
    const auto source = root / "study.yaml";
    put(source, document);
    return run_pipeline(
        load_pipeline(source.string()), registry, root.string(), output.string(), nullptr, options);
  }

  void expect_no_published_outputs() const {
    EXPECT_FALSE(fs::exists(output / "response.csv"));
    EXPECT_FALSE(fs::exists(output / "evidence.json"));
    if (fs::exists(output)) {
      for (const auto& item : fs::recursive_directory_iterator(output)) {
        EXPECT_FALSE(item.is_regular_file()) << item.path();
      }
    }
  }
};

std::string hex_bytes(const std::string& bytes) {
  constexpr char digits[] = "0123456789abcdef";
  std::string result;
  result.reserve(bytes.size() * 2);
  for (const char character : bytes) {
    const auto byte = static_cast<unsigned char>(character);
    result += digits[byte >> 4U];
    result += digits[byte & 0x0fU];
  }
  return result;
}

double affine_reference(int steps) {
  // Independent rational stability polynomial for h=1/8 second.
  double power = 1.0;
  for (int index = 0; index < steps; ++index) {
    power *= 86753.0 / 98304.0;
  }
  return 1.0 - power;
}

TEST_F(ModelWorkflow, CompiledFeedbackProducesLabeledSamplesAndExplicitEvidenceLimits) {
  const auto result = run(study());
  const auto* model_artifact = result.find("model");
  const auto* trajectory_artifact = result.find("response");
  ASSERT_NE(model_artifact, nullptr);
  ASSERT_NE(trajectory_artifact, nullptr);
  const auto& model =
      model_artifact->payload_as<galata::modeling::CompiledModel>("executable_model");
  const auto& trajectory =
      trajectory_artifact->payload_as<galata::modeling::SimulationResult>("model_trajectory");
  EXPECT_EQ(model_artifact->produced_by_capability, "model.compile");
  EXPECT_EQ(trajectory_artifact->produced_by_capability, "sim.model");
  ASSERT_EQ(trajectory.state_ids, (std::vector<std::string>{"x"}));
  ASSERT_EQ(trajectory.output_ids, (std::vector<std::string>{"y"}));
  ASSERT_EQ(trajectory.times_s, (std::vector<double>{0.0, 0.375, 0.75, 1.0}));
  ASSERT_EQ(trajectory.states.size(), 4U);
  ASSERT_EQ(trajectory.outputs.size(), 4U);
  constexpr std::array<int, 4> ticks{0, 3, 6, 8};
  // B01: <=64 elementary operations/block/step across RK/output work, plus 32
  // for the independent recurrence; scale4 covers the affine contributions.
  const double operations =
      (64.0 * static_cast<double>(model.source_model().blocks.size()) + 32.0) * 9.0;
  const double accumulated = operations * std::numeric_limits<double>::epsilon();
  const double floating = 4.0 * accumulated / (1.0 - accumulated);
  for (std::size_t sample = 0; sample < ticks.size(); ++sample) {
    EXPECT_NEAR(trajectory.states[sample](0), affine_reference(ticks[sample]), floating);
    EXPECT_EQ(trajectory.outputs[sample](0), trajectory.states[sample](0));
  }

  const auto csv_bytes = read_file_bytes((output / "response.csv").string());
  std::istringstream csv(csv_bytes);
  csv.imbue(std::locale::classic());
  std::string line;
  ASSERT_TRUE(static_cast<bool>(std::getline(csv, line)));
  EXPECT_EQ(line, "time_s,state:x,output:y");
  std::size_t sample = 0;
  while (std::getline(csv, line)) {
    ASSERT_LT(sample, trajectory.times_s.size());
    std::istringstream row(line);
    row.imbue(std::locale::classic());
    double time = 0.0;
    double state = 0.0;
    double value = 0.0;
    char first_comma = '\0';
    char second_comma = '\0';
    ASSERT_TRUE(static_cast<bool>(row >> time >> first_comma >> state >> second_comma >> value));
    EXPECT_EQ(first_comma, ',');
    EXPECT_EQ(second_comma, ',');
    row >> std::ws;
    EXPECT_TRUE(row.eof());
    EXPECT_EQ(time, trajectory.times_s[sample]);
    EXPECT_EQ(state, trajectory.states[sample](0));
    EXPECT_EQ(value, trajectory.outputs[sample](0));
    ++sample;
  }
  EXPECT_EQ(sample, trajectory.times_s.size());

  const auto evidence_bytes = read_file_bytes((output / "evidence.json").string());
  const auto evidence = YAML::Load(evidence_bytes);
  EXPECT_EQ(evidence["schema"].as<std::string>(), "galata.model-run.v1");
  EXPECT_EQ(evidence["model_semantic_sha256"].as<std::string>(), model.semantic_sha256());
  EXPECT_EQ(evidence["model_source_yaml"].as<std::string>(),
            galata::modeling::write_model_yaml(model.source_model()));
  EXPECT_EQ(evidence["state_ids"].as<std::vector<std::string>>(), model.state_ids());
  EXPECT_EQ(evidence["output_ids"].as<std::vector<std::string>>(), model.output_ids());
  EXPECT_EQ(evidence["schedule_ids"].as<std::vector<std::string>>(), model.schedule_ids());
  EXPECT_EQ(evidence["solver"]["method"].as<std::string>(), "rk4");
  EXPECT_EQ(evidence["solver"]["initial_time_s"].as<double>(), 0.0);
  EXPECT_EQ(evidence["solver"]["step_s"].as<double>(), 0.125);
  EXPECT_EQ(evidence["solver"]["steps"].as<int>(), 8);
  EXPECT_EQ(evidence["solver"]["sample_stride"].as<int>(), 3);
  EXPECT_EQ(evidence["execution"].as<std::string>(), "completed");
  for (const auto* field : {"numerical_accuracy", "model_validity", "engineering_acceptance"}) {
    EXPECT_EQ(evidence[field].as<std::string>(), "not_assessed");
  }
  EXPECT_EQ(evidence["sample_count"].as<std::size_t>(), 4U);
  EXPECT_EQ(evidence["trajectory_sha256"].as<std::string>(), sha256(csv_bytes));
  EXPECT_EQ(fs::path(evidence["trajectory_path"].as<std::string>()).filename(), "response.csv");

  const auto manifest_bytes = read_file_bytes(result.manifest_path);
  EXPECT_EQ(fs::path(result.manifest_path).filename().string(),
            "run-" + sha256(manifest_bytes) + ".json");
  const auto manifest = YAML::Load(manifest_bytes);
  bool found_model = false;
  for (const auto& input : manifest["inputs"]) {
    if (input["path"].as<std::string>() == fs::canonical(root / "model.yaml").string()) {
      found_model = true;
      EXPECT_EQ(input["sha256"].as<std::string>(), sha256(raw_model));
      EXPECT_EQ(input["bytes_hex"].as<std::string>(), hex_bytes(raw_model));
    }
  }
  EXPECT_TRUE(found_model);
  ASSERT_EQ(manifest["outputs"].size(), 2U);
  const std::map<std::string, std::string> expected_outputs{
      {"response.csv", sha256(csv_bytes)}, {"evidence.json", sha256(evidence_bytes)}};
  for (const auto& record : manifest["outputs"]) {
    const auto name = fs::path(record["path"].as<std::string>()).filename().string();
    ASSERT_TRUE(expected_outputs.contains(name));
    EXPECT_EQ(record["sha256"].as<std::string>(), expected_outputs.at(name));
  }
}

TEST_F(ModelWorkflow, RepeatRunIdentityDistinguishesRawSourceBytesFromEquivalentSemantics) {
  const auto first = run(study(), {.overwrite = true});
  const auto first_csv = read_file_bytes((output / "response.csv").string());
  const auto first_evidence = read_file_bytes((output / "evidence.json").string());
  const auto repeated = run(study(), {.overwrite = true});
  EXPECT_EQ(repeated.manifest_path, first.manifest_path);
  EXPECT_EQ(read_file_bytes((output / "response.csv").string()), first_csv);
  EXPECT_EQ(read_file_bytes((output / "evidence.json").string()), first_evidence);

  put(root / "model.yaml", "# A source-byte change with identical equations.\n" + raw_model);
  const auto changed_raw = run(study(), {.overwrite = true});
  EXPECT_NE(changed_raw.manifest_path, first.manifest_path);
  EXPECT_EQ(read_file_bytes((output / "response.csv").string()), first_csv);
  EXPECT_EQ(read_file_bytes((output / "evidence.json").string()), first_evidence);
  EXPECT_EQ(first.find("model")
                ->payload_as<galata::modeling::CompiledModel>("executable_model")
                .semantic_sha256(),
            changed_raw.find("model")
                ->payload_as<galata::modeling::CompiledModel>("executable_model")
                .semantic_sha256());
}

TEST_F(ModelWorkflow, InvalidCountsMissingOutputsAndOversizedRunsPublishNothing) {
  const std::vector<std::string> invalid_inputs{
      "{model: {from: model}, step_s: 0.125, steps: 8.5, csv_path: response.csv, evidence_path: "
      "evidence.json}",
      "{model: {from: model}, step_s: 0.125, steps: 8, sample_stride: 1.5, csv_path: response.csv, "
      "evidence_path: evidence.json}",
      "{model: {from: model}, step_s: 0.125, steps: 8, evidence_path: evidence.json}",
      "{model: {from: model}, step_s: 0.125, steps: 8, csv_path: response.csv}",
      "{model: {from: model}, step_s: 0.125, steps: 1000001, csv_path: response.csv, "
      "evidence_path: evidence.json}",
      "{model: {from: model}, step_s: 0.125, steps: 333333, csv_path: response.csv, evidence_path: "
      "evidence.json}",
      "{model: {from: model}, step_s: 0.125, steps: 8, csv_path: response.csv, evidence_path: "
      "response.csv}"};
  for (const auto& input : invalid_inputs) {
    EXPECT_THROW((void)run(study(input), {.write_manifest = false}), std::runtime_error) << input;
    expect_no_published_outputs();
  }
}

TEST_F(ModelWorkflow, WrongUpstreamArtifactKindIsRejectedBeforeTrajectoryPublication) {
  auto registry = builtin_registry();
  Capability wrong;
  wrong.id = "test.unrelated";
  wrong.summary = "Produce an unrelated fixture artifact";
  wrong.produces = "unrelated";
  wrong.run = [](const StageContext&) {
    Artifact artifact;
    artifact.kind = "unrelated";
    artifact.payload = 1;
    return artifact;
  };
  registry.add(std::move(wrong));
  const auto document =
      "version: 1\nstages:\n"
      "  - id: model\n    capability: test.unrelated\n    input: {}\n"
      "  - id: response\n    capability: sim.model\n"
      "    input: {model: {from: model}, step_s: 0.125, steps: 8, csv_path: response.csv, "
      "evidence_path: evidence.json}\n";
  EXPECT_THROW((void)run(document, {.write_manifest = false}, registry), std::runtime_error);
  expect_no_published_outputs();
}

TEST_F(ModelWorkflow, ModelInputCannotBeOverwrittenThroughASimulationOutputRole) {
  output = root;
  const auto document = study(
      "{model: {from: model}, step_s: 0.125, steps: 8, csv_path: model.yaml, evidence_path: "
      "evidence.json}");
  EXPECT_THROW((void)run(document, {.overwrite = true, .write_manifest = false}),
               std::runtime_error);
  EXPECT_EQ(read_file_bytes((root / "model.yaml").string()), raw_model);
  EXPECT_FALSE(fs::exists(root / "evidence.json"));
}
}  // namespace
