// SPDX-License-Identifier: Apache-2.0
// Product contracts: runnable study, physical model -> design -> time histories;
// invalid wiring/configuration must fail rather than produce a plausible report.
#include "galata/pipeline/pipeline.hpp"
#include "galata/sim/nonlinear.hpp"
#include "galata/synth/control.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>

namespace {
std::string read(const std::filesystem::path& path) {
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

TEST(DesignWorkflow, AircraftStudyProducesAStabilisingLawAndCompletedTimeHistories) {
  const auto base = std::filesystem::path(GALATA_EXAMPLES_DIR) / "nt33a-control-design";
  const auto output = std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR) / "control-design";
  auto pipeline = galata::pipeline::load_pipeline((base / "study.yaml").string());
  galata::pipeline::RunOptions options;
  options.overwrite = true;
  const auto result = galata::pipeline::run_pipeline(pipeline,
                                                     galata::pipeline::builtin_registry(),
                                                     base.string(),
                                                     output.string(),
                                                     nullptr,
                                                     options);
  const auto* law_artifact = result.find("law");
  ASSERT_NE(law_artifact, nullptr);
  const auto& law = law_artifact->payload_as<galata::synth::LqrDesign>("control_law");
  EXPECT_EQ(law.plant.input_names, std::vector<std::string>{"elevator"});
  for (const auto pole : law.riccati.closed_loop_eigenvalues) {
    EXPECT_LT(pole.real(), 0);
  }
  EXPECT_LE(law.riccati.relative_residual, law.riccati.residual_budget);
  const auto* response = result.find("nonlinear_response");
  ASSERT_NE(response, nullptr);
  const auto& trajectory =
      response->payload_as<galata::sim::NonlinearResult>("nonlinear_trajectory");
  EXPECT_TRUE(trajectory.completed);
  EXPECT_FALSE(trajectory.outside_envelope_encountered);
  ASSERT_FALSE(trajectory.samples.empty());
  EXPECT_DOUBLE_EQ(trajectory.samples.back().time_s, 20);
  EXPECT_TRUE(std::filesystem::exists(result.manifest_path));
  const auto report = read(output / "control-design.md");
  EXPECT_NE(report.find("Relative CARE residual"), std::string::npos);
  EXPECT_NE(report.find("Position-limited steps"), std::string::npos);
  EXPECT_EQ(report.find("No Markdown writer"), std::string::npos);
  EXPECT_NE(report.find("Conservative alpha lower bound"), std::string::npos);
  const auto html = read(output / "control-design.html");
  EXPECT_NE(html.find("<!doctype html>"), std::string::npos);
  EXPECT_NE(html.find("Relative CARE residual"), std::string::npos);
  const auto csv = read(output / "nonlinear-response.csv");
  EXPECT_NE(csv.find("elevator_rad"), std::string::npos);
  EXPECT_EQ(csv.find("nan"), std::string::npos);
}

TEST(DesignWorkflow, SynthesisRejectsInvalidCostDimensions) {
  const auto text = R"(version: 1
stages:
  - id: care
    capability: synth.care
    input: {a: [[0,1],[0,0]], b: [[0],[1]], q: [[1]], r: [[1]]}
)";
  const auto pipeline = galata::pipeline::parse_pipeline(text);
  const auto output = std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR) / "invalid-design";
  EXPECT_THROW((void)galata::pipeline::run_pipeline(
                   pipeline, galata::pipeline::builtin_registry(), ".", output.string()),
               std::runtime_error);
}

TEST(DesignWorkflow, SynthesisRequiresAnExplicitLoopBreak) {
  const auto base = std::filesystem::path(GALATA_EXAMPLES_DIR) / "nt33a-control-design";
  auto text = read(base / "study.yaml");
  const std::string declaration = "      break_at: plant_input\n";
  const auto index = text.find(declaration);
  ASSERT_NE(index, std::string::npos);
  text.erase(index, declaration.size());
  const auto pipeline = galata::pipeline::parse_pipeline(text);
  const auto output = std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR) / "missing-break";
  galata::pipeline::RunOptions options;
  options.overwrite = true;
  EXPECT_THROW((void)galata::pipeline::run_pipeline(pipeline,
                                                    galata::pipeline::builtin_registry(),
                                                    base.string(),
                                                    output.string(),
                                                    nullptr,
                                                    options),
               std::runtime_error);
}
}  // namespace
