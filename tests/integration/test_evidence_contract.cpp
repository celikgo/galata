// SPDX-License-Identifier: Apache-2.0
// Evidence is a public capability boundary: source identity, finite values and
// matrix meaning must survive extension registration and downstream execution.
#include "galata/linearize/finite_difference.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {
namespace fs = std::filesystem;
using galata::linearize::Linearisation;
using namespace galata::pipeline;

// An independently specified scalar source: dx/dt = -2*x + u, y = x.
// These records exercise artifact contracts, not an aircraft-model oracle.
Linearisation scalar_evidence() {
  Linearisation record;
  record.a = Eigen::MatrixXd::Constant(1, 1, -2.0);
  record.b = Eigen::MatrixXd::Ones(1, 1);
  record.c = Eigen::MatrixXd::Ones(1, 1);
  record.d = Eigen::MatrixXd::Zero(1, 1);
  record.state_names = {"u"};
  record.input_names = {"elevator"};
  record.state_steps = Eigen::VectorXd::Constant(1, 1e-4);
  record.control_steps = Eigen::VectorXd::Constant(1, 1e-5);
  record.a_truncation = Eigen::MatrixXd::Constant(1, 1, 1e-9);
  record.b_truncation = Eigen::MatrixXd::Constant(1, 1, 1e-10);
  record.worst_relative_truncation = 5e-10;
  record.chart_conditioning = 1.0;
  record.neglected_coupling = 0.0;
  record.trim_altitude_m = 1000.0;
  record.trim_airspeed_m_s = 60.0;
  record.trim_alpha_rad = 0.1;
  record.trim_delta_isa_k = 0.0;
  record.trim_residual_norm = 0.0;
  record.trim_residual_tolerance = 1e-10;
  return record;
}

Artifact source_artifact(const std::string& source, std::shared_ptr<const Linearisation> record) {
  Artifact artifact;
  artifact.kind = "evidence_fixture";
  artifact.linearization_evidence.emplace(source, std::move(record));
  return artifact;
}

void add(Registry& registry,
         const std::string& id,
         CapabilityFunction function,
         std::vector<std::string> inputs = {}) {
  Capability capability;
  capability.id = id;
  capability.produces = "evidence_fixture";
  capability.run = std::move(function);
  capability.input_keys = std::move(inputs);
  registry.add(std::move(capability));
}

class EvidenceContract : public ::testing::Test {
 protected:
  fs::path root;
  unsigned run_sequence = 0;

  void SetUp() override {
    static std::atomic<unsigned> sequence{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-evidence-contract-" + std::to_string(tick) + "-"
              + std::to_string(sequence.fetch_add(1)));
    ASSERT_TRUE(fs::create_directory(root));
  }

  void TearDown() override {
    std::error_code error;
    fs::remove_all(root, error);
  }

  RunResult run(const std::string& yaml,
                const Registry& registry,
                const ProgressCallback& progress = nullptr,
                bool manifest = false) {
    return run_pipeline(parse_pipeline(yaml),
                        registry,
                        root.string(),
                        (root / ("run-" + std::to_string(run_sequence++))).string(),
                        progress,
                        RunOptions{.write_manifest = manifest});
  }
};

TEST_F(EvidenceContract, PreservesScalarEvidenceThroughAutomaticAndExplicitInheritance) {
  Registry registry;
  const auto evidence = std::make_shared<const Linearisation>(scalar_evidence());
  add(registry, "fixture.source", [evidence](const StageContext& context) {
    return source_artifact(context.stage_id, evidence);
  });
  add(registry,
      "fixture.automatic",
      [](const StageContext&) {
        Artifact artifact;
        artifact.kind = "evidence_fixture";
        return artifact;
      },
      {"upstream"});
  add(registry,
      "fixture.explicit",
      [](const StageContext& context) { return context.upstream_at("upstream"); },
      {"upstream"});
  const auto result = run(R"(
version: 1
stages:
  - {id: source, capability: fixture.source}
  - id: automatic
    capability: fixture.automatic
    input: {upstream: {from: source}}
  - id: explicit
    capability: fixture.explicit
    input: {upstream: {from: automatic}}
)",
                          registry);
  ASSERT_EQ(result.stages.size(), 3U);
  for (const auto& stage : result.stages) {
    ASSERT_EQ(stage.artifact.linearization_evidence.size(), 1U);
    EXPECT_EQ(stage.artifact.linearization_evidence.at("source"), evidence);
    EXPECT_DOUBLE_EQ(stage.artifact.linearization_evidence.at("source")->a(0, 0), -2.0);
  }
}

TEST_F(EvidenceContract, RejectsAnOriginForgedByAnIndependentStageBeforeCompletion) {
  Registry registry;
  add(registry, "fixture.source", [](const StageContext& context) {
    return source_artifact(context.stage_id,
                           std::make_shared<const Linearisation>(scalar_evidence()));
  });
  add(registry, "fixture.forgery", [](const StageContext&) {
    auto different = scalar_evidence();
    different.a(0, 0) = -3.0;
    return source_artifact("source", std::make_shared<const Linearisation>(different));
  });
  std::vector<std::string> completed;
  const ProgressCallback progress =
      [&completed](const std::string& id, const std::string&, bool finished, const std::string&) {
        if (finished) {
          completed.push_back(id);
        }
      };
  EXPECT_THROW((void)run(R"(
version: 1
stages:
  - {id: source, capability: fixture.source}
  - {id: independent, capability: fixture.forgery}
)",
                         registry,
                         progress),
               std::runtime_error);
  EXPECT_EQ(completed, std::vector<std::string>{"source"});
}

TEST_F(EvidenceContract, RejectsReplacingInheritedEvidenceEvenWithEqualNumericValues) {
  Registry registry;
  add(registry, "fixture.source", [](const StageContext& context) {
    return source_artifact(context.stage_id,
                           std::make_shared<const Linearisation>(scalar_evidence()));
  });
  add(registry,
      "fixture.replacement",
      [](const StageContext&) {
        return source_artifact("source", std::make_shared<const Linearisation>(scalar_evidence()));
      },
      {"upstream"});
  EXPECT_THROW((void)run(R"(
version: 1
stages:
  - {id: source, capability: fixture.source}
  - id: replacement
    capability: fixture.replacement
    input: {upstream: {from: source}}
)",
                         registry),
               std::runtime_error);
}

TEST_F(EvidenceContract, RejectsNonfiniteEvidenceBeforeStageCompletionWithoutAManifest) {
  using Change = std::function<void(Linearisation&, double)>;
  const std::vector<std::pair<std::string, Change>> changes = {
      {"a", [](Linearisation& r, double v) { r.a(0, 0) = v; }},
      {"b", [](Linearisation& r, double v) { r.b(0, 0) = v; }},
      {"c", [](Linearisation& r, double v) { r.c(0, 0) = v; }},
      {"d", [](Linearisation& r, double v) { r.d(0, 0) = v; }},
      {"state_steps", [](Linearisation& r, double v) { r.state_steps(0) = v; }},
      {"control_steps", [](Linearisation& r, double v) { r.control_steps(0) = v; }},
      {"a_truncation", [](Linearisation& r, double v) { r.a_truncation(0, 0) = v; }},
      {"b_truncation", [](Linearisation& r, double v) { r.b_truncation(0, 0) = v; }},
      {"worst_relative_truncation",
       [](Linearisation& r, double v) { r.worst_relative_truncation = v; }},
      {"chart_conditioning", [](Linearisation& r, double v) { r.chart_conditioning = v; }},
      {"neglected_coupling", [](Linearisation& r, double v) { r.neglected_coupling = v; }},
      {"trim_altitude_m", [](Linearisation& r, double v) { r.trim_altitude_m = v; }},
      {"trim_airspeed_m_s", [](Linearisation& r, double v) { r.trim_airspeed_m_s = v; }},
      {"trim_alpha_rad", [](Linearisation& r, double v) { r.trim_alpha_rad = v; }},
      {"trim_delta_isa_k", [](Linearisation& r, double v) { r.trim_delta_isa_k = v; }},
      {"trim_residual_norm", [](Linearisation& r, double v) { r.trim_residual_norm = v; }},
      {"trim_residual_tolerance",
       [](Linearisation& r, double v) { r.trim_residual_tolerance = v; }},
  };
  for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
    for (const auto& [name, change] : changes) {
      SCOPED_TRACE(name);
      SCOPED_TRACE(invalid);
      auto record = scalar_evidence();
      change(record, invalid);
      Registry registry;
      add(registry, "fixture.source", [record](const StageContext& context) {
        return source_artifact(context.stage_id, std::make_shared<const Linearisation>(record));
      });
      bool completed = false;
      const ProgressCallback progress =
          [&completed](const std::string&, const std::string&, bool finished, const std::string&) {
            completed = completed || finished;
          };
      EXPECT_THROW((void)run("version: 1\nstages: [{id: source, capability: fixture.source}]\n",
                             registry,
                             progress),
                   std::runtime_error);
      EXPECT_FALSE(completed);
    }
  }
}

TEST_F(EvidenceContract, RejectsInconsistentMatrixNamesStepsAndErrorShapes) {
  using Change = std::function<void(Linearisation&)>;
  const std::vector<std::pair<std::string, Change>> changes = {
      {"nonsquare a", [](Linearisation& r) { r.a = Eigen::MatrixXd::Zero(2, 1); }},
      {"b state dimension", [](Linearisation& r) { r.b = Eigen::MatrixXd::Zero(2, 1); }},
      {"c state dimension", [](Linearisation& r) { r.c = Eigen::MatrixXd::Zero(1, 2); }},
      {"d output dimension", [](Linearisation& r) { r.d = Eigen::MatrixXd::Zero(2, 1); }},
      {"state names", [](Linearisation& r) { r.state_names.push_back("v"); }},
      {"input names", [](Linearisation& r) { r.input_names.push_back("aileron"); }},
      {"state steps", [](Linearisation& r) { r.state_steps = Eigen::VectorXd::Ones(2); }},
      {"control steps", [](Linearisation& r) { r.control_steps = Eigen::VectorXd::Ones(2); }},
      {"a error", [](Linearisation& r) { r.a_truncation = Eigen::MatrixXd::Zero(2, 1); }},
      {"b error", [](Linearisation& r) { r.b_truncation = Eigen::MatrixXd::Zero(1, 2); }},
  };
  for (const auto& [name, change] : changes) {
    SCOPED_TRACE(name);
    auto record = scalar_evidence();
    change(record);
    Registry registry;
    add(registry, "fixture.source", [record](const StageContext& context) {
      return source_artifact(context.stage_id, std::make_shared<const Linearisation>(record));
    });
    EXPECT_THROW(
        (void)run("version: 1\nstages: [{id: source, capability: fixture.source}]\n", registry),
        std::runtime_error);
  }
}

TEST_F(EvidenceContract, SerializesDistinctIndependentOriginsAndTheirInheritedUnion) {
  Registry registry;
  add(registry, "fixture.source", [](const StageContext& context) {
    auto record = scalar_evidence();
    record.a(0, 0) = context.stage_id == "first" ? -2.0 : -3.0;
    return source_artifact(context.stage_id, std::make_shared<const Linearisation>(record));
  });
  add(registry,
      "fixture.merge",
      [](const StageContext&) {
        Artifact artifact;
        artifact.kind = "evidence_fixture";
        return artifact;
      },
      {"left", "right"});
  const auto result = run(R"(
version: 1
stages:
  - {id: first, capability: fixture.source}
  - {id: second, capability: fixture.source}
  - id: merge
    capability: fixture.merge
    input: {left: {from: first}, right: {from: second}}
)",
                          registry,
                          nullptr,
                          true);
  ASSERT_EQ(result.stages.size(), 3U);
  EXPECT_EQ(result.stages.back().artifact.linearization_evidence.size(), 2U);
  ASSERT_FALSE(result.manifest_path.empty());
  const auto manifest = YAML::Load(read_file_bytes(result.manifest_path));
  const auto evidence = manifest["linearization_evidence"];
  ASSERT_EQ(evidence.size(), 2U);
  EXPECT_DOUBLE_EQ(evidence["first"]["a"][0][0].as<double>(), -2.0);
  EXPECT_DOUBLE_EQ(evidence["second"]["a"][0][0].as<double>(), -3.0);
  EXPECT_DOUBLE_EQ(evidence["first"]["state_steps"][0][0].as<double>(), 1e-4);
  EXPECT_DOUBLE_EQ(evidence["second"]["equilibrium_tolerance"].as<double>(), 1e-10);
  EXPECT_EQ(manifest["stage_linearization_sources"]["merge"].as<std::vector<std::string>>(),
            (std::vector<std::string>{"first", "second"}));
}
}  // namespace
