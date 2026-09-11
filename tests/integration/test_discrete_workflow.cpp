// SPDX-License-Identifier: Apache-2.0
//
// The discrete-time capabilities through the pipeline, against their frozen
// contracts: `model.discretize`, `synth.dare` and `synth.sampled_lqr`, and the
// boundary between the two time domains that every one of them enforces a
// piece of.
//
// The references are the library's, carried up a level so that the SCHEMA is
// checked against them too: the scalar DARE's closed form, Riccati value
// iteration — a different algorithm, with no symplectic matrix and no Schur
// decomposition — and the refusals F14 names. A capability wrapper that passed
// the wrong matrix to a correct solver would pass every library test and fail
// here.
//
// Written from the capability schemas and the headers, not from the capability
// implementations (docs/TESTING.md).

#include "galata/model/discrete_system.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"
#include "galata/synth/discrete_control.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

namespace {
namespace fs = std::filesystem;
using namespace galata::pipeline;

class DiscreteWorkflow : public ::testing::Test {
 protected:
  fs::path root;
  fs::path output;

  void SetUp() override {
    static std::atomic<unsigned> sequence{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-discrete-workflow-" + std::to_string(tick) + "-"
              + std::to_string(sequence.fetch_add(1)));
    ASSERT_TRUE(fs::create_directory(root));
    output = root / "output";
    put(root / "plant.yaml", galata::model::serialize_linear_system(double_integrator()));
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

  RunResult run(const std::string& document) {
    const auto source = root / "study.yaml";
    put(source, document);
    return run_pipeline(load_pipeline(source.string()),
                        builtin_registry(),
                        root.string(),
                        output.string(),
                        nullptr,
                        RunOptions{.overwrite = true, .write_manifest = false});
  }

  // Refused, and the refusal says `expected` somewhere in it.
  void expect_refusal(const std::string& document, const std::string& expected) {
    try {
      (void)run(document);
      ADD_FAILURE() << "ran, and should have been refused:\n" << document;
    } catch (const std::runtime_error& error) {
      EXPECT_NE(std::string(error.what()).find(expected), std::string::npos)
          << "refused, but the message does not say '" << expected << "': " << error.what();
    }
  }

  static galata::model::LinearSystem double_integrator() {
    galata::model::LinearSystem system;
    system.a = Eigen::MatrixXd::Zero(2, 2);
    system.a(0, 1) = 1.0;
    system.b = Eigen::MatrixXd::Zero(2, 1);
    system.b(1, 0) = 1.0;
    system.state_names = {"position_m", "velocity_m_s"};
    system.input_names = {"acceleration_m_s2"};
    system.description = "Double integrator";
    system.citation = "Analytic test plant";
    system.units = "SI";
    return system;
  }

  static constexpr const char* kLoad =
      "version: 1\nstages:\n"
      "  - id: plant\n    capability: model.linear.statespace\n    input: {path: plant.yaml}\n";
};

// THE INDEPENDENT SOLVER, the same algorithm the library tests use and for the
// same reason: X <- A'XA + Q - (A'XB + N)(R + B'XB)^-1(B'XA + N') from X = 0
// reaches the stabilising solution by a route that shares no step with the
// ordered-Schur construction. A fixed count, per ADR-0004.
Eigen::MatrixXd value_iteration(const Eigen::MatrixXd& a,
                                const Eigen::MatrixXd& b,
                                const Eigen::MatrixXd& q,
                                const Eigen::MatrixXd& r,
                                const Eigen::MatrixXd& n) {
  Eigen::MatrixXd x = Eigen::MatrixXd::Zero(a.rows(), a.rows());
  for (int i = 0; i < 20000; ++i) {
    const Eigen::MatrixXd weighted = r + b.transpose() * x * b;
    const Eigen::MatrixXd coupling = a.transpose() * x * b + n;
    x = a.transpose() * x * a + q - coupling * weighted.llt().solve(coupling.transpose());
    x = 0.5 * (x + x.transpose()).eval();
  }
  return x;
}

double relative(const Eigen::MatrixXd& value, const Eigen::MatrixXd& reference) {
  return (value - reference).norm() / reference.norm();
}

}  // namespace

// The scalar problem a = 2, b = 1, q = 1, r = 1 reduces to x^2 - 4x - 1 = 0, so
// x = 2 + sqrt(5), the gain is (1 + sqrt(5)) / 2 and the closed-loop pole is
// (3 - sqrt(5)) / 2 — written as those expressions, not as decimals. Declared
// as bare matrices, which is admissible only WITH a sample time.
TEST_F(DiscreteWorkflow, TheScalarDareThroughThePipelineMatchesItsClosedForm) {
  const RunResult result =
      run("version: 1\nstages:\n"
          "  - id: dare\n    capability: synth.dare\n"
          "    input: {a: [[2]], b: [[1]], q: [[1]], r: [[1]], sample_time_s: 1.0}\n");
  const auto& dare = result.find("dare")->payload_as<DareArtifact>("dare_solution");
  const double root5 = std::sqrt(5.0);
  EXPECT_NEAR(dare.solution.x(0, 0), 2.0 + root5, 1e-12);
  EXPECT_NEAR(dare.solution.k(0, 0), 0.5 * (1.0 + root5), 1e-12);
  EXPECT_NEAR(dare.solution.spectral_radius, 0.5 * (3.0 - root5), 1e-12);
  EXPECT_EQ(dare.sample_time_s, 1.0);
  EXPECT_FALSE(dare.from_discrete_model);
  EXPECT_LE(dare.solution.relative_residual, dare.solution.residual_budget);
}

// The whole sampled path on an analytic plant, and the discrete Riccati
// equation posed on the explicit adapter's output with a declared cross term.
// Both against value iteration; the design's gain against the discrete gain
// formula; the cross term against the evidence file it must survive into.
TEST_F(DiscreteWorkflow, ASampledDesignThroughThePipelineAgreesWithValueIteration) {
  const RunResult result = run(
      std::string(kLoad)
      + "  - id: discrete\n    capability: model.discretize\n"
        "    input: {system: {from: plant}, sample_time_s: 0.05, hold: zero_order}\n"
        "  - id: dare\n    capability: synth.dare\n"
        "    input: {system: {from: discrete}, q: [[1, 0], [0, 1]], r: [[1]], "
        "n: [[0.1], [0.05]]}\n"
        "  - id: design\n    capability: synth.sampled_lqr\n"
        "    input: {system: {from: plant}, sample_time_s: 0.05, hold: zero_order, "
        "q: [[10, 0], [0, 1]], r: [[0.1]], evidence_path: design.yaml}\n"
        "  - id: report\n    capability: report.markdown\n"
        "    input: {sections: [{from: discrete}, {from: dare}, {from: design}], "
        "path: report.md}\n");

  // synth.dare, on the adapter's output, with the declared cross term.
  const auto& discretised =
      result.find("discrete")->payload_as<galata::model::Discretisation>("discrete_linear_system");
  const auto& dare = result.find("dare")->payload_as<DareArtifact>("dare_solution");
  EXPECT_TRUE(dare.from_discrete_model);
  EXPECT_EQ(dare.sample_time_s, 0.05);
  EXPECT_LT(
      relative(dare.solution.x,
               value_iteration(discretised.system.a, discretised.system.b, dare.q, dare.r, dare.n)),
      1e-9);

  // synth.sampled_lqr: the design is the DARE of the DISCRETISED problem.
  const auto& design =
      result.find("design")->payload_as<galata::synth::SampledLqrDesign>("sampled_control_law");
  const auto& ad = design.discretisation.system.a;
  const auto& bd = design.discretisation.system.b;
  EXPECT_EQ(ad, discretised.system.a) << "the adapter and the design discretise identically";
  EXPECT_EQ(bd, discretised.system.b);
  EXPECT_LT(relative(design.riccati.x,
                     value_iteration(ad, bd, design.cost.q, design.cost.r, design.cost.n)),
            1e-9);
  const Eigen::MatrixXd gain =
      (design.cost.r + bd.transpose() * design.riccati.x * bd)
          .llt()
          .solve(bd.transpose() * design.riccati.x * ad + design.cost.n.transpose());
  EXPECT_LT(relative(design.riccati.k, gain), 1e-12)
      << "the returned gain is K = (R + B'XB)^-1 (B'XA + N'), cross term included";
  EXPECT_LT(design.riccati.spectral_radius, 1.0);

  // THE CROSS TERM, exported. The continuous cost declared none; the hold's
  // own N must reach the file bit for bit, since max_digits10 round-trips.
  ASSERT_GT(design.cost.n.norm(), 0.0);
  const YAML::Node evidence = YAML::LoadFile((output / "design.yaml").string());
  EXPECT_EQ(evidence["schema"].as<std::string>(), "galata.sampled-lqr.v1");
  const YAML::Node written = evidence["discretised_cost"]["n"];
  ASSERT_EQ(written.size(), 2U);
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(written[i][0].as<double>(), design.cost.n(static_cast<Eigen::Index>(i), 0))
        << "discretised N row " << i;
  }
  EXPECT_EQ(evidence["riccati"]["relative_residual"].as<double>(),
            design.riccati.relative_residual);
  EXPECT_EQ(evidence["riccati"]["spectral_radius"].as<double>(), design.riccati.spectral_radius);
  EXPECT_TRUE(evidence["conventions"]["gain"]) << "the gain convention is stated in the file";
  EXPECT_TRUE(evidence["not_established"]);

  // And the report states the conventions and what the design does not give.
  std::ifstream report_file(output / "report.md");
  const std::string report((std::istreambuf_iterator<char>(report_file)),
                           std::istreambuf_iterator<char>());
  EXPECT_NE(report.find("Discretised cross term N"), std::string::npos);
  EXPECT_NE(report.find("u[k] = -K x[k]"), std::string::npos);
  EXPECT_NE(report.find("no capability here computes one"), std::string::npos);
}

// MIXED TIME DOMAINS ARE REJECTED WITHOUT AN EXPLICIT ADAPTER, in both
// directions, by name. The discrete refusals name the adapter; a discrete model
// reaching a continuous capability is refused by the artefact kind with both
// kinds in the message.
TEST_F(DiscreteWorkflow, EachTimeDomainRefusesTheOtherByName) {
  const std::string discrete =
      std::string(kLoad)
      + "  - id: discrete\n    capability: model.discretize\n"
        "    input: {system: {from: plant}, sample_time_s: 0.05, hold: zero_order}\n";

  expect_refusal(std::string(kLoad)
                     + "  - id: dare\n    capability: synth.dare\n"
                       "    input: {system: {from: plant}, q: [[1, 0], [0, 1]], r: [[1]]}\n",
                 "model.discretize");
  expect_refusal(discrete
                     + "  - id: design\n    capability: synth.sampled_lqr\n"
                       "    input: {system: {from: discrete}, sample_time_s: 0.05, "
                       "hold: zero_order, q: [[1, 0], [0, 1]], r: [[1]], evidence_path: d.yaml}\n",
                 "synth.dare");
  expect_refusal(discrete
                     + "  - id: again\n    capability: model.discretize\n"
                       "    input: {system: {from: discrete}, sample_time_s: 0.1, "
                       "hold: zero_order}\n",
                 "DISCRETE-time");
  expect_refusal(discrete
                     + "  - id: modes\n    capability: analyze.modes\n"
                       "    input: {system: {from: discrete}}\n",
                 "discrete_linear_system");
  expect_refusal(discrete
                     + "  - id: response\n    capability: sim.linear\n"
                       "    input: {system: {from: discrete}, step_s: 0.01, steps: 10}\n",
                 "discrete_linear_system");
}

// The sample time and the hold are the two facts a discrete model is a model
// OF. Neither has a default, and neither may be stated twice.
TEST_F(DiscreteWorkflow, TheSampleTimeAndTheHoldAreDeclaredAndNeverDefaulted) {
  const std::string stage =
      std::string(kLoad) + "  - id: discrete\n    capability: model.discretize\n    input: ";
  expect_refusal(stage + "{system: {from: plant}, hold: zero_order}\n", "sample_time_s");
  expect_refusal(stage + "{system: {from: plant}, sample_time_s: 0.05}\n", "hold");
  expect_refusal(stage + "{system: {from: plant}, sample_time_s: 0.05, hold: first_order}\n",
                 "not implemented");
  expect_refusal(stage + "{system: {from: plant}, sample_time_s: 0, hold: zero_order}\n",
                 "positive finite");
  expect_refusal(stage + "{system: {from: plant}, sample_time_s: -0.05, hold: zero_order}\n",
                 "positive finite");
  expect_refusal(std::string(kLoad)
                     + "  - id: design\n    capability: synth.sampled_lqr\n"
                       "    input: {system: {from: plant}, sample_time_s: 0.05, "
                       "q: [[1, 0], [0, 1]], r: [[1]], evidence_path: d.yaml}\n",
                 "hold");
  expect_refusal(std::string(kLoad)
                     + "  - id: design\n    capability: synth.sampled_lqr\n"
                       "    input: {system: {from: plant}, sample_time_s: 0.05, hold: zero_order, "
                       "q: [[1, 0], [0, 1]], r: [[1]]}\n",
                 "evidence_path");

  const std::string dare = "version: 1\nstages:\n  - id: dare\n    capability: synth.dare\n";
  expect_refusal(dare + "    input: {a: [[2]], b: [[1]], q: [[1]], r: [[1]]}\n", "sample_time_s");
  expect_refusal(std::string(kLoad)
                     + "  - id: discrete\n    capability: model.discretize\n"
                       "    input: {system: {from: plant}, sample_time_s: 0.05, hold: zero_order}\n"
                       "  - id: dare\n    capability: synth.dare\n"
                       "    input: {system: {from: discrete}, sample_time_s: 0.05, "
                       "q: [[1, 0], [0, 1]], r: [[1]]}\n",
                 "already carries one");
  expect_refusal(dare + "    input: {q: [[1]], r: [[1]]}\n", "exactly one of");
}

// STRICT SCHEMAS. The closed vocabulary is checked before any stage runs, so a
// misspelt or invented key is refused rather than ignored.
TEST_F(DiscreteWorkflow, UnknownKeysAreRefusedBeforeAnythingRuns) {
  expect_refusal(
      "version: 1\nstages:\n  - id: dare\n    capability: synth.dare\n"
      "    input: {a: [[2]], b: [[1]], q: [[1]], r: [[1]], sample_time_s: 1.0, "
      "tolerance: 1e-9}\n",
      "unknown input key 'tolerance'");
  expect_refusal(std::string(kLoad)
                     + "  - id: discrete\n    capability: model.discretize\n"
                       "    input: {system: {from: plant}, sample_time_s: 0.05, "
                       "method: tustin}\n",
                 "unknown input key 'method'");
  expect_refusal(std::string(kLoad)
                     + "  - id: design\n    capability: synth.sampled_lqr\n"
                       "    input: {system: {from: plant}, sample_time_s: 0.05, hold: zero_order, "
                       "q: [[1, 0], [0, 1]], r: [[1]], evidence_path: d.yaml, "
                       "break_at: plant_input}\n",
                 "unknown input key 'break_at'");
}

// F14's acceptance cases through the schema: stabilisable and detectable
// problems solve, an unstabilisable one, an undetectable one and an invalid
// cost are refused. Weights of the wrong shape are refused in the plant's own
// terms before the solver is reached.
TEST_F(DiscreteWorkflow, UnstabilisableUndetectableAndInvalidCostsAreRefused) {
  const std::string dare = "version: 1\nstages:\n  - id: dare\n    capability: synth.dare\n";
  // A stable mode left unpenalised is detectable, and solves.
  const RunResult stable = run(dare
                               + "    input: {a: [[0.5]], b: [[1]], q: [[0]], r: [[1]], "
                                 "sample_time_s: 0.1}\n");
  EXPECT_LT(stable.find("dare")->payload_as<DareArtifact>("dare_solution").solution.spectral_radius,
            1.0);

  expect_refusal(dare
                     + "    input: {a: [[2, 0], [0, 0.5]], b: [[0], [1]], q: [[1, 0], [0, 1]], "
                       "r: [[1]], sample_time_s: 0.1}\n",
                 "synth.dare");
  expect_refusal(dare + "    input: {a: [[2]], b: [[1]], q: [[0]], r: [[1]], sample_time_s: 0.1}\n",
                 "unpenalised");
  expect_refusal(
      dare + "    input: {a: [[2]], b: [[1]], q: [[1]], r: [[-1]], sample_time_s: 0.1}\n",
      "positive definite");
  expect_refusal(dare
                     + "    input: {a: [[1, 0.1], [0, 1]], b: [[0], [1]], q: [[1, 2], [0, 1]], "
                       "r: [[1]], sample_time_s: 0.1}\n",
                 "symmetric");
  expect_refusal(dare
                     + "    input: {a: [[1, 0.1], [0, 1]], b: [[0], [1]], q: [[1]], r: [[1]], "
                       "sample_time_s: 0.1}\n",
                 "must be 2x2");
}

// COMPLETE EVIDENCE PROPAGATION. A fixed-wing linearisation carries its
// Jacobian's diagnostics as `linearization_evidence`, and that record must
// survive the time-domain adapter and the sampled design as the SAME object —
// not a copy, which could drift, and not dropped, which would leave a discrete
// gain with no account of the matrices it came from. The design's evidence file
// names the stage that took them.
TEST_F(DiscreteWorkflow, LinearisationEvidenceSurvivesDiscretisationAndDesign) {
  const std::string model = (fs::path(GALATA_MODELS_DIR) / "nt33a" / "nt33a-fc1.yaml").string();
  const RunResult result = run(
      "version: 1\nstages:\n"
      "  - id: aircraft\n    capability: model.aircraft.derivatives\n    input: {path: "
      + model
      + "}\n"
        "  - id: trim\n    capability: trim.level\n"
        "    input: {aircraft: {from: aircraft}, altitude_m: 0, airspeed_m_s: 69.4944}\n"
        "  - id: full_plant\n    capability: linearize.finitediff\n"
        "    input: {trim_point: {from: trim}, axes: longitudinal}\n"
        "  - id: plant\n    capability: model.channels\n"
        "    input: {system: {from: full_plant}, inputs: [elevator]}\n"
        "  - id: discrete\n    capability: model.discretize\n"
        "    input: {system: {from: plant}, sample_time_s: 0.02, hold: zero_order}\n"
        "  - id: design\n    capability: synth.sampled_lqr\n"
        "    input: {system: {from: plant}, sample_time_s: 0.02, hold: zero_order, "
        "q: [[0.001, 0, 0, 0], [0, 0.01, 0, 0], [0, 0, 1, 0], [0, 0, 0, 10]], r: [[1]], "
        "evidence_path: design.yaml}\n");

  const auto& source = result.find("full_plant")->linearization_evidence;
  ASSERT_EQ(source.count("full_plant"), 1U);
  for (const char* stage : {"discrete", "design"}) {
    const auto& carried = result.find(stage)->linearization_evidence;
    ASSERT_EQ(carried.count("full_plant"), 1U) << stage << " dropped the linearisation evidence";
    EXPECT_EQ(carried.at("full_plant"), source.at("full_plant"))
        << stage << " carries a different object from the one the linearisation produced";
  }
  const YAML::Node evidence = YAML::LoadFile((output / "design.yaml").string());
  const YAML::Node sources = evidence["plant"]["upstream_linearization_evidence"];
  ASSERT_EQ(sources.size(), 1U);
  EXPECT_EQ(sources[0].as<std::string>(), "full_plant");
}
