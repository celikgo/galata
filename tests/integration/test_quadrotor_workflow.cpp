// SPDX-License-Identifier: Apache-2.0
//
// The multirotor chain end to end, against its documented contract:
// model.quadrotor -> trim.hover -> linearize.extended -> model.linear.export,
// the exported file read back by model.linear.statespace, and the result
// consumed unchanged by the analysis and simulation capabilities that already
// existed. Completing this chain is a statement about wiring; it is not a
// validation of any aircraft.
//
// Written from the capability schemas and the headers, not from the capability
// implementations (docs/TESTING.md).

#include "galata/analyze/gramians.hpp"
#include "galata/analyze/margins.hpp"
#include "galata/core/state.hpp"
#include "galata/linearize/extended.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/modeling/linear_adapter.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"
#include "galata/synth/control.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using namespace galata::pipeline;

class QuadrotorWorkflow : public ::testing::Test {
 protected:
  fs::path root;
  fs::path output;

  void SetUp() override {
    // A unique scratch directory per test. RFC-0002's housekeeping section
    // records a two-worker ctest race caused by a shared one; this does not
    // repeat it.
    static std::atomic<unsigned> sequence{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-quadrotor-workflow-" + std::to_string(tick) + "-"
              + std::to_string(sequence.fetch_add(1)));
    ASSERT_TRUE(fs::create_directory(root));
    output = root / "output";
    const std::string model =
        read_file_bytes((fs::path(GALATA_MODELS_DIR) / "souxmar-quad/souxmar-quad.yaml").string());
    put(root / "quad.yaml", model);
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

  RunResult run(const std::string& document,
                RunOptions options = {},
                const std::string& output_directory = {}) {
    const auto source = root / "study.yaml";
    put(source, document);
    return run_pipeline(load_pipeline(source.string()),
                        builtin_registry(),
                        root.string(),
                        output_directory.empty() ? output.string() : output_directory,
                        nullptr,
                        options);
  }

  // The producing study. It stops at the export, because the pipeline
  // snapshots every declared input file BEFORE any stage runs — so a study
  // cannot read a file one of its own stages is about to write. Consuming the
  // export is a second study, which is also how the requesting programme
  // actually uses it: one run computes the model, a later one analyses it.
  // A diagonal weight matrix as study YAML. The weights are the study's choice
  // wherever they appear, and writing them out keeps that visible.
  static std::string identity_weight(int size, double value) {
    std::ostringstream out;
    out << "[";
    for (int row = 0; row < size; ++row) {
      out << (row == 0 ? "[" : ", [");
      for (int column = 0; column < size; ++column) {
        out << (column == 0 ? "" : ", ") << (row == column ? value : 0.0);
      }
      out << "]";
    }
    out << "]";
    return out.str();
  }

  static std::string chain(const std::string& trim_input = "{quadrotor: {from: plant}}") {
    return "version: 1\nstages:\n"
           "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
           "  - id: hover\n    capability: trim.hover\n    input: "
           + trim_input
           + "\n"
             "  - id: linear\n    capability: linearize.extended\n    input: "
             "{trim: {from: hover}, evidence_path: operating-point.yaml}\n"
             "  - id: exported\n    capability: model.linear.export\n    input: "
             "{system: {from: linear}, path: quad-hover.yaml}\n";
  }

  // The consuming study: the existing loader, then the existing analysis and the
  // existing simulation, both unchanged. RFC-0002 requires that `analyze.*` and
  // `sim.linear` consume the quadrotor without modification, so the point of
  // this chain is that nothing in it was written for a multirotor.
  static std::string reload_chain() {
    return "version: 1\nstages:\n"
           "  - id: reloaded\n    capability: model.linear.statespace\n    input: "
           "{path: output/quad-hover.yaml}\n"
           "  - id: modes\n    capability: analyze.modes\n    input: "
           "{system: {from: reloaded}, classify: false}\n"
           "  - id: response\n    capability: sim.linear\n    input: "
           "{system: {from: reloaded}, step_s: 0.002, steps: 500, sample_stride: 25}\n";
  }
};

TEST_F(QuadrotorWorkflow, TheChainRunsAndTheExportIsReadBackByTheExistingLoader) {
  const RunResult result = run(chain());

  const Artifact* linear = result.find("linear");
  ASSERT_NE(linear, nullptr);
  const auto& computed = linear->payload_as<galata::model::LinearSystem>("linear_system");
  EXPECT_EQ(computed.state_count(), 16) << "twelve chart coordinates and four rotor states";
  EXPECT_EQ(computed.input_count(), 7) << "four rotor commands and three named wind columns";
  EXPECT_EQ(computed.output_count(), 13);

  // The wind columns are NAMED, which is what lets model.channels select or
  // drop the disturbance rather than guess at it by position.
  EXPECT_EQ(computed.input_names[4], "wind_north_m_s");
  EXPECT_EQ(computed.input_names[5], "wind_east_m_s");
  EXPECT_EQ(computed.input_names[6], "wind_down_m_s");

  // The exported file is the shape model.linear.statespace already reads, and
  // reading it back gives the same matrices to the last bit: the serializer
  // writes at max_digits10 precisely so this holds.
  ASSERT_TRUE(fs::exists(output / "quad-hover.yaml"));
  const RunResult consumed = run(reload_chain(), {}, (root / "analysis").string());
  const Artifact* reloaded = consumed.find("reloaded");
  ASSERT_NE(reloaded, nullptr);
  const auto& read_back = reloaded->payload_as<galata::model::LinearSystem>("linear_system");
  ASSERT_EQ(read_back.state_count(), computed.state_count());
  EXPECT_TRUE(read_back.a.isApprox(computed.a, 0.0)) << "A did not survive the round trip";
  EXPECT_TRUE(read_back.b.isApprox(computed.b, 0.0)) << "B did not survive the round trip";
  EXPECT_TRUE(read_back.c.isApprox(computed.c, 0.0)) << "C did not survive the round trip";
  EXPECT_TRUE(read_back.d.isApprox(computed.d, 0.0)) << "D did not survive the round trip";
  EXPECT_EQ(read_back.state_names, computed.state_names);
  EXPECT_EQ(read_back.input_names, computed.input_names);
  EXPECT_EQ(read_back.output_names, computed.output_names);

  // Downstream analysis and simulation consume it unchanged. `classify: false`
  // because these are not the fixed-wing modes and RFC-0002 says the labels must
  // not be applied to a multirotor.
  EXPECT_NE(consumed.find("modes"), nullptr);
  const Artifact* response = consumed.find("response");
  ASSERT_NE(response, nullptr) << "sim.linear must integrate a multirotor model unchanged";
  EXPECT_EQ(response->kind, "linear_trajectory");
}

TEST_F(QuadrotorWorkflow, TheOperatingPointTravelsWithTheMatricesAsItsOwnEvidenceFile) {
  (void)run(chain());

  const fs::path evidence = output / "operating-point.yaml";
  ASSERT_TRUE(fs::exists(evidence));
  const YAML::Node node = YAML::Load(read_file_bytes(evidence.string()));
  EXPECT_EQ(node["schema"].as<std::string>(), "galata.operating-point.v1");
  EXPECT_EQ(node["stage"].as<std::string>(), "linear");

  // Charter rule 9: the evidence that the point was an equilibrium, and how far
  // to trust the differentiation of it, cannot be separated from the matrices.
  ASSERT_TRUE(node["trim"]);
  EXPECT_LE(node["trim"]["residual_norm"].as<double>(),
            node["trim"]["residual_tolerance"].as<double>());
  EXPECT_GT(node["trim"]["smallest_rotor_margin_fraction"].as<double>(), 0.0);
  EXPECT_EQ(node["trim"]["extended_state"].size(), 17u);

  // The shipped model carries no battery, so nothing was frozen and the
  // residual covers every coordinate it did not deliberately exclude. The key
  // is present and empty rather than absent: a reader must be able to tell
  // "nothing was frozen" from "this writer does not report freezing".
  ASSERT_TRUE(node["trim"]["frozen_states"]);
  EXPECT_EQ(node["trim"]["frozen_states"].size(), 0u);

  ASSERT_TRUE(node["linearisation"]);
  EXPECT_LE(node["linearisation"]["equilibrium_residual_norm"].as<double>(),
            node["linearisation"]["equilibrium_tolerance"].as<double>());
  // All four matrices are qualified, not only A and B.
  for (const char* key : {"worst_relative_truncation_a",
                          "worst_relative_truncation_b",
                          "worst_relative_truncation_c",
                          "worst_relative_truncation_d"}) {
    ASSERT_TRUE(node["linearisation"][key]) << key;
    EXPECT_GE(node["linearisation"][key].as<double>(), 0.0) << key;
  }
}

TEST_F(QuadrotorWorkflow, TheDeclaredConditionReachesTheAnswerAndAnUnknownKeyIsRefused) {
  const RunResult result =
      run(chain("{quadrotor: {from: plant}, altitude_m: 80.0, wind_ned_m_s: [5.0, 0.0, 0.0], "
                "heading_deg: 90.0}"));
  const Artifact* linear = result.find("linear");
  ASSERT_NE(linear, nullptr);

  const YAML::Node node = YAML::Load(read_file_bytes((output / "operating-point.yaml").string()));
  EXPECT_DOUBLE_EQ(node["trim"]["altitude_m"].as<double>(), 80.0);
  // Degrees at the boundary, radians below it: ninety degrees is pi/2.
  EXPECT_NEAR(node["trim"]["yaw_rad"].as<double>(), 1.5707963267948966, 1e-12);
  EXPECT_NEAR(node["trim"]["airspeed_m_s"].as<double>(), 5.0, 1e-9);
  // Down is the negative of altitude, in the state the matrices were taken about.
  EXPECT_DOUBLE_EQ(node["trim"]["extended_state"][2].as<double>(), -80.0);

  // The schema is closed. A key nobody declared is an error, not a value
  // silently ignored — which is how a study that thinks it set a condition ends
  // up reporting one it did not.
  EXPECT_THROW((void)run(chain("{quadrotor: {from: plant}, altitude_ft: 260.0}"),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error);
  EXPECT_THROW((void)run("version: 1\nstages:\n"
                         "  - id: plant\n    capability: model.quadrotor\n"
                         "    input: {path: quad.yaml}\n"
                         "  - id: hover\n    capability: trim.hover\n"
                         "    input: {quadrotor: {from: plant}, wind_ned_m_s: [1.0, 2.0]}\n",
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error)
      << "a two-component NED vector must be refused, not padded";
}

TEST_F(QuadrotorWorkflow, TheSeventeenStateBatteryVariantLowersThroughTheTypedGraphAdapter) {
  // ADR-0013's cap was sixteen, which the fixed-voltage model sits exactly on,
  // so one battery state used to put this variant one over and cost it the
  // typed linear-graph path. The amendment raised the cap; this is the gate on
  // that, at the width that actually arises rather than at a round number.
  EXPECT_GE(galata::modeling::kMaxLinearChannels, 17u);

  std::string battery_model =
      read_file_bytes((fs::path(GALATA_MODELS_DIR) / "souxmar-quad/souxmar-quad.yaml").string());
  battery_model +=
      "\nbattery:\n"
      "  energy_j: 360000.0\n"
      "  full_voltage_v: 25.2\n"
      "  empty_voltage_v: 19.8\n"
      "  internal_resistance_ohm: 0.03\n"
      "  speed_at_full_voltage_rad_s: 1102.4200493562662\n";
  put(root / "quad.yaml", battery_model);

  const RunResult result = run(chain());
  const Artifact* linear = result.find("linear");
  ASSERT_NE(linear, nullptr);
  const auto& system = linear->payload_as<galata::model::LinearSystem>("linear_system");
  ASSERT_EQ(system.state_count(), 17)
      << "twelve chart coordinates, four rotors and one state of charge";
  EXPECT_EQ(system.state_names.back(), "battery_soc");

  // The battery row is zero because it was DECLARED frozen, and the evidence
  // file reports the rate that was declared away so the declaration can be
  // audited rather than merely trusted.
  EXPECT_LT(system.a.row(16).cwiseAbs().maxCoeff(), 1e-12);
  const YAML::Node node = YAML::Load(read_file_bytes((output / "operating-point.yaml").string()));

  // BOTH stages freeze it, and both say so. The trim held the state of charge
  // and excluded its row from the residual; without this marker the exported
  // trim reads as an equilibrium in a coordinate the solver never balanced.
  const YAML::Node trim_frozen = node["trim"]["frozen_states"];
  ASSERT_EQ(trim_frozen.size(), 1u);
  EXPECT_EQ(trim_frozen[0]["name"].as<std::string>(), "battery_soc");
  EXPECT_TRUE(trim_frozen[0]["excluded_from_residual"].as<bool>());
  EXPECT_DOUBLE_EQ(trim_frozen[0]["held_at"].as<double>(),
                   node["trim"]["battery_state_of_charge"].as<double>());

  const YAML::Node frozen = node["linearisation"]["frozen_states"];
  ASSERT_EQ(frozen.size(), 1u);
  EXPECT_EQ(frozen[0]["name"].as<std::string>(), "battery_soc");
  EXPECT_LT(frozen[0]["declared_zero_actual_rate"].as<double>(), 0.0)
      << "a hovering pack discharges; a zero here would mean the freeze changed nothing";

  // The adapter now admits it. Types are declared explicitly, as ADR-0013
  // requires: nothing is inferred from a channel name.
  galata::modeling::LinearChannels channels;
  const galata::modeling::SignalType scalar{};
  channels.states.assign(static_cast<std::size_t>(system.state_count()), scalar);
  channels.inputs.assign(static_cast<std::size_t>(system.input_count()), scalar);
  channels.outputs.assign(static_cast<std::size_t>(system.output_count()), scalar);

  galata::modeling::LinearGraphOptions adapter_options;
  adapter_options.initial_state = Eigen::VectorXd::Zero(system.state_count());
  adapter_options.command = Eigen::VectorXd::Zero(system.input_count());

  const galata::modeling::LinearGraph graph =
      galata::modeling::lower_linear_system(system, channels, adapter_options);
  EXPECT_EQ(graph.state_ids.size(), 17u);
}

// A conjugate pair is one mode, so a sixteen-state model that oscillates
// reports fewer modes than it has states. That is the intended convention —
// src/analyze/modes.cpp consumes the partner deliberately — but a bare "12
// modes" about a sixteen-state system reads as though four states went missing,
// and a reader who believes that has been told something false about what was
// analysed. The summary now carries the reconciliation, and this pins it.
//
// The hover model is the case that makes the count differ from the state count
// only once a loop is closed around it; open-loop hover is entirely real, so it
// reports sixteen modes over sixteen states and must NOT carry the clause.
TEST_F(QuadrotorWorkflow, TheModeCountReconcilesItselfWithTheStateCount) {
  const RunResult result = run(chain()
                               + "  - id: modes\n    capability: analyze.modes\n    input: "
                                 "{system: {from: linear}, classify: false}\n",
                               {.overwrite = true, .write_manifest = false});
  const Artifact* modes = result.find("modes");
  ASSERT_NE(modes, nullptr);
  // Open-loop hover: sixteen real eigenvalues, sixteen modes, no clause needed.
  EXPECT_NE(modes->summary.find("16 modes"), std::string::npos) << modes->summary;
  EXPECT_EQ(modes->summary.find("over 16 states"), std::string::npos)
      << "a count that already equals the state count must not be padded with the "
         "reconciliation: "
      << modes->summary;
}

// --- sim.plant -------------------------------------------------------------
//
// The nonlinear multirotor, integrated through a PUBLIC capability. Before this
// existed the plant was reachable only from C++: this repository could integrate
// a quadrotor in its own tests and a user could not integrate one at all.
// `sim.nonlinear` is the fixed-wing path — it takes a `trim.level` point and
// actuators named elevator, aileron, rudder and thrust — and is untouched.
//
// The strongest thing a trim and an integrator can be asked together is whether
// the trim STAYS. A point that satisfies the residual gate but drifts under the
// plant's own dynamics was never an equilibrium, and no gate on the residual
// alone can tell the difference.
TEST_F(QuadrotorWorkflow, IntegratingFromAHoverTrimLeavesItWhereItStarted) {
  const RunResult result =
      run("version: 1\nstages:\n"
          "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
          "  - id: hover\n    capability: trim.hover\n    input: "
          "{quadrotor: {from: plant}, altitude_m: 120.0}\n"
          "  - id: fly\n    capability: sim.plant\n    input: "
          "{trim: {from: hover}, step_s: 0.002, steps: 2500, sample_stride: 250}\n",
          {.overwrite = true, .write_manifest = false});

  const Artifact* flown = result.find("fly");
  ASSERT_NE(flown, nullptr);
  const auto& run_data = flown->payload_as<PlantRun>("plant_trajectory");
  ASSERT_FALSE(run_data.trajectory.states.empty());

  const Eigen::VectorXd& first = run_data.trajectory.states.front();
  const Eigen::VectorXd& last = run_data.trajectory.states.back();
  ASSERT_EQ(first.size(), last.size());
  // Five seconds of RK4 at 2 ms. The budget is round-off accumulated over 2500
  // steps on quantities of order 100, not a tolerance chosen to pass: an
  // equilibrium that drifts by more than this is not one.
  EXPECT_LT((last - first).norm(), 1e-9)
      << "the hover trim did not stay put under the plant's own dynamics";
  EXPECT_NEAR(last(galata::core::kPositionDown), -120.0, 1e-9)
      << "the declared altitude did not survive the integration";
}

// Cruise is a RELATIVE equilibrium: the dynamic accelerations vanish and the
// position rate does not. An integrator that quietly held the position still
// would satisfy every dynamic residual and be wrong about where the aircraft is.
TEST_F(QuadrotorWorkflow, CruiseKeepsItsPositionRateUnderIntegration) {
  const RunResult result = run(
      "version: 1\nstages:\n"
      "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
      "  - id: cruise\n    capability: trim.hover\n    input: "
      "{quadrotor: {from: plant}, altitude_m: 120.0, ground_velocity_ned_m_s: [8.0, 0.0, 0.0]}\n"
      "  - id: fly\n    capability: sim.plant\n    input: "
      "{trim: {from: cruise}, step_s: 0.002, steps: 2500, sample_stride: 2500}\n",
      {.overwrite = true, .write_manifest = false});

  const Artifact* flown = result.find("fly");
  ASSERT_NE(flown, nullptr);
  const auto& run_data = flown->payload_as<PlantRun>("plant_trajectory");
  const Eigen::VectorXd& last = run_data.trajectory.states.back();
  // Five seconds north at 8 m/s is 40 m, exactly, by construction.
  EXPECT_NEAR(last(galata::core::kPositionNorth), 40.0, 1e-6)
      << "a relative equilibrium must keep travelling";
  EXPECT_NEAR(last(galata::core::kPositionDown), -120.0, 1e-6)
      << "cruise must hold its altitude while it travels";
}

// The two ways in are exclusive, and a request that would do nothing is refused
// rather than ignored — a caller who asks to freeze a battery that is not there
// has misunderstood their own model.
TEST_F(QuadrotorWorkflow, SimPlantRefusesAmbiguousAndVacuousRequests) {
  EXPECT_THROW((void)run("version: 1\nstages:\n"
                         "  - id: plant\n    capability: model.quadrotor\n"
                         "    input: {path: quad.yaml}\n"
                         "  - id: hover\n    capability: trim.hover\n"
                         "    input: {quadrotor: {from: plant}}\n"
                         "  - id: fly\n    capability: sim.plant\n    input: "
                         "{trim: {from: hover}, quadrotor: {from: plant}, step_s: 0.002, "
                         "steps: 10}\n",
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error)
      << "a trim and a model together leaves it unsaid which state was integrated";

  EXPECT_THROW((void)run("version: 1\nstages:\n"
                         "  - id: plant\n    capability: model.quadrotor\n"
                         "    input: {path: quad.yaml}\n"
                         "  - id: hover\n    capability: trim.hover\n"
                         "    input: {quadrotor: {from: plant}}\n"
                         "  - id: fly\n    capability: sim.plant\n    input: "
                         "{trim: {from: hover}, step_s: 0.002, steps: 10, "
                         "freeze_battery: true}\n",
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error)
      << "freezing a battery the shipped model does not carry must be refused, not ignored";
}

// --- wind, and the coordinate the state actually holds ---------------------
//
// ADR-0002's velocity state is AIR-RELATIVE and `Quadrotor::derivative` takes
// the wind as steady: it carries no -R^T dw/dt term. A wind that changes
// therefore needs the simulator's help, in two different ways.
//
// A STEP is an event, not a large derivative. No force acts at the instant the
// air mass changes speed, so the GROUND velocity is continuous and the
// air-relative velocity must jump by exactly minus the wind change, rotated
// into the body frame. Integrating through the step instead injects the entire
// wind increment as a ground-velocity error — permanently, and with nothing to
// show for it in any residual. This is WP1's first finding, and it is the one
// property of time-varying wind that a plausible-looking trajectory will hide.
//
// The vehicle is level at hover, so the body-to-NED rotation is the identity
// and the expected jump is exactly the negated wind, written down rather than
// computed by the code under test.
TEST_F(QuadrotorWorkflow, AWindStepMovesTheAirRelativeVelocityAndNotTheGroundVelocity) {
  const RunResult result =
      run("version: 1\nstages:\n"
          "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
          "  - id: hover\n    capability: trim.hover\n    input: "
          "{quadrotor: {from: plant}, altitude_m: 120.0}\n"
          "  - id: fly\n    capability: sim.plant\n    input:\n"
          "      trim: {from: hover}\n"
          "      step_s: 0.002\n"
          "      steps: 1000\n"
          "      sample_stride: 1\n"
          "      wind_schedule:\n"
          "        hold: zero_order\n"
          "        extrapolation: hold\n"
          "        samples:\n"
          "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
          "          - {time_s: 1.0, values: [3.0, 2.0, 0.0]}\n",
          {.overwrite = true, .write_manifest = false});

  const Artifact* flown = result.find("fly");
  ASSERT_NE(flown, nullptr);
  const auto& data = flown->payload_as<PlantRun>("plant_trajectory");
  ASSERT_EQ(data.trajectory.states.size(), data.wind_samples_ned_m_s.size());

  std::size_t at_step = 0;
  for (std::size_t i = 0; i < data.trajectory.times_s.size(); ++i) {
    if (std::fabs(data.trajectory.times_s[i] - 1.0) < 1e-9) {
      at_step = i;
      break;
    }
  }
  ASSERT_GT(at_step, 0u) << "the wind step is not among the recorded samples";

  const Eigen::VectorXd& before = data.trajectory.states[at_step - 1];
  const Eigen::VectorXd& after = data.trajectory.states[at_step];
  const Eigen::Vector3d air_before = before.segment<3>(galata::core::kVelocityU);
  const Eigen::Vector3d air_after = after.segment<3>(galata::core::kVelocityU);

  // Level hover: the rotation is the identity, so the air-relative velocity
  // must jump by exactly -(3, 2, 0).
  EXPECT_NEAR((air_after - air_before - Eigen::Vector3d(-3.0, -2.0, 0.0)).norm(), 0.0, 1e-9)
      << "the air-relative velocity did not absorb the whole wind change";

  // The property that matters: ground velocity, air-relative plus wind, does
  // not move across the step.
  const Eigen::Vector3d ground_before = air_before + data.wind_samples_ned_m_s[at_step - 1];
  const Eigen::Vector3d ground_after = air_after + data.wind_samples_ned_m_s[at_step];
  EXPECT_NEAR((ground_after - ground_before).norm(), 0.0, 1e-12)
      << "a wind step moved the ground velocity, which no force did";
}

// A discontinuity strictly inside a step is not representable — RK4's stages
// would straddle it and the re-basing has no instant to happen at — so it is
// refused rather than rounded to the nearest step, which would move the event
// and say nothing about having done so.
TEST_F(QuadrotorWorkflow, AScheduleThatMissesTheStepLatticeIsRefused) {
  EXPECT_THROW((void)run("version: 1\nstages:\n"
                         "  - id: plant\n    capability: model.quadrotor\n"
                         "    input: {path: quad.yaml}\n"
                         "  - id: hover\n    capability: trim.hover\n"
                         "    input: {quadrotor: {from: plant}}\n"
                         "  - id: fly\n    capability: sim.plant\n    input:\n"
                         "      trim: {from: hover}\n"
                         "      step_s: 0.002\n"
                         "      steps: 1000\n"
                         "      wind_schedule:\n"
                         "        hold: zero_order\n"
                         "        extrapolation: hold\n"
                         "        samples:\n"
                         "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                         "          - {time_s: 0.9993, values: [3.0, 2.0, 0.0]}\n",
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error);
}

// --- sim.sampled -----------------------------------------------------------

namespace {

// A study that designs a gain and then flies it. Everything here goes through
// public capabilities; nothing reaches into C++ to build a controller. The law
// stage's head — which capability, and the keys before the weights — is the
// caller's, so the same chain serves a continuous and a discrete design.
std::string law_chain(const std::string& trim_input,
                      const std::string& law_head,
                      const std::string& sampled_extra,
                      int steps) {
  std::ostringstream q;
  q << "[";
  for (int i = 0; i < 16; ++i) {
    q << (i ? ", [" : "[");
    for (int j = 0; j < 16; ++j) {
      double value = 0.0;
      if (i == j) {
        value = (i < 3) ? 5.0 : (i >= 6 && i < 9) ? 20.0 : (i >= 12) ? 0.001 : 1.0;
      }
      q << (j ? ", " : "") << value;
    }
    q << "]";
  }
  q << "]";
  std::ostringstream r;
  r << "[";
  for (int i = 0; i < 4; ++i) {
    r << (i ? ", [" : "[");
    for (int j = 0; j < 4; ++j) {
      r << (j ? ", " : "") << (i == j ? 0.02 : 0.0);
    }
    r << "]";
  }
  r << "]";

  std::ostringstream out;
  out << "version: 1\nstages:\n"
      << "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
      << "  - id: hover\n    capability: trim.hover\n    input: " << trim_input << "\n"
      << "  - id: linear\n    capability: linearize.extended\n    input: "
         "{trim: {from: hover}, evidence_path: op.yaml}\n"
      << "  - id: rotors\n    capability: model.channels\n    input:\n"
         "      system: {from: linear}\n"
         "      inputs: [omega_command_0, omega_command_1, omega_command_2, omega_command_3]\n"
         "      outputs: [position_north_m, position_east_m, altitude_m]\n"
      << law_head << q.str() << ", r: " << r.str() << "}\n"
      << "  - id: closed\n    capability: sim.sampled\n    input:\n"
         "      trim: {from: hover}\n      law: {from: lqr}\n"
         "      step_s: 0.002\n      steps: "
      << steps << "\n"
      << sampled_extra;
  return out.str();
}

std::string sampled_chain(const std::string& trim_input,
                          const std::string& sampled_extra,
                          int steps = 400) {
  return law_chain(trim_input,
                   "  - id: lqr\n    capability: synth.lqr\n    input: {system: {from: rotors}, "
                   "break_at: plant_input, q: ",
                   sampled_extra,
                   steps);
}

// The same chain with the law DESIGNED in discrete time, at 250 Hz unless the
// caller says otherwise.
std::string discrete_chain(const std::string& sampled_extra,
                           const std::string& design_keys =
                               "sample_time_s: 0.004, hold: zero_order, evidence_path: lqr.yaml",
                           int steps = 400) {
  return law_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                   "  - id: lqr\n    capability: synth.sampled_lqr\n    input: {system: {from: "
                   "rotors}, "
                       + design_keys + ", q: ",
                   sampled_extra,
                   steps);
}

}  // namespace

// THE INDEPENDENT CHECK. The trajectory is not re-derived here — that would be
// checking the integrator against itself. What is re-derived is the SAMPLED
// LOGIC: from the states the run recorded at each tick, the law, the saturation
// and the delay line are recomputed by hand and required to reproduce the
// commands the run says it issued. A loop that sampled at the wrong instant,
// fed back on the wrong coordinates, or shifted its delay by one tick passes
// every stability check and fails this.
TEST_F(QuadrotorWorkflow, TheSampledCommandSequenceIsReproducibleByHand) {
  const RunResult result =
      run(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                        "      controller_period_s: 0.004\n"
                        "      delay_periods: 2\n"
                        "      initial_chart_perturbation: "
                        "[2.0, -1.0, 1.5, 0,0,0, 0.05,0.02,0.0, 0,0,0, 0,0,0,0]\n"),
          {.overwrite = true, .write_manifest = false});

  const Artifact* closed = result.find("closed");
  ASSERT_NE(closed, nullptr);
  const auto& sampled = closed->payload_as<SampledRun>("sampled_trajectory");
  const auto& control = sampled.control;
  ASSERT_GT(control.tick_times_s.size(), 10u);
  EXPECT_EQ(control.delay_periods, 2);

  const Artifact* law_stage = result.find("lqr");
  ASSERT_NE(law_stage, nullptr);
  const auto& law = law_stage->payload_as<galata::synth::LqrDesign>("control_law");
  const Artifact* trim_stage = result.find("hover");
  ASSERT_NE(trim_stage, nullptr);
  const auto& trimmed = trim_stage->payload_as<HoverTrimArtifact>("hover_trim");

  for (std::size_t tick = 0; tick < control.tick_times_s.size(); ++tick) {
    // The law, recomputed from the chart error the run recorded.
    const Eigen::VectorXd expected_request =
        trimmed.point.command_rad_s - law.riccati.k * control.chart_error[tick];
    EXPECT_LT((expected_request - control.requested_rad_s[tick]).norm(), 1e-12)
        << "the requested command at tick " << tick << " is not u_trim - K e";

    // The delay line: what the plant received is what was computed
    // `delay_periods` ticks ago, and the trim command before that.
    const Eigen::VectorXd& applied = control.applied_rad_s[tick];
    if (tick < static_cast<std::size_t>(control.delay_periods)) {
      EXPECT_LT((applied - trimmed.point.command_rad_s).norm(), 1e-12)
          << "before the delay line has filled, the plant must receive the TRIM command at "
             "tick "
          << tick << " — zero would be a multirotor switched off";
    } else {
      const std::size_t source = tick - static_cast<std::size_t>(control.delay_periods);
      EXPECT_LT((applied - control.saturated_rad_s[source]).norm(), 1e-12)
          << "tick " << tick << " did not receive the command computed at tick " << source;
    }
  }
}

// Zero delay is the degenerate case of the same schedule and must not be a
// separate code path: what is applied is what was just computed.
TEST_F(QuadrotorWorkflow, ZeroDelayAppliesTheCommandComputedAtThatTick) {
  const RunResult result = run(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                                             "      controller_period_s: 0.004\n"
                                             "      delay_periods: 0\n"
                                             "      initial_chart_perturbation: "
                                             "[1.0, 0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0,0]\n"),
                               {.overwrite = true, .write_manifest = false});
  const auto& control = result.find("closed")->payload_as<SampledRun>("sampled_trajectory").control;
  for (std::size_t tick = 0; tick < control.tick_times_s.size(); ++tick) {
    EXPECT_LT((control.applied_rad_s[tick] - control.saturated_rad_s[tick]).norm(), 1e-12);
  }
}

// A large displacement drives the law past what the rotors can deliver. The
// commands must clamp at the model's own ceiling — not at a number this
// capability chose — and the run must SAY it saturated rather than reporting a
// trajectory that looks like ordinary flight.
TEST_F(QuadrotorWorkflow, SaturationClampsAtTheModelsOwnCeilingAndIsReported) {
  const RunResult result =
      run(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                        "      controller_period_s: 0.004\n"
                        "      delay_periods: 0\n"
                        "      initial_chart_perturbation: "
                        "[400.0, -300.0, 250.0, 0,0,0, 0,0,0, 0,0,0, 0,0,0,0]\n"),
          {.overwrite = true, .write_manifest = false});
  const auto& sampled = result.find("closed")->payload_as<SampledRun>("sampled_trajectory");
  EXPECT_GT(sampled.control.saturated_tick_count, 0)
      << "a 400 m displacement must ask for more than the rotors have";
  EXPECT_GT(sampled.control.worst_saturation_residual_rad_s, 0.0);

  const Artifact* plant_stage = result.find("plant");
  ASSERT_NE(plant_stage, nullptr);
  const auto& model = plant_stage->payload_as<QuadrotorArtifact>("quadrotor").model;
  for (const Eigen::VectorXd& applied : sampled.control.applied_rad_s) {
    for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
      const auto& description = model.rotors[static_cast<std::size_t>(rotor)];
      EXPECT_GE(applied(rotor), description.minimum_speed_rad_s - 1e-12);
      EXPECT_LE(applied(rotor), description.maximum_speed_rad_s + 1e-12)
          << "an applied command exceeded the rotor's own ceiling";
    }
  }
}

// CRUISE IS NOT HOVER WITH A NUMBER CHANGED. At a relative equilibrium the
// reference position moves; a reference held at the trim's starting point
// becomes, a second later, a demand to fly back to where the aircraft began.
// The capability refuses to guess which was meant.
TEST_F(QuadrotorWorkflow, ACruiseTrimMustDeclareWhetherItsReferenceTravels) {
  const std::string cruise =
      "{quadrotor: {from: plant}, altitude_m: 120.0, ground_velocity_ned_m_s: [6.0, 0.0, 0.0]}";
  EXPECT_THROW((void)run(sampled_chain(cruise,
                                       "      controller_period_s: 0.004\n"
                                       "      delay_periods: 0\n"),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error)
      << "a moving equilibrium with no declared reference motion must be refused";

  // Declared, and the aircraft holds the equilibrium rather than fighting it.
  const RunResult result = run(sampled_chain(cruise,
                                             "      controller_period_s: 0.004\n"
                                             "      delay_periods: 0\n"
                                             "      reference_motion: follow_trim_velocity\n",
                                             2000),
                               {.overwrite = true, .write_manifest = false});
  const auto& sampled = result.find("closed")->payload_as<SampledRun>("sampled_trajectory");
  const Eigen::VectorXd& last = sampled.plant.trajectory.states.back();
  // Four seconds north at 6 m/s is 24 m. Started at the trim, so the error is
  // what the controller allowed, not what it was asked to remove.
  EXPECT_NEAR(last(galata::core::kPositionNorth), 24.0, 0.5)
      << "a followed reference must let the aircraft travel along its equilibrium";
  EXPECT_LT(sampled.control.chart_error.back().segment<3>(0).norm(), 0.5)
      << "the position error must stay bounded when the reference travels with the trim";
}

// Timing that the schedule cannot represent is refused rather than rounded.
TEST_F(QuadrotorWorkflow, UnsupportedSampledTimingIsRefused) {
  const std::string hover = "{quadrotor: {from: plant}, altitude_m: 120.0}";
  // A period that the step does not divide.
  EXPECT_THROW((void)run(sampled_chain(hover, "      controller_period_s: 0.003\n"),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error);
  // A horizon that is not a whole number of periods.
  EXPECT_THROW((void)run(sampled_chain(hover, "      controller_period_s: 0.006\n", 400),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error);
  // A delay that is not a whole number of periods.
  EXPECT_THROW((void)run(sampled_chain(hover,
                                       "      controller_period_s: 0.004\n"
                                       "      delay_periods: 1.5\n"),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error);
}

// --- sim.sampled under a declared wind history -------------------------------
//
// THE SAME CONTRACT AS `sim.plant`, NOT A WEAKER ONE. `sim.sampled` reads the
// same `wind_schedule` into a plant whose velocity state is, by ADR-0002, the
// same AIR-RELATIVE one, so everything stated above
// `AWindStepMovesTheAirRelativeVelocityAndNotTheGroundVelocity` holds here
// unchanged: a zero-order step is an event, no force acts at it, the ground
// velocity is continuous across it, the air-relative velocity jumps by exactly
// -R^T dw, and an event strictly inside an integration step is refused. Until
// these tests the schedule was accepted here and no study drove one through it,
// and a closed loop is exactly where WP1's first finding hides: the controller
// quietly flies off whatever ground-velocity error the integrator injected, and
// the trajectory still looks like a vehicle rejecting a gust.
//
// What the sampled loop adds is TIMING. The event need not fall on a controller
// tick, the controller must not learn of it before its next tick, and the
// rotors must not move before the declared delay has elapsed.
//
// THE FLIGHT, identical in the three tests below: from the exact trim with no
// chart perturbation, a 2 ms step, a 0.01 s controller period and five periods
// of delay.
//
//   t = 0.026 s  (step 13)  wind (0, 0, 0) -> (3, 2, 0)    between ticks 2 and 3
//   t = 0.040 s  (step 20)  wind (3, 2, 0) -> (1, 2, -1)   exactly on tick 4
//
// The delay line holds the TRIM command for its first five ticks, so the rotors
// are commanded to the trim, bitwise, over the whole of [0, 0.05 s]: no feedback
// of any kind acts inside that window. Tick 3 is the first at or after 0.026 s,
// so the first command that can know of the wind is computed there and reaches
// the rotors at tick 3 + 5 = 8.
//
// R IS THE IDENTITY, and that is asserted rather than assumed. A still-air hover
// at zero heading is level, and nothing in the window can tilt it: the rotors
// hold trim and the plant's drag acts at the centre of gravity, so no moment
// changes. The expected jumps are therefore the negated wind changes, written
// down: -(3, 2, 0) and -(-2, 0, -1).
//
// THE PHYSICS IS KNOWN IN CLOSED FORM. Level, with the rotors at trim, thrust
// still balances weight and the only force that changes is the drag on the
// air-relative velocity, per axis m dv/dt = -(c1 v + c2 v|v|) with the model
// file's coefficients. That equation separates: with a = c1/m and k = c2/c1,
// |v| = s0 e^(-at) / (1 + k s0 (1 - e^(-at))), whose time integral is
// (m/c2) ln(1 + k s0 (1 - e^(-at))), sign preserved. The expectation is exact —
// not a Taylor expansion with a remainder to budget — so the budget is only
// what the idealisation neglects.
//
// THE BUDGET, derived here before any run of these tests and not from one. Over
// the 0.05 s window, in each coordinate's own unit:
//
//   the trim's unbalanced acceleration, at most its declared gate of 1e-10
//     m/s^2 (rad/s^2), acting for 0.05 s ............................. 5e-12
//   the level premise, asserted below at 1e-12 rad for the trim and for the
//     flown attitude, misreading a velocity of at most 4 m/s ........... 8e-12
//   the body rate that gate allows, 5e-12 rad/s, turning that velocity
//     for 0.05 s ...................................................... 1e-12
//   RK4's truncation on a decay whose rate is below 0.25 1/s, at 2 ms . < 1e-15
//   round-off: a few ulps of 4 m/s, or of 120 m, per step, 25 steps .. < 1e-12
//
// The sum is below 2e-11. The budget is 1e-10, five times the sum, because the
// round-off entries are estimates rather than bounds. Positions integrate these
// errors over at most 0.05 s and sit inside the same figure. The smallest defect
// the budget has to see is a re-basing applied one step late, which leaves the
// air-relative velocity one 2 ms step of drag short: the drag at (3, 2, 0) m/s
// is about 0.4 m/s^2 on this 1.6 kg vehicle, so about 8e-4 m/s, seven orders
// above the budget.

namespace {

constexpr double kWindWindowBudget = 1e-10;  // each chart coordinate, in its own unit

// The flight above. Only the time of the first wind step varies, so that the
// refusal test can move it off the step lattice and change nothing else.
std::string windy_sampled_chain(const std::string& first_step_time_s) {
  return sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                       "      controller_period_s: 0.01\n"
                       "      delay_periods: 5\n"
                       "      wind_schedule:\n"
                       "        hold: zero_order\n"
                       "        extrapolation: hold\n"
                       "        samples:\n"
                       "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                       "          - {time_s: "
                           + first_step_time_s
                           + ", values: [3.0, 2.0, 0.0]}\n"
                             "          - {time_s: 0.04, values: [1.0, 2.0, -1.0]}\n",
                       50);
}

// Drag alone, in closed form, on a level vehicle whose rotors hold trim: the
// air-relative velocity after `duration_s`, and the GROUND displacement over
// it under a constant wind. Body axes are NED axes at level, zero heading.
struct DragOnly {
  Eigen::Vector3d air_m_s = Eigen::Vector3d::Zero();         // m/s
  Eigen::Vector3d displacement_m = Eigen::Vector3d::Zero();  // m, NED
};

DragOnly drag_only(const galata::model::Quadrotor& model,
                   const Eigen::Vector3d& air_m_s,
                   const Eigen::Vector3d& wind_ned_m_s,
                   double duration_s) {
  DragOnly out;
  const double mass_kg = model.mass.mass_kg;
  for (int axis = 0; axis < 3; ++axis) {
    const double linear = model.drag_linear_n_s_m(axis);
    const double quadratic = model.drag_quadratic_n_s2_m2(axis);
    const double speed = std::fabs(air_m_s(axis));
    const double sign = air_m_s(axis) < 0.0 ? -1.0 : 1.0;
    const double rate = linear / mass_kg;
    const double decayed = -std::expm1(-rate * duration_s);  // 1 - e^(-at)
    const double spread = (quadratic / linear) * speed * decayed;
    out.air_m_s(axis) = sign * speed * std::exp(-rate * duration_s) / (1.0 + spread);
    out.displacement_m(axis) =
        sign * std::log1p(spread) * mass_kg / quadratic + wind_ned_m_s(axis) * duration_s;
  }
  return out;
}

double largest_magnitude(const Eigen::VectorXd& values) {
  return values.cwiseAbs().maxCoeff();
}

// The worst discrepancy each check left, so the margin the budget holds is
// visible in the test report and not only its verdict.
void record_discrepancy(const std::string& key, double value) {
  std::ostringstream out;
  out << std::scientific << std::setprecision(3) << value;
  ::testing::Test::RecordProperty(key, out.str());
}

// What the analytic flight predicts at the recorded ticks inside the window.
struct WindWindow {
  DragOnly at_tick_3;            // 0.004 s after the first step
  DragOnly before_second;        // 0.014 s after the first step, just before the second
  Eigen::Vector3d after_second;  // the air-relative velocity just after it, m/s
  DragOnly at_tick_5;            // 0.010 s after the second step
};

WindWindow predict_wind_window(const galata::model::Quadrotor& model) {
  // The wind as declared, and the air-relative jump each step must cause at
  // level attitude: minus the wind change, written down rather than derived by
  // the code under test.
  const Eigen::Vector3d first_wind(3.0, 2.0, 0.0);
  const Eigen::Vector3d first_jump(-3.0, -2.0, 0.0);
  const Eigen::Vector3d second_jump(2.0, 0.0, 1.0);
  const Eigen::Vector3d second_wind(1.0, 2.0, -1.0);

  WindWindow window;
  // Still air until 0.026 s: the air-relative velocity is zero, and the first
  // step sets it to the jump, because the ground velocity cannot move.
  window.at_tick_3 = drag_only(model, first_jump, first_wind, 0.030 - 0.026);
  window.before_second = drag_only(model, first_jump, first_wind, 0.040 - 0.026);
  window.after_second = window.before_second.air_m_s + second_jump;
  window.at_tick_5 = drag_only(model, window.after_second, second_wind, 0.050 - 0.040);
  return window;
}

}  // namespace

// Contract items (1) and (4): continuity at each event, and the only ground
// acceleration between the first event and the first command change is the
// drag the wind change causes.
TEST_F(QuadrotorWorkflow, AWindStepBetweenControllerTicksKeepsTheGroundVelocityContinuous) {
  const RunResult result =
      run(windy_sampled_chain("0.026"), {.overwrite = true, .write_manifest = false});
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& sampled = result.find("closed")->payload_as<SampledRun>("sampled_trajectory");
  const galata::model::Quadrotor& model = trimmed.model;
  const auto& states = sampled.plant.trajectory.states;
  const auto& winds = sampled.plant.wind_samples_ned_m_s;
  const auto& control = sampled.control;
  ASSERT_EQ(control.tick_times_s.size(), 10U);
  ASSERT_EQ(states.size(), 11U) << "one sample per tick, and the state after the last hold";
  ASSERT_EQ(winds.size(), states.size());

  // THE PREMISES THE BUDGET RESTS ON. Each is asserted, so a change that
  // invalidates the derivation above fails here, by name, rather than moving a
  // number the budget no longer describes.
  ASSERT_LE(trimmed.point.residual_tolerance, 1e-10) << "the budget assumes trim.hover's gate";
  ASSERT_LE(trimmed.point.residual_norm, trimmed.point.residual_tolerance);
  ASSERT_TRUE(trimmed.point.wind_ned_m_s.isZero(0.0)) << "the trim must be a still-air hover";
  ASSERT_EQ(trimmed.point.yaw_rad, 0.0);
  ASSERT_LE(std::fabs(trimmed.point.roll_rad), 1e-12) << "a still-air hover must be level";
  ASSERT_LE(std::fabs(trimmed.point.pitch_rad), 1e-12) << "a still-air hover must be level";
  for (int axis = 0; axis < 3; ++axis) {
    // The closed form divides by both coefficients; a model without either
    // needs a different expectation, not this one with a zero in it.
    ASSERT_GT(model.drag_linear_n_s_m(axis), 0.0);
    ASSERT_GT(model.drag_quadratic_n_s2_m2(axis), 0.0);
  }
  using galata::linearize::kChartAttitudeErrorX;
  for (std::size_t tick = 0; tick <= 5; ++tick) {
    EXPECT_LE(largest_magnitude(control.chart_error[tick].segment<3>(kChartAttitudeErrorX)), 1e-12)
        << "the vehicle tilted inside the trim-held window, at tick " << tick
        << ", so R is not the identity the expected jumps assume";
  }
  // Nothing moves before the first event: every chart coordinate, at every
  // tick before 0.026 s, is still the trim.
  for (std::size_t tick = 0; tick <= 2; ++tick) {
    EXPECT_LE(largest_magnitude(control.chart_error[tick]), kWindWindowBudget)
        << "the trim moved before any wind changed, at tick " << tick;
  }

  const WindWindow expected = predict_wind_window(model);
  const auto air = [&](std::size_t tick) -> Eigen::Vector3d {
    return states[tick].segment<3>(galata::core::kVelocityU);
  };
  // R is the identity (asserted above), so body and NED components coincide.
  const auto ground = [&](std::size_t tick) -> Eigen::Vector3d { return air(tick) + winds[tick]; };
  const Eigen::Vector3d first_wind(3.0, 2.0, 0.0);
  const Eigen::Vector3d second_wind(1.0, 2.0, -1.0);

  // (1) THE JUMP. The first event is between recorded samples, so it is seen
  // through its consequence at tick 3: the jump -(3, 2, 0) followed by 4 ms of
  // drag, and nothing else. The second is ON tick 4, and the state recorded
  // there must be the one AFTER it — the analytic air-relative velocity just
  // before the event moved by exactly -(-2, 0, -1).
  const double at_tick_3 = largest_magnitude(air(3) - expected.at_tick_3.air_m_s);
  EXPECT_LE(at_tick_3, kWindWindowBudget)
      << "the air-relative velocity 4 ms after the 0.026 s step is not minus the wind change "
         "decayed by drag: "
      << air(3).transpose() << " against " << expected.at_tick_3.air_m_s.transpose();
  const double at_event = largest_magnitude(air(4) - expected.after_second);
  EXPECT_LE(at_event, kWindWindowBudget)
      << "at the 0.04 s step the air-relative velocity did not jump by exactly minus the wind "
         "change: "
      << air(4).transpose() << " against " << expected.after_second.transpose();
  const double at_tick_5 = largest_magnitude(air(5) - expected.at_tick_5.air_m_s);
  EXPECT_LE(at_tick_5, kWindWindowBudget)
      << air(5).transpose() << " against " << expected.at_tick_5.air_m_s.transpose();

  // CONTINUITY, AS A CONSISTENCY CHECK. The ground velocity here is the recorded
  // air-relative velocity plus the recorded wind, and the next test pins the
  // wind to the declared table. So these two checks restate the jump checks
  // above rather than add to them. They are kept because they state the
  // property in its own terms. The INDEPENDENT witness of continuity is the
  // position at tick 5 below: it integrates the ground velocity and has no
  // event of its own, and a re-basing through the wrong vector moves it by
  // millimetres or more.
  const double across_first =
      largest_magnitude(ground(3) - ground(2) - (expected.at_tick_3.air_m_s + first_wind));
  EXPECT_LE(across_first, kWindWindowBudget)
      << "the ground velocity moved across the 0.026 s wind step by more than the drag it "
         "causes: from "
      << ground(2).transpose() << " to " << ground(3).transpose();
  const double across_second =
      largest_magnitude(ground(4) - (expected.before_second.air_m_s + first_wind));
  EXPECT_LE(across_second, kWindWindowBudget)
      << "the ground velocity at the 0.04 s wind step is not the one just before it: "
      << ground(4).transpose() << " against "
      << (expected.before_second.air_m_s + first_wind).transpose();

  // (4) THE PHYSICAL RESULT. From tick 2, before any wind, to tick 5, the last
  // state the trim command alone produced, the ground velocity changed by the
  // drag the two wind changes caused — a few hundredths of a metre per second,
  // against wind changes of metres per second — and the position by its
  // integral.
  const Eigen::Vector3d drag_change = expected.at_tick_5.air_m_s + second_wind;
  const double velocity_change = largest_magnitude(ground(5) - ground(2) - drag_change);
  EXPECT_LE(velocity_change, kWindWindowBudget)
      << "between the first wind step and the first command change the ground velocity "
         "changed by "
      << (ground(5) - ground(2)).transpose() << "; drag alone accounts for "
      << drag_change.transpose();
  const Eigen::Vector3d displacement =
      states[5].segment<3>(galata::core::kPositionNorth)
      - trimmed.point.extended_state.segment<3>(galata::core::kPositionNorth);
  const Eigen::Vector3d drag_displacement =
      expected.before_second.displacement_m + expected.at_tick_5.displacement_m;
  const double position_change = largest_magnitude(displacement - drag_displacement);
  EXPECT_LE(position_change, kWindWindowBudget)
      << "the vehicle moved " << displacement.transpose() << " m over the window; drag alone "
      << "moves it " << drag_displacement.transpose() << " m";

  record_discrepancy("air_at_tick_3_m_s", at_tick_3);
  record_discrepancy("air_at_event_on_tick_4_m_s", at_event);
  record_discrepancy("air_at_tick_5_m_s", at_tick_5);
  record_discrepancy("ground_across_first_step_m_s", across_first);
  record_discrepancy("ground_across_second_step_m_s", across_second);
  record_discrepancy("ground_change_to_tick_5_m_s", velocity_change);
  record_discrepancy("position_change_to_tick_5_m", position_change);
}

// Contract item (2): the recorded wind is the declared value in force at every
// sample, and the change reaches the controller at its next tick and the rotors
// only after the declared delay — ticks 3 and 8, derived above from the period,
// the delay and the event time, and written down here rather than searched for.
TEST_F(QuadrotorWorkflow, AWindStepReachesTheRotorsOnlyAtTheNextTickPlusTheDeclaredDelay) {
  const RunResult result =
      run(windy_sampled_chain("0.026"), {.overwrite = true, .write_manifest = false});
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& law = result.find("lqr")->payload_as<galata::synth::LqrDesign>("control_law");
  const auto& sampled = result.find("closed")->payload_as<SampledRun>("sampled_trajectory");
  const galata::model::Quadrotor& model = trimmed.model;
  const auto& control = sampled.control;
  const auto& winds = sampled.plant.wind_samples_ned_m_s;
  ASSERT_EQ(control.tick_times_s.size(), 10U);
  ASSERT_EQ(winds.size(), 11U);
  EXPECT_EQ(control.delay_periods, 5);

  // RIGHT-CONTINUOUS, AT EVERY SAMPLE. Ticks 0 to 2 are before the first step,
  // tick 3 is inside the first wind, and tick 4 is AT the second step and
  // carries the value that starts there — the one the next step integrates
  // under and the one that makes the recorded air-relative velocity add up to
  // the right ground velocity. The final sample, at 0.1 s, holds the last value.
  const Eigen::Vector3d still(0.0, 0.0, 0.0);
  const Eigen::Vector3d first_wind(3.0, 2.0, 0.0);
  const Eigen::Vector3d second_wind(1.0, 2.0, -1.0);
  const std::vector<Eigen::Vector3d> declared = {still,
                                                 still,
                                                 still,
                                                 first_wind,
                                                 second_wind,
                                                 second_wind,
                                                 second_wind,
                                                 second_wind,
                                                 second_wind,
                                                 second_wind,
                                                 second_wind};
  for (std::size_t i = 0; i < winds.size(); ++i) {
    EXPECT_TRUE(winds[i] == declared[i])
        << "sample " << i << " at t = " << sampled.plant.trajectory.times_s[i]
        << " s recorded the wind " << winds[i].transpose() << "; the schedule declares "
        << declared[i].transpose();
  }

  // THE CONTROLLER'S VIEW. At tick 2 (0.02 s) the wind has not changed, so the
  // error it feeds back on is still the trim's. Tick 3 is the first at or after
  // the 0.026 s step, and its error is the analytic one: position and
  // air-relative velocity from the closed form, attitude, rates and rotors
  // untouched. The second step is on tick 4 and is seen there.
  using galata::linearize::kChartPositionNorth;
  using galata::linearize::kChartVelocityU;
  for (std::size_t tick = 0; tick <= 2; ++tick) {
    EXPECT_LE(largest_magnitude(control.chart_error[tick]), kWindWindowBudget)
        << "the controller saw a change at tick " << tick << ", before any wind changed";
  }
  const WindWindow expected = predict_wind_window(model);
  Eigen::VectorXd first_error = Eigen::VectorXd::Zero(control.chart_error[3].size());
  first_error.segment<3>(kChartPositionNorth) = expected.at_tick_3.displacement_m;
  first_error.segment<3>(kChartVelocityU) = expected.at_tick_3.air_m_s;
  EXPECT_LE(largest_magnitude(control.chart_error[3] - first_error), kWindWindowBudget)
      << "tick 3 is the first to see the 0.026 s step, and it did not see the analytic error: "
      << control.chart_error[3].transpose();
  EXPECT_LE(
      largest_magnitude(control.chart_error[4].segment<3>(kChartVelocityU) - expected.after_second),
      kWindWindowBudget)
      << "the step ON tick 4 must be seen at tick 4, after it, not before";

  // THE ROTORS. What a chart error within the budget in every coordinate can
  // move a command by, at most: the largest row sum of |K| times the budget.
  const Eigen::VectorXd& trim_command = trimmed.point.command_rad_s;
  const double trim_level_rad_s =
      law.riccati.k.cwiseAbs().rowwise().sum().maxCoeff() * kWindWindowBudget;

  // Ticks 0 to 4 receive the delay line's own fill, the trim, bitwise.
  for (std::size_t tick = 0; tick <= 4; ++tick) {
    EXPECT_TRUE(control.applied_rad_s[tick] == trim_command)
        << "tick " << tick << " must receive the delay line's trim fill";
  }
  // Ticks 5 to 7 receive what ticks 0 to 2 computed — before any wind changed.
  for (std::size_t tick = 5; tick <= 7; ++tick) {
    EXPECT_LE(largest_magnitude(control.applied_rad_s[tick] - trim_command), trim_level_rad_s)
        << "tick " << tick << " applied a command computed at tick " << tick - 5
        << ", before the 0.026 s step, and it is not the trim";
  }
  // Tick 8 is the first to change: it receives exactly what tick 3 computed,
  // and that is the trim minus the gain times the analytic error, clamped to
  // the rotors' own limits.
  EXPECT_LT((control.applied_rad_s[8] - control.saturated_rad_s[3]).norm(), 1e-12)
      << "tick 8 did not receive the command computed at tick 3";
  EXPECT_LT((control.applied_rad_s[9] - control.saturated_rad_s[4]).norm(), 1e-12)
      << "tick 9 did not receive the command computed at tick 4";
  Eigen::VectorXd first_response = trim_command - law.riccati.k * first_error;
  for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
    const auto& description = model.rotors[static_cast<std::size_t>(rotor)];
    first_response(rotor) = std::clamp(
        first_response(rotor), description.minimum_speed_rad_s, description.maximum_speed_rad_s);
  }
  EXPECT_LE(largest_magnitude(control.applied_rad_s[8] - first_response), trim_level_rad_s)
      << "the first command to answer the wind is not u_trim - K e at the analytic error";
  EXPECT_GT(largest_magnitude(control.applied_rad_s[8] - trim_command), trim_level_rad_s)
      << "tick 8 is where the wind first reaches the rotors, and it applied the trim";
  record_discrepancy("first_response_rad_s",
                     largest_magnitude(control.applied_rad_s[8] - first_response));
  record_discrepancy("trim_level_rad_s", trim_level_rad_s);
}

// Contract item (3): a wind step half-way through an integration step is
// refused by `sim.sampled`, and the refusal names the schedule and says why.
// (Every stage error carries its capability's name, added by the pipeline's own
// wrapper, so matching that name here would prove nothing.) The same flight
// with the step on the lattice runs, so the refusal is about WHEN, not about
// the key. That lattice time is 0.026 s: thirteen 2 ms steps, but not
// 13 * 0.002 in floating point. So an implementation that looks the event up
// at k * step_s rather than at its declared time fails here as well.
TEST_F(QuadrotorWorkflow, SimSampledRefusesAWindStepInsideAnIntegrationStep) {
  EXPECT_NO_THROW(
      (void)run(windy_sampled_chain("0.026"), {.overwrite = true, .write_manifest = false}));
  try {
    (void)run(windy_sampled_chain("0.027"), {.overwrite = true, .write_manifest = false});
    ADD_FAILURE() << "a wind step at 0.027 s, half-way through the 2 ms step from 0.026 s, was "
                     "flown: its re-basing had no instant to happen at, and RK4's stages "
                     "straddled it";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("wind_schedule"), std::string::npos)
        << "the refusal must name the schedule whose event missed the lattice: " << message;
    EXPECT_NE(message.find("not a whole number of"), std::string::npos)
        << "the refusal must say that the event is not on the step lattice: " << message;
  }
}

// THE ROTATION, which the level, zero-heading flight above cannot see. There R
// is the identity, so -R^T dw, -R dw and an unrotated -dw are one vector, and a
// re-basing through the wrong rotation passes every check. Here the trim is
// level at a 30 degree heading, so the body jump is -Rz(30 deg)^T dw. At these
// winds that differs from the other two by metres per second. The rotation is
// written with cos and sin here, not taken from the code under test. The flight,
// the window and the budget are the ones derived above. The body axes differ
// from NED only by the heading, and nothing turns them inside the window, so
// the per-axis drag closed form holds unchanged in body axes.
TEST_F(QuadrotorWorkflow, AWindStepAtANonzeroHeadingIsAbsorbedThroughTheBodyRotation) {
  const RunResult result =
      run(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0, heading_deg: 30.0}",
                        "      controller_period_s: 0.01\n"
                        "      delay_periods: 5\n"
                        "      wind_schedule:\n"
                        "        hold: zero_order\n"
                        "        extrapolation: hold\n"
                        "        samples:\n"
                        "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                        "          - {time_s: 0.026, values: [3.0, 2.0, 0.0]}\n"
                        "          - {time_s: 0.04, values: [1.0, 2.0, -1.0]}\n",
                        50),
          {.overwrite = true, .write_manifest = false});
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& sampled = result.find("closed")->payload_as<SampledRun>("sampled_trajectory");
  const galata::model::Quadrotor& model = trimmed.model;
  const auto& states = sampled.plant.trajectory.states;
  const auto& control = sampled.control;
  ASSERT_EQ(states.size(), 11U);

  // The premises: a still-air hover, level, at the declared heading, and still
  // level through the whole trim-held window.
  ASSERT_TRUE(trimmed.point.wind_ned_m_s.isZero(0.0));
  ASSERT_NEAR(trimmed.point.yaw_rad, std::numbers::pi / 6.0, 1e-15);
  ASSERT_LE(std::fabs(trimmed.point.roll_rad), 1e-12);
  ASSERT_LE(std::fabs(trimmed.point.pitch_rad), 1e-12);
  using galata::linearize::kChartAttitudeErrorX;
  for (std::size_t tick = 0; tick <= 4; ++tick) {
    ASSERT_LE(largest_magnitude(control.chart_error[tick].segment<3>(kChartAttitudeErrorX)), 1e-12)
        << "the vehicle tilted inside the trim-held window, at tick " << tick;
  }

  const double c = std::cos(trimmed.point.yaw_rad);
  const double s = std::sin(trimmed.point.yaw_rad);
  Eigen::Matrix3d ned_from_body;
  ned_from_body << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  const Eigen::Vector3d first_wind(3.0, 2.0, 0.0);
  const Eigen::Vector3d second_wind(1.0, 2.0, -1.0);
  const Eigen::Vector3d first_jump = -(ned_from_body.transpose() * first_wind);
  const Eigen::Vector3d second_jump = -(ned_from_body.transpose() * (second_wind - first_wind));

  // Drag only, in body axes; the wind's own contribution to the position is
  // added in NED below.
  const DragOnly at_tick_3 = drag_only(model, first_jump, Eigen::Vector3d::Zero(), 0.004);
  const DragOnly before_second = drag_only(model, first_jump, Eigen::Vector3d::Zero(), 0.014);
  const Eigen::Vector3d after_second = before_second.air_m_s + second_jump;
  const auto air = [&](std::size_t tick) -> Eigen::Vector3d {
    return states[tick].segment<3>(galata::core::kVelocityU);
  };

  const double at_tick_3_miss = largest_magnitude(air(3) - at_tick_3.air_m_s);
  EXPECT_LE(at_tick_3_miss, kWindWindowBudget)
      << "4 ms after the 0.026 s step at a 30 degree heading, the body-axis air-relative "
         "velocity is "
      << air(3).transpose() << "; -Rz^T dw decayed by drag is " << at_tick_3.air_m_s.transpose();
  const double at_event_miss = largest_magnitude(air(4) - after_second);
  EXPECT_LE(at_event_miss, kWindWindowBudget)
      << "at the 0.04 s step the body-axis jump is not -Rz^T dw: " << air(4).transpose()
      << " against " << after_second.transpose();

  // The independent witness of ground-velocity continuity: the POSITION, which
  // integrates the ground velocity and has no event of its own. Still air
  // until 0.026 s, then 14 ms of the rotated air-relative motion plus the wind.
  const Eigen::Vector3d displacement =
      states[4].segment<3>(galata::core::kPositionNorth)
      - trimmed.point.extended_state.segment<3>(galata::core::kPositionNorth);
  const Eigen::Vector3d expected_displacement =
      ned_from_body * before_second.displacement_m + first_wind * 0.014;
  const double displacement_miss = largest_magnitude(displacement - expected_displacement);
  EXPECT_LE(displacement_miss, kWindWindowBudget)
      << "the vehicle moved " << displacement.transpose() << " m by the 0.04 s step; the "
      << "continuous ground velocity moves it " << expected_displacement.transpose() << " m";

  record_discrepancy("heading_air_at_tick_3_m_s", at_tick_3_miss);
  record_discrepancy("heading_air_at_event_on_tick_4_m_s", at_event_miss);
  record_discrepancy("heading_position_at_tick_4_m", displacement_miss);
}

// A RAMP, the other half of the contract. Under a linear hold the wind has a
// finite rate, and the air-relative velocity's rate must gain -R^T dw/dt,
// which the plant, taking the wind as steady, does not supply. Without it the
// air-relative velocity would sit still while the wind ramps, and the ground
// velocity would follow the wind by metres per second that no force produced.
//
// THE BUDGET is a bound rather than a closed form, computed from the model file
// before the run. The vehicle is level at zero heading, and the rotors hold the
// trim bitwise over [0, 0.05 s], by the delay line's fill as above. So the only
// force that changes is the drag on the air-relative velocity. The ground
// velocity changes by at most the drag's magnitude times the time it acts:
//     (c1 |v| + c2 |v|^2) / m * T,
// using the largest coefficient on any axis. Here |v| <= 2.1 m/s, because the
// wind reaches 2 m/s and the bound itself keeps the ground velocity far below
// 0.1 m/s, and T = 0.04 s covers the 0.038 s from the ramp's start to the end
// of the window. A missing ramp term moves the ground velocity by the whole
// 2 m/s. The position moves by at most the bound times the 0.05 s window.
//
// The ramp's slope changes, at 0.012 s and 0.036 s, fall on the integration
// lattice but strictly INSIDE controller periods. So the hold must be split at
// them as well as at ticks. Otherwise the last stage of the step ending at
// either change reads the next segment's slope, which moves the ground velocity
// by h / 6 times the 83 m/s^2 slope: twice the bound.
TEST_F(QuadrotorWorkflow, AWindRampThroughSimSampledMovesTheGroundVelocityOnlyThroughDrag) {
  const RunResult result =
      run(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                        "      controller_period_s: 0.01\n"
                        "      delay_periods: 5\n"
                        "      wind_schedule:\n"
                        "        hold: linear\n"
                        "        extrapolation: hold\n"
                        "        samples:\n"
                        "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                        "          - {time_s: 0.012, values: [0.0, 0.0, 0.0]}\n"
                        "          - {time_s: 0.036, values: [2.0, 0.0, 0.0]}\n",
                        50),
          {.overwrite = true, .write_manifest = false});
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& sampled = result.find("closed")->payload_as<SampledRun>("sampled_trajectory");
  const galata::model::Quadrotor& model = trimmed.model;
  const auto& states = sampled.plant.trajectory.states;
  const auto& winds = sampled.plant.wind_samples_ned_m_s;
  const auto& control = sampled.control;
  ASSERT_EQ(states.size(), 11U);
  ASSERT_TRUE(trimmed.point.wind_ned_m_s.isZero(0.0));
  ASSERT_EQ(trimmed.point.yaw_rad, 0.0);
  using galata::linearize::kChartAttitudeErrorX;
  for (std::size_t tick = 0; tick <= 5; ++tick) {
    ASSERT_LE(largest_magnitude(control.chart_error[tick].segment<3>(kChartAttitudeErrorX)), 1e-12)
        << "the vehicle tilted inside the trim-held window, at tick " << tick;
    EXPECT_TRUE(control.applied_rad_s[std::min<std::size_t>(tick, 4)]
                == trimmed.point.command_rad_s);
  }
  // The recorded wind is the interpolated one: at 0.02 s, a third of the way
  // up the ramp.
  EXPECT_NEAR(winds[2](0), 2.0 / 3.0, 1e-12);

  const double speed_m_s = 2.1;
  const double linear = model.drag_linear_n_s_m.maxCoeff();
  const double quadratic = model.drag_quadratic_n_s2_m2.maxCoeff();
  const double velocity_bound =
      (linear * speed_m_s + quadratic * speed_m_s * speed_m_s) / model.mass.mass_kg * 0.04;
  const auto ground = [&](std::size_t tick) -> Eigen::Vector3d {
    return states[tick].segment<3>(galata::core::kVelocityU) + winds[tick];
  };
  double worst_velocity = 0.0;
  for (std::size_t tick = 1; tick <= 5; ++tick) {
    worst_velocity = std::fmax(worst_velocity, largest_magnitude(ground(tick) - ground(0)));
  }
  EXPECT_LE(worst_velocity, velocity_bound)
      << "the ground velocity moved by " << worst_velocity << " m/s while the wind ramped; drag "
      << "alone can move it by at most " << velocity_bound << " m/s";
  const double moved =
      largest_magnitude(states[5].segment<3>(galata::core::kPositionNorth)
                        - trimmed.point.extended_state.segment<3>(galata::core::kPositionNorth));
  EXPECT_LE(moved, velocity_bound * 0.05)
      << "the vehicle moved " << moved << " m while the wind ramped; drag alone moves it at most "
      << velocity_bound * 0.05 << " m";
  record_discrepancy("ramp_ground_velocity_change_m_s", worst_velocity);
  record_discrepancy("ramp_velocity_bound_m_s", velocity_bound);
}

// THE SAME RAMP THROUGH `sim.plant`, which carried the ramp term before this
// closure and had no test of it. Open loop, from the still-air trim, with the
// rotors commanded to the trim throughout, so the only force that changes is the
// drag. The budget is the drag bound derived above the `sim.sampled` ramp test,
// over the 0.04 s of ramp and hold. A ramp rate read one stage early at a slope
// change moves the ground velocity by a sixth of a step of the whole slope, which
// this bound does not allow for.
TEST_F(QuadrotorWorkflow, AWindRampThroughSimPlantMovesTheGroundVelocityOnlyThroughDrag) {
  const RunResult result =
      run("version: 1\nstages:\n"
          "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
          "  - id: hover\n    capability: trim.hover\n    input: "
          "{quadrotor: {from: plant}, altitude_m: 120.0}\n"
          "  - id: fly\n    capability: sim.plant\n    input:\n"
          "      trim: {from: hover}\n"
          "      step_s: 0.002\n"
          "      steps: 25\n"
          "      sample_stride: 1\n"
          "      wind_schedule:\n"
          "        hold: linear\n"
          "        extrapolation: hold\n"
          "        samples:\n"
          "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
          "          - {time_s: 0.01, values: [0.0, 0.0, 0.0]}\n"
          "          - {time_s: 0.04, values: [2.0, 0.0, 0.0]}\n",
          {.overwrite = true, .write_manifest = false});
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& data = result.find("fly")->payload_as<PlantRun>("plant_trajectory");
  const galata::model::Quadrotor& model = trimmed.model;
  ASSERT_EQ(data.trajectory.states.size(), 26U);
  ASSERT_TRUE(trimmed.point.wind_ned_m_s.isZero(0.0));
  ASSERT_EQ(trimmed.point.yaw_rad, 0.0);
  ASSERT_LE(std::fabs(trimmed.point.roll_rad), 1e-12);
  ASSERT_LE(std::fabs(trimmed.point.pitch_rad), 1e-12);

  const double speed_m_s = 2.1;
  const double velocity_bound = (model.drag_linear_n_s_m.maxCoeff() * speed_m_s
                                 + model.drag_quadratic_n_s2_m2.maxCoeff() * speed_m_s * speed_m_s)
                                / model.mass.mass_kg * 0.04;
  const auto ground = [&](std::size_t sample) -> Eigen::Vector3d {
    return data.trajectory.states[sample].segment<3>(galata::core::kVelocityU)
           + data.wind_samples_ned_m_s[sample];
  };
  double worst_velocity = 0.0;
  for (std::size_t sample = 1; sample < data.trajectory.states.size(); ++sample) {
    worst_velocity = std::fmax(worst_velocity, largest_magnitude(ground(sample) - ground(0)));
  }
  EXPECT_LE(worst_velocity, velocity_bound)
      << "the ground velocity moved by " << worst_velocity << " m/s while the wind ramped; drag "
      << "alone can move it by at most " << velocity_bound << " m/s";
  record_discrepancy("plant_ramp_ground_velocity_change_m_s", worst_velocity);
}

// `extrapolation: refuse` MEANS REFUSE, FOR A ZERO-ORDER HISTORY TOO. A
// zero-order history is read only at t = 0 and at its declared events, all of
// which lie inside its span. So nothing would read it past its end unless the
// horizon is checked on purpose. Each capability refuses before its first step
// a history that ends before the run does. Both kinds of schedule, in both
// capabilities.
TEST_F(QuadrotorWorkflow, AZeroOrderHistoryShorterThanTheRunIsRefusedUnderRefuse) {
  const std::string plant_head =
      "version: 1\nstages:\n"
      "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
      "  - id: hover\n    capability: trim.hover\n    input: "
      "{quadrotor: {from: plant}, altitude_m: 120.0}\n"
      "  - id: fly\n    capability: sim.plant\n    input:\n"
      "      trim: {from: hover}\n      step_s: 0.002\n      steps: 50\n";
  const auto expect_refused = [&](const std::string& document, const std::string& what) {
    try {
      (void)run(document, {.overwrite = true, .write_manifest = false});
      ADD_FAILURE() << what << " ends at 0.05 s under `extrapolation: refuse` and the run "
                    << "reaches 0.1 s, but it was flown";
    } catch (const std::runtime_error& error) {
      EXPECT_NE(std::string(error.what()).find("outside the declared span"), std::string::npos)
          << what << ": " << error.what();
    }
  };
  expect_refused(plant_head
                     + "      wind_schedule:\n        hold: zero_order\n"
                       "        extrapolation: refuse\n        samples:\n"
                       "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                       "          - {time_s: 0.05, values: [1.0, 0.0, 0.0]}\n",
                 "a sim.plant wind_schedule");
  expect_refused(plant_head
                     + "      command_schedule:\n        hold: zero_order\n"
                       "        extrapolation: refuse\n        samples:\n"
                       "          - {time_s: 0.0, values: [620.0, 620.0, 620.0, 620.0]}\n"
                       "          - {time_s: 0.05, values: [630.0, 630.0, 630.0, 630.0]}\n",
                 "a sim.plant command_schedule");
  expect_refused(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                               "      controller_period_s: 0.01\n"
                               "      delay_periods: 5\n"
                               "      wind_schedule:\n"
                               "        hold: zero_order\n"
                               "        extrapolation: refuse\n"
                               "        samples:\n"
                               "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                               "          - {time_s: 0.05, values: [1.0, 0.0, 0.0]}\n",
                               50),
                 "a sim.sampled wind_schedule");
}

// A RAMP THAT ENDS EXACTLY AT THE HORIZON. The run is 13 steps of 2 ms, and
// 13 * 0.002 overshoots the declared 0.026 s by one unit in the last place.
// Two things must not follow from that:
//   - Refusal by rounding. Under `extrapolation: refuse` the history covers the
//     run exactly, so reading it at the overshooting horizon must not refuse it.
//   - A misread last step. The slope change at the horizon is a segment end like
//     any other. The last RK4 stage must read the ramp's own slope, not the zero
//     beyond it, which would put a sixth of a step of the whole slope into the
//     air-relative velocity.
// The budget is the drag bound of the ramp tests above, over the 0.02 s during
// which the wind is nonzero. A misread last step moves the ground velocity by
// h / 6 times the 100 m/s^2 slope, several times that bound.
TEST_F(QuadrotorWorkflow, ARampEndingAtTheHorizonIsNeitherRefusedByRoundingNorMisreadAtTheEnd) {
  const RunResult result =
      run("version: 1\nstages:\n"
          "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
          "  - id: hover\n    capability: trim.hover\n    input: "
          "{quadrotor: {from: plant}, altitude_m: 120.0}\n"
          "  - id: fly\n    capability: sim.plant\n    input:\n"
          "      trim: {from: hover}\n"
          "      step_s: 0.002\n"
          "      steps: 13\n"
          "      sample_stride: 1\n"
          "      wind_schedule:\n"
          "        hold: linear\n"
          "        extrapolation: refuse\n"
          "        samples:\n"
          "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
          "          - {time_s: 0.006, values: [0.0, 0.0, 0.0]}\n"
          "          - {time_s: 0.026, values: [2.0, 0.0, 0.0]}\n",
          {.overwrite = true, .write_manifest = false});
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& data = result.find("fly")->payload_as<PlantRun>("plant_trajectory");
  const galata::model::Quadrotor& model = trimmed.model;
  ASSERT_EQ(data.trajectory.states.size(), 14U);
  ASSERT_EQ(trimmed.point.yaw_rad, 0.0);
  EXPECT_TRUE(data.wind_samples_ned_m_s.back() == Eigen::Vector3d(2.0, 0.0, 0.0))
      << "the last sample must carry the history's last declared value";

  const double speed_m_s = 2.1;
  const double velocity_bound = (model.drag_linear_n_s_m.maxCoeff() * speed_m_s
                                 + model.drag_quadratic_n_s2_m2.maxCoeff() * speed_m_s * speed_m_s)
                                / model.mass.mass_kg * 0.02;
  const auto ground = [&](std::size_t sample) -> Eigen::Vector3d {
    return data.trajectory.states[sample].segment<3>(galata::core::kVelocityU)
           + data.wind_samples_ned_m_s[sample];
  };
  double worst_velocity = 0.0;
  for (std::size_t sample = 1; sample < data.trajectory.states.size(); ++sample) {
    worst_velocity = std::fmax(worst_velocity, largest_magnitude(ground(sample) - ground(0)));
  }
  EXPECT_LE(worst_velocity, velocity_bound)
      << "the ground velocity moved by " << worst_velocity << " m/s over a ramp ending at the "
      << "horizon; drag alone can move it by at most " << velocity_bound << " m/s";
  record_discrepancy("horizon_ramp_ground_velocity_change_m_s", worst_velocity);
}

// TWO EVENTS ON ONE STEP. Declared 1e-13 s apart, both round to step 500 of a
// 1 ms run. The value in force after that step is the second one's, 3 m/s, and
// the air-relative velocity must absorb the whole change to it, not only the
// first event's 1 m/s. Otherwise the recorded wind and the state disagree by a
// ground-velocity jump of 2 m/s that no force produced.
TEST_F(QuadrotorWorkflow, EveryWindEventThatLandsOnOneStepIsAbsorbed) {
  const RunResult result =
      run("version: 1\nstages:\n"
          "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
          "  - id: hover\n    capability: trim.hover\n    input: "
          "{quadrotor: {from: plant}, altitude_m: 120.0}\n"
          "  - id: fly\n    capability: sim.plant\n    input:\n"
          "      trim: {from: hover}\n"
          "      step_s: 0.001\n"
          "      steps: 600\n"
          "      sample_stride: 1\n"
          "      wind_schedule:\n"
          "        hold: zero_order\n"
          "        extrapolation: hold\n"
          "        samples:\n"
          "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
          "          - {time_s: 0.5, values: [1.0, 0.0, 0.0]}\n"
          "          - {time_s: 0.5000000000001, values: [3.0, 0.0, 0.0]}\n",
          {.overwrite = true, .write_manifest = false});
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& data = result.find("fly")->payload_as<PlantRun>("plant_trajectory");
  ASSERT_EQ(data.trajectory.states.size(), 601U);
  ASSERT_EQ(trimmed.point.yaw_rad, 0.0);
  ASSERT_LE(std::fabs(trimmed.point.roll_rad), 1e-12);
  ASSERT_LE(std::fabs(trimmed.point.pitch_rad), 1e-12);
  const auto ground = [&](std::size_t sample) -> Eigen::Vector3d {
    return data.trajectory.states[sample].segment<3>(galata::core::kVelocityU)
           + data.wind_samples_ned_m_s[sample];
  };
  // THE BUDGET, derived from premises that are asserted. The ground velocity
  // above adds body components to NED ones, which is exact only at R = I. The
  // flown attitude at sample 500 is asserted within 2e-11 rad of it: the level
  // trim accounts for at most 1.5e-12 rad, and 0.5 s of open-loop drift under
  // the trim's residual gate, 1e-10 rad/s^2, for at most 1.25e-11 rad. Through
  // that rotation the 3 m/s jump is misread by at most 6e-11 m/s. The 1 ms step
  // before the event adds the trim's residual, 1e-13 m/s, and the gravity the
  // drift misprojects, about 1.2e-13 m/s. The budget is 1e-10 m/s, above the
  // sum of about 6.1e-11. A lost event leaves a step of 2 m/s.
  ASSERT_LE(trimmed.point.residual_tolerance, 1e-10) << "the budget assumes trim.hover's gate";
  ASSERT_LE(trimmed.point.residual_norm, trimmed.point.residual_tolerance);
  const double rotation_rad =
      2.0 * data.trajectory.states[500].segment<3>(galata::core::kQuaternionX).norm();
  ASSERT_LE(rotation_rad, 2e-11) << "the budget assumes the flown attitude is within 2e-11 rad "
                                    "of level at zero heading";
  constexpr double kCoincidentEventBudget = 1e-10;  // m/s
  EXPECT_TRUE(data.wind_samples_ned_m_s[500] == Eigen::Vector3d(3.0, 0.0, 0.0));
  EXPECT_LE(largest_magnitude(ground(500) - ground(499)), kCoincidentEventBudget)
      << "two wind events on one step moved the ground velocity from " << ground(499).transpose()
      << " to " << ground(500).transpose();
}

// A RAMP THE INTEGRATOR CANNOT SEE IS REFUSED. A linear wind's rate enters the
// air-relative velocity, so a change of slope inside a step is integrated as if
// it happened at one of RK4's stages, and the error stays in the ground
// velocity. A ramp 1e-4 s long inside one 2 ms step falls between the stages
// and is missed entirely: its 3 m/s would reach the recorded wind and never the
// state. Both capabilities refuse a sample off the lattice, as they refuse a
// step there. Two samples 1e-13 s apart both land on step 13, so each is on
// the lattice, and the pair is refused by name: a step written as a ramp.
TEST_F(QuadrotorWorkflow, ALinearWindThatChangesSlopeInsideAStepIsRefused) {
  const std::string plant_head =
      "version: 1\nstages:\n"
      "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
      "  - id: hover\n    capability: trim.hover\n    input: "
      "{quadrotor: {from: plant}, altitude_m: 120.0}\n"
      "  - id: fly\n    capability: sim.plant\n    input:\n"
      "      trim: {from: hover}\n      step_s: 0.002\n      steps: 25\n";
  const auto expect_refused =
      [&](const std::string& document, const std::string& what, const std::string& reason) {
        try {
          (void)run(document, {.overwrite = true, .write_manifest = false});
          ADD_FAILURE() << what << " was flown";
        } catch (const std::runtime_error& error) {
          const std::string message = error.what();
          EXPECT_NE(message.find("wind_schedule"), std::string::npos) << what << ": " << message;
          EXPECT_NE(message.find(reason), std::string::npos) << what << ": " << message;
        }
      };
  const std::string sub_step_ramp =
      "        samples:\n"
      "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
      "          - {time_s: 0.0251, values: [0.0, 0.0, 0.0]}\n"
      "          - {time_s: 0.0252, values: [3.0, 0.0, 0.0]}\n";
  expect_refused(plant_head
                     + "      wind_schedule:\n        hold: linear\n        extrapolation: hold\n"
                     + sub_step_ramp,
                 "a sim.plant ramp inside one step",
                 "not a whole number of");
  expect_refused(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                               "      controller_period_s: 0.01\n"
                               "      delay_periods: 5\n"
                               "      wind_schedule:\n"
                               "        hold: linear\n"
                               "        extrapolation: hold\n"
                                   + sub_step_ramp,
                               50),
                 "a sim.sampled ramp inside one step",
                 "not a whole number of");
  expect_refused(plant_head
                     + "      wind_schedule:\n        hold: linear\n        extrapolation: hold\n"
                       "        samples:\n"
                       "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                       "          - {time_s: 0.026, values: [0.0, 0.0, 0.0]}\n"
                       "          - {time_s: 0.0260000000001, values: [3.0, 0.0, 0.0]}\n",
                 "a sim.plant ramp of 1e-13 s on one lattice step",
                 "on the same integration step");
}

// A LINEAR COMMAND HISTORY ENDING AT THE HORIZON. The ramp test above cannot see
// this, because its history is the wind. The same 13 steps of 2 ms overshoot
// the declared 0.026 s by a unit in the last place. Under `extrapolation:
// refuse` the command history must be neither refused by that rounding nor read
// past its end. The last recorded command is its last declared value.
TEST_F(QuadrotorWorkflow, ALinearCommandHistoryEndingAtTheHorizonIsNotRefusedByRounding) {
  const RunResult result =
      run("version: 1\nstages:\n"
          "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
          "  - id: hover\n    capability: trim.hover\n    input: "
          "{quadrotor: {from: plant}, altitude_m: 120.0}\n"
          "  - id: fly\n    capability: sim.plant\n    input:\n"
          "      trim: {from: hover}\n      step_s: 0.002\n      steps: 13\n"
          "      sample_stride: 1\n"
          "      command_schedule:\n        hold: linear\n        extrapolation: refuse\n"
          "        samples:\n"
          "          - {time_s: 0.0, values: [620.0, 620.0, 620.0, 620.0]}\n"
          "          - {time_s: 0.026, values: [630.0, 630.0, 630.0, 630.0]}\n",
          {.overwrite = true, .write_manifest = false});
  const auto& data = result.find("fly")->payload_as<PlantRun>("plant_trajectory");
  ASSERT_EQ(data.command_samples_rad_s.size(), 14U);
  EXPECT_TRUE((data.command_samples_rad_s.back().array() == 630.0).all())
      << "the last sample must carry the history's last declared command, "
      << data.command_samples_rad_s.back().transpose();
}

// THE SAME HORIZON THROUGH `sim.sampled`, which handles its horizon in its own
// code. A linear wind ends exactly at a horizon of 13 steps of 2 ms, under
// `extrapolation: refuse`, with the controller running every step. The run must
// not be refused by the rounding, and its last recorded wind is the history's
// last declared value.
TEST_F(QuadrotorWorkflow, ALinearWindHistoryEndingAtTheHorizonIsNotRefusedBySimSampled) {
  const RunResult result =
      run(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                        "      controller_period_s: 0.002\n"
                        "      delay_periods: 5\n"
                        "      wind_schedule:\n"
                        "        hold: linear\n"
                        "        extrapolation: refuse\n"
                        "        samples:\n"
                        "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                        "          - {time_s: 0.006, values: [0.0, 0.0, 0.0]}\n"
                        "          - {time_s: 0.026, values: [2.0, 0.0, 0.0]}\n",
                        13),
          {.overwrite = true, .write_manifest = false});
  const auto& sampled = result.find("closed")->payload_as<SampledRun>("sampled_trajectory");
  ASSERT_EQ(sampled.plant.wind_samples_ned_m_s.size(), 14U);
  EXPECT_TRUE(sampled.plant.wind_samples_ned_m_s.back() == Eigen::Vector3d(2.0, 0.0, 0.0))
      << "the last sample must carry the history's last declared wind";
}

// COINCIDENT EVENTS THROUGH `sim.sampled`. This is the flight of the first sampled
// wind test, except that its first step is split into two events 1e-13 s apart.
// Both land on step 13: first to (1, 0, 0), then to (3, 2, 0). Absorbed as their
// sum, they leave exactly the flight the single step does. The same premises
// are asserted here, so the analytic window and its budget apply unchanged.
TEST_F(QuadrotorWorkflow, CoincidentWindEventsBetweenTicksAreAbsorbedBySimSampled) {
  const RunResult result =
      run(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                        "      controller_period_s: 0.01\n"
                        "      delay_periods: 5\n"
                        "      wind_schedule:\n"
                        "        hold: zero_order\n"
                        "        extrapolation: hold\n"
                        "        samples:\n"
                        "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                        "          - {time_s: 0.026, values: [1.0, 0.0, 0.0]}\n"
                        "          - {time_s: 0.0260000000001, values: [3.0, 2.0, 0.0]}\n"
                        "          - {time_s: 0.04, values: [1.0, 2.0, -1.0]}\n",
                        50),
          {.overwrite = true, .write_manifest = false});
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& sampled = result.find("closed")->payload_as<SampledRun>("sampled_trajectory");
  const galata::model::Quadrotor& model = trimmed.model;
  const auto& states = sampled.plant.trajectory.states;
  const auto& winds = sampled.plant.wind_samples_ned_m_s;
  const auto& control = sampled.control;
  ASSERT_EQ(states.size(), 11U);
  ASSERT_EQ(winds.size(), states.size());
  ASSERT_LE(trimmed.point.residual_tolerance, 1e-10) << "the budget assumes trim.hover's gate";
  ASSERT_LE(trimmed.point.residual_norm, trimmed.point.residual_tolerance);
  ASSERT_TRUE(trimmed.point.wind_ned_m_s.isZero(0.0)) << "the trim must be a still-air hover";
  ASSERT_EQ(trimmed.point.yaw_rad, 0.0);
  ASSERT_LE(std::fabs(trimmed.point.roll_rad), 1e-12) << "a still-air hover must be level";
  ASSERT_LE(std::fabs(trimmed.point.pitch_rad), 1e-12) << "a still-air hover must be level";
  for (int axis = 0; axis < 3; ++axis) {
    ASSERT_GT(model.drag_linear_n_s_m(axis), 0.0);
    ASSERT_GT(model.drag_quadratic_n_s2_m2(axis), 0.0);
  }
  using galata::linearize::kChartAttitudeErrorX;
  for (std::size_t tick = 0; tick <= 5; ++tick) {
    ASSERT_LE(largest_magnitude(control.chart_error[tick].segment<3>(kChartAttitudeErrorX)), 1e-12)
        << "the vehicle tilted inside the trim-held window, at tick " << tick;
  }
  EXPECT_TRUE(winds[3] == Eigen::Vector3d(3.0, 2.0, 0.0))
      << "the wind after both events must be the later one's";

  const WindWindow expected = predict_wind_window(model);
  const auto air = [&](std::size_t tick) -> Eigen::Vector3d {
    return states[tick].segment<3>(galata::core::kVelocityU);
  };
  EXPECT_LE(largest_magnitude(air(3) - expected.at_tick_3.air_m_s), kWindWindowBudget)
      << "two events on one step were not absorbed as their sum: " << air(3).transpose()
      << " against " << expected.at_tick_3.air_m_s.transpose();
  EXPECT_LE(largest_magnitude(air(4) - expected.after_second), kWindWindowBudget)
      << air(4).transpose() << " against " << expected.after_second.transpose();
  EXPECT_LE(largest_magnitude(air(5) - expected.at_tick_5.air_m_s), kWindWindowBudget)
      << air(5).transpose() << " against " << expected.at_tick_5.air_m_s.transpose();
}

// A WIND STEP ON THE LATTICE, AT A TIME THE STEP DOES NOT MULTIPLY TO EXACTLY.
// 0.026 s is thirteen 2 ms steps, but 13 * 0.002 is 0.026000000000000002 in
// floating point. Until 2026-09-11 `sim.plant` looked the jump up at
// k * step_s, which the history does not recognise as its declared 0.026 s, so
// it refused a step that is on the lattice. The trim here is level at a 30
// degree heading, so the jump is -Rz(30 deg)^T dw. The zero-heading test above
// cannot tell that apart from the unrotated -dw.
TEST_F(QuadrotorWorkflow, SimPlantAcceptsALatticeWindStepAtATimeTheStepDoesNotMultiplyTo) {
  const RunResult result =
      run("version: 1\nstages:\n"
          "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
          "  - id: hover\n    capability: trim.hover\n    input: "
          "{quadrotor: {from: plant}, altitude_m: 120.0, heading_deg: 30.0}\n"
          "  - id: fly\n    capability: sim.plant\n    input:\n"
          "      trim: {from: hover}\n"
          "      step_s: 0.002\n"
          "      steps: 50\n"
          "      sample_stride: 1\n"
          "      wind_schedule:\n"
          "        hold: zero_order\n"
          "        extrapolation: hold\n"
          "        samples:\n"
          "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
          "          - {time_s: 0.026, values: [3.0, 2.0, 0.0]}\n",
          {.overwrite = true, .write_manifest = false});
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& data = result.find("fly")->payload_as<PlantRun>("plant_trajectory");
  ASSERT_EQ(data.trajectory.states.size(), 51U);
  ASSERT_NEAR(trimmed.point.yaw_rad, std::numbers::pi / 6.0, 1e-15);
  ASSERT_LE(std::fabs(trimmed.point.roll_rad), 1e-12);
  ASSERT_LE(std::fabs(trimmed.point.pitch_rad), 1e-12);

  const double c = std::cos(trimmed.point.yaw_rad);
  const double s = std::sin(trimmed.point.yaw_rad);
  Eigen::Matrix3d ned_from_body;
  ned_from_body << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  const Eigen::Vector3d wind(3.0, 2.0, 0.0);
  // Sample 13 is the state after the event at 0.026 s and sample 12 the one
  // before it, in still air, so nothing but the jump separates them.
  //
  // THE BUDGET, derived from premises that are asserted. The code applies
  // -R^T dw at the attitude FLOWN at sample 13; the expectation uses the trim's
  // heading alone. The trim's roll and pitch, asserted at 1e-12 rad each, put
  // its attitude within 1.5e-12 rad of Rz(30 deg). The flown attitude is
  // asserted within 1e-13 rad of the trim's; 26 ms of drift under the residual
  // gate allows about 3.4e-14. Through at most 1.6e-12 rad the 3.6 m/s jump is
  // misread by at most 5.8e-12 m/s, and the 2 ms still-air step before it adds
  // the trim's residual, at most 2e-13 m/s. The budget is 1e-11 m/s. A jump
  // through the unrotated -dw is off by about 1.8 m/s.
  ASSERT_LE(trimmed.point.residual_tolerance, 1e-10) << "the budget assumes trim.hover's gate";
  ASSERT_LE(trimmed.point.residual_norm, trimmed.point.residual_tolerance);
  const Eigen::Vector4d trim_attitude =
      trimmed.point.extended_state.segment<4>(galata::core::kQuaternionW);
  const Eigen::Vector4d flown_attitude =
      data.trajectory.states[13].segment<4>(galata::core::kQuaternionW);
  ASSERT_LE(2.0 * (flown_attitude - trim_attitude).norm(), 1e-13)
      << "the budget assumes the vehicle has not turned from its trim by sample 13";
  constexpr double kLatticeJumpBudget = 1e-11;  // m/s
  const Eigen::Vector3d air_before =
      data.trajectory.states[12].segment<3>(galata::core::kVelocityU);
  const Eigen::Vector3d air_after = data.trajectory.states[13].segment<3>(galata::core::kVelocityU);
  EXPECT_LE(largest_magnitude(air_after - air_before + ned_from_body.transpose() * wind),
            kLatticeJumpBudget)
      << "the air-relative velocity did not jump by -Rz^T dw: "
      << (air_after - air_before).transpose();
  EXPECT_TRUE(data.wind_samples_ned_m_s[12].isZero(0.0));
  EXPECT_TRUE(data.wind_samples_ned_m_s[13] == wind)
      << "the sample at the event must carry the wind that starts there";
}

// --- a law designed in discrete time -----------------------------------------

// THE PERIOD IS THE DESIGN'S. A discrete gain is optimal for the period its
// plant and cost were discretised at and for no other, and no transformation
// between rates is supported — so running it at any other period is refused,
// faster and slower alike, and the refusal names the remedy.
TEST_F(QuadrotorWorkflow, ADiscreteLawIsRefusedAtAPeriodItWasNotDesignedFor) {
  for (const char* period : {"0.008", "0.002"}) {
    try {
      (void)run(discrete_chain(std::string("      controller_period_s: ") + period
                               + "\n      hold: zero_order\n      delay_periods: 0\n"),
                {.overwrite = true, .write_manifest = false});
      ADD_FAILURE() << "a law designed at 0.004 s ran at " << period << " s";
    } catch (const std::runtime_error& error) {
      const std::string message = error.what();
      EXPECT_NE(message.find("designed at a sample time of"), std::string::npos) << message;
      EXPECT_NE(message.find("synth.sampled_lqr"), std::string::npos)
          << "the refusal must name the redesign that would make it admissible: " << message;
    }
  }
}

// THE TIMING IS DECLARED. The design modelled a hold and no delay, so a study
// flying it must state both rather than inherit defaults written for a
// continuous law.
TEST_F(QuadrotorWorkflow, ADiscreteLawMustDeclareItsHoldAndItsDelay) {
  EXPECT_THROW((void)run(discrete_chain("      controller_period_s: 0.004\n"
                                        "      delay_periods: 0\n"),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error)
      << "no hold declared";
  EXPECT_THROW((void)run(discrete_chain("      controller_period_s: 0.004\n"
                                        "      hold: zero_order\n"),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error)
      << "no delay declared";
  EXPECT_THROW((void)run(discrete_chain("      controller_period_s: 0.004\n"
                                        "      hold: first_order\n"
                                        "      delay_periods: 0\n"),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error)
      << "a hold this schedule does not execute";
  // The design itself refuses the same hold, by the same name.
  EXPECT_THROW(
      (void)run(discrete_chain("      controller_period_s: 0.004\n"
                               "      hold: zero_order\n"
                               "      delay_periods: 0\n",
                               "sample_time_s: 0.004, hold: tustin, evidence_path: lqr.yaml"),
                {.overwrite = true, .write_manifest = false}),
      std::runtime_error);
}

// THE INDEPENDENT CHECK, for a discrete law. As for the continuous one, the
// trajectory is not re-derived; the SAMPLED LOGIC is. From the chart errors the
// run recorded, the law is recomputed with the DISCRETE gain and required to
// reproduce every requested command, the delay line is required to shift by
// exactly the declared period, and the prediction is required to start where
// the run did.
TEST_F(QuadrotorWorkflow, TheDiscreteLawIsExecutedAsDesigned) {
  const RunResult result = run(discrete_chain("      controller_period_s: 0.004\n"
                                              "      hold: zero_order\n"
                                              "      delay_periods: 1\n"
                                              "      initial_chart_perturbation: "
                                              "[0.2, -0.1, 0.1, 0,0,0, 0.01,0.0,0.0, 0,0,0, "
                                              "0,0,0,0]\n"),
                               {.overwrite = true, .write_manifest = false});
  const auto& design =
      result.find("lqr")->payload_as<galata::synth::SampledLqrDesign>("sampled_control_law");
  const auto& trimmed = result.find("hover")->payload_as<HoverTrimArtifact>("hover_trim");
  const auto& control = result.find("closed")->payload_as<SampledRun>("sampled_trajectory").control;

  EXPECT_EQ(control.law_time_domain, "discrete_design");
  EXPECT_EQ(control.design_sample_time_s, 0.004);
  ASSERT_GT(control.tick_times_s.size(), 10U);
  for (std::size_t tick = 0; tick < control.tick_times_s.size(); ++tick) {
    const Eigen::VectorXd expected =
        trimmed.point.command_rad_s - design.riccati.k * control.chart_error[tick];
    EXPECT_LT((expected - control.requested_rad_s[tick]).norm(), 1e-12) << "tick " << tick;
    if (tick >= 1) {
      EXPECT_LT((control.applied_rad_s[tick] - control.saturated_rad_s[tick - 1]).norm(), 1e-12)
          << "tick " << tick << " did not receive the command computed one period earlier";
    }
  }
  ASSERT_TRUE(control.prediction.available) << control.prediction.unavailable_reason;
  EXPECT_EQ(control.prediction.predicted_chart.size(), control.tick_times_s.size() + 1);
  EXPECT_EQ(control.prediction.predicted_chart.front(), control.chart_error.front());
  EXPECT_FALSE(control.prediction.budget_declared)
      << "no budget was declared, so the comparison is reported without a verdict";
}

// A GAIN ON THE WRONG BASIS IS REFUSED. With the allocation the identity, a law
// whose inputs are the rotor commands in another order would drive each rotor
// with another rotor's row of the gain — a plausible, wrong aircraft. The
// check applies to a continuous law as much as to a discrete one.
TEST_F(QuadrotorWorkflow, ALawWhoseInputsAreNotTheRotorCommandsInOrderIsRefused) {
  for (const bool discrete : {false, true}) {
    std::string study = discrete ? discrete_chain(
                                       "      controller_period_s: 0.004\n"
                                       "      hold: zero_order\n"
                                       "      delay_periods: 0\n")
                                 : sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                                                 "      controller_period_s: 0.004\n");
    const std::string in_order = "inputs: [omega_command_0, omega_command_1,";
    const auto at = study.find(in_order);
    ASSERT_NE(at, std::string::npos);
    study.replace(at, in_order.size(), "inputs: [omega_command_1, omega_command_0,");
    try {
      (void)run(study, {.overwrite = true, .write_manifest = false});
      ADD_FAILURE() << "a permuted " << (discrete ? "discrete" : "continuous") << " law ran";
    } catch (const std::runtime_error& error) {
      EXPECT_NE(std::string(error.what()).find("rotor commands"), std::string::npos)
          << error.what();
    }
  }
}

// A BUDGET ON NOTHING IS REFUSED. A continuous law has no discrete prediction,
// and a declared wind history acts on the plant and not on a prediction; in
// both cases a declared agreement budget would have nothing to hold.
TEST_F(QuadrotorWorkflow, AnAgreementBudgetWithNothingToHoldIsRefused) {
  EXPECT_THROW((void)run(sampled_chain("{quadrotor: {from: plant}, altitude_m: 120.0}",
                                       "      controller_period_s: 0.004\n"
                                       "      linear_agreement_budget: 0.05\n"),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error)
      << "a continuous law";
  EXPECT_THROW((void)run(discrete_chain("      controller_period_s: 0.004\n"
                                        "      hold: zero_order\n"
                                        "      delay_periods: 0\n"
                                        "      linear_agreement_budget: 0.05\n"
                                        "      wind_schedule:\n"
                                        "        hold: zero_order\n"
                                        "        extrapolation: hold\n"
                                        "        samples:\n"
                                        "          - {time_s: 0.0, values: [0.0, 0.0, 0.0]}\n"
                                        "          - {time_s: 0.4, values: [1.0, 0.0, 0.0]}\n"),
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error)
      << "a declared wind history";
}

// AN UNDEFINED COMPARISON HAS NO VERDICT. A run that starts exactly at the
// reference makes the design predict nothing but zero, so its discrepancy
// relative to the prediction's peak does not exist. A declared budget can then
// be neither met nor missed, and the report must say that rather than print
// OUTSIDE beside "undefined".
TEST_F(QuadrotorWorkflow, ABudgetOnAnUndefinedComparisonGetsNoVerdict) {
  std::string study = discrete_chain(
      "      controller_period_s: 0.004\n"
      "      hold: zero_order\n"
      "      delay_periods: 0\n"
      "      linear_agreement_budget: 0.05\n");
  study +=
      "  - id: report\n    capability: report.markdown\n    input: "
      "{sections: [{from: closed}], path: closed.md}\n";
  const RunResult result = run(study, {.overwrite = true, .write_manifest = false});
  const auto& prediction =
      result.find("closed")->payload_as<SampledRun>("sampled_trajectory").control.prediction;
  ASSERT_TRUE(prediction.available) << prediction.unavailable_reason;
  EXPECT_TRUE(prediction.budget_declared);
  EXPECT_FALSE(prediction.relative_discrepancy_defined)
      << "a run started at the reference has an identically zero prediction";

  std::ifstream file(output / "closed.md");
  const std::string report((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
  EXPECT_EQ(report.find("OUTSIDE the declared budget"), std::string::npos)
      << "an undefined comparison was reported as a budget miss";
  EXPECT_NE(report.find("can be neither met nor missed"), std::string::npos);
}

// A GAIN DESIGNED ON ANOTHER CHART IS REFUSED, even when its shape and its
// inputs match. The linearisation is exported, one appended coordinate is
// renamed in the file, and the renamed model is read back, designed on and
// flown: the gain is the right size for this vehicle and commands the right
// rotors, and it still describes a state this vehicle does not have.
TEST_F(QuadrotorWorkflow, ALawDesignedOnAnotherChartIsRefused) {
  (void)run(chain("{quadrotor: {from: plant}, altitude_m: 120.0}"),
            {.overwrite = true, .write_manifest = false});
  const fs::path exported = output / "quad-hover.yaml";
  std::string text = read_file_bytes(exported.string());
  const auto at = text.find("\"omega_3\"");
  ASSERT_NE(at, std::string::npos) << "the export no longer names the rotor state omega_3";
  text.replace(at, std::string("\"omega_3\"").size(), "\"omega_rear_right\"");
  put(root / "other-chart.yaml", text);

  const std::string study =
      "version: 1\nstages:\n"
      "  - id: plant\n    capability: model.quadrotor\n    input: {path: quad.yaml}\n"
      "  - id: hover\n    capability: trim.hover\n"
      "    input: {quadrotor: {from: plant}, altitude_m: 120.0}\n"
      "  - id: other\n    capability: model.linear.statespace\n"
      "    input: {path: other-chart.yaml}\n"
      "  - id: rotors\n    capability: model.channels\n    input:\n"
      "      system: {from: other}\n"
      "      inputs: [omega_command_0, omega_command_1, omega_command_2, omega_command_3]\n"
      "  - id: lqr\n    capability: synth.sampled_lqr\n"
      "    input: {system: {from: rotors}, sample_time_s: 0.004, hold: zero_order, "
      "evidence_path: lqr.yaml, q: "
      + identity_weight(16, 1.0) + ", r: " + identity_weight(4, 0.02)
      + "}\n"
        "  - id: closed\n    capability: sim.sampled\n    input:\n"
        "      trim: {from: hover}\n      law: {from: lqr}\n"
        "      controller_period_s: 0.004\n      hold: zero_order\n      delay_periods: 0\n"
        "      step_s: 0.002\n      steps: 40\n";
  try {
    (void)run(study, {.overwrite = true, .write_manifest = false}, (root / "second").string());
    ADD_FAILURE() << "a law designed on a chart with a coordinate this vehicle lacks was flown";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("chart coordinate"), std::string::npos) << message;
    EXPECT_NE(message.find("omega_rear_right"), std::string::npos) << message;
  }
}

// SAMPLED-LOOP MARGINS ARE NOT DELIVERED, and the continuous margin path does
// not quietly supply them. `model.control_system` builds loops from a
// CONTINUOUS design's plant; handed a discrete design it is refused by the
// artefact kind, so no continuous-domain margin can be read off a sampled law.
TEST_F(QuadrotorWorkflow, TheContinuousMarginPathRefusesADiscreteDesign) {
  std::string study = discrete_chain(
      "      controller_period_s: 0.004\n"
      "      hold: zero_order\n"
      "      delay_periods: 0\n");
  study +=
      "  - id: loop\n    capability: model.control_system\n    input: "
      "{law: {from: lqr}, use: single_loop, channel: omega_command_0}\n";
  try {
    (void)run(study, {.overwrite = true, .write_manifest = false});
    ADD_FAILURE() << "a discrete design reached the continuous margin path";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("sampled_control_law"), std::string::npos)
        << error.what();
  }
}

// --- the two analyses that were unavailable on this plant --------------------

// RFC-0002 asked for reachability and observability reporting "so that an
// exported model's defective integrator chains and any unobservable direction
// are reported rather than discovered from a failed synthesis". This is that,
// on the real hover linearisation, through the public capability.
TEST_F(QuadrotorWorkflow, ReachabilityAndObservabilityAreReportedBeforeAnyDesign) {
  const RunResult result = run(chain()
                                   + "  - id: gram\n    capability: analyze.gramians\n"
                                     "    input: {system: {from: linear}, horizon_s: 4.0}\n",
                               {.overwrite = true, .write_manifest = false});
  const Artifact* gram = result.find("gram");
  ASSERT_NE(gram, nullptr);
  const auto& analysis = gram->payload_as<galata::analyze::GramianAnalysis>("gramians");

  // Every chart direction is reachable: four rotor commands plus the wind
  // columns move the whole sixteen-coordinate state.
  EXPECT_EQ(analysis.reachability.rank, analysis.reachability.state_count);
  EXPECT_TRUE(analysis.reachability.missing_directions.empty());

  // THE INFINITE-HORIZON GRAMIANS DO NOT EXIST FOR THIS MODEL, and the analysis
  // must say so rather than returning a Lyapunov solution that is not a Gramian
  // of anything. Six eigenvalues sit at the origin — three because nothing reads
  // position, three because nothing reads attitude at a level hover.
  EXPECT_FALSE(analysis.spectrum_is_strictly_stable);
  EXPECT_NEAR(analysis.rightmost_eigenvalue_real_part, 0.0, 1e-9);
  EXPECT_NE(analysis.assumptions.find("do not exist for this model"), std::string::npos)
      << analysis.assumptions;
  EXPECT_GT(analysis.horizon_s, 0.0);

  // The heading is unobservable in THE OBSERVATION MODEL THIS CHAIN DECLARES:
  // body rates, position, altitude, ground velocity and specific force, none of
  // which is a heading reference. It is a statement about that declared output
  // set and about nothing else — not about any airframe's sensors, which may
  // well include a heading reference; a model that omits one is unobservable in
  // yaw whether or not the aircraft is. Adding a magnetic observation would be
  // a model extension, not a fix to this. What the assertion is for is that the
  // answer arrives HERE rather than three stages later as a Riccati diagnostic
  // a reader has to work backwards from.
  ASSERT_FALSE(analysis.observability.missing_directions.empty())
      << "a hover observation model with no heading reference must leave yaw unobservable";
  bool names_yaw = false;
  for (const auto& direction : analysis.observability.missing_directions) {
    for (const std::string& state : direction.dominant_states) {
      names_yaw = names_yaw || state.find("attitude_error_z") != std::string::npos;
    }
  }
  EXPECT_TRUE(names_yaw) << "the unobservable direction must be NAMED, not merely counted";
}

// The frequency-domain margins the audit found unavailable on this plant. The
// refusal was correct: breaking one channel of the MIMO return ratio leaves the
// other three OPEN, and that closure is not internally stable. The loop-at-a-
// time reading, with the other loops closed, is well posed and is what a margin
// can be computed from.
TEST_F(QuadrotorWorkflow, SingleLoopMarginsAreAvailableWhereTheBrokenLoopIsRefused) {
  const std::string design = chain()
                             + "  - id: rotors\n    capability: model.channels\n    input:\n"
                               "      system: {from: linear}\n"
                               "      inputs: [omega_command_0, omega_command_1, "
                               "omega_command_2, omega_command_3]\n"
                               "      outputs: [position_north_m, position_east_m, altitude_m]\n"
                             + "  - id: lqr\n    capability: synth.lqr\n    input: "
                               "{system: {from: rotors}, break_at: plant_input, q: "
                             + identity_weight(16, 1.0) + ", r: " + identity_weight(4, 0.02)
                             + "}\n";

  // The MIMO return ratio, handed to a SISO margin routine: refused, and the
  // diagnostic names the cause rather than advising a rescale.
  try {
    (void)run(design
                  + "  - id: brk\n    capability: model.control_system\n"
                    "    input: {law: {from: lqr}, use: broken_loop}\n"
                    "  - id: margins\n    capability: analyze.margins\n"
                    "    input: {system: {from: brk}}\n",
              {.overwrite = true, .write_manifest = false});
    FAIL() << "margins of a loop that leaves three channels open must be refused";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("internal stability unresolved"), std::string::npos) << message;
    EXPECT_NE(message.find("unstabilised mode"), std::string::npos)
        << "the diagnostic must name the cause: " << message;
  }

  // The same design, read loop at a time with the others closed: a margin.
  const RunResult result =
      run(design
              + "  - id: loop0\n    capability: model.control_system\n"
                "    input: {law: {from: lqr}, use: single_loop, channel: omega_command_0}\n"
                "  - id: margins0\n    capability: analyze.margins\n"
                "    input: {system: {from: loop0}}\n"
                "  - id: disk0\n    capability: analyze.diskmargin\n"
                "    input: {system: {from: loop0}}\n",
          {.overwrite = true, .write_manifest = false});

  const Artifact* loop = result.find("loop0");
  ASSERT_NE(loop, nullptr);
  const auto& single = loop->payload_as<galata::model::LinearSystem>("linear_system");
  EXPECT_EQ(single.input_count(), 1);
  EXPECT_EQ(single.output_count(), 1);
  EXPECT_EQ(single.state_count(), 16);

  const Artifact* margins = result.find("margins0");
  ASSERT_NE(margins, nullptr);
  EXPECT_EQ(margins->kind, "stability_margins");
  // A phase margin exists and is reported. Its VALUE is a property of the
  // weights this test chose and is deliberately not asserted: what is asserted
  // is that the analysis is now AVAILABLE where the other reading refused it.
  // The payload type is private to the capability layer, so the claim is read
  // off the summary a user sees.
  EXPECT_NE(margins->summary.find("PM "), std::string::npos) << margins->summary;
  EXPECT_EQ(margins->summary.find("unresolved"), std::string::npos) << margins->summary;

  const Artifact* disk = result.find("disk0");
  ASSERT_NE(disk, nullptr);
  EXPECT_NE(disk->summary.find("alpha"), std::string::npos) << disk->summary;
}

// A SANITY COMPARISON WITH A STATED APPROXIMATION, AND NOT A STABILITY
// CONDITION IN EITHER DIRECTION.
//
// `sim.sampled` applies a whole-period transport delay plus a zero-order hold.
// The continuous loop has a DELAY MARGIN, which `analyze.margins` reports, and
// comparing the two is worth doing — but an earlier version of this comment
// called being inside that margin a NECESSARY condition for the sampled loop,
// and that was wrong. Three reasons, all of which the comparison has to carry:
//
//   THE HOLD IS NOT A DELAY. A zero-order hold's low-frequency phase lag is
//     approximately that of a half-period delay, and only well below the sample
//     rate; it also reshapes the loop's magnitude. Adding half a period to the
//     transport delay is an APPROXIMATION of the hold, not a model of it.
//   A CONTINUOUS DELAY MARGIN BOUNDS A CONTINUOUS PERTURBATION. Applying it to
//     a sampled loop compares a figure computed for one system against a lag
//     appearing in a different one. Nothing here makes that a proof.
//   SO IT IS NEITHER NECESSARY NOR SUFFICIENT. A sampled loop can be stable
//     with an equivalent lag past the continuous margin, and unstable inside it.
//
// What the comparison IS: a warning sign in one direction. A design whose
// continuous delay margin is a small multiple of its transport delay is one to
// look at with discrete-time tools before flying, and galata has none. This
// test establishes that the comparison has teeth — a faster design on this
// plant at this rate falls the wrong side of it — so the shipped example's
// comfortable figure is a measurement rather than a number nothing could fail.
//
// EVERY FIGURE BELOW IS A PROPERTY OF THIS LQR DESIGN AND THIS LOOP
// CONSTRUCTION. It is not a property of the plant, of the sample rate, or of
// any other controller that happens to run at the same rate.
TEST_F(QuadrotorWorkflow, AFasterDesignRunsOutOfDelayMarginAtTheSameSampleRate) {
  // Unit state weights and the same control weight the shipped study uses. That
  // penalises the rotor-speed states as hard as position, which the shipped
  // study deliberately does not — it weights them at a thousandth — and the
  // result is a much faster loop.
  const RunResult result =
      run(chain()
              + "  - id: rotors\n    capability: model.channels\n    input:\n"
                "      system: {from: linear}\n"
                "      inputs: [omega_command_0, omega_command_1, omega_command_2, "
                "omega_command_3]\n"
                "      outputs: [position_north_m, position_east_m, altitude_m]\n"
              + "  - id: lqr\n    capability: synth.lqr\n    input: "
                "{system: {from: rotors}, break_at: plant_input, q: "
              + identity_weight(16, 1.0) + ", r: " + identity_weight(4, 0.02) + "}\n",
          {.overwrite = true, .write_manifest = false});
  const Artifact* law_stage = result.find("lqr");
  ASSERT_NE(law_stage, nullptr);
  const auto& law = law_stage->payload_as<galata::synth::LqrDesign>("control_law");

  // The equivalent lag the shipped sampled study applies: two controller periods
  // of transport delay at 250 Hz, plus about half a period AS AN APPROXIMATION
  // of the hold. The hold's contribution is counted rather than dropped because
  // leaving it out would flatter the comparison, and it is called an
  // approximation because that is what it is.
  const double controller_period_s = 0.004;
  const double equivalent_lag_s = 2.0 * controller_period_s + 0.5 * controller_period_s;
  RecordProperty("equivalent_lag_s", std::to_string(equivalent_lag_s));

  double smallest = std::numeric_limits<double>::infinity();
  for (int channel = 0; channel < law.plant.input_count(); ++channel) {
    const galata::model::LinearSystem loop = galata::synth::single_loop_others_closed(law, channel);
    const galata::analyze::StabilityMargins margins =
        galata::analyze::stability_margins(loop, 0, 0, {});
    ASSERT_TRUE(margins.has_delay_margin)
        << law.plant.input_names[static_cast<std::size_t>(channel)]
        << ": no delay margin was found, so this test bounds nothing";
    RecordProperty(law.plant.input_names[static_cast<std::size_t>(channel)] + "_delay_margin_s",
                   std::to_string(margins.delay_margin_s));
    smallest = std::fmin(smallest, margins.delay_margin_s);
  }
  RecordProperty("smallest_delay_margin_s", std::to_string(smallest));

  // The finding this test exists to pin: this design's worst channel tolerates
  // LESS delay than the sampled implementation applies. It is not a defect in
  // the plant or in `sim.sampled` — it is what unit state weights buy on this
  // vehicle — and it is the reason the shipped study's weights are declared in
  // its own file rather than defaulted.
  EXPECT_LT(smallest, equivalent_lag_s)
      << "a unit-weighted design on this plant is expected to fall the wrong side of this "
         "comparison at 250 Hz with two periods of delay. If it no longer does, the shipped "
         "example's comfortable figure has stopped being a measurement, because the "
         "comparison would then be one nothing can fail; find a faster design, or delete this "
         "test and say why. Note that falling the wrong side is not a proof of sampled "
         "instability, any more than falling the right side is a proof of stability";
}

// A channel the plant does not have is refused by name, with the vocabulary
// listed: picking a channel for the caller would be choosing which number to
// report, and there is one loop per input with different margins.
TEST_F(QuadrotorWorkflow, ASingleLoopNeedsAChannelAndRefusesOneTheDesignDoesNotHave) {
  const std::string design = chain()
                             + "  - id: rotors\n    capability: model.channels\n    input:\n"
                               "      system: {from: linear}\n"
                               "      inputs: [omega_command_0, omega_command_1, "
                               "omega_command_2, omega_command_3]\n"
                               "      outputs: [position_north_m, position_east_m, altitude_m]\n"
                             + "  - id: lqr\n    capability: synth.lqr\n    input: "
                               "{system: {from: rotors}, break_at: plant_input, q: "
                             + identity_weight(16, 1.0) + ", r: " + identity_weight(4, 0.02)
                             + "}\n";
  EXPECT_THROW((void)run(design
                             + "  - id: loop\n    capability: model.control_system\n"
                               "    input: {law: {from: lqr}, use: single_loop}\n",
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error);
  try {
    (void)run(design
                  + "  - id: loop\n    capability: model.control_system\n"
                    "    input: {law: {from: lqr}, use: single_loop, channel: elevator}\n",
              {.overwrite = true, .write_manifest = false});
    FAIL() << "a channel the plant does not have must be refused";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("no input 'elevator'"), std::string::npos) << message;
    EXPECT_NE(message.find("omega_command_0"), std::string::npos)
        << "the refusal must list the vocabulary: " << message;
  }
}

TEST_F(QuadrotorWorkflow, TheExistingStateSpaceFilesStillLoadUnchanged) {
  // The file contract did not move. `serialize_linear_system` was added beside
  // the reader, not in place of it, and every model file already in the tree
  // must still parse and still say what it said.
  for (const char* relative : {"nt33a-lateral-modes/nt33a-lateral.yaml",
                               "nt33a-longitudinal-modes/nt33a-longitudinal.yaml",
                               "nt33a-lateral-mimo/nt33a-lateral-mimo.yaml",
                               "nt33a-bank-loop-margins/nt33a-bank-loop.yaml"}) {
    const std::string path = (fs::path(GALATA_EXAMPLES_DIR) / relative).string();
    const galata::model::LinearSystem original = galata::model::load_linear_system(path);

    // And a system galata was GIVEN and a system galata COMPUTED are the same
    // kind of object: writing the first one out and reading it back returns it.
    const galata::model::LinearSystem round_tripped =
        galata::model::parse_linear_system(galata::model::serialize_linear_system(original), path);
    EXPECT_TRUE(round_tripped.a.isApprox(original.a, 0.0)) << relative;
    EXPECT_TRUE(round_tripped.b.isApprox(original.b, 0.0)) << relative;
    EXPECT_EQ(round_tripped.state_names, original.state_names) << relative;
    EXPECT_EQ(round_tripped.output_labels(), original.output_labels()) << relative;
    EXPECT_EQ(round_tripped.description, original.description) << relative;
    EXPECT_EQ(round_tripped.citation, original.citation) << relative;
  }
}

TEST_F(QuadrotorWorkflow, RepeatingTheRunProducesByteIdenticalMatrices) {
  // ADR-0004 on the same platform: same bits. The export is the place this is
  // visible, because it is the only artefact of the chain that reaches a file.
  const RunResult first = run(chain());
  ASSERT_NE(first.find("exported"), nullptr);
  const std::string once = read_file_bytes((output / "quad-hover.yaml").string());

  const RunResult second = run(chain(), {.overwrite = true, .write_manifest = false});
  ASSERT_NE(second.find("exported"), nullptr);
  const std::string twice = read_file_bytes((output / "quad-hover.yaml").string());

  EXPECT_EQ(once, twice);
}

}  // namespace
