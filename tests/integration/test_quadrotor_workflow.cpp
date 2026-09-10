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
#include "galata/model/linear_system.hpp"
#include "galata/modeling/linear_adapter.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"
#include "galata/synth/control.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

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
// public capabilities; nothing reaches into C++ to build a controller.
std::string sampled_chain(const std::string& trim_input,
                          const std::string& sampled_extra,
                          int steps = 400) {
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
      << "  - id: lqr\n    capability: synth.lqr\n    input: {system: {from: rotors}, "
         "break_at: plant_input, q: "
      << q.str() << ", r: " << r.str() << "}\n"
      << "  - id: closed\n    capability: sim.sampled\n    input:\n"
         "      trim: {from: hover}\n      law: {from: lqr}\n"
         "      step_s: 0.002\n      steps: "
      << steps << "\n"
      << sampled_extra;
  return out.str();
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

  // The heading is unobservable from this observation model: it has body rates,
  // position, altitude, ground velocity and specific force, and none of them
  // measures an absolute yaw angle. That is a physical fact about the sensor
  // set, and it is the kind of answer a failed synthesis would have delivered
  // as a Riccati diagnostic three stages later.
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
