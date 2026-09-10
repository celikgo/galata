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

#include "galata/core/state.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/modeling/linear_adapter.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
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
