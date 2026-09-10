// SPDX-License-Identifier: Apache-2.0
//
// The identification chain end to end, against its documented contract:
// data.import.csv -> data.window -> identify.greybox -> model.quadrotor.export,
// the exported file read back by model.quadrotor, the fitted model consumed
// unchanged by trim.hover, and identify.validate reading the estimation
// record's identity out of the model's own provenance rather than off a string
// the study typed.
//
// Completing this chain is a statement about wiring. It is not a validation of
// any aircraft, and the record here is galata's own output rather than a
// measurement.
//
// Written from the capability schemas, ADR-0018 and the headers, not from the
// capability implementations (docs/TESTING.md).

#include "galata/data/record.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {
namespace fs = std::filesystem;
using namespace galata::pipeline;

class IdentifyWorkflow : public ::testing::Test {
 protected:
  fs::path root;
  fs::path output;

  // A unique scratch directory per test, for the reason RFC-0002's
  // housekeeping section records: a shared one races under parallel ctest.
  void SetUp() override {
    static std::atomic<unsigned> sequence{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-identify-workflow-" + std::to_string(tick) + "-"
              + std::to_string(sequence.fetch_add(1)));
    ASSERT_TRUE(fs::create_directory(root));
    output = root / "output";

    const fs::path example = fs::path(GALATA_EXAMPLES_DIR) / "quadrotor-identification";
    put(root / "flight.csv", read_file_bytes((example / "flight.csv").string()));
    put(root / "quad-base.yaml", read_file_bytes((example / "quad-base.yaml").string()));
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

  // The example's import block, reused so these tests are about the stages
  // after it rather than about CSV mapping.
  static std::string import_and_split() {
    std::ostringstream out;
    out << "  - id: base\n    capability: model.quadrotor\n"
           "    input: {path: quad-base.yaml}\n"
           "  - id: flight\n    capability: data.import.csv\n    input:\n"
           "      path: flight.csv\n      time_column: time_s\n      channels:\n";
    const char* names[] = {"p_n",
                           "p_e",
                           "p_d",
                           "u",
                           "v",
                           "w",
                           "q_w",
                           "q_x",
                           "q_y",
                           "q_z",
                           "p",
                           "q",
                           "r",
                           "omega_0",
                           "omega_1",
                           "omega_2",
                           "omega_3"};
    for (const char* name : names) {
      out << "        - {column: \"state:" << name << "\", name: " << name
          << ", unit: \"si\", frame: none}\n";
    }
    for (int r = 0; r < 4; ++r) {
      out << "        - {column: \"command:omega_command_" << r << "_rad_s\", name: cmd_" << r
          << ", unit: \"rad/s\", frame: none}\n";
    }
    out << "      ignore_columns: [\"wind:north_m_s\", \"wind:east_m_s\", \"wind:down_m_s\"]\n"
           "  - id: estimation\n    capability: data.window\n"
           "    input: {record: {from: flight}, start_time_s: 0.0, end_time_s: 1.2}\n"
           "  - id: heldout\n    capability: data.window\n"
           "    input: {record: {from: flight}, start_time_s: 1.2, end_time_s: 2.01}\n";
    return out.str();
  }

  static std::string seed_block(const char* indent) {
    const char* names[] = {"p_n",
                           "p_e",
                           "p_d",
                           "u",
                           "v",
                           "w",
                           "q_w",
                           "q_x",
                           "q_y",
                           "q_z",
                           "p",
                           "q",
                           "r",
                           "omega_0",
                           "omega_1",
                           "omega_2",
                           "omega_3"};
    std::ostringstream out;
    out << indent << "initial_extended_state: [0,0,-100, 0,0,0, 1,0,0,0, 0,0,0, 0,0,0,0]\n"
        << indent << "initial_state_from_record:\n";
    for (const char* name : names) {
      out << indent << "  - {state: " << name << ", channel: " << name << "}\n";
    }
    return out.str();
  }

  static std::string fit_stage(const std::string& record = "estimation",
                               const std::string& extra = {}) {
    std::ostringstream out;
    out << "  - id: fit\n    capability: identify.greybox\n    input:\n"
           "      model: {from: base}\n      record: {from: "
        << record << "}\n"
        << seed_block("      ") << "      step_s: 0.004\n      iterations: 30\n"
        << "      parameters:\n"
           "        - {path: \"mass.mass_kg\", lower: 1.0, upper: 3.0, initial: 1.45}\n"
           "        - {path: \"drag.angular_n_m_s[1]\", lower: 0.0005, upper: 0.02, "
           "initial: 0.004}\n"
           "      outputs:\n"
           "        - {channel: p_d, state: p_d, scale: 0.1}\n"
           "        - {channel: w, state: w, scale: 0.1}\n"
           "        - {channel: q, state: q, scale: 0.05}\n"
           "      command_channels: [cmd_0, cmd_1, cmd_2, cmd_3]\n"
        << extra;
    return out.str();
  }
};

}  // namespace

// --- the strict schemas ----------------------------------------------------

TEST_F(IdentifyWorkflow, EveryNewCapabilityRefusesAnUnknownInputKey) {
  // The registry validates the vocabulary before any stage runs, so a typo in a
  // study is an error rather than a silently ignored intent.
  for (const char* stage : {
           "  - id: w\n    capability: data.window\n    input: {record: {from: flight}, "
           "start_time_s: 0.0, end_time_s: 1.0, stride: 2}\n",
           "  - id: e\n    capability: model.quadrotor.export\n    input: {model: {from: base}, "
           "path: m.yaml, evidence_path: e.yaml, compress: true}\n",
       }) {
    EXPECT_THROW((void)run("version: 1\nstages:\n" + import_and_split() + stage,
                           {.overwrite = true, .write_manifest = false}),
                 std::runtime_error)
        << stage;
  }
}

// --- the fitted model is a plant -------------------------------------------

TEST_F(IdentifyWorkflow, TheFittedModelIsConsumedByTrimAndLinearisationWithNoAdapter) {
  const RunResult result =
      run("version: 1\nstages:\n" + import_and_split() + fit_stage()
              + "  - id: hover\n    capability: trim.hover\n"
                "    input: {quadrotor: {from: fit}, altitude_m: 100.0}\n"
                "  - id: linear\n    capability: linearize.extended\n"
                "    input: {trim: {from: hover}, evidence_path: op.yaml}\n"
                "  - id: fly\n    capability: sim.plant\n"
                "    input: {trim: {from: hover}, step_s: 0.004, steps: 50}\n",
          {.overwrite = true, .write_manifest = false});

  const Artifact* fit = result.find("fit");
  ASSERT_NE(fit, nullptr);
  // The same artefact kind `model.quadrotor` produces, which is what makes the
  // three stages above need no special case (ADR-0018).
  EXPECT_EQ(fit->kind, "quadrotor");
  const auto& fitted = fit->payload_as<QuadrotorArtifact>("quadrotor");
  EXPECT_TRUE(fitted.identity.is_fitted());
  EXPECT_EQ(fitted.identity.origin, "fit");
  ASSERT_NE(fitted.identity.fit, nullptr);

  // The trim carries the identity forward, so a chain that trims has not lost
  // track of what it trimmed.
  const Artifact* hover = result.find("hover");
  ASSERT_NE(hover, nullptr);
  const auto& trimmed = hover->payload_as<HoverTrimArtifact>("hover_trim");
  EXPECT_TRUE(trimmed.model_identity.is_fitted());
  EXPECT_EQ(trimmed.model_identity.fit, fitted.identity.fit)
      << "the provenance must be the same object, not a structurally equal copy";

  ASSERT_NE(result.find("linear"), nullptr);
  ASSERT_NE(result.find("fly"), nullptr);
}

TEST_F(IdentifyWorkflow, ALoadedModelCarriesItsFileDigestAndIsNotMarkedFitted) {
  const RunResult result = run("version: 1\nstages:\n" + import_and_split(),
                               {.overwrite = true, .write_manifest = false});
  const Artifact* base = result.find("base");
  ASSERT_NE(base, nullptr);
  const auto& loaded = base->payload_as<QuadrotorArtifact>("quadrotor");
  EXPECT_FALSE(loaded.identity.is_fitted());
  EXPECT_EQ(loaded.identity.origin, "file");
  EXPECT_EQ(loaded.identity.path, "quad-base.yaml");
  EXPECT_EQ(loaded.identity.sha256.size(), 64U);
  EXPECT_EQ(loaded.identity.fit, nullptr);
  // The bytes, not the path: the same digest the run manifest records.
  EXPECT_EQ(loaded.identity.sha256,
            galata::pipeline::sha256(read_file_bytes((root / "quad-base.yaml").string())));
}

// --- the durable export ----------------------------------------------------

TEST_F(IdentifyWorkflow, TheExportedModelReloadsThroughTheOrdinaryLoaderAndIsTheSameAircraft) {
  const RunResult result =
      run("version: 1\nstages:\n" + import_and_split() + fit_stage()
              + "  - id: out\n    capability: model.quadrotor.export\n    input:\n"
                "      model: {from: fit}\n      path: fitted.yaml\n"
                "      evidence_path: fitted.provenance.yaml\n",
          {.overwrite = true, .write_manifest = false});

  const auto& fitted = result.find("fit")->payload_as<QuadrotorArtifact>("quadrotor");
  const auto written = output / "fitted.yaml";
  ASSERT_TRUE(fs::exists(written));
  const galata::model::Quadrotor reloaded = galata::model::load_quadrotor(written.string());

  EXPECT_EQ(reloaded.mass.mass_kg, fitted.model.mass.mass_kg);
  EXPECT_EQ(reloaded.angular_drag_n_m_s, fitted.model.angular_drag_n_m_s);
  EXPECT_EQ(reloaded.rotor_count(), fitted.model.rotor_count());
  // The file says what it is, because the format cannot: a reader who has only
  // the model file must still be able to tell a fit from a measurement.
  EXPECT_NE(reloaded.description.find("identified"), std::string::npos) << reloaded.description;
  EXPECT_NE(reloaded.citation.find("identify.greybox"), std::string::npos) << reloaded.citation;
  EXPECT_NE(reloaded.citation.find("not measured aircraft data"), std::string::npos)
      << reloaded.citation;
}

TEST_F(IdentifyWorkflow, TheEvidenceFileIsRequiredAndMayNotBeTheModelFile) {
  const std::string chain = "version: 1\nstages:\n" + import_and_split() + fit_stage();
  // Absent: ADR-0018 makes it mandatory, for a fitted model and a loaded one
  // alike, so there is no branch for a caller to take.
  EXPECT_THROW((void)run(chain
                             + "  - id: out\n    capability: model.quadrotor.export\n"
                               "    input: {model: {from: fit}, path: fitted.yaml}\n",
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error);
  // The same file for both: one document would overwrite the other.
  EXPECT_THROW((void)run(chain
                             + "  - id: out\n    capability: model.quadrotor.export\n"
                               "    input: {model: {from: fit}, path: both.yaml, "
                               "evidence_path: both.yaml}\n",
                         {.overwrite = true, .write_manifest = false}),
               std::runtime_error);
}

TEST_F(IdentifyWorkflow, ExportingALoadedModelRecordsThatNothingWasFitted) {
  const RunResult result =
      run("version: 1\nstages:\n" + import_and_split()
              + "  - id: out\n    capability: model.quadrotor.export\n    input:\n"
                "      model: {from: base}\n      path: copy.yaml\n"
                "      evidence_path: copy.provenance.yaml\n",
          {.overwrite = true, .write_manifest = false});
  ASSERT_NE(result.find("out"), nullptr);
  const std::string evidence = read_file_bytes((output / "copy.provenance.yaml").string());
  EXPECT_NE(evidence.find("origin: \"file\""), std::string::npos) << evidence;
  EXPECT_NE(evidence.find("fitted_parameter_count: 0"), std::string::npos) << evidence;
}

// THE GUARANTEE ADR-0018 RESTS ON, and it is the executor's rather than the
// exporter's: the base model file is a recorded run input, and no output may
// overwrite one. That is why "the source model is not modified" is a property
// of the run and not of one capability's good behaviour.
TEST_F(IdentifyWorkflow, AStudyThatWouldOverwriteItsSourceModelIsRefusedBeforeAnythingRuns) {
  const std::string before = read_file_bytes((root / "quad-base.yaml").string());
  EXPECT_THROW((void)run("version: 1\nstages:\n"
                         "  - id: base\n    capability: model.quadrotor\n"
                         "    input: {path: quad-base.yaml}\n"
                         "  - id: out\n    capability: model.quadrotor.export\n    input:\n"
                         "      model: {from: base}\n      path: quad-base.yaml\n"
                         "      evidence_path: quad-base.provenance.yaml\n",
                         {.overwrite = true, .write_manifest = false},
                         root.string()),
               std::runtime_error);
  EXPECT_EQ(read_file_bytes((root / "quad-base.yaml").string()), before)
      << "the source model must be byte-identical after a refused study";
}

// --- honest exposure -------------------------------------------------------

// The record's commands produce no net yaw torque — the diagonal rotor pairs
// spin opposite ways, so their reaction torques cancel — and the yaw rate is
// therefore identically zero throughout. Yaw angular drag acts on that rate, so
// its column of the sensitivity matrix is exactly zero: this record measures
// nothing about it, however small the residual becomes. The refusal must say so
// rather than report where the optimiser happened to stop.
TEST_F(IdentifyWorkflow, AnUnidentifiableParameterPairIsRefusedWithTheReasonStated) {
  const std::string chain =
      "version: 1\nstages:\n" + import_and_split()
      + "  - id: fit\n    capability: identify.greybox\n    input:\n"
        "      model: {from: base}\n      record: {from: estimation}\n"
      + seed_block("      ")
      + "      step_s: 0.004\n      iterations: 10\n"
        "      parameters:\n"
        "        - {path: \"mass.mass_kg\", lower: 1.0, upper: 3.0, initial: 1.45}\n"
        "        - {path: \"drag.angular_n_m_s[2]\", lower: 0.0005, "
        "upper: 0.02, initial: 0.003}\n"
        "      outputs:\n"
        "        - {channel: p_d, state: p_d, scale: 0.1}\n"
        "      command_channels: [cmd_0, cmd_1, cmd_2, cmd_3]\n";
  try {
    (void)run(chain, {.overwrite = true, .write_manifest = false});
    FAIL() << "an unidentifiable parameter direction must be refused";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("does not constrain every declared parameter"), std::string::npos)
        << message;
    EXPECT_NE(message.find("condition number"), std::string::npos) << message;
    // The distinction ADR-0008 requires: finishing is not measuring.
    EXPECT_NE(message.find("The optimiser finished"), std::string::npos) << message;
  }
}

TEST_F(IdentifyWorkflow, AParameterPathTheModelDoesNotHaveIsRefusedRatherThanIgnored) {
  const std::string chain =
      "version: 1\nstages:\n" + import_and_split()
      + "  - id: fit\n    capability: identify.greybox\n    input:\n"
        "      model: {from: base}\n      record: {from: estimation}\n"
      + seed_block("      ")
      + "      step_s: 0.004\n      iterations: 5\n"
        "      parameters:\n"
        "        - {path: \"rotors[9].thrust_coefficient_n_s2\", lower: 1.0, upper: 2.0, "
        "initial: 1.5}\n"
        "      outputs:\n"
        "        - {channel: p_d, state: p_d, scale: 0.1}\n"
        "      command_channels: [cmd_0, cmd_1, cmd_2, cmd_3]\n";
  try {
    (void)run(chain, {.overwrite = true, .write_manifest = false});
    FAIL() << "a parameter path the model does not have must be refused";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("silently not fitted"), std::string::npos)
        << error.what();
  }
}

TEST_F(IdentifyWorkflow, GivingBothATrimAndADeclaredStateIsRefusedRatherThanResolved) {
  const std::string chain =
      "version: 1\nstages:\n" + import_and_split()
      + "  - id: hover\n    capability: trim.hover\n"
        "    input: {quadrotor: {from: base}, altitude_m: 100.0}\n"
        "  - id: fit\n    capability: identify.greybox\n    input:\n"
        "      model: {from: base}\n      record: {from: estimation}\n"
        "      trim: {from: hover}\n"
        "      initial_extended_state: [0,0,-100, 0,0,0, 1,0,0,0, 0,0,0, 0,0,0,0]\n"
        "      step_s: 0.004\n      iterations: 5\n"
        "      parameters:\n"
        "        - {path: \"mass.mass_kg\", lower: 1.0, upper: 3.0, initial: 1.45}\n"
        "      outputs:\n"
        "        - {channel: p_d, state: p_d, scale: 0.1}\n"
        "      command_channels: [cmd_0, cmd_1, cmd_2, cmd_3]\n";
  EXPECT_THROW((void)run(chain, {.overwrite = true, .write_manifest = false}), std::runtime_error);
}

// --- validation reads the model's provenance -------------------------------

TEST_F(IdentifyWorkflow, ValidationTakesTheEstimationIdentityFromTheModelRatherThanTheStudy) {
  const std::string chain =
      "version: 1\nstages:\n" + import_and_split() + fit_stage()
      + "  - id: check\n    capability: identify.validate\n    input:\n"
        "      model: {from: fit}\n      record: {from: heldout}\n"
        "      estimation_record: {from: estimation}\n"
      + seed_block("      ")
      + "      step_s: 0.004\n"
        "      outputs:\n        - {channel: p_d, state: p_d}\n"
        "      command_channels: [cmd_0, cmd_1, cmd_2, cmd_3]\n";
  const RunResult result = run(chain, {.overwrite = true, .write_manifest = false});
  const Artifact* check = result.find("check");
  ASSERT_NE(check, nullptr);
  const auto& validation = check->payload_as<ValidationArtifact>("validation");
  const auto& fitted = result.find("fit")->payload_as<QuadrotorArtifact>("quadrotor");

  // The study states no digest anywhere. The identity came from the model.
  EXPECT_EQ(validation.result.estimation_record_sha256,
            fitted.identity.fit->estimation_record_sha256);
  EXPECT_EQ(validation.result.separation, galata::identify::RecordSeparation::VerifiedDisjoint);
  EXPECT_TRUE(validation.model_identity.is_fitted());
}

TEST_F(IdentifyWorkflow, ADeclaredDigestThatDisagreesWithTheModelsOwnProvenanceIsRefused) {
  const std::string chain =
      "version: 1\nstages:\n" + import_and_split() + fit_stage()
      + "  - id: check\n    capability: identify.validate\n    input:\n"
        "      model: {from: fit}\n      record: {from: heldout}\n"
        "      estimation_record_sha256: "
        "\"0000000000000000000000000000000000000000000000000000000000000000\"\n"
      + seed_block("      ")
      + "      step_s: 0.004\n"
        "      outputs:\n        - {channel: p_d, state: p_d}\n"
        "      command_channels: [cmd_0, cmd_1, cmd_2, cmd_3]\n";
  try {
    (void)run(chain, {.overwrite = true, .write_manifest = false});
    FAIL() << "a study that contradicts the model's own fit provenance must be refused";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("the model's own fit provenance"), std::string::npos) << message;
  }
}

TEST_F(IdentifyWorkflow, WiringTheWrongRecordInAsTheTrainingDataIsRefused) {
  const std::string chain =
      "version: 1\nstages:\n" + import_and_split() + fit_stage()
      + "  - id: check\n    capability: identify.validate\n    input:\n"
        "      model: {from: fit}\n      record: {from: heldout}\n"
        // The model was fitted to `estimation`; this names the whole flight.
        "      estimation_record: {from: flight}\n"
      + seed_block("      ")
      + "      step_s: 0.004\n"
        "      outputs:\n        - {channel: p_d, state: p_d}\n"
        "      command_channels: [cmd_0, cmd_1, cmd_2, cmd_3]\n";
  try {
    (void)run(chain, {.overwrite = true, .write_manifest = false});
    FAIL() << "supplying a record the model was not fitted to must be refused";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("is not the record this model was fitted to"), std::string::npos)
        << message;
    // The digests are EQUAL here — a window keeps the digest of the file it was
    // cut from — so the refusal has to rest on the interval and the sample
    // count, and the message has to show both sides.
    EXPECT_NE(message.find("a window [0, 1.2) s of"), std::string::npos) << message;
    EXPECT_NE(message.find("the whole of"), std::string::npos) << message;
  }
}
