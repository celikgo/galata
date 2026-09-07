// SPDX-License-Identifier: Apache-2.0
// Offline study contract: operator-owned output directory, explicit overwrite,
// strict file schemas, and an immutable record of the exact consumed bytes.
// These are filesystem/configuration invariants, not numerical reference values.
#include "galata/linearize/finite_difference.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/pipeline/charts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace {
namespace fs = std::filesystem;
using namespace galata::pipeline;

class ProductFiles : public ::testing::Test {
 protected:
  fs::path root;
  fs::path output;

  void SetUp() override {
    static std::atomic<unsigned> sequence{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-hardening-" + std::to_string(tick) + "-"
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

  RunResult run(const std::string& yaml, RunOptions options = {}) {
    const auto source = root / "study.yaml";
    put(source, yaml);
    return run_pipeline(load_pipeline(source.string()),
                        builtin_registry(),
                        root.string(),
                        output.string(),
                        nullptr,
                        options);
  }

  static std::string report(const std::string& path, const std::string& title = "Audit") {
    return "version: 1\nstages:\n  - id: report\n    capability: report.markdown\n    input:\n"
           "      sections: []\n      title: "
           + json_quote(title) + "\n      path: " + json_quote(path) + "\n";
  }
};

TEST_F(ProductFiles, RejectsRelativeAndAbsoluteEscapesWithoutTouchingExistingFiles) {
  const auto sentinel = root / "outside.md";
  put(sentinel, "keep this file");
  for (const auto& path : {std::string("../outside.md"),
                           sentinel.string(),
                           std::string("inside/../../outside.md"),
                           std::string("C:\\outside.md")}) {
    EXPECT_THROW((void)run(report(path), RunOptions{.overwrite = true}), std::runtime_error)
        << path;
    EXPECT_EQ(read_file_bytes(sentinel.string()), "keep this file");
  }
}

TEST_F(ProductFiles, RefusesSymlinkParentsAndTargetsEvenWithOverwrite) {
  fs::create_directory(output);
  const auto outside = root / "outside";
  fs::create_directory(outside);
  put(outside / "keep.md", "keep this file");
  std::error_code error;
  fs::create_directory_symlink(outside, output / "escape", error);
  if (error) {
    GTEST_SKIP() << "symlink creation unavailable: " << error.message();
  }
  EXPECT_THROW((void)run(report("escape/keep.md"), RunOptions{.overwrite = true}),
               std::runtime_error);
  fs::create_symlink(outside / "keep.md", output / "alias.md");
  EXPECT_THROW((void)run(report("alias.md"), RunOptions{.overwrite = true}), std::runtime_error);
  EXPECT_EQ(read_file_bytes((outside / "keep.md").string()), "keep this file");
}

TEST_F(ProductFiles, CreatesOutputDirectoriesAndRequiresExplicitOverwrite) {
  (void)run(report("nested/report.md", "First"));
  const auto path = output / "nested/report.md";
  const auto first = read_file_bytes(path.string());
  EXPECT_THROW((void)run(report("nested/report.md", "Second")), std::runtime_error);
  EXPECT_EQ(read_file_bytes(path.string()), first);
  (void)run(report("nested/report.md", "Second"), RunOptions{.overwrite = true});
  EXPECT_NE(read_file_bytes(path.string()), first);
  for (const auto& file : fs::recursive_directory_iterator(output)) {
    EXPECT_EQ(file.path().filename().string().find(".galata-write-"), std::string::npos);
  }
}

TEST_F(ProductFiles, NoClobberPublicationIsAtomicAcrossCompetingWriters) {
  RunFiles first(output.string());
  RunFiles second(output.string());
  first.reserve_output("result.txt");
  second.reserve_output("result.txt");
  first.write_output("result.txt", "complete first result");
  EXPECT_THROW(second.write_output("result.txt", "second result"), std::runtime_error);
  EXPECT_EQ(read_file_bytes((output / "result.txt").string()), "complete first result");
}

TEST_F(ProductFiles, RefusesTwoStagesWritingOneDestinationBeforeEitherWrites) {
  const auto yaml = report("same.md") +
      "  - id: other\n    capability: report.markdown\n"
      "    input: {sections: [], path: same.md}\n";
  EXPECT_THROW((void)run(yaml, RunOptions{.overwrite = true}), std::runtime_error);
  EXPECT_FALSE(fs::exists(output / "same.md"));
}

TEST_F(ProductFiles, OutputNamesCannotAliasOnCaseInsensitivePlatforms) {
  const auto yaml = report("Report.md") +
      "  - id: other\n    capability: report.markdown\n"
      "    input: {sections: [], path: report.md}\n";
  EXPECT_THROW((void)run(yaml, RunOptions{.overwrite = true}), std::runtime_error);
  EXPECT_FALSE(fs::exists(output / "Report.md"));
}

TEST_F(ProductFiles, DoesNotOverwriteTheStudyEvenWithExplicitOverwrite) {
  output = root;
  const auto yaml = report("study.yaml");
  EXPECT_THROW((void)run(yaml, RunOptions{.overwrite = true}), std::runtime_error);
  EXPECT_EQ(read_file_bytes((root / "study.yaml").string()), yaml);
}

TEST_F(ProductFiles, AnEarlyWriterCannotOverwriteAnInputOfALaterStage) {
  output = root;
  const std::string model = "states: [x]\na: [[-1]]\n";
  put(root / "model.yaml", model);
  const auto yaml = report("model.yaml") +
      "  - id: model\n    capability: model.linear.statespace\n"
      "    input: {path: model.yaml}\n";
  EXPECT_THROW((void)run(yaml, RunOptions{.overwrite = true}), std::runtime_error);
  EXPECT_EQ(read_file_bytes((root / "model.yaml").string()), model);
}

TEST_F(ProductFiles, InputSnapshotsRecordTheExactBytesUsedEvenIfTheFileChanges) {
  put(root / "model.yaml", "original model bytes\r\n");
  RunFiles files(output.string());
  EXPECT_EQ(files.read_input((root / "model.yaml").string()), "original model bytes\r\n");
  put(root / "model.yaml", "changed model bytes\n");
  EXPECT_EQ(files.read_input((root / "model.yaml").string()), "original model bytes\r\n");
  ASSERT_EQ(files.inputs().size(), 1U);
  EXPECT_EQ(files.inputs().begin()->second.sha256, sha256("original model bytes\r\n"));
}

TEST_F(ProductFiles, RuntimeSnapshotDetectsReplacementAndProtectsAnExecutableAlias) {
  // Exercise the runtime-file policy on an inert stand-in, never by targeting
  // the real test runner or CLI executable with a destructive test write.
  fs::create_directory(output);
  const auto binary = output / "galata-copy";
  put(binary, "inert executable stand-in");
  const auto snapshot = snapshot_file(binary.string());
  EXPECT_NO_THROW(verify_file_unchanged(snapshot));
  RunFiles files(output.string(), true);
  files.protect_input_path(binary.string());
  EXPECT_THROW(files.reserve_output("galata-copy"), std::runtime_error);
  EXPECT_THROW(files.write_output("galata-copy", "report bytes"), std::runtime_error);
  EXPECT_TRUE(files.inputs().empty());  // runtime bytes are not embedded as model input
  std::error_code error;
  fs::create_hard_link(binary, output / "binary-alias", error);
  if (!error) {
    EXPECT_THROW(files.reserve_output("binary-alias"), std::runtime_error);
    EXPECT_THROW(files.write_output("binary-alias", "report bytes"), std::runtime_error);
  }
  EXPECT_EQ(read_file_bytes(binary.string()), snapshot.bytes);
  put(binary, "replacement executable bytes");
  EXPECT_THROW(verify_file_unchanged(snapshot), std::runtime_error);
}

TEST_F(ProductFiles, UnknownCapabilityInputFailsBeforeAnEarlierWriterRuns) {
  const auto yaml = report("must-not-exist.md") +
      "  - id: trim\n    capability: trim.level\n"
      "    input: {flight_path_angel_deg: 3.0}\n";
  try {
    (void)run(yaml);
    FAIL() << "misspelled input was accepted";
  } catch (const std::exception& error) {
    EXPECT_NE(std::string(error.what()).find("flight_path_angel_deg"), std::string::npos);
  }
  EXPECT_FALSE(fs::exists(output / "must-not-exist.md"));
}

TEST(PipelineInputContract, RejectsDuplicateUnknownAndAmbiguousYaml) {
  for (const auto& yaml :
       {"version: 1\nversion: 1\nstages: []\n",
        "version: 1\nunknown: x\nstages: []\n",
        "version: 1\nstages: [{id: a, capability: report.markdown, unexpected: true}]\n",
        "version: 1\nstages: [{id: a, id: b, capability: report.markdown}]\n",
        "version: 1\nstages: [{id: a, capability: report.markdown, input: {path: a, path: b}}]\n",
        "version: 1\nstages: [{id: a, capability: x, input: &cycle {value: *cycle}}]\n",
        "version: 1\nstages: [{id: a, capability: x, input: {value: !!str 123}}]\n",
        "version: 1\nstages: [{id: a, capability: x}]\n---\nignored: true\n",
        "version: 1\nstages: [{id: a, capability: x, input: {points: .nan}}]\n"}) {
    EXPECT_THROW((void)parse_pipeline(yaml), std::runtime_error) << yaml;
  }
}

TEST(PipelineInputContract, QuotedScalarsRemainStringsAndIndicesMustBeIntegers) {
  const auto pipeline = parse_pipeline(
      "version: 1\nstages: [{id: a, capability: x, input: {word: 'true', number: '123'}}]");
  EXPECT_EQ(pipeline.stages.front().input->string_at("word"), "true");
  EXPECT_EQ(pipeline.stages.front().input->string_at("number"), "123");
  for (const double value : {2.5, 1.0e30}) {
    const auto input = Value::map({{"points", Value::number(value)}});
    EXPECT_THROW((void)input->integer_at("points", 10), std::runtime_error);
  }
}

TEST(PipelineInputContract, ProgrammaticValuesCannotBreakGraphTraversalOrJsonSerialization) {
  EXPECT_THROW((void)Value::number(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW((void)Value::number(std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW((void)Value::list({nullptr}), std::invalid_argument);
  EXPECT_THROW((void)Value::map({{"input", nullptr}}), std::invalid_argument);
  EXPECT_THROW((void)Value::stage_reference(""), std::invalid_argument);
  EXPECT_NO_THROW((void)Value::list({Value::null()}));
}

TEST(PipelineInputContract, ProgrammaticPipelinesRejectNullInputsBeforeGraphTraversal) {
  Pipeline pipeline;
  pipeline.version = 1;
  pipeline.stages.push_back(Stage{"test", "report.markdown", nullptr});
  EXPECT_THROW((void)pipeline.execution_order(), std::runtime_error);
}

TEST(ModelInputContract, RejectsUnknownAndDuplicateLinearModelKeys) {
  for (const auto& bytes :
       {"states: [x]\na: [[-1]]\nextra: 2\n", "states: [x]\na: [[-1]]\na: [[-2]]\n"}) {
    EXPECT_THROW((void)galata::model::parse_linear_system(bytes), std::invalid_argument);
  }
}

TEST(ModelInputContract, RejectsMisspelledOptionalAircraftDerivatives) {
  const auto model = fs::path(GALATA_EXAMPLES_DIR).parent_path() / "models/nt33a/nt33a-fc1.yaml";
  auto bytes = read_file_bytes(model.string());
  const auto position = bytes.find("aero:");
  ASSERT_NE(position, std::string::npos);
  bytes.insert(position + 5, "\n  rolling_moment_roll_ratte: 1.0");
  try {
    (void)galata::model::parse_aircraft(bytes);
    FAIL() << "misspelled derivative was accepted";
  } catch (const std::exception& error) {
    EXPECT_NE(std::string(error.what()).find("rolling_moment_roll_ratte"), std::string::npos);
  }
}

TEST(ContentDigest, MatchesPublishedSha256Vectors) {
  // NIST FIPS 180-4 / NIST SHA examples: empty, "abc", and the two-block vector.
  EXPECT_EQ(sha256(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(sha256("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(sha256("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(sha256(std::string(1000000, 'a')),
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST_F(ProductFiles, ManifestContainsInputSnapshotsOutputHashesAndExecutableIdentity) {
  const auto yaml = report("report.md");
  const auto result = run(yaml);
  ASSERT_FALSE(result.manifest_path.empty());
  const auto bytes = read_file_bytes(result.manifest_path);
  const auto manifest = YAML::Load(bytes);
  EXPECT_EQ(fs::path(result.manifest_path).filename().string(), "run-" + sha256(bytes) + ".json");
  EXPECT_EQ(manifest["schema"].as<std::string>(), "galata.run.v1");
  EXPECT_EQ(manifest["study"]["sha256"].as<std::string>(), sha256(yaml));
  EXPECT_EQ(manifest["study"]["bytes_hex"].as<std::string>().size(), yaml.size() * 2);
  ASSERT_EQ(manifest["outputs"].size(), 1U);
  EXPECT_EQ(manifest["outputs"][0]["sha256"].as<std::string>(),
            sha256(read_file_bytes((output / "report.md").string())));
  EXPECT_EQ(manifest["executable"]["sha256"].as<std::string>(),
            sha256(read_file_bytes(current_executable())));
  EXPECT_FALSE(manifest["build"]["source_commit"].as<std::string>().empty());
  EXPECT_EQ(manifest["build"]["source_tree_sha256"].as<std::string>().size(), 64U);
  EXPECT_EQ(manifest["build"]["configuration_sha256"].as<std::string>().size(), 64U);
  EXPECT_TRUE(manifest["build"]["configuration"].IsMap());
  ASSERT_GT(manifest["runtime"]["modules"].size(), 0U);
  EXPECT_FALSE(manifest["runtime"]["os_identity"].as<std::string>().empty());
  for (const auto& module : manifest["runtime"]["modules"]) {
    EXPECT_NE(module["path"].as<std::string>(), manifest["executable"]["path"].as<std::string>());
    if (module["storage"].as<std::string>() == "file") {
      EXPECT_EQ(module["sha256"].as<std::string>(),
                sha256(read_file_bytes(module["path"].as<std::string>())));
    } else {
      // OS-provided virtual/shared-cache images must not masquerade as hashed files.
      EXPECT_TRUE(module["sha256"].IsNull());
    }
  }
  EXPECT_FALSE(manifest["build"]["dependencies"]["Eigen"].as<std::string>().empty());
  EXPECT_FALSE(manifest["build"]["dependencies"]["yaml-cpp"].as<std::string>().empty());
}

TEST_F(ProductFiles, ExplicitRepeatReusesIdenticalManifestAndChangedInputMakesANewOne) {
  const auto yaml = report("report.md");
  const auto first = run(yaml, RunOptions{.overwrite = true});
  const auto before = read_file_bytes(first.manifest_path);
  const auto second = run(yaml, RunOptions{.overwrite = true});
  EXPECT_EQ(first.manifest_path, second.manifest_path);
  const auto changed = run("# input provenance changed\n" + yaml, RunOptions{.overwrite = true});
  EXPECT_NE(first.manifest_path, changed.manifest_path);
  EXPECT_EQ(read_file_bytes(first.manifest_path), before);
}

TEST_F(ProductFiles, ImmutableManifestCannotBeClobberedWithOverwrite) {
  RunFiles files(output.string(), true);
  files.write_output("record.json", "original", true);
  RunFiles next(output.string(), true);
  EXPECT_THROW(next.write_output("record.json", "altered", true), std::runtime_error);
  EXPECT_EQ(read_file_bytes((output / "record.json").string()), "original");
}

TEST_F(ProductFiles, HtmlReportEscapesUntrustedTextAndEmbedsTablesWithoutRemoteResources) {
  put(root / "model.yaml",
      "states: [x]\na: [[-1]]\ndescription: "
          + json_quote("<img src=\"https://untrusted.invalid/tracker\"> & model") + "\n");
  const auto title = "</title><script>alert('unsafe')</script>";
  const auto yaml =
      "version: 1\nstages:\n"
      "  - id: model\n    capability: model.linear.statespace\n"
      "    input: {path: model.yaml}\n"
      "  - id: modes\n    capability: analyze.modes\n"
      "    input: {system: {from: model}}\n"
      "  - id: response\n    capability: sim.linear\n"
      "    input: {system: {from: model}, initial_state: [1], step_s: 0.1, steps: 10}\n"
      "  - id: report\n    capability: report.html\n    input:\n"
      "      sections: [{from: model}, {from: modes}, {from: response}]\n"
      "      path: report.html\n      title: "
      + json_quote(title) + "\n";
  const auto result = run(yaml);
  const auto html = read_file_bytes((output / "report.html").string());
  EXPECT_NE(html.find("<!doctype html>"), std::string::npos);
  EXPECT_NE(html.find("<table>"), std::string::npos);
  EXPECT_NE(html.find("<pre><code>"), std::string::npos);
  EXPECT_NE(html.find("<svg"), std::string::npos);
  EXPECT_NE(html.find("<polyline"), std::string::npos);
  EXPECT_NE(html.find("Time (s)"), std::string::npos);
  EXPECT_NE(html.find("<style>"), std::string::npos);
  EXPECT_NE(html.find("&lt;script&gt;"), std::string::npos);
  EXPECT_NE(html.find("&lt;img"), std::string::npos);
  EXPECT_NE(html.find("default-src 'none'"), std::string::npos);
  EXPECT_EQ(html.find("<script"), std::string::npos);
  EXPECT_EQ(html.find("<img"), std::string::npos);
  EXPECT_EQ(html.find("<link"), std::string::npos);
  EXPECT_EQ(html.find("src=\""), std::string::npos);
  EXPECT_FALSE(result.manifest_path.empty());
}

TEST(HtmlCharts, RejectsMalformedSamplesAndEscapesEveryLabel) {
  TimeSeriesChart chart{"<script>title</script>", "<img>units", {0, 1}, {{"<svg>name", {1, 2}}}};
  const auto html = render_timeseries_svg(chart);
  EXPECT_NE(html.find("&lt;script&gt;title&lt;/script&gt;"), std::string::npos);
  EXPECT_NE(html.find("&lt;img&gt;units"), std::string::npos);
  EXPECT_NE(html.find("&lt;svg&gt;name"), std::string::npos);
  EXPECT_EQ(html.find("<script>"), std::string::npos);
  chart.series.front().values.pop_back();
  EXPECT_THROW((void)render_timeseries_svg(chart), std::invalid_argument);
  chart.series.front().values = {1, std::numeric_limits<double>::infinity()};
  EXPECT_THROW((void)render_timeseries_svg(chart), std::invalid_argument);
  chart.series.front().values = {1, 2};
  chart.times_s = {1, 1};
  EXPECT_THROW((void)render_timeseries_svg(chart), std::invalid_argument);
}

TEST_F(ProductFiles, ReportsRefuseUnsupportedArtifactsRatherThanEmittingAnEmptySection) {
  Registry registry;
  registry.add(Capability{"test.unknown",
                          "unknown artifact",
                          "unrendered",
                          Capability::State::ImplementedUnvalidated,
                          [](const StageContext&) {
                            Artifact artifact;
                            artifact.kind = "unrendered";
                            return artifact;
                          }});
  registry.add(*builtin_registry().find("report.markdown"));
  const auto pipeline = parse_pipeline(
      "version: 1\nstages:\n"
      "  - {id: source, capability: test.unknown}\n"
      "  - id: report\n    capability: report.markdown\n"
      "    input: {path: result.md, sections: [{from: source}]}\n");
  EXPECT_THROW((void)run_pipeline(pipeline, registry, root.string(), output.string()),
               std::runtime_error);
  EXPECT_FALSE(fs::exists(output / "result.md"));
}
}  // namespace

// Contract: diagnostics of a source Jacobian survive every downstream stage,
// including branches/joins, without becoming an error bound on a derived model.
TEST_F(ProductFiles, LinearizationEvidenceSurvivesAnalysisAndReport) {
  const auto base = fs::path(GALATA_EXAMPLES_DIR) / "nt33a-trim-and-linearise";
  const auto result = run_pipeline(load_pipeline((base / "study.yaml").string()),
                                   builtin_registry(),
                                   base.string(),
                                   output.string());
  const auto* source = result.find("longitudinal");
  const auto* modes = result.find("longitudinal_modes");
  const auto* report_artifact = result.find("report");
  ASSERT_NE(source, nullptr);
  ASSERT_NE(modes, nullptr);
  ASSERT_NE(report_artifact, nullptr);
  ASSERT_EQ(source->linearization_evidence.size(), 1U);
  EXPECT_EQ(modes->linearization_evidence.at("longitudinal"),
            source->linearization_evidence.at("longitudinal"));
  EXPECT_EQ(report_artifact->linearization_evidence.size(), 2U);
  const auto manifest = YAML::Load(read_file_bytes(result.manifest_path));
  const auto record = manifest["linearization_evidence"]["longitudinal"];
  EXPECT_GE(record["chart_conditioning"].as<double>(),
            galata::linearize::kMinimumChartConditioning);
  EXPECT_LE(record["equilibrium_residual"].as<double>(),
            record["equilibrium_tolerance"].as<double>());
  EXPECT_EQ(record["state_steps"].size(), 4U);
  EXPECT_EQ(record["a_truncation"].size(), 4U);
  EXPECT_EQ(manifest["stage_linearization_sources"]["report"].size(), 2U);
  const auto text = read_file_bytes((output / "trim-and-modes.md").string());
  EXPECT_NE(text.find("Upstream linearization evidence"), std::string::npos);
  EXPECT_NE(text.find("Recomputed equilibrium residual"), std::string::npos);
}

// Analytic adversary: the cubic k-scaled loop has a crossover below the default
// band. Passing a frequency search must never promote its peaks to guarantees.
TEST_F(ProductFiles, SampledSensitivityReportCannotClaimGuaranteedMargins) {
  put(root / "model.yaml",
      "states: [x, y, z]\ninputs: [u]\noutputs: [v]\n"
      "a: [[0, 0.00035714285714285714, 0], [0, 0, 0.00035714285714285714], "
      "[-0.0035714285714285713, -0.0035714285714285713, -0.0035714285714285713]]\n"
      "b: [[0], [0], [0.00035714285714285714]]\nc: [[25, 0, 0]]\nd: [[0]]\n");
  (void)run(
      "version: 1\nstages:\n"
      "  - id: model\n    capability: model.linear.statespace\n"
      "    input: {path: model.yaml}\n"
      "  - id: sensitivity\n    capability: analyze.sensitivity\n"
      "    input: {system: {from: model}}\n"
      "  - id: report\n    capability: report.markdown\n"
      "    input: {sections: [{from: sensitivity}], path: report.md}\n");
  const auto text = read_file_bytes((output / "report.md").string());
  EXPECT_EQ(text.find("gain margin at least"), std::string::npos);
  EXPECT_EQ(text.find("actual margins are at least"), std::string::npos);
  EXPECT_NE(text.find("No guaranteed margins are derived from sampled peaks"), std::string::npos);
}
