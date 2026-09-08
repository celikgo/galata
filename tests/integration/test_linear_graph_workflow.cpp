// SPDX-License-Identifier: Apache-2.0
// ADR-0013 capability contract, authored without reading the implementation.
// The NT-33A comparison is shared-engine integration consistency, not aircraft
// validation. The rounding allowance is predeclared in ADR-0013; its reference
// amplification is the independent classical RK4 stability polynomial.
#include "galata/linearize/finite_difference.hpp"
#include "galata/modeling/linear_adapter.hpp"
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
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using namespace galata::pipeline;
using galata::modeling::CompiledModel;
using galata::modeling::SimulationResult;

class LinearGraphWorkflow : public ::testing::Test {
 protected:
  fs::path root;
  unsigned sequence = 0;

  void SetUp() override {
    static std::atomic<unsigned> counter{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-linear-graph-" + std::to_string(tick) + "-"
              + std::to_string(counter.fetch_add(1)));
    ASSERT_TRUE(fs::create_directory(root));
  }

  void TearDown() override {
    std::error_code error;
    fs::remove_all(root, error);
  }

  fs::path next_output() {
    return root / ("run-" + std::to_string(sequence++));
  }

  void put(const std::string& name, const std::string& bytes) {
    std::ofstream file(root / name, std::ios::binary | std::ios::trunc);
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    ASSERT_TRUE(file.good());
  }
};

std::size_t column(const std::vector<std::string>& names, const std::string& name) {
  const auto found = std::find(names.begin(), names.end(), name);
  if (found == names.end()) {
    throw std::runtime_error("missing expected channel: " + name);
  }
  return static_cast<std::size_t>(std::distance(names.begin(), found));
}

Eigen::Index slot(const std::vector<std::string>& names, const std::string& name) {
  return static_cast<Eigen::Index>(column(names, name));
}

std::vector<std::string> split(const std::string& row) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  bool closed = false;
  for (std::size_t index = 0; index < row.size(); ++index) {
    const char character = row[index];
    if (quoted) {
      if (character != '"') {
        field += character;
      } else if (index + 1 < row.size() && row[index + 1] == '"') {
        field += '"';
        ++index;
      } else {
        quoted = false;
        closed = true;
      }
    } else if (character == ',') {
      fields.push_back(field);
      field.clear();
      closed = false;
    } else if (character == '"' && field.empty() && !closed) {
      quoted = true;
    } else if (closed || character == '"') {
      throw std::runtime_error("invalid quoted trajectory CSV field");
    } else {
      field += character;
    }
  }
  if (quoted)
    throw std::runtime_error("unterminated trajectory CSV field");
  fields.push_back(field);
  return fields;
}

struct Csv {
  std::vector<std::string> header;
  std::vector<std::vector<double>> rows;
};

Csv read_csv(const fs::path& path) {
  std::istringstream input(read_file_bytes(path.string()));
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("missing trajectory CSV header");
  }
  Csv csv;
  csv.header = split(line);
  while (std::getline(input, line)) {
    const auto fields = split(line);
    if (fields.size() != csv.header.size()) {
      throw std::runtime_error("trajectory CSV row width mismatch");
    }
    std::vector<double> row;
    for (const auto& field : fields) {
      std::istringstream number(field);
      number.imbue(std::locale::classic());
      double value = 0.0;
      if (!(number >> value) || !std::isfinite(value)) {
        throw std::runtime_error("invalid trajectory CSV number");
      }
      number >> std::ws;
      if (!number.eof()) {
        throw std::runtime_error("trailing trajectory CSV value");
      }
      row.push_back(value);
    }
    csv.rows.push_back(std::move(row));
  }
  return csv;
}

std::string hex_bytes(const std::string& bytes) {
  constexpr char digits[] = "0123456789abcdef";
  std::string output;
  output.reserve(bytes.size() * 2);
  for (const char character : bytes) {
    const auto byte = static_cast<unsigned char>(character);
    output.push_back(digits[byte >> 4U]);
    output.push_back(digits[byte & 15U]);
  }
  return output;
}

void expect_matrix(const YAML::Node& node, const Eigen::MatrixXd& expected) {
  ASSERT_TRUE(node.IsSequence());
  ASSERT_EQ(node.size(), static_cast<std::size_t>(expected.rows()));
  for (Eigen::Index row = 0; row < expected.rows(); ++row) {
    ASSERT_EQ(node[static_cast<std::size_t>(row)].size(),
              static_cast<std::size_t>(expected.cols()));
    for (Eigen::Index col = 0; col < expected.cols(); ++col) {
      EXPECT_EQ(node[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)].as<double>(),
                expected(row, col));
    }
  }
}

double consistency_budget(const CompiledModel& graph, const galata::synth::LqrDesign& law) {
  // ADR-0013/B01: bounds depend only on declared equations, solver options and
  // operation counts, before reading either trajectory. All state components
  // here are normalized by one unit of their explicitly declared SI quantity.
  using Matrix = Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>;
  constexpr int steps = 4000;
  constexpr long double h = 0.005L;
  const Matrix z = h * law.closed_loop.a.cast<long double>();
  const Matrix identity = Matrix::Identity(z.rows(), z.cols());
  const Matrix polynomial =
      identity + z + (z * z) / 2.0L + (z * z * z) / 6.0L + (z * z * z * z) / 24.0L;
  Matrix power = identity;
  long double amplification = 1.0L;
  for (int step = 0; step < steps; ++step) {
    power = (power * polynomial).eval();
    amplification = std::max(amplification, power.cwiseAbs().rowwise().sum().maxCoeff());
  }
  const double derivative_scale =
      (law.plant.a.cwiseAbs() + law.plant.b.cwiseAbs() * law.riccati.k.cwiseAbs())
          .rowwise()
          .sum()
          .maxCoeff();
  std::size_t terms = 0;
  for (const auto& block : graph.source_model().blocks) {
    if (const auto* row = std::get_if<galata::modeling::LinearCombination>(&block.parameters)) {
      terms += row->terms.size();
    }
  }
  const double n = static_cast<double>(law.plant.state_count());
  const double m = static_cast<double>(law.plant.input_count());
  const double q =
      64.0
      * (static_cast<double>(graph.source_model().blocks.size() + terms) + n * (n + m) + n + 1.0);
  const double accumulation =
      q * static_cast<double>(steps + 1) * std::numeric_limits<double>::epsilon();
  if (!(accumulation < 0.01) || !std::isfinite(amplification)) {
    throw std::runtime_error("consistency budget outside its declared small-error domain");
  }
  const double gamma = accumulation / (1.0 - accumulation);
  const double initial_scale = std::max(1.0, graph.initial_state().cwiseAbs().maxCoeff());
  const double stage_scale = initial_scale * std::pow(1.0 + 0.005 * derivative_scale, 4);
  const double growth = static_cast<double>(amplification);
  const double budget = 8.0 * gamma * growth * growth * stage_scale;
  if (!std::isfinite(budget) || !(budget > 0.0)) {
    throw std::runtime_error("nonfinite or nonpositive integration-consistency budget");
  }
  return budget;
}

TEST_F(LinearGraphWorkflow, Nt33aGraphMatchesEveryReferenceSampleAndRetainsSourceEvidence) {
  const auto base = fs::path(GALATA_EXAMPLES_DIR) / "nt33a-graph-design";
  const auto output = next_output();
  const auto result = run_pipeline(load_pipeline((base / "study.yaml").string()),
                                   builtin_registry(),
                                   base.string(),
                                   output.string());
  const auto* graph_artifact = result.find("graph");
  const auto* response_artifact = result.find("graph_response");
  const auto* law_artifact = result.find("law");
  const auto* source_artifact = result.find("full_plant");
  ASSERT_NE(graph_artifact, nullptr);
  ASSERT_NE(response_artifact, nullptr);
  ASSERT_NE(law_artifact, nullptr);
  ASSERT_NE(source_artifact, nullptr);
  const auto& graph = graph_artifact->payload_as<CompiledModel>("executable_model");
  const auto& response = response_artifact->payload_as<SimulationResult>("model_trajectory");
  const auto& law = law_artifact->payload_as<galata::synth::LqrDesign>("control_law");
  EXPECT_EQ(graph_artifact->produced_by_capability, "model.linear_graph");
  EXPECT_FALSE(graph_artifact->produced_by_build.empty());
  ASSERT_EQ(law.plant.state_names, (std::vector<std::string>{"u", "w", "q", "theta"}));
  ASSERT_EQ(law.plant.input_names, (std::vector<std::string>{"elevator"}));
  EXPECT_EQ(response.options.step_s, 0.005);
  EXPECT_EQ(response.options.step_count, 4000);
  EXPECT_EQ(response.options.sample_stride, 10);
  ASSERT_EQ(response.times_s.size(), 401U);
  ASSERT_EQ(response.states.size(), 401U);
  ASSERT_EQ(response.outputs.size(), 401U);

  const auto adapter = YAML::Load(read_file_bytes((output / "adapter.json").string()));
  EXPECT_EQ(adapter["schema"].as<std::string>(), "galata.linear-adapter.v1");
  EXPECT_EQ(adapter["lowering"].as<std::string>(), "linear-rows.v1");
  EXPECT_EQ(adapter["model_semantic_sha256"].as<std::string>(), graph.semantic_sha256());
  const auto state_ids = adapter["mapping"]["state_ids"].as<std::vector<std::string>>();
  const auto output_ids = adapter["mapping"]["output_ids"].as<std::vector<std::string>>();
  const auto control_ids = adapter["mapping"]["control_output_ids"].as<std::vector<std::string>>();
  ASSERT_EQ(state_ids.size(), 4U);
  ASSERT_EQ(output_ids.size(), 4U);
  ASSERT_EQ(control_ids.size(), 1U);
  expect_matrix(adapter["source_system"]["a"], law.plant.a);
  expect_matrix(adapter["source_system"]["b"], law.plant.b);
  expect_matrix(adapter["source_system"]["c"], law.plant.output_matrix());
  expect_matrix(adapter["source_system"]["d"], law.plant.feedthrough_matrix());
  expect_matrix(adapter["feedback_gain"], law.riccati.k);
  expect_matrix(adapter["lqr_origin"]["q"], law.q);
  expect_matrix(adapter["lqr_origin"]["r"], law.r);
  EXPECT_EQ(adapter["source_system"]["state_names"].as<std::vector<std::string>>(),
            law.plant.state_names);
  ASSERT_EQ(adapter["channel_types"]["states"].size(), 4U);
  EXPECT_EQ(adapter["channel_types"]["states"][0]["frame"].as<std::string>(), "body");
  EXPECT_EQ(adapter["channel_types"]["states"][0]["dimension"].as<std::vector<int>>(),
            (std::vector<int>{1, 0, -1, 0, 0, 0, 0, 0}));
  EXPECT_EQ(adapter["channel_types"]["states"][2]["dimension"].as<std::vector<int>>(),
            (std::vector<int>{0, 0, -1, 0, 0, 0, 0, 1}));
  EXPECT_EQ(adapter["channel_types"]["states"][3]["dimension"].as<std::vector<int>>(),
            (std::vector<int>{0, 0, 0, 0, 0, 0, 0, 1}));
  const auto saved_graph = galata::modeling::compile_model(
      galata::modeling::parse_model_yaml(read_file_bytes((output / "model.yaml").string())));
  EXPECT_EQ(saved_graph.semantic_sha256(), graph.semantic_sha256());

  // Compute the complete predeclared budget before reading reference values.
  const double state_budget = consistency_budget(graph, law);
  RecordProperty("state_consistency_budget", ::testing::PrintToString(state_budget));
  const auto csv = read_csv(output / "linear-response.csv");
  ASSERT_EQ(csv.rows.size(), 401U);
  const auto time_column = column(csv.header, "time_s");
  double worst_state_error = 0.0;
  const auto c = law.plant.output_matrix();
  const auto d = law.plant.feedthrough_matrix();
  for (std::size_t sample = 0; sample < csv.rows.size(); ++sample) {
    SCOPED_TRACE(sample);
    const double time = static_cast<double>(sample * 10) * 0.005;
    EXPECT_EQ(response.times_s[sample], time);
    EXPECT_EQ(csv.rows[sample][time_column], time);
    Eigen::VectorXd reference(4);
    for (std::size_t state = 0; state < state_ids.size(); ++state) {
      reference(static_cast<Eigen::Index>(state)) =
          csv.rows[sample][column(csv.header, "state:" + law.plant.state_names[state])];
      const double actual = response.states[sample](slot(response.state_ids, state_ids[state]));
      worst_state_error = std::max(worst_state_error,
                                   std::abs(actual - reference(static_cast<Eigen::Index>(state))));
      EXPECT_NEAR(actual, reference(static_cast<Eigen::Index>(state)), state_budget);
    }
    const Eigen::VectorXd controls = -law.riccati.k * reference;
    const Eigen::VectorXd plant_outputs = c * reference + d * controls;
    for (std::size_t output_index = 0; output_index < output_ids.size(); ++output_index) {
      const auto row = static_cast<Eigen::Index>(output_index);
      const double multiplier = 1.0 + c.row(row).cwiseAbs().sum()
                                + (d.cwiseAbs() * law.riccati.k.cwiseAbs()).row(row).sum();
      EXPECT_NEAR(response.outputs[sample](slot(response.output_ids, output_ids[output_index])),
                  plant_outputs(row),
                  state_budget * multiplier);
    }
    for (std::size_t input = 0; input < control_ids.size(); ++input) {
      const auto row = static_cast<Eigen::Index>(input);
      EXPECT_NEAR(response.outputs[sample](slot(response.output_ids, control_ids[input])),
                  controls(row),
                  state_budget * (1.0 + law.riccati.k.row(row).cwiseAbs().sum()));
    }
  }
  RecordProperty("maximum_state_difference", ::testing::PrintToString(worst_state_error));

  ASSERT_EQ(source_artifact->linearization_evidence.size(), 1U);
  const auto source = source_artifact->linearization_evidence.at("full_plant");
  ASSERT_NE(source, nullptr);
  EXPECT_EQ(graph_artifact->linearization_evidence.at("full_plant"), source);
  EXPECT_EQ(response_artifact->linearization_evidence.at("full_plant"), source);
  const auto manifest = YAML::Load(read_file_bytes(result.manifest_path));
  const auto evidence = manifest["linearization_evidence"]["full_plant"];
  expect_matrix(evidence["a"], source->a);
  expect_matrix(evidence["b"], source->b);
  expect_matrix(evidence["a_truncation"], source->a_truncation);
  expect_matrix(evidence["b_truncation"], source->b_truncation);
  EXPECT_EQ(evidence["chart_conditioning"].as<double>(), source->chart_conditioning);
  EXPECT_EQ(evidence["equilibrium_residual"].as<double>(), source->trim_residual_norm);
  EXPECT_EQ(evidence["equilibrium_tolerance"].as<double>(), source->trim_residual_tolerance);
  EXPECT_EQ(manifest["stage_linearization_sources"]["graph"].as<std::vector<std::string>>(),
            (std::vector<std::string>{"full_plant"}));
  EXPECT_EQ(
      manifest["stage_linearization_sources"]["graph_response"].as<std::vector<std::string>>(),
      (std::vector<std::string>{"full_plant"}));
  EXPECT_EQ(manifest["build"]["source_tree_sha256"].as<std::string>().size(), 64U);
  EXPECT_EQ(manifest["build"]["configuration_sha256"].as<std::string>().size(), 64U);
  EXPECT_EQ(manifest["executable"]["sha256"].as<std::string>().size(), 64U);
  EXPECT_FALSE(manifest["runtime"]["os_identity"].as<std::string>().empty());
  EXPECT_EQ(manifest["study"]["sha256"].as<std::string>(),
            sha256(read_file_bytes((base / "study.yaml").string())));
  bool found_aircraft = false;
  for (const auto& input : manifest["inputs"]) {
    if (fs::path(input["path"].as<std::string>()).filename() == "nt33a-fc1.yaml") {
      found_aircraft = true;
      const auto bytes = read_file_bytes(input["path"].as<std::string>());
      EXPECT_EQ(input["sha256"].as<std::string>(), sha256(bytes));
      EXPECT_EQ(input["bytes_hex"].as<std::string>(), hex_bytes(bytes));
    }
  }
  EXPECT_TRUE(found_aircraft);
  std::map<std::string, std::string> retained_outputs;
  for (const auto& output_record : manifest["outputs"]) {
    retained_outputs.emplace(fs::path(output_record["path"].as<std::string>()).filename().string(),
                             output_record["sha256"].as<std::string>());
  }
  for (const auto* name :
       {"model.yaml", "adapter.json", "graph-response.csv", "graph-evidence.json"}) {
    ASSERT_TRUE(retained_outputs.contains(name));
    EXPECT_EQ(retained_outputs.at(name), sha256(read_file_bytes((output / name).string())));
  }
  const auto run_evidence = YAML::Load(read_file_bytes((output / "graph-evidence.json").string()));
  EXPECT_EQ(run_evidence["execution"].as<std::string>(), "completed");
  for (const auto* name : {"numerical_accuracy", "model_validity", "engineering_acceptance"}) {
    EXPECT_EQ(run_evidence[name].as<std::string>(), "not_assessed");
  }
}

Registry synthetic_registry() {
  auto registry = builtin_registry();
  Capability source;
  source.id = "fixture.linear_graph_source";
  source.produces = "linear_system";
  source.run = [](const StageContext&) {
    // Independent scalar fixture: xdot=-x+u, y=2x+u/2.
    galata::model::LinearSystem system;
    system.a = Eigen::MatrixXd::Constant(1, 1, -1.0);
    system.b = Eigen::MatrixXd::Ones(1, 1);
    system.c = Eigen::MatrixXd::Constant(1, 1, 2.0);
    system.d = Eigen::MatrixXd::Constant(1, 1, 0.5);
    system.state_names = {"distance"};
    system.input_names = {"speed"};
    system.output_names = {"measured"};
    Artifact artifact;
    artifact.kind = "linear_system";
    artifact.payload = std::move(system);
    return artifact;
  };
  registry.add(std::move(source));
  return registry;
}

std::string synthetic_study() {
  return R"(version: 1
stages:
  - id: source
    capability: fixture.linear_graph_source
  - id: graph
    capability: model.linear_graph
    input:
      system: {from: source}
      channel_types:
        states: [{dimension: [1,0,0,0,0,0,0,0], frame: body}]
        inputs: [{dimension: [1,0,-1,0,0,0,0,0], frame: none}]
        outputs: [{dimension: [1,0,0,0,0,0,0,0], frame: ned}]
      initial_state: [2]
      command: [0.5]
      model_path: model.yaml
      adapter_path: adapter.json
  - id: response
    capability: sim.model
    input: {model: {from: graph}, step_s: 0.125, steps: 0, csv_path: response.csv, evidence_path: evidence.json}
)";
}

TEST_F(LinearGraphWorkflow, SystemRouteKeepsDyadicFeedthroughAndExplicitCoordinateMetadata) {
  const auto output = next_output();
  const auto result = run_pipeline(
      parse_pipeline(synthetic_study()), synthetic_registry(), root.string(), output.string());
  ASSERT_NE(result.find("graph"), nullptr);
  const auto& graph = result.find("graph")->payload_as<CompiledModel>("executable_model");
  const auto direct = graph.evaluate(0.0, graph.initial_state());
  EXPECT_EQ(direct.derivatives(0), -1.5);
  EXPECT_EQ(direct.outputs(slot(graph.output_ids(), "output_000")), 4.25);
  EXPECT_EQ(direct.outputs(slot(graph.output_ids(), "control_output_000")), 0.5);
  const auto adapter = YAML::Load(read_file_bytes((output / "adapter.json").string()));
  EXPECT_EQ(adapter["channel_types"]["states"][0]["frame"].as<std::string>(), "body");
  EXPECT_EQ(adapter["channel_types"]["outputs"][0]["frame"].as<std::string>(), "ned");
}

TEST_F(LinearGraphWorkflow, AdapterRefusesAmbiguousSourcesAndMissingOrMisshapedTypes) {
  const std::vector<std::pair<std::string, std::string>> changes{
      {"      system: {from: source}\n",
       "      system: {from: source}\n      law: {from: source}\n"},
      {"      system: {from: source}\n", ""},
      {"states: [{dimension: [1,0,0,0,0,0,0,0], frame: body}]", "states: []"},
      {"      command: [0.5]", "      command: [0.5, 1]"},
      {"      model_path: model.yaml", "      model_path: adapter.json"}};
  for (const auto& [from, to] : changes) {
    auto document = synthetic_study();
    const auto position = document.find(from);
    ASSERT_NE(position, std::string::npos);
    document.replace(position, from.size(), to);
    const auto output = next_output();
    EXPECT_THROW(
        (void)run_pipeline(
            parse_pipeline(document), synthetic_registry(), root.string(), output.string()),
        std::runtime_error);
    EXPECT_FALSE(fs::exists(output / "model.yaml"));
    EXPECT_FALSE(fs::exists(output / "adapter.json"));
    EXPECT_FALSE(fs::exists(output / "response.csv"));
  }
}

std::string context_study(bool include_context) {
  // An independent report precedes compilation, so the oversized case checks
  // global bounded-input preflight rather than just the compiler's read limit.
  return "version: 1\nstages:\n"
         "  - id: early_report\n    capability: report.markdown\n"
         "    input: {title: Context preflight sentinel, sections: [], path: early.md}\n"
         "  - id: model\n    capability: model.compile\n"
         "    input: {path: model.yaml"
         + std::string(include_context ? ", context_path: origin.yaml" : "") + "}\n";
}

TEST_F(LinearGraphWorkflow, OptionalCompileContextRetainsExactBytesWithoutChangingModelIdentity) {
  const auto source =
      read_file_bytes((fs::path(GALATA_EXAMPLES_DIR) / "continuous-feedback/model.yaml").string());
  put("model.yaml", source);
  const auto ordinary_output = next_output();
  const auto ordinary = run_pipeline(parse_pipeline(context_study(false)),
                                     builtin_registry(),
                                     root.string(),
                                     ordinary_output.string());
  ASSERT_NE(ordinary.find("model"), nullptr);
  EXPECT_TRUE(fs::exists(ordinary_output / "early.md"));
  const auto plain_manifest = YAML::Load(read_file_bytes(ordinary.manifest_path));
  for (const auto& input : plain_manifest["inputs"]) {
    EXPECT_NE(fs::path(input["path"].as<std::string>()).filename(), "origin.yaml");
  }

  // MODEL_FILES.md declares an opaque source context with a checked outer
  // schema. Comments, quoting and whitespace are part of its byte identity.
  const std::string context =
      "# source attachment bytes, retained verbatim\n"
      "schema: galata.project-origin.v1\n"
      "note: 'Original source only; no edited-model validity claim'\n";
  put("origin.yaml", context);
  const auto attached_output = next_output();
  const auto attached = run_pipeline(parse_pipeline(context_study(true)),
                                     builtin_registry(),
                                     root.string(),
                                     attached_output.string());
  ASSERT_NE(attached.find("model"), nullptr);
  EXPECT_EQ(
      attached.find("model")->payload_as<CompiledModel>("executable_model").semantic_sha256(),
      ordinary.find("model")->payload_as<CompiledModel>("executable_model").semantic_sha256());
  EXPECT_TRUE(attached.find("model")->linearization_evidence.empty());
  const auto manifest = YAML::Load(read_file_bytes(attached.manifest_path));
  bool found = false;
  for (const auto& input : manifest["inputs"]) {
    if (fs::path(input["path"].as<std::string>()).filename() == "origin.yaml") {
      EXPECT_FALSE(found);
      found = true;
      EXPECT_EQ(input["sha256"].as<std::string>(), sha256(context));
      EXPECT_EQ(input["bytes_hex"].as<std::string>(), hex_bytes(context));
      EXPECT_EQ(input["size_bytes"].as<std::size_t>(), context.size());
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(LinearGraphWorkflow, CompileContextRejectsUnsupportedOuterSchemaWithoutSuccessfulManifest) {
  put("model.yaml",
      read_file_bytes((fs::path(GALATA_EXAMPLES_DIR) / "continuous-feedback/model.yaml").string()));
  put("origin.yaml", "schema: galata.project-origin.v999\n");
  const auto output = next_output();
  const auto document =
      "version: 1\nstages:\n"
      "  - id: model\n    capability: model.compile\n"
      "    input: {path: model.yaml, context_path: origin.yaml}\n";
  try {
    (void)run_pipeline(
        parse_pipeline(document), builtin_registry(), root.string(), output.string());
    FAIL() << "unsupported source-context schema was accepted";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("galata.project-origin.v1"), std::string::npos);
  }
  if (fs::exists(output)) {
    for (const auto& entry : fs::directory_iterator(output)) {
      EXPECT_FALSE(entry.path().filename().string().starts_with("run-"));
    }
  }
}

TEST_F(LinearGraphWorkflow, OversizedCompileContextIsRefusedBeforeAnyReportExecutes) {
  put("model.yaml",
      read_file_bytes((fs::path(GALATA_EXAMPLES_DIR) / "continuous-feedback/model.yaml").string()));
  std::string oversized = "schema: galata.project-origin.v1\n#";
  oversized.resize(8 * 1024 * 1024 + 1, 'x');
  put("origin.yaml", oversized);
  const auto output = next_output();
  std::vector<std::string> started;
  const ProgressCallback progress =
      [&](const std::string& stage, const std::string&, bool finished, const std::string&) {
        if (!finished)
          started.push_back(stage);
      };
  EXPECT_THROW((void)run_pipeline(parse_pipeline(context_study(true)),
                                  builtin_registry(),
                                  root.string(),
                                  output.string(),
                                  progress),
               std::runtime_error);
  EXPECT_TRUE(started.empty());
  EXPECT_FALSE(fs::exists(output / "early.md"));
}

TEST_F(LinearGraphWorkflow, LawRouteRefusesAnEmptyFeedbackGainFromACustomArtifact) {
  auto registry = builtin_registry();
  Capability capability;
  capability.id = "fixture.empty_law";
  capability.produces = "control_law";
  capability.run = [](const StageContext&) {
    galata::synth::LqrDesign law;
    law.plant.a = Eigen::MatrixXd::Constant(1, 1, -1.0);
    law.plant.b = Eigen::MatrixXd::Ones(1, 1);
    law.plant.state_names = {"distance"};
    law.plant.input_names = {"speed"};
    law.q = Eigen::MatrixXd::Ones(1, 1);
    law.r = Eigen::MatrixXd::Ones(1, 1);
    law.n = Eigen::MatrixXd::Zero(1, 1);
    // Empty K cannot silently change a declared feedback law to open loop.
    Artifact artifact;
    artifact.kind = "control_law";
    artifact.payload = std::move(law);
    return artifact;
  };
  registry.add(std::move(capability));
  auto document = synthetic_study();
  const auto source_at = document.find("fixture.linear_graph_source");
  ASSERT_NE(source_at, std::string::npos);
  document.replace(
      source_at, std::string("fixture.linear_graph_source").size(), "fixture.empty_law");
  const auto selector_at = document.find("system: {from: source}");
  ASSERT_NE(selector_at, std::string::npos);
  document.replace(
      selector_at, std::string("system: {from: source}").size(), "law: {from: source}");
  const auto output = next_output();
  try {
    (void)run_pipeline(parse_pipeline(document), registry, root.string(), output.string());
    FAIL() << "empty controller K was accepted as open loop";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("feedback gain"), std::string::npos);
  }
  EXPECT_FALSE(fs::exists(output / "model.yaml"));
  EXPECT_FALSE(fs::exists(output / "adapter.json"));
}
}  // namespace
