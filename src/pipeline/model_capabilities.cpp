// SPDX-License-Identifier: Apache-2.0
// Study adapters for the continuous scalar model contract. Numerical algorithms
// and semantic validation remain below this layer in galata::modeling.
#include "galata/modeling/model.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {
ValuePtr required(const StageContext& context, const std::string& key) {
  const auto value = context.input->get(key);
  if (!value)
    throw std::invalid_argument("missing required input '" + key + "'");
  return value;
}

int integer(const ValuePtr& value, const std::string& key) {
  const double number = value->as_number();
  if (!std::isfinite(number) || number < 0 || number > std::numeric_limits<int>::max()
      || std::floor(number) != number) {
    throw std::invalid_argument(key + ": expected a nonnegative representable integer");
  }
  return static_cast<int>(number);
}

Artifact compile(const StageContext& context) {
  const auto path = required(context, "path")->as_string();
  auto compiled = modeling::compile_model(
      modeling::parse_model_yaml(context.read_input(path, modeling::kMaxSourceBytes)));
  Artifact result;
  result.kind = "executable_model";
  result.summary = std::string(modeling::kProfile) + "; "
                   + std::to_string(compiled.state_ids().size()) + " states, "
                   + std::to_string(compiled.output_ids().size()) + " outputs; semantic SHA-256 "
                   + compiled.semantic_sha256();
  result.payload = std::move(compiled);
  return result;
}

std::string csv(const modeling::SimulationResult& result) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << "time_s";
  // IDs contain only the model contract's safe ASCII identifier characters.
  for (const auto& id : result.state_ids)
    out << ",state:" << id;
  for (const auto& id : result.output_ids)
    out << ",output:" << id;
  out << '\n';
  for (std::size_t i = 0; i < result.times_s.size(); ++i) {
    out << result.times_s[i];
    for (const auto value : result.states[i])
      out << ',' << value;
    for (const auto value : result.outputs[i])
      out << ',' << value;
    out << '\n';
  }
  return out.str();
}

std::string ids(const std::vector<std::string>& values) {
  std::string result = "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0)
      result += ',';
    result += json_quote(values[i]);
  }
  return result + ']';
}

std::string evidence(const modeling::CompiledModel& model,
                     const modeling::SimulationResult& result,
                     const std::string& trajectory_path,
                     const std::string& trajectory_bytes) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "{\n  \"schema\":\"galata.model-run.v1\",\n  \"profile\":"
      << json_quote(modeling::kProfile)
      << ",\n  \"model_semantic_sha256\":" << json_quote(result.semantic_sha256)
      << ",\n  \"model_source_yaml\":"
      << json_quote(modeling::write_model_yaml(model.source_model()))
      << ",\n  \"state_ids\":" << ids(result.state_ids)
      << ",\n  \"output_ids\":" << ids(result.output_ids)
      << ",\n  \"schedule_ids\":" << ids(model.schedule_ids())
      << ",\n  \"solver\":{\"method\":\"rk4\",\"initial_time_s\":" << result.options.initial_time_s
      << ",\"step_s\":" << result.options.step_s << ",\"steps\":" << result.options.step_count
      << ",\"sample_stride\":" << result.options.sample_stride << "},"
      << "\n  \"sample_count\":" << result.times_s.size()
      << ",\n  \"trajectory_path\":" << json_quote(trajectory_path)
      << ",\n  \"trajectory_sha256\":" << json_quote(sha256(trajectory_bytes))
      << ",\n  \"execution\":\"completed\",\n  \"numerical_accuracy\":\"not_assessed\","
      << "\n  \"model_validity\":\"not_assessed\",\n  "
         "\"engineering_acceptance\":\"not_assessed\"\n}\n";
  return out.str();
}

Artifact simulate(const StageContext& context) {
  const auto& model =
      context.upstream_at("model").payload_as<modeling::CompiledModel>("executable_model");
  // Validate required output roles and options before allocating or executing.
  const auto csv_path = required(context, "csv_path")->as_string();
  const auto evidence_path = required(context, "evidence_path")->as_string();
  const auto resolved_csv = context.resolve_output_path(csv_path);
  if (resolved_csv == context.resolve_output_path(evidence_path)) {
    throw std::invalid_argument("trajectory and evidence require distinct output paths");
  }
  modeling::SimulationOptions options;
  options.step_s = required(context, "step_s")->as_number();
  options.step_count = integer(required(context, "steps"), "steps");
  if (const auto origin = context.input->get("initial_time_s"))
    options.initial_time_s = origin->as_number();
  if (const auto stride = context.input->get("sample_stride"))
    options.sample_stride = integer(stride, "sample_stride");
  auto trajectory = modeling::simulate(model, options);
  const auto csv_bytes = csv(trajectory);
  const auto evidence_bytes = evidence(model, trajectory, csv_path, csv_bytes);
  context.write_output(csv_path, csv_bytes);
  context.write_output(evidence_path, evidence_bytes);
  Artifact result;
  result.kind = "model_trajectory";
  result.summary =
      "execution completed; " + std::to_string(trajectory.times_s.size())
      + " samples; numerical accuracy, model validity and engineering acceptance not assessed";
  result.payload = std::move(trajectory);
  return result;
}
}  // namespace

void register_model_capabilities(Registry& registry) {
  registry.add(Capability{"model.compile",
                          "Compile the continuous scalar model profile with typed ports and "
                          "explicit feedback semantics",
                          "executable_model",
                          Capability::State::ImplementedUnvalidated,
                          compile,
                          {"path"},
                          {"path"},
                          {},
                          {{"path", modeling::kMaxSourceBytes}}});
  registry.add(Capability{
      "sim.model",
      "Run a compiled continuous scalar model with fixed-step RK4 and write CSV plus scoped "
      "evidence",
      "model_trajectory",
      Capability::State::ImplementedUnvalidated,
      simulate,
      {"model", "initial_time_s", "step_s", "steps", "sample_stride", "csv_path", "evidence_path"},
      {},
      {"csv_path", "evidence_path"}});
}
}  // namespace galata::pipeline
