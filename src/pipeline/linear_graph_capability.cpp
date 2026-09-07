// SPDX-License-Identifier: Apache-2.0
// File and provenance adapter for deterministic typed state-space lowering.
// The graph compiler and RK4 executor remain the numerical implementation.
#include "galata/modeling/linear_adapter.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/synth/control.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {
constexpr std::size_t kMaxAdapterBytes = 2 * 1024 * 1024;

ValuePtr required(const ValuePtr& value, const std::string& key) {
  const auto entry = value->get(key);
  if (!entry)
    throw std::invalid_argument("model.linear_graph: missing required '" + key + "'");
  return entry;
}

void keys(const ValuePtr& value,
          std::initializer_list<std::string> allowed,
          const std::string& path) {
  for (const auto& [key, entry] : value->as_map()) {
    (void)entry;
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end())
      throw std::invalid_argument(path + ": unknown key '" + key + "'");
  }
}

std::vector<modeling::SignalType> channels(const ValuePtr& value,
                                           Eigen::Index count,
                                           const std::string& path) {
  const auto& entries = value->as_list();
  if (count < 1 || count > static_cast<Eigen::Index>(modeling::kMaxLinearChannels)
      || entries.size() != static_cast<std::size_t>(count)) {
    throw std::invalid_argument(path + ": requires one declared type per source channel (1..16)");
  }
  std::vector<modeling::SignalType> result;
  for (std::size_t i = 0; i < entries.size(); ++i) {
    const auto& entry = entries[i];
    const auto location = path + '[' + std::to_string(i) + ']';
    keys(entry, {"dimension", "frame"}, location);
    modeling::SignalType type;
    const auto& dimensions = required(entry, "dimension")->as_list();
    if (dimensions.size() != type.dimension.size())
      throw std::invalid_argument(location + ": dimension requires eight integer exponents");
    for (std::size_t d = 0; d < dimensions.size(); ++d) {
      const double number = dimensions[d]->as_number();
      if (!std::isfinite(number) || std::floor(number) != number || number < -16 || number > 16)
        throw std::invalid_argument(location
                                    + ": dimension exponents must be integers in [-16,16]");
      type.dimension[d] = static_cast<int>(number);
    }
    const auto frame = required(entry, "frame")->as_string();
    if (frame == "none")
      type.frame = modeling::Frame::None;
    else if (frame == "body")
      type.frame = modeling::Frame::Body;
    else if (frame == "ned")
      type.frame = modeling::Frame::Ned;
    else
      throw std::invalid_argument(location + ": frame must be none, body or ned");
    result.push_back(type);
  }
  return result;
}

Eigen::VectorXd vector(const ValuePtr& value, Eigen::Index count, const std::string& path) {
  const auto& entries = value->as_list();
  if (count < 1 || count > static_cast<Eigen::Index>(modeling::kMaxLinearChannels)
      || entries.size() != static_cast<std::size_t>(count))
    throw std::invalid_argument(path + ": requires one finite value per source channel (1..16)");
  Eigen::VectorXd result(count);
  for (Eigen::Index i = 0; i < count; ++i)
    result(i) = entries[static_cast<std::size_t>(i)]->as_number();
  if (!result.allFinite())
    throw std::invalid_argument(path + ": requires finite values");
  return result;
}

void number(std::ostream& out, double value) {
  if (!std::isfinite(value))
    throw std::invalid_argument("model.linear_graph: source evidence contains a nonfinite number");
  out << value;
}

template <typename Derived>
void matrix_json(std::ostream& out, const Eigen::MatrixBase<Derived>& value) {
  out << '[';
  for (Eigen::Index i = 0; i < value.rows(); ++i) {
    if (i != 0)
      out << ',';
    out << '[';
    for (Eigen::Index j = 0; j < value.cols(); ++j) {
      if (j != 0)
        out << ',';
      number(out, value(i, j));
    }
    out << ']';
  }
  out << ']';
}

void vector_json(std::ostream& out, const Eigen::VectorXd& value) {
  out << '[';
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i != 0)
      out << ',';
    number(out, value(i));
  }
  out << ']';
}

void strings_json(std::ostream& out, const std::vector<std::string>& values) {
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0)
      out << ',';
    out << json_quote(values[i]);
  }
  out << ']';
}

void channels_json(std::ostream& out, const std::vector<modeling::SignalType>& types) {
  out << '[';
  for (std::size_t i = 0; i < types.size(); ++i) {
    if (i != 0)
      out << ',';
    out << "{\"dimension\":[";
    for (std::size_t d = 0; d < types[i].dimension.size(); ++d) {
      if (d != 0)
        out << ',';
      out << types[i].dimension[d];
    }
    const char* frame = types[i].frame == modeling::Frame::Body  ? "body"
                        : types[i].frame == modeling::Frame::Ned ? "ned"
                                                                 : "none";
    out << "],\"frame\":" << json_quote(frame) << '}';
  }
  out << ']';
}

void lqr_json(std::ostream& out, const synth::LqrDesign& law) {
  out << ",\n  \"lqr_origin\":{\"q\":";
  matrix_json(out, law.q);
  out << ",\"r\":";
  matrix_json(out, law.r);
  out << ",\"n\":";
  matrix_json(out, law.n);
  out << ",\"k\":";
  matrix_json(out, law.riccati.k);
  out << ",\"care\":{\"x\":";
  matrix_json(out, law.riccati.x);
  out << ",\"relative_residual\":";
  number(out, law.riccati.relative_residual);
  out << ",\"residual_budget\":";
  number(out, law.riccati.residual_budget);
  out << ",\"symmetry_defect\":";
  number(out, law.riccati.symmetry_defect);
  out << ",\"subspace_condition\":";
  number(out, law.riccati.subspace_condition);
  out << ",\"hamiltonian_separation\":";
  number(out, law.riccati.hamiltonian_separation);
  out << ",\"closed_loop_eigenvalues\":[";
  for (std::size_t i = 0; i < law.riccati.closed_loop_eigenvalues.size(); ++i) {
    if (i != 0)
      out << ',';
    out << "{\"real\":";
    number(out, law.riccati.closed_loop_eigenvalues[i].real());
    out << ",\"imag\":";
    number(out, law.riccati.closed_loop_eigenvalues[i].imag());
    out << '}';
  }
  out << "]}}";
}

std::string adapter_json(const model::LinearSystem& system,
                         const modeling::LinearChannels& types,
                         const modeling::LinearGraphOptions& options,
                         const modeling::LinearGraph& graph,
                         const modeling::CompiledModel& compiled,
                         const std::string& model_path,
                         const synth::LqrDesign* law) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "{\n  \"schema\":\"galata.linear-adapter.v1\",\n  \"lowering\":\"linear-rows.v1\","
      << "\n  \"model_path\":" << json_quote(model_path)
      << ",\n  \"model_semantic_sha256\":" << json_quote(compiled.semantic_sha256())
      << ",\n  \"source_system\":{\"a\":";
  matrix_json(out, system.a);
  out << ",\"b\":";
  matrix_json(out, system.b);
  out << ",\"c\":";
  matrix_json(out, system.output_matrix());
  out << ",\"d\":";
  matrix_json(out, system.feedthrough_matrix());
  out << ",\"state_names\":";
  strings_json(out, system.state_names);
  out << ",\"input_names\":";
  strings_json(out, system.input_names);
  out << ",\"output_names\":";
  strings_json(out, system.output_labels());
  out << ",\"description\":" << json_quote(system.description)
      << ",\"citation\":" << json_quote(system.citation)
      << ",\"units\":" << json_quote(system.units) << "},\n  \"channel_types\":{\"states\":";
  channels_json(out, types.states);
  out << ",\"inputs\":";
  channels_json(out, types.inputs);
  out << ",\"outputs\":";
  channels_json(out, types.outputs);
  out << "},\n  \"initial_state\":";
  vector_json(out, options.initial_state);
  out << ",\n  \"command\":";
  vector_json(out, options.command);
  out << ",\n  \"feedback_gain\":";
  matrix_json(out, options.feedback_gain);
  out << ",\n  \"mapping\":{\"state_ids\":";
  strings_json(out, graph.state_ids);
  out << ",\"command_ids\":";
  strings_json(out, graph.command_ids);
  out << ",\"control_ids\":";
  strings_json(out, graph.control_ids);
  out << ",\"output_ids\":";
  strings_json(out, graph.output_ids);
  out << ",\"control_output_ids\":";
  strings_json(out, graph.control_output_ids);
  out << "},\n  \"assumptions\":";
  strings_json(out,
               {"Continuous LTI perturbation model: x_dot=A*x+B*u, y=C*x+D*u; "
                "constant command, with u=command-K*x when feedback is supplied.",
                "Explicit channel types follow source matrix order in canonical SI; "
                "declared row couplings do not infer physical frame transformations.",
                "Limits: 1..16 states, inputs and outputs; model YAML at most 1 MiB; "
                "adapter JSON at most 2 MiB; ordinary graph execution limits apply.",
                "The adapter introduces no actuator dynamics, saturation, estimator, "
                "sampled controller or new numerical solver.",
                "Inherited linearization diagnostics and LQR CARE diagnostics qualify "
                "their source operations only; they do not establish graph simulation "
                "accuracy, model validity or engineering acceptance.",
                "Model edits retain this origin record but require separate assessment; "
                "they do not preserve source Jacobian or controller equivalence."});
  if (law)
    lqr_json(out, *law);
  out << "\n}\n";
  auto bytes = out.str();
  if (bytes.size() > kMaxAdapterBytes)
    throw std::invalid_argument("model.linear_graph: adapter JSON exceeds 2 MiB");
  return bytes;
}

Artifact lower(const StageContext& context) {
  keys(context.input,
       {"system", "law", "channel_types", "initial_state", "command", "model_path", "adapter_path"},
       "model.linear_graph");
  const bool has_system = static_cast<bool>(context.input->get("system"));
  const bool has_law = static_cast<bool>(context.input->get("law"));
  if (has_system == has_law)
    throw std::invalid_argument("model.linear_graph requires exactly one of system or law");
  const auto model_path = required(context.input, "model_path")->as_string();
  const auto adapter_path = required(context.input, "adapter_path")->as_string();
  if (context.resolve_output_path(model_path) == context.resolve_output_path(adapter_path))
    throw std::invalid_argument(
        "model.linear_graph requires distinct model and adapter output paths");
  const synth::LqrDesign* law = nullptr;
  if (has_law)
    law = &context.upstream_at("law").payload_as<synth::LqrDesign>("control_law");
  const auto& system =
      law ? law->plant
          : context.upstream_at("system").payload_as<model::LinearSystem>("linear_system");
  if (law
      && (law->riccati.k.rows() != system.input_count()
          || law->riccati.k.cols() != system.state_count()))
    throw std::invalid_argument(
        "model.linear_graph: law feedback gain must have one row per input and column per state");
  const auto declared = required(context.input, "channel_types");
  keys(declared, {"states", "inputs", "outputs"}, "channel_types");
  modeling::LinearChannels types;
  types.states =
      channels(required(declared, "states"), system.state_count(), "channel_types.states");
  types.inputs =
      channels(required(declared, "inputs"), system.input_count(), "channel_types.inputs");
  types.outputs =
      channels(required(declared, "outputs"), system.output_count(), "channel_types.outputs");
  modeling::LinearGraphOptions options;
  options.initial_state =
      vector(required(context.input, "initial_state"), system.state_count(), "initial_state");
  options.command = vector(required(context.input, "command"), system.input_count(), "command");
  if (law)
    options.feedback_gain = law->riccati.k;
  const auto graph = modeling::lower_linear_system(system, types, options);
  auto compiled = modeling::compile_model(graph.model);
  const auto model_bytes = modeling::write_model_yaml(graph.model);
  if (model_bytes.size() > modeling::kMaxSourceBytes)
    throw std::invalid_argument("model.linear_graph: generated model YAML exceeds 1 MiB");
  const auto adapter_bytes = adapter_json(system, types, options, graph, compiled, model_path, law);
  if (context.cancelled && context.cancelled())
    throw modeling::Error(modeling::ErrorCode::Cancelled, "linear graph export cancelled");
  context.write_output(model_path, model_bytes);
  context.write_output(adapter_path, adapter_bytes);
  Artifact result;
  result.kind = "executable_model";
  result.summary = compiled.source_model().profile + "; " + std::to_string(system.state_count())
                   + " source states, " + std::to_string(system.input_count())
                   + " controls; semantic SHA-256 " + compiled.semantic_sha256()
                   + "; source evidence only, validity not assessed";
  result.payload = std::move(compiled);
  return result;
}
}  // namespace

void register_linear_graph_capability(Registry& registry) {
  registry.add(Capability{
      "model.linear_graph",
      "Lower a typed linear system or LQR plant and feedback into an executable graph with origin "
      "evidence",
      "executable_model",
      Capability::State::ImplementedUnvalidated,
      lower,
      {"system", "law", "channel_types", "initial_state", "command", "model_path", "adapter_path"},
      {},
      {"model_path", "adapter_path"}});
}
}  // namespace galata::pipeline
