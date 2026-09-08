// SPDX-License-Identifier: Apache-2.0

#include "galata/pipeline/pipeline.hpp"

#include "galata/pipeline/files.hpp"
#include "galata/version.hpp"

#include "../io/strict_yaml.hpp"
#include "provenance.hpp"
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {

ValuePtr convert(const YAML::Node& node, const std::string& path);

void check_keys(const YAML::Node& node,
                const std::string& path,
                std::initializer_list<std::string> allowed) {
  try {
    io::yaml_keys(node, path, allowed);
  } catch (const std::exception& error) {
    throw std::runtime_error(error.what());
  }
}

// `{from: stage_id}` is wiring, not data. Recognised here and nowhere else.
bool is_stage_reference(const YAML::Node& node) {
  return node.IsMap() && node.size() == 1 && node["from"] && node["from"].IsScalar();
}

ValuePtr convert_scalar(const YAML::Node& node) {
  if (node.Tag() == "!") {
    return Value::string(node.Scalar());
  }
  // yaml-cpp does not tag scalar types, so the kind is recovered by trying the
  // narrowest interpretation first. Order matters: "true" parses as a bool and
  // must not become the string "true", and "1" must not become the string "1"
  // or a stage id could never be compared with a number.
  bool as_bool = false;
  if (YAML::convert<bool>::decode(node, as_bool)) {
    return Value::boolean(as_bool);
  }
  double as_number = 0.0;
  if (YAML::convert<double>::decode(node, as_number)) {
    if (!std::isfinite(as_number)) {
      throw std::runtime_error("pipeline: numeric inputs must be finite");
    }
    return Value::number(as_number);
  }
  return Value::string(node.Scalar());
}

ValuePtr convert(const YAML::Node& node, const std::string& path) {
  switch (node.Type()) {
    case YAML::NodeType::Null:
    case YAML::NodeType::Undefined:
      return Value::null();
    case YAML::NodeType::Scalar:
      return convert_scalar(node);
    case YAML::NodeType::Sequence: {
      std::vector<ValuePtr> items;
      items.reserve(node.size());
      for (std::size_t i = 0; i < node.size(); ++i) {
        items.push_back(convert(node[i], path + "[" + std::to_string(i) + "]"));
      }
      return Value::list(std::move(items));
    }
    case YAML::NodeType::Map: {
      if (is_stage_reference(node)) {
        return Value::stage_reference(node["from"].Scalar());
      }
      std::map<std::string, ValuePtr> entries;
      for (const auto& entry : node) {
        if (!entry.first.IsScalar()) {
          throw std::runtime_error("pipeline: non-scalar map key at " + path);
        }
        const std::string key = entry.first.Scalar();
        entries[key] = convert(entry.second, path.empty() ? key : path + "." + key);
      }
      return Value::map(std::move(entries));
    }
  }
  return Value::null();
}

}  // namespace

std::vector<std::string> Pipeline::execution_order() const {
  // Kahn's algorithm, with ready stages taken in declaration order so the
  // execution sequence is the one a reader of the file predicts.
  std::map<std::string, std::size_t> index_of;
  for (std::size_t i = 0; i < stages.size(); ++i) {
    if (stages[i].id.empty() || !index_of.emplace(stages[i].id, i).second) {
      throw std::runtime_error("pipeline: stage ids must be unique non-empty strings");
    }
    if (!stages[i].input || stages[i].input->kind() != Value::Kind::Map) {
      throw std::runtime_error("stage '" + stages[i].id + "': input must be a map");
    }
  }

  std::vector<std::set<std::string>> pending(stages.size());
  for (std::size_t i = 0; i < stages.size(); ++i) {
    for (const std::string& reference : stages[i].input->referenced_stages()) {
      if (index_of.count(reference) == 0) {
        throw std::runtime_error("stage '" + stages[i].id + "' refers to stage '" + reference
                                 + "', which is not defined in this pipeline");
      }
      if (reference == stages[i].id) {
        throw std::runtime_error("stage '" + stages[i].id + "' refers to itself");
      }
      pending[i].insert(reference);
    }
  }

  std::vector<std::string> order;
  std::set<std::string> done;
  order.reserve(stages.size());

  while (order.size() < stages.size()) {
    bool progressed = false;
    for (std::size_t i = 0; i < stages.size(); ++i) {
      if (done.count(stages[i].id) != 0) {
        continue;
      }
      const bool ready =
          std::all_of(pending[i].begin(), pending[i].end(), [&done](const std::string& id) {
            return done.count(id) != 0;
          });
      if (!ready) {
        continue;
      }
      order.push_back(stages[i].id);
      done.insert(stages[i].id);
      progressed = true;
      break;  // restart the scan so declaration order is honoured
    }
    if (!progressed) {
      // Name the stages involved: "there is a cycle" is not actionable, but
      // "these four stages are in a cycle" is.
      std::ostringstream message;
      message << "pipeline contains a dependency cycle among stages:";
      for (const Stage& stage : stages) {
        if (done.count(stage.id) == 0) {
          message << " " << stage.id;
        }
      }
      throw std::runtime_error(message.str());
    }
  }
  return order;
}

Pipeline parse_pipeline(const std::string& yaml_text) {
  YAML::Node root;
  try {
    root = io::load_yaml(yaml_text, "pipeline");
  } catch (const std::exception& error) {
    throw std::runtime_error(std::string("pipeline is not valid YAML: ") + error.what());
  }

  if (!root.IsMap()) {
    throw std::runtime_error("pipeline: the document must be a map with 'version' and 'stages'");
  }

  Pipeline pipeline;
  pipeline.source_name = "<memory>";
  pipeline.source_bytes = yaml_text;
  check_keys(root, "pipeline", {"version", "stages"});
  if (!root["version"]) {
    throw std::runtime_error("pipeline: missing 'version'");
  }
  pipeline.version = root["version"].as<int>();
  if (pipeline.version != 1) {
    throw std::runtime_error("pipeline: version " + std::to_string(pipeline.version)
                             + " is not supported; this build understands version 1");
  }

  if (!root["stages"] || !root["stages"].IsSequence()) {
    throw std::runtime_error("pipeline: 'stages' must be a sequence");
  }

  std::set<std::string> seen;
  for (const YAML::Node& node : root["stages"]) {
    if (!node.IsMap()) {
      throw std::runtime_error("pipeline: each stage must be a map");
    }
    Stage stage;
    check_keys(node, "pipeline stage", {"id", "capability", "input"});
    if (!node["id"]) {
      throw std::runtime_error("pipeline: a stage is missing its 'id'");
    }
    stage.id = node["id"].Scalar();
    if (!node["id"].IsScalar() || stage.id.empty()) {
      throw std::runtime_error("pipeline: each stage id must be a non-empty string");
    }
    if (!seen.insert(stage.id).second) {
      throw std::runtime_error("pipeline: stage id '" + stage.id + "' is used twice");
    }
    if (!node["capability"]) {
      throw std::runtime_error("pipeline: stage '" + stage.id + "' is missing its 'capability'");
    }
    stage.capability = node["capability"].Scalar();
    if (!node["capability"].IsScalar() || stage.capability.empty()) {
      throw std::runtime_error("stage '" + stage.id + "': capability must be a non-empty string");
    }
    stage.input = node["input"] ? convert(node["input"], stage.id) : Value::map({});
    if (stage.input->kind() != Value::Kind::Map) {
      throw std::runtime_error("stage '" + stage.id + "': input must be a map");
    }
    pipeline.stages.push_back(std::move(stage));
  }

  if (pipeline.stages.empty()) {
    throw std::runtime_error("pipeline: 'stages' is empty");
  }

  // Validate the graph now rather than at run time, so a malformed pipeline
  // fails before any expensive stage has run.
  (void)pipeline.execution_order();
  return pipeline;
}

Pipeline load_pipeline(const std::string& path) {
  auto pipeline = parse_pipeline(read_file_bytes(path));
  pipeline.source_name = std::filesystem::canonical(path).string();
  return pipeline;
}

const Artifact* RunResult::find(const std::string& stage_id) const {
  for (const StageResult& result : stages) {
    if (result.stage_id == stage_id) {
      return &result.artifact;
    }
  }
  return nullptr;
}

RunResult run_pipeline(const Pipeline& pipeline,
                       const Registry& registry,
                       const std::string& base_directory,
                       const std::string& output_directory,
                       const ProgressCallback& progress,
                       const RunOptions& options) {
  const auto check_cancelled = [&]() {
    if (options.cancelled && options.cancelled())
      throw std::runtime_error("pipeline cancelled");
  };
  check_cancelled();
  if (pipeline.version != 1 || pipeline.stages.empty()) {
    throw std::runtime_error("pipeline: version 1 and at least one stage are required");
  }
  const std::vector<std::string> order = pipeline.execution_order();

  // Validate every stage before a writer or an expensive solver can run.
  for (const Stage& stage : pipeline.stages) {
    const Capability* capability = registry.find(stage.capability);
    if (!capability) {
      throw std::runtime_error("stage '" + stage.id + "': no capability named '" + stage.capability
                               + "'");
    }
    if (!stage.input || stage.input->kind() != Value::Kind::Map) {
      throw std::runtime_error("stage '" + stage.id + "': input must be a map");
    }
    for (const auto& [key, value] : stage.input->as_map()) {
      (void)value;
      if (std::find(capability->input_keys.begin(), capability->input_keys.end(), key)
          == capability->input_keys.end()) {
        throw std::runtime_error("stage '" + stage.id + "' (" + stage.capability
                                 + "): unknown input key '" + key + "'");
      }
    }
  }

  std::map<std::string, const Stage*> by_id;
  for (const Stage& stage : pipeline.stages) {
    by_id[stage.id] = &stage;
  }

  RunResult result;
  const auto executable_path = current_executable();
  // Snapshot runtime identity before any stage can execute. A concurrent
  // replacement must not attribute this study to a newer on-disk executable.
  const auto executable = options.write_manifest ? snapshot_file(executable_path) : FileRecord{};
  const auto runtime = snapshot_runtime(options.write_manifest);
  const auto files = std::make_shared<RunFiles>(output_directory, options.overwrite);
  files->protect_input_path(executable_path);
  for (const auto& module : runtime.modules) {
    if (module.storage == "file") {
      files->protect_input_path(module.path);
    }
  }
  if (!pipeline.source_name.empty() && pipeline.source_name != "<memory>") {
    files->record_input(pipeline.source_name, pipeline.source_bytes);
  }

  // Snapshot bounded input roles first, strictest limits first. A legacy
  // unbounded role referencing the same file must not bypass a model's cap.
  // All inputs still precede output reservations and capability execution.
  struct InputRole {
    const Stage* stage;
    std::string key;
    std::size_t limit;
  };

  std::vector<InputRole> bounded_inputs;
  for (const auto& stage : pipeline.stages) {
    const auto* capability = registry.find(stage.capability);
    for (const auto& [key, limit] : capability->input_file_byte_limits) {
      bounded_inputs.push_back({&stage, key, limit});
    }
  }
  std::stable_sort(bounded_inputs.begin(), bounded_inputs.end(), [](const auto& a, const auto& b) {
    return a.limit < b.limit;
  });
  const auto read_role =
      [&](const Stage& stage, const std::string& key, const std::optional<std::size_t> limit) {
        const auto* capability = registry.find(stage.capability);
        if (!stage.input->get(key)
            && std::find(capability->optional_input_file_keys.begin(),
                         capability->optional_input_file_keys.end(),
                         key)
                   != capability->optional_input_file_keys.end())
          return;
        StageContext context;
        context.base_directory = base_directory;
        context.files = files;
        try {
          const auto path = stage.input->string_at(key);
          if (limit)
            (void)context.read_input(path, *limit);
          else
            (void)context.read_input(path);
        } catch (const std::exception& error) {
          throw std::runtime_error("stage '" + stage.id + "' (" + stage.capability + ") input '"
                                   + key + "': " + error.what());
        }
      };
  for (const auto& role : bounded_inputs)
    read_role(*role.stage, role.key, role.limit);
  for (const auto& stage : pipeline.stages) {
    const auto* capability = registry.find(stage.capability);
    for (const auto& key : capability->input_file_keys) {
      if (!capability->input_file_byte_limits.contains(key))
        read_role(stage, key, std::nullopt);
    }
  }
  for (const auto& stage : pipeline.stages) {
    const auto* capability = registry.find(stage.capability);
    for (const auto& key : capability->output_file_keys) {
      files->reserve_output(stage.input->string_at(key));
    }
  }
  std::map<std::string, Artifact> produced;

  for (const std::string& stage_id : order) {
    check_cancelled();
    const Stage& stage = *by_id.at(stage_id);
    const Capability* capability = registry.find(stage.capability);
    if (capability == nullptr) {
      throw std::runtime_error("stage '" + stage_id + "': no capability named '" + stage.capability
                               + "'. Run `galata capabilities` to see what this build provides.");
    }

    if (progress) {
      progress(stage_id, stage.capability, false, "");
    }

    StageContext context;
    context.input = stage.input;
    context.stage_id = stage_id;
    context.base_directory = base_directory;
    context.output_directory = output_directory;
    context.files = files;
    context.cancelled = options.cancelled;
    for (const std::string& reference : stage.input->referenced_stages()) {
      context.upstream[reference] = produced.at(reference);
    }

    Artifact artifact;
    try {
      check_cancelled();
      artifact = capability->run(context);
      check_cancelled();
    } catch (const std::exception& error) {
      throw std::runtime_error("stage '" + stage_id + "' (" + stage.capability
                               + ") failed: " + error.what());
    }
    for (const auto& [source, evidence] : artifact.linearization_evidence) {
      if (!evidence) {
        throw std::runtime_error("stage '" + stage_id + "': null linearization evidence for '"
                                 + source + "'");
      }
      if (source != stage_id) {
        bool inherited = false;
        for (const auto& [upstream_id, upstream] : context.upstream) {
          (void)upstream_id;
          const auto origin = upstream.linearization_evidence.find(source);
          inherited =
              inherited
              || (origin != upstream.linearization_evidence.end() && origin->second == evidence);
        }
        if (!inherited) {
          throw std::runtime_error("stage '" + stage_id
                                   + "': cannot invent linearization evidence for stage '" + source
                                   + "'");
        }
      }
      try {
        validate_linearization_evidence(*evidence);
      } catch (const std::exception& error) {
        throw std::runtime_error("stage '" + stage_id
                                 + "': invalid linearization evidence: " + error.what());
      }
    }
    artifact.produced_by_capability = stage.capability;
    artifact.produced_by_build = std::string(galata::build_identification());
    for (const auto& [upstream_id, upstream] : context.upstream) {
      (void)upstream_id;
      for (const auto& [source, evidence] : upstream.linearization_evidence) {
        const auto [record, inserted] =
            artifact.linearization_evidence.try_emplace(source, evidence);
        if (!inserted && record->second != evidence) {
          throw std::runtime_error("stage '" + stage_id
                                   + "': cannot replace upstream linearization evidence '" + source
                                   + "'");
        }
      }
    }

    if (progress) {
      progress(stage_id, stage.capability, true, artifact.summary);
    }

    produced[stage_id] = artifact;
    result.stages.push_back(StageResult{stage_id, stage.capability, std::move(artifact)});
  }
  check_cancelled();
  if (options.write_manifest) {
    verify_file_unchanged(executable);
    verify_runtime_unchanged(runtime);
    result.manifest_path =
        write_run_manifest(pipeline, result, *files, options, base_directory, executable, runtime);
  }
  return result;
}

}  // namespace galata::pipeline
