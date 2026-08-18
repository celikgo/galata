// SPDX-License-Identifier: Apache-2.0

#include "galata/pipeline/pipeline.hpp"

#include "galata/version.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {

ValuePtr convert(const YAML::Node& node, const std::string& path);

// `{from: stage_id}` is wiring, not data. Recognised here and nowhere else.
bool is_stage_reference(const YAML::Node& node) {
  return node.IsMap() && node.size() == 1 && node["from"] && node["from"].IsScalar();
}

ValuePtr convert_scalar(const YAML::Node& node) {
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
    index_of[stages[i].id] = i;
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
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception& error) {
    throw std::runtime_error(std::string("pipeline is not valid YAML: ") + error.what());
  }

  if (!root.IsMap()) {
    throw std::runtime_error("pipeline: the document must be a map with 'version' and 'stages'");
  }

  Pipeline pipeline;
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
    if (!node["id"]) {
      throw std::runtime_error("pipeline: a stage is missing its 'id'");
    }
    stage.id = node["id"].Scalar();
    if (!seen.insert(stage.id).second) {
      throw std::runtime_error("pipeline: stage id '" + stage.id + "' is used twice");
    }
    if (!node["capability"]) {
      throw std::runtime_error("pipeline: stage '" + stage.id + "' is missing its 'capability'");
    }
    stage.capability = node["capability"].Scalar();
    stage.input = node["input"] ? convert(node["input"], stage.id) : Value::map({});
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
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open pipeline file: " + path);
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return parse_pipeline(buffer.str());
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
                       const ProgressCallback& progress) {
  const std::vector<std::string> order = pipeline.execution_order();

  std::map<std::string, const Stage*> by_id;
  for (const Stage& stage : pipeline.stages) {
    by_id[stage.id] = &stage;
  }

  RunResult result;
  std::map<std::string, Artifact> produced;

  for (const std::string& stage_id : order) {
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
    context.base_directory = base_directory;
    context.output_directory = output_directory;
    for (const std::string& reference : stage.input->referenced_stages()) {
      context.upstream[reference] = produced.at(reference);
    }

    Artifact artifact;
    try {
      artifact = capability->run(context);
    } catch (const std::exception& error) {
      throw std::runtime_error("stage '" + stage_id + "' (" + stage.capability
                               + ") failed: " + error.what());
    }
    artifact.produced_by_capability = stage.capability;
    artifact.produced_by_build = std::string(galata::build_identification());

    if (progress) {
      progress(stage_id, stage.capability, true, artifact.summary);
    }

    produced[stage_id] = artifact;
    result.stages.push_back(StageResult{stage_id, stage.capability, std::move(artifact)});
  }
  return result;
}

}  // namespace galata::pipeline
