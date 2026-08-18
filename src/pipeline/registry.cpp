// SPDX-License-Identifier: Apache-2.0

#include "galata/pipeline/registry.hpp"

#include <filesystem>
#include <stdexcept>
#include <utility>

namespace galata::pipeline {

std::string to_string(Capability::State state) {
  switch (state) {
    case Capability::State::Implemented:
      return "implemented and validated";
    case Capability::State::ImplementedUnvalidated:
      return "implemented, unvalidated";
    case Capability::State::Stub:
      return "stub";
  }
  return "unknown";
}

const Artifact& StageContext::upstream_at(const std::string& key) const {
  const ValuePtr found = input ? input->get(key) : nullptr;
  if (!found) {
    throw std::runtime_error("required input '" + key + "' is missing");
  }
  if (found->kind() != Value::Kind::StageReference) {
    throw std::runtime_error("input '" + key + "' must be a stage reference, written "
                             "{from: stage_id}, but it is a " +
                             Value::kind_name(found->kind()));
  }
  const std::string& stage_id = found->as_stage_reference();
  const auto artifact = upstream.find(stage_id);
  if (artifact == upstream.end()) {
    throw std::runtime_error("input '" + key + "' refers to stage '" + stage_id
                             + "', which produced no output");
  }
  return artifact->second;
}

std::string StageContext::resolve_input_path(const std::string& path) const {
  const std::filesystem::path candidate(path);
  if (candidate.is_absolute() || base_directory.empty()) {
    return path;
  }
  return (std::filesystem::path(base_directory) / candidate).lexically_normal().string();
}

std::string StageContext::resolve_output_path(const std::string& path) const {
  const std::filesystem::path candidate(path);
  if (candidate.is_absolute() || output_directory.empty()) {
    return path;
  }
  return (std::filesystem::path(output_directory) / candidate).lexically_normal().string();
}

void Registry::add(Capability capability) {
  if (capability.id.empty()) {
    throw std::invalid_argument("Registry::add: capability id is empty");
  }
  if (!capability.run) {
    throw std::invalid_argument("Registry::add: capability '" + capability.id
                                + "' has no implementation");
  }
  if (capabilities_.count(capability.id) != 0) {
    throw std::invalid_argument("Registry::add: capability '" + capability.id
                                + "' is already registered");
  }
  const std::string id = capability.id;
  capabilities_.emplace(id, std::move(capability));
}

const Capability* Registry::find(const std::string& id) const {
  const auto found = capabilities_.find(id);
  return (found == capabilities_.end()) ? nullptr : &found->second;
}

std::vector<const Capability*> Registry::all() const {
  std::vector<const Capability*> result;
  result.reserve(capabilities_.size());
  // std::map iterates in key order, so this listing is stable across runs and
  // across platforms — which matters because the README's status table is
  // generated from it.
  for (const auto& [id, capability] : capabilities_) {
    (void)id;
    result.push_back(&capability);
  }
  return result;
}

std::vector<const Capability*> Registry::with_prefix(const std::string& prefix) const {
  std::vector<const Capability*> result;
  for (const auto& [id, capability] : capabilities_) {
    if (id == prefix
        || (id.size() > prefix.size() && id.compare(0, prefix.size(), prefix) == 0
            && id[prefix.size()] == '.')) {
      result.push_back(&capability);
    }
  }
  return result;
}

}  // namespace galata::pipeline
