// SPDX-License-Identifier: Apache-2.0
//
// The capability registry and the artefacts capabilities produce.
//
// A capability is a named numerical operation with a typed input and a typed
// output. Capability identifiers are dotted and dispatched by prefix —
// `analyze.modes`, `report.markdown` — so that a whole new vertical can be
// added without touching the dispatcher. That property is load-bearing for the
// plugin ABI and is asserted by a test rather than assumed.
//
// WHAT THIS IS NOT: not yet a plugin host. Every capability here is compiled
// in. The C ABI in include/galata-c/ does not exist yet, and when it does, an
// out-of-tree plugin will register through this same registry.

#ifndef GALATA_PIPELINE_REGISTRY_HPP
#define GALATA_PIPELINE_REGISTRY_HPP

#include "galata/pipeline/value.hpp"

#include <any>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
// <stdexcept> for the std::runtime_error thrown by payload_as below. libstdc++
// and libc++ both pull it in transitively through one of the headers above;
// MSVC's standard library does not, and this header failed to compile on
// Windows because of it. A header includes what it uses.
#include <stdexcept>
#include <string>
#include <vector>

namespace galata::linearize {
struct Linearisation;
}

namespace galata::pipeline {

class RunFiles;

// Everything a stage produces, with the provenance charter rule 9 requires: a
// number never reaches the user without knowing which capability made it, from
// what, and on which build.
struct Artifact {
  // What this is, e.g. "linear_system", "modal_table", "report". Used to give
  // a legible error when a stage is wired to the wrong kind of upstream.
  std::string kind;
  std::string produced_by_capability;
  std::string produced_by_build;
  // A one-line human summary, printed by the CLI as each stage completes.
  std::string summary;
  std::any payload;

  // Immutable diagnostics of each source Jacobian, keyed by its study stage.
  // The executor preserves the union through downstream transformations. These
  // qualify the source linearization, not the numerical error of derived results.
  // New records must use the executing stage ID; inherited records retain the
  // same pointer. Null, nonfinite or malformed source diagnostics are refused.
  std::map<std::string, std::shared_ptr<const linearize::Linearisation>> linearization_evidence;

  // Throws with a message naming both kinds when the cast fails, rather than
  // returning a null pointer the caller might not check.
  template <typename T>
  [[nodiscard]] const T& payload_as(const std::string& expected_kind) const {
    if (kind != expected_kind) {
      throw std::runtime_error("stage produced a '" + kind + "' where a '" + expected_kind
                               + "' was required");
    }
    return std::any_cast<const T&>(payload);
  }
};

// What a capability is handed when it runs.
struct StageContext {
  // The stage's `input:` block, with stage references left as references.
  ValuePtr input;
  // Upstream artefacts, keyed by stage id.
  std::map<std::string, Artifact> upstream;
  // Directory the pipeline file lives in. Relative paths in the pipeline
  // resolve against this, not against the process's working directory, so a
  // study is runnable from anywhere.
  std::string base_directory;
  // Where output files go. Relative output paths resolve against this.
  std::string output_directory;
  // Shared run-scoped file access: contains writes and records the bytes used.
  std::shared_ptr<RunFiles> files;
  // Study-local identity of the executing stage; empty for direct invocations.
  std::string stage_id;

  // Resolves `{from: id}` and returns the referenced artefact.
  [[nodiscard]] const Artifact& upstream_at(const std::string& key) const;
  [[nodiscard]] std::string resolve_input_path(const std::string& path) const;
  [[nodiscard]] std::string resolve_output_path(const std::string& path) const;
  [[nodiscard]] std::string read_input(const std::string& path) const;
  [[nodiscard]] std::string read_input(const std::string& path, std::size_t max_bytes) const;
  void write_output(const std::string& path, const std::string& bytes) const;
};

using CapabilityFunction = std::function<Artifact(const StageContext&)>;

struct Capability {
  std::string id;
  // One line, imperative. Shown by `galata capabilities`.
  std::string summary;
  // The artefact kind this produces.
  std::string produces;
  // Honest state, printed alongside the summary. The README's status table is
  // generated from these, which is what stops it drifting.
  enum class State { Implemented, ImplementedUnvalidated, Stub };
  State state = State::ImplementedUnvalidated;
  CapabilityFunction run;
  // Closed input vocabulary. Empty means no inputs, never "accept anything".
  // Validated before any stage executes, including out-of-tree registrations.
  std::vector<std::string> input_keys = {};
  // Optional file roles, used to snapshot inputs and preflight all outputs
  // before execution. The keys must also occur in input_keys.
  std::vector<std::string> input_file_keys = {};
  std::vector<std::string> output_file_keys = {};
  // Optional positive byte limits for declared input file roles. Preflight
  // snapshots these before unbounded roles, and the reader also checks caches.
  std::map<std::string, std::size_t> input_file_byte_limits = {};
};

[[nodiscard]] std::string to_string(Capability::State state);

class Registry {
 public:
  void add(Capability capability);

  [[nodiscard]] const Capability* find(const std::string& id) const;
  [[nodiscard]] std::vector<const Capability*> all() const;

  // Every capability whose id starts with `prefix` followed by a dot, or which
  // equals it. This is the prefix dispatch the ABI depends on.
  [[nodiscard]] std::vector<const Capability*> with_prefix(const std::string& prefix) const;

 private:
  std::map<std::string, Capability> capabilities_;
};

// The capabilities compiled into galata.
[[nodiscard]] const Registry& builtin_registry();

}  // namespace galata::pipeline

#endif  // GALATA_PIPELINE_REGISTRY_HPP
