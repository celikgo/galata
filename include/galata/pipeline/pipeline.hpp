// SPDX-License-Identifier: Apache-2.0
//
// The pipeline: a directed acyclic graph of capability invocations, parsed
// from YAML and executed in dependency order.
//
// The CLI and — later — the desktop application execute the identical document.
// The graphical surface is a view over this file, never a separate code path,
// because two code paths means two behaviours and only one of them gets tested.
//
// WHAT THIS IS NOT
// * No caching. Every run executes every stage. Content-addressed caching is
//   named in the design and is not built; a stage that takes an hour takes an
//   hour every time.
// * No parallelism. Independent branches run one after another, in a
//   deterministic order. Running them concurrently would need a decision about
//   reduction order that ADR-0004 has not yet been extended to cover.
// * No conditionals, loops or variables. A pipeline is a DAG of stages, not a
//   programming language, and it stays that way until something real needs
//   otherwise.

#ifndef GALATA_PIPELINE_PIPELINE_HPP
#define GALATA_PIPELINE_PIPELINE_HPP

#include "galata/pipeline/registry.hpp"
#include "galata/pipeline/value.hpp"

#include <functional>
#include <string>
#include <vector>

namespace galata::pipeline {

struct Stage {
  std::string id;
  std::string capability;
  ValuePtr input;
};

struct Pipeline {
  int version = 0;
  std::vector<Stage> stages;

  // Stage ids in an execution order that satisfies every dependency.
  //
  // Deterministic: where several stages are ready, the one declared first in
  // the file goes first. An arbitrary-but-consistent order would still be
  // reproducible, but declaration order is the one a reader can predict.
  [[nodiscard]] std::vector<std::string> execution_order() const;
};

// Parses a pipeline from YAML text. Throws with the offending stage id in the
// message on any structural error.
[[nodiscard]] Pipeline parse_pipeline(const std::string& yaml_text);
[[nodiscard]] Pipeline load_pipeline(const std::string& path);

struct StageResult {
  std::string stage_id;
  std::string capability;
  Artifact artifact;
};

struct RunResult {
  std::vector<StageResult> stages;
  [[nodiscard]] const Artifact* find(const std::string& stage_id) const;
};

// Called before and after each stage, so the CLI can stream progress.
using ProgressCallback = std::function<void(const std::string& stage_id,
                                            const std::string& capability,
                                            bool finished,
                                            const std::string& summary)>;

// Executes the pipeline. Throws on the first stage that fails, with the stage
// id and the capability in the message — a failure that does not say which
// stage failed is a failure the user has to bisect by hand.
[[nodiscard]] RunResult run_pipeline(const Pipeline& pipeline,
                                     const Registry& registry,
                                     const std::string& base_directory,
                                     const std::string& output_directory,
                                     const ProgressCallback& progress = nullptr);

}  // namespace galata::pipeline

#endif  // GALATA_PIPELINE_PIPELINE_HPP
