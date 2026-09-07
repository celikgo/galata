// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_PIPELINE_PROVENANCE_HPP
#define GALATA_PIPELINE_PROVENANCE_HPP
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"

#include "runtime_identity.hpp"

namespace galata::pipeline {
// Public custom capabilities must meet the same finite, shaped source-evidence
// contract as built-in linearization before the executor can retain a record.
void validate_linearization_evidence(const linearize::Linearisation& record);
[[nodiscard]] std::string write_run_manifest(const Pipeline& pipeline,
                                             const RunResult& result,
                                             RunFiles& files,
                                             const RunOptions& options,
                                             const std::string& input_directory,
                                             const FileRecord& executable,
                                             const RuntimeIdentity& runtime);
}  // namespace galata::pipeline
#endif
