// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_PIPELINE_ARTIFACTS_HPP
#define GALATA_PIPELINE_ARTIFACTS_HPP

#include "galata/model/aircraft.hpp"
#include "galata/pipeline/registry.hpp"
#include "galata/trim/level.hpp"

#include <iosfwd>

namespace galata::pipeline {
struct TrimArtifact {
  trim::TrimPoint point;
  model::Aircraft aircraft;
};

void register_design_capabilities(Registry& registry);
void register_model_capabilities(Registry& registry);
bool write_design_section(std::ostream& out, const Artifact& artifact);
}  // namespace galata::pipeline
#endif
