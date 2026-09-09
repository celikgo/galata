// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_PIPELINE_ARTIFACTS_HPP
#define GALATA_PIPELINE_ARTIFACTS_HPP

#include "galata/model/aircraft.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/pipeline/registry.hpp"
#include "galata/trim/hover.hpp"
#include "galata/trim/level.hpp"

#include <iosfwd>

namespace galata::pipeline {
struct TrimArtifact {
  trim::TrimPoint point;
  model::Aircraft aircraft;
};

// The multirotor equilibrium travels with the plant it is an equilibrium of.
// `linearize.extended` needs both — the point to linearise about, and the model
// whose f it differentiates — and pairing them here is what stops a study
// wiring a trim of one vehicle into a linearisation of another.
struct HoverTrimArtifact {
  trim::HoverTrim point;
  model::Quadrotor model;
};

void register_design_capabilities(Registry& registry);
void register_model_capabilities(Registry& registry);
void register_quadrotor_capabilities(Registry& registry);
void register_linear_graph_capability(Registry& registry);
bool write_design_section(std::ostream& out, const Artifact& artifact);
}  // namespace galata::pipeline
#endif
