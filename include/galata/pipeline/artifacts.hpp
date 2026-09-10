// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_PIPELINE_ARTIFACTS_HPP
#define GALATA_PIPELINE_ARTIFACTS_HPP

#include "galata/data/record.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/numerics/integrator.hpp"
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

void register_data_capabilities(Registry& registry);

// What `sim.plant` produces. The state NAMES travel with the samples because the
// appended block's width is the model's — four rotors, or six, with or without a
// battery — so a reader cannot infer the columns from the trajectory alone.
struct PlantRun {
  std::vector<std::string> state_names;
  numerics::Trajectory trajectory;
  // The command and wind AT EACH RECORDED SAMPLE, not the constants the run was
  // configured with. A schedule makes those constants a half-truth: writing
  // them beside a trajectory they did not drive is how a reader reconstructs a
  // ground velocity that never happened.
  std::vector<Eigen::VectorXd> command_samples_rad_s;
  std::vector<Eigen::Vector3d> wind_samples_ned_m_s;
  // What the run was configured with, kept for the constant case and for
  // provenance. Equal to every entry above when no schedule was given.
  Eigen::VectorXd command_rad_s;
  Eigen::Vector3d wind_ned_m_s = Eigen::Vector3d::Zero();
  bool battery_present = false;
  // Declared, not discovered: a powered pack is always discharging, so this is a
  // statement about the RUN. Recorded so a reader of the trajectory can see
  // which was chosen rather than inferring it from a flat column.
  bool battery_frozen = false;
  double step_s = 0.0;
  int step_count = 0;
};

// What `sim.sampled` adds to a plant run: the controller's own record, one
// entry per CONTROLLER TICK rather than per integration sample.
//
// REQUESTED AND APPLIED ARE BOTH KEPT. The requested command is what the law
// asked for; the applied command is what the plant received after saturation
// and after the delay line. Reporting only the second hides a controller that
// spent the whole run against its limits; reporting only the first describes a
// vehicle that was never flown. The residual between them is the honest measure
// of how much authority the law asked for and did not get.
struct SampledControlRecord {
  std::vector<double> tick_times_s;
  std::vector<Eigen::VectorXd> requested_rad_s;  // what the law asked for
  std::vector<Eigen::VectorXd> saturated_rad_s;  // after limits, before delay
  std::vector<Eigen::VectorXd> applied_rad_s;    // what the plant received
  std::vector<Eigen::VectorXd> chart_error;      // the coordinates it fed back on
  double controller_period_s = 0.0;
  int delay_periods = 0;
  // Ticks at which saturation changed the command at all, and the worst single
  // channel residual over the run.
  int saturated_tick_count = 0;
  double worst_saturation_residual_rad_s = 0.0;
  // Declared, not inferred: whether the reference position translates.
  bool reference_follows_trim_velocity = false;
};

// What `sim.sampled` produces. The plant run and the controller's own record
// travel together because neither answers the interesting question alone: the
// trajectory says what the aircraft did, and the record says what it was asked
// to do and how much of that it was allowed.
struct SampledRun {
  PlantRun plant;
  SampledControlRecord control;
};

void register_design_capabilities(Registry& registry);
void register_model_capabilities(Registry& registry);
void register_quadrotor_capabilities(Registry& registry);
void register_linear_graph_capability(Registry& registry);
bool write_design_section(std::ostream& out, const Artifact& artifact);
}  // namespace galata::pipeline
#endif
