// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_PIPELINE_ARTIFACTS_HPP
#define GALATA_PIPELINE_ARTIFACTS_HPP

#include "galata/data/record.hpp"
#include "galata/identify/greybox.hpp"
#include "galata/identify/validate.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/numerics/integrator.hpp"
#include "galata/pipeline/registry.hpp"
#include "galata/trim/hover.hpp"
#include "galata/trim/level.hpp"

#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

namespace galata::pipeline {

struct FittedModelProvenance;

// WHERE A MODEL CAME FROM, travelling with the model.
//
// A `quadrotor` artefact used to be a bare `model::Quadrotor`, which is a
// complete description of a plant and no description at all of its standing. It
// stopped being enough the moment a capability could PRODUCE a model rather
// than only load one: a fitted plant and a hand-written one are the same C++
// object and the same YAML, and a study that mixes them must be able to say
// which is which without the reader diffing files.
//
// `sha256` is of the model file's BYTES, never of its path, for the reason
// `data::Record` gives about its own source: a path is not an identity, and two
// runs citing one path can have read different files.
struct ModelIdentity {
  // "file" for a model read from YAML, "fit" for one an identification
  // produced. Closed vocabulary; a third value means a third way for a model to
  // exist and needs its own record.
  std::string origin = "file";
  std::string path;    // as the study wrote it; empty for a fit
  std::string sha256;  // of the file's bytes; empty for a fit
  std::string summary;

  // Present exactly when `origin == "fit"`. Shared rather than copied for the
  // reason `Artifact::linearization_evidence` is: it is immutable evidence that
  // passes through several stages and must stay one object, so that two
  // artefacts claiming the same fit are claiming the same fit.
  std::shared_ptr<const FittedModelProvenance> fit;

  [[nodiscard]] bool is_fitted() const noexcept {
    return origin == "fit";
  }
};

// What every `quadrotor` artefact carries: the plant, and its standing.
struct QuadrotorArtifact {
  model::Quadrotor model;
  ModelIdentity identity;
};

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
  // Carried through unchanged from the plant this is an equilibrium of, so a
  // chain that trims and then linearises has not lost the identity of the model
  // it did that to. See `ModelIdentity`.
  ModelIdentity model_identity;
};

// One estimated parameter, with everything a reader needs to judge the number
// beside the number itself. The bounds are here because an estimate resting on
// one is not an interior estimate and the value alone does not say so; the unit
// is here because a coefficient without one is not a measurement; the standard
// error is here with its own flag because "no uncertainty could be estimated"
// and "the uncertainty is zero" are different statements and a bare 0.0 reads
// as the second.
struct FittedParameter {
  std::string path;
  std::string unit;
  double lower = 0.0;
  double upper = 0.0;
  double initial = 0.0;
  double value = 0.0;
  double standard_error = 0.0;
  bool standard_error_is_estimable = false;
  bool at_bound = false;
};

// The record of what an identification did, travelling with the model it
// produced.
//
// WHY THIS IS NOT OPTIONAL. A model file written from a fit is byte-for-byte
// the same KIND of object as a model file written from a bench measurement, and
// nothing in the format distinguishes them (see `model::serialize_quadrotor`).
// The only thing that can is a record kept beside it, so this is carried by the
// artefact, written to a file by `model.quadrotor.export`, and refused rather
// than defaulted where it is required.
struct FittedModelProvenance {
  // WHAT IT STARTED FROM. The base model is not modified and is not replaced:
  // the fit produces a NEW model whose unnamed parameters are the base's, and
  // this says which base that was, by its bytes.
  std::string base_model_path;
  std::string base_model_sha256;
  std::string base_model_description;

  // WHAT IT WAS FITTED TO. `identify.validate` reads this rather than trusting
  // a digest the caller types in beside it.
  std::string estimation_record_path;
  std::string estimation_record_sha256;
  bool estimation_record_is_window = false;
  double estimation_window_start_s = 0.0;
  double estimation_window_end_s = 0.0;
  double estimation_first_sample_s = 0.0;
  double estimation_last_sample_s = 0.0;
  int estimation_sample_count = 0;

  // WHAT MOVED, AND WHAT DID NOT. `preserved_parameter_paths` is written out in
  // full rather than left as "everything else": a reader asking which numbers in
  // the exported file are measurements and which are inherited must be able to
  // answer it from this record alone, without diffing two YAML files.
  std::vector<FittedParameter> fitted;
  std::vector<std::string> preserved_parameter_paths;

  // WHAT WAS MINIMISED. Stated in words because every residual below is
  // relative to it, and a weighted sum whose weights are unstated is a
  // weighting decision made by accident.
  std::string objective_definition;
  std::vector<std::string> output_matches;  // "channel -> state (scale unit)"
  std::vector<std::string> command_channels;

  // THREE THINGS THAT ARE NOT THE SAME, kept apart here as
  // `include/galata/identify/greybox.hpp` keeps them apart: the optimiser
  // finished, the parameters are identifiable, the fit is acceptable. This
  // record carries the first two and never the third.
  double objective = 0.0;
  double residual_rms = 0.0;
  int iterations = 0;
  int residual_count = 0;
  double last_step_norm = 0.0;
  double jacobian_condition_number = 0.0;
  double identifiability_ratio = 0.0;
  bool optimiser_finished = false;
  bool uncertainty_is_estimable = false;
  std::string uncertainty_assumptions;
  double step_s = 0.0;
};

void register_data_capabilities(Registry& registry);
void register_identify_capabilities(Registry& registry);

// What `identify.validate` produces. The model's identity travels with the
// verdict so a report can say WHICH plant was scored, and whether that plant was
// itself fitted, without re-walking the study graph.
struct ValidationArtifact {
  identify::ValidationResult result;
  ModelIdentity model_identity;
};

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
