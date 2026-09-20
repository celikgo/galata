// SPDX-License-Identifier: Apache-2.0
//
// identify.* — fitting a model to measured data, and asking whether the result
// predicts anything.
//
// THE THREE THINGS THESE CAPABILITIES KEEP APART, because conflating them is
// how an identification run produces a confident wrong answer, and because a
// pipeline is exactly where they get conflated: a stage that completes looks
// like a stage that succeeded.
//
//   the OPTIMISER FINISHED — it ran its declared iterations. Says nothing about
//     whether the parameters mean anything, and with a fixed iteration count it
//     is true even of a run that moved nothing. `objective_improved` is the
//     question a reader actually has.
//   the PARAMETERS ARE IDENTIFIABLE — the data constrains each one separately.
//     A direction the Jacobian cannot see is a parameter this record did not
//     measure, and `identify.greybox` REFUSES rather than reporting a number
//     for it.
//   the FIT IS ACCEPTABLE — an engineering judgement about whether the residual
//     is small enough for the use. Nothing here makes it and nothing here
//     implies it. ADR-0008 requires the three to be separable in the evidence;
//     this file is where that separation is built rather than asserted.

#include "galata/identify/flight_test_campaign.hpp"
#include "galata/identify/greybox.hpp"
#include "galata/identify/static_fit.hpp"
#include "galata/identify/validate.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/version.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace galata::pipeline {
namespace {

// The unit a parameter path implies, read off the path's own suffix rather than
// declared a second time. ADR-0003 makes the key carry the unit, so the path
// already says it and a study restating it could disagree with the model.
std::string unit_of_parameter(const std::string& path) {
  static const std::pair<const char*, const char*> kSuffixes[] = {
      {"mass_kg", "kg"},
      {"thrust_coefficient_n_s2", "N s^2"},
      {"torque_coefficient_n_m_s2", "N m s^2"},
      {"speed_time_constant_s", "s"},
      {"linear_n_s_m", "N s/m"},
      {"quadratic_n_s2_m2", "N s^2/m^2"},
      {"angular_n_m_s", "N m s"},
  };
  for (const auto& [suffix, unit] : kSuffixes) {
    const std::string text(suffix);
    if (path.size() >= text.size()
        && path.compare(path.size() - text.size(), text.size(), text) == 0) {
      return unit;
    }
    // `drag.linear_n_s_m[2]` carries its unit before the index.
    const std::string indexed = text + "[";
    const auto found = path.find(indexed);
    if (found != std::string::npos) {
      return unit;
    }
  }
  // Refused rather than defaulted to the empty string: a coefficient reaching a
  // provenance record without a unit is the exact failure ADR-0003 exists to
  // stop, and a new parameter path added to the library without a unit here
  // should fail loudly at the first study that fits it.
  throw std::invalid_argument(
      "identify.greybox: no unit is known for the parameter path '" + path
      + "'. Every fitted coefficient is written into a provenance record with its unit, and a "
        "number without one is not a measurement; add the path's unit beside the path");
}

// Every parameter path the model has, so that what a fit did NOT touch can be
// written out in full rather than left as "everything else". A reader asking
// which numbers in an exported file are measurements and which are inherited
// must be able to answer it from the record alone.
std::vector<std::string> every_parameter_path(const model::Quadrotor& model) {
  std::vector<std::string> paths{"mass.mass_kg"};
  for (int r = 0; r < model.rotor_count(); ++r) {
    const std::string rotor = "rotors[" + std::to_string(r) + "].";
    paths.push_back(rotor + "thrust_coefficient_n_s2");
    paths.push_back(rotor + "torque_coefficient_n_m_s2");
    paths.push_back(rotor + "speed_time_constant_s");
  }
  for (const char* axis : {"linear_n_s_m", "quadratic_n_s2_m2", "angular_n_m_s"}) {
    for (int k = 0; k < 3; ++k) {
      paths.push_back(std::string("drag.") + axis + "[" + std::to_string(k) + "]");
    }
  }
  return paths;
}

Eigen::VectorXd numbers_at(const ValuePtr& node, const std::string& what, Eigen::Index expected) {
  const std::vector<ValuePtr>& entries = node->as_list();
  if (static_cast<Eigen::Index>(entries.size()) != expected) {
    std::ostringstream message;
    message << what << " must hold " << expected << " number(s); it holds " << entries.size();
    throw std::invalid_argument(message.str());
  }
  Eigen::VectorXd out(expected);
  for (Eigen::Index k = 0; k < expected; ++k) {
    out(k) = entries[static_cast<std::size_t>(k)]->as_number();
  }
  return out;
}

// Two ways to say where the simulation starts, exactly one of which must be
// taken — the same contract `sim.plant` states, and for the same reason: a trim
// carries its own equilibrium state, a bare declaration does not, and accepting
// both would leave it unsaid which state was integrated.
//
// AND ONE OPTIONAL CORRECTION ON TOP, which a windowed record makes necessary
// rather than convenient. A record cut to [t1, t2) does not begin at the
// equilibrium the whole flight began at, and simulating it from that
// equilibrium scores the model on an initial-condition error rather than on its
// dynamics — the residual is real, large, and about the wrong thing.
// `initial_state_from_record` names, one at a time, which state each channel's
// FIRST sample sets. It is a declaration and not an inference: a state nobody
// names keeps whatever the base declaration gave it, and this refuses to guess
// what a flight log does not measure.
Eigen::VectorXd initial_state_for(const StageContext& context,
                                  const model::Quadrotor& model,
                                  const data::Record& record,
                                  const std::string& capability) {
  const bool from_trim = context.input->get("trim") != nullptr;
  const bool declared = context.input->get("initial_extended_state") != nullptr;
  if (from_trim == declared) {
    throw std::invalid_argument(
        capability
        + ": give exactly one of `trim` (an operating point, which carries its own state) or "
          "`initial_extended_state` (the state declared outright). Both together leaves it "
          "unsaid which state the record is simulated from, and neither leaves it undefined");
  }

  Eigen::VectorXd initial;
  if (from_trim) {
    const auto& trimmed = context.upstream_at("trim").payload_as<HoverTrimArtifact>("hover_trim");
    if (trimmed.point.extended_state.size() != model.extended_state_size()) {
      throw std::invalid_argument(
          capability + ": the trim is an equilibrium of a model of a different width than the "
                       "one being simulated");
    }
    initial = trimmed.point.extended_state;
  } else {
    initial = numbers_at(context.input->get("initial_extended_state"),
                         capability + ": `initial_extended_state`",
                         model.extended_state_size());
  }

  const ValuePtr from_record = context.input->get("initial_state_from_record");
  if (!from_record) {
    return initial;
  }
  const std::vector<std::string> state_names = model.extended_state_names();
  for (const ValuePtr& entry : from_record->as_list()) {
    const std::string state = entry->string_at("state");
    const std::string channel = entry->string_at("channel");
    const auto found = std::find(state_names.begin(), state_names.end(), state);
    if (found == state_names.end()) {
      throw std::invalid_argument(capability + ": `initial_state_from_record` names '" + state
                                  + "', which is not a state of this model");
    }
    const data::Channel* samples = record.find(channel);
    if (samples == nullptr) {
      throw std::invalid_argument(capability + ": `initial_state_from_record` reads channel '"
                                  + channel + "', which this record does not have");
    }
    // Bound rather than cast: the iterator difference is already `ptrdiff_t`,
    // which is what `Eigen::Index` is, so an explicit cast is one GCC refuses
    // under -Wuseless-cast. The implicit conversion keeps the intent and would
    // still warn under -Wconversion if the two types ever stopped matching.
    const Eigen::Index component = found - state_names.begin();
    initial(component) = samples->samples.front();
  }
  // The quaternion may have been set component by component from four channels
  // that were themselves recorded to finite precision, so it is renormalised
  // here — the same projection the integrator applies at every step, applied
  // once at the start rather than left for the first step to do silently.
  model.project(initial);
  return initial;
}

Eigen::VectorXd initial_vehicle_state_for(const StageContext& context,
                                          const VehicleArtifact& subject,
                                          const data::Record& record,
                                          const std::string& capability) {
  const model::VehicleModel& model = *subject.model;
  const bool from_trim = context.input->get("trim") != nullptr;
  const bool declared = context.input->get("initial_extended_state") != nullptr;
  if (from_trim == declared) {
    throw std::invalid_argument(
        capability
        + ": give exactly one of trim or initial_extended_state; the starting state must "
          "be explicit and unambiguous");
  }

  Eigen::VectorXd initial;
  if (from_trim) {
    const auto& trimmed =
        context.upstream_at("trim").payload_as<VehicleTrimArtifact>("vehicle_trim");
    if (trimmed.model.get() != subject.model.get()) {
      throw std::invalid_argument(capability
                                  + ": the trim belongs to a different vehicle artifact");
    }
    if (trimmed.extended_state.size() != model.extended_state_size()) {
      throw std::invalid_argument(capability
                                  + ": the trim state width does not match the vehicle model");
    }
    initial = trimmed.extended_state;
  } else {
    initial = numbers_at(context.input->get("initial_extended_state"),
                         capability + ": initial_extended_state",
                         model.extended_state_size());
  }

  const ValuePtr from_record = context.input->get("initial_state_from_record");
  if (from_record) {
    const std::vector<std::string> state_names = model.state_names();
    for (const ValuePtr& entry : from_record->as_list()) {
      const std::string state = entry->string_at("state");
      const std::string channel = entry->string_at("channel");
      const auto found = std::find(state_names.begin(), state_names.end(), state);
      if (found == state_names.end()) {
        throw std::invalid_argument(capability + ": initial_state_from_record names '" + state
                                    + "', which is not a state of this model");
      }
      const data::Channel* samples = record.find(channel);
      if (samples == nullptr || samples->samples.empty()) {
        throw std::invalid_argument(capability + ": initial_state_from_record reads channel '"
                                    + channel + "', which is absent or empty");
      }
      const Eigen::Index index = found - state_names.begin();
      initial(index) = samples->samples.front();
    }
  }
  model::VehicleModel::project(initial);
  return initial;
}

model::Environment validation_environment_for(const StageContext& context) {
  if (context.input->get("trim")) {
    const auto& trimmed =
        context.upstream_at("trim").payload_as<VehicleTrimArtifact>("vehicle_trim");
    return trimmed.environment;
  }
  model::Environment environment = model::Environment::at_geometric_altitude(
      context.input->number_at("altitude_m", 0.0), context.input->number_at("delta_isa_k", 0.0));
  if (const ValuePtr wind = context.input->get("wind_ned_m_s")) {
    environment.wind_ned_m_s = numbers_at(wind, "identify.validate.vehicle: wind_ned_m_s", 3);
  }
  return environment;
}

const QuadrotorArtifact& quadrotor_input(const StageContext& context, const char* key) {
  return context.upstream_at(key).payload_as<QuadrotorArtifact>("quadrotor");
}

std::string scientific(double value, int digits = 3) {
  std::ostringstream out;
  out << std::scientific << std::setprecision(digits) << value;
  return out.str();
}

struct ValidationAcceptance {
  std::vector<identify::ValidationCriterion> criteria;
  identify::FlightTestEvidence flight_test_evidence;
  std::string campaign_manifest_path;
};

std::optional<ValidationAcceptance> validation_acceptance_from_input(const StageContext& context) {
  const ValuePtr acceptance = context.input->get("acceptance");
  if (!acceptance) {
    return std::nullopt;
  }
  if (acceptance->kind() != Value::Kind::Map) {
    throw std::invalid_argument("identify.validate.vehicle: acceptance must be a map");
  }
  const ValuePtr criteria = acceptance->get("outputs");
  if (!criteria) {
    throw std::invalid_argument(
        "identify.validate.vehicle: acceptance.outputs is required when acceptance is given");
  }
  ValidationAcceptance parsed;
  for (const ValuePtr& entry : criteria->as_list()) {
    if (!entry || entry->kind() != Value::Kind::Map) {
      throw std::invalid_argument(
          "identify.validate.vehicle: acceptance.outputs entries must be maps");
    }
    identify::ValidationCriterion criterion;
    criterion.channel = entry->string_at("channel");
    criterion.state_name = entry->string_at("state");
    criterion.max_rmse_is_defined = entry->get("max_rmse") != nullptr;
    if (criterion.max_rmse_is_defined) {
      criterion.max_rmse = entry->number_at("max_rmse");
    }
    criterion.max_absolute_error_is_defined = entry->get("max_absolute_error") != nullptr;
    if (criterion.max_absolute_error_is_defined) {
      criterion.max_absolute_error = entry->number_at("max_absolute_error");
    }
    criterion.minimum_fit_fraction_is_defined = entry->get("min_fit_fraction") != nullptr;
    if (criterion.minimum_fit_fraction_is_defined) {
      criterion.minimum_fit_fraction = entry->number_at("min_fit_fraction");
    }
    if (!criterion.max_rmse_is_defined && !criterion.max_absolute_error_is_defined
        && !criterion.minimum_fit_fraction_is_defined) {
      throw std::invalid_argument(
          "identify.validate.vehicle: every acceptance output needs at least one numerical budget");
    }
    parsed.criteria.push_back(std::move(criterion));
  }

  const ValuePtr evidence = acceptance->get("evidence");
  if (evidence) {
    if (evidence->kind() != Value::Kind::Map) {
      throw std::invalid_argument("identify.validate.vehicle: acceptance.evidence must be a map");
    }
    parsed.flight_test_evidence.evidence_class = evidence->string_at("evidence_class", "");
    parsed.flight_test_evidence.aircraft_id = evidence->string_at("aircraft_id", "");
    parsed.flight_test_evidence.aircraft_configuration =
        evidence->string_at("aircraft_configuration", "");
    parsed.flight_test_evidence.test_plan_id = evidence->string_at("test_plan_id", "");
    parsed.flight_test_evidence.calibration_manifest_sha256 =
        evidence->string_at("calibration_manifest_sha256", "");
    parsed.flight_test_evidence.configuration_manifest_sha256 =
        evidence->string_at("configuration_manifest_sha256", "");
    parsed.flight_test_evidence.reviewer_id = evidence->string_at("reviewer_id", "");
    parsed.flight_test_evidence.reviewer_attestation_sha256 =
        evidence->string_at("reviewer_attestation_sha256", "");
    parsed.flight_test_evidence.safety_review_complete =
        evidence->bool_at("safety_review_complete", false);
  }
  if (const ValuePtr campaign_manifest = acceptance->get("campaign_manifest")) {
    parsed.campaign_manifest_path = campaign_manifest->as_string();
    if (parsed.campaign_manifest_path.empty()) {
      throw std::invalid_argument(
          "identify.validate.vehicle: acceptance.campaign_manifest must not be empty");
    }
  }
  return parsed;
}

// --- identify.greybox ------------------------------------------------------

Artifact greybox_capability(const StageContext& context) {
  const QuadrotorArtifact& base = quadrotor_input(context, "model");
  const Artifact& record_artifact = context.upstream_at("record");
  const auto& record = record_artifact.payload_as<data::Record>("measured_record");

  identify::GreyboxRequest request;
  request.step_s = context.input->number_at("step_s");
  request.iterations = context.input->integer_at("iterations", 40);
  request.identifiability_ratio = context.input->number_at("identifiability_ratio", 1e-6);
  request.initial_extended_state =
      initial_state_for(context, base.model, record, "identify.greybox");

  const ValuePtr parameters = context.input->get("parameters");
  if (!parameters) {
    throw std::invalid_argument(
        "identify.greybox: `parameters` is required. Nothing is "
        "discovered here: a parameter the study did not name is held "
        "at the value the model file gave it");
  }
  for (const ValuePtr& entry : parameters->as_list()) {
    identify::Parameter parameter;
    parameter.path = entry->string_at("path");
    // No defaults for any of the three. A bound the study did not choose is a
    // bound the software chose, and it is the software's choice that then
    // decides whether an estimate rests on a limit.
    parameter.lower = entry->number_at("lower");
    parameter.upper = entry->number_at("upper");
    parameter.initial = entry->number_at("initial");
    request.parameters.push_back(std::move(parameter));
  }

  const ValuePtr outputs = context.input->get("outputs");
  if (!outputs) {
    throw std::invalid_argument("identify.greybox: `outputs` is required");
  }
  for (const ValuePtr& entry : outputs->as_list()) {
    identify::OutputMatch match;
    match.channel = entry->string_at("channel");
    match.state_name = entry->string_at("state");
    // Required, and required for a reason the summary repeats: without it,
    // channels in different units are added together as though a metre and a
    // radian per second were the same size.
    match.scale = entry->number_at("scale");
    request.outputs.push_back(std::move(match));
  }

  const ValuePtr commands = context.input->get("command_channels");
  if (!commands) {
    throw std::invalid_argument(
        "identify.greybox: `command_channels` is required, one per "
        "rotor, in rotor order");
  }
  for (const ValuePtr& entry : commands->as_list()) {
    request.command_channels.push_back(entry->as_string());
  }

  const identify::GreyboxResult fit = identify::fit_greybox(base.model, record, request);

  auto provenance = std::make_shared<FittedModelProvenance>();
  provenance->base_model_path = base.identity.path;
  provenance->base_model_sha256 = base.identity.sha256;
  provenance->base_model_description = base.model.description;
  if (base.identity.is_fitted()) {
    // A fit of a fit is not refused — refitting a subset against a second record
    // is legitimate — but the base must still be identified, and a fitted base
    // has no file digest to be identified by. Say so rather than writing an
    // empty field a reader would take for a missing one.
    provenance->base_model_sha256 = "none: the base model was itself produced by a fit";
  }

  provenance->estimation_record_path = record.source_path;
  provenance->estimation_record_sha256 = record.source_sha256;
  provenance->estimation_record_is_window = record.is_window;
  provenance->estimation_window_start_s = record.window_start_s;
  provenance->estimation_window_end_s = record.window_end_s;
  provenance->estimation_first_sample_s = record.first_time_s();
  provenance->estimation_last_sample_s = record.last_time_s();
  provenance->estimation_sample_count = static_cast<int>(record.sample_count());

  for (std::size_t p = 0; p < fit.names.size(); ++p) {
    FittedParameter parameter;
    parameter.path = fit.names[p];
    parameter.unit = unit_of_parameter(parameter.path);
    parameter.lower = request.parameters[p].lower;
    parameter.upper = request.parameters[p].upper;
    parameter.initial = request.parameters[p].initial;
    parameter.value = fit.value(static_cast<Eigen::Index>(p));
    parameter.standard_error = fit.standard_error(static_cast<Eigen::Index>(p));
    parameter.standard_error_is_estimable = fit.uncertainty_is_estimable;
    parameter.at_bound = fit.at_bound[p];
    provenance->fitted.push_back(std::move(parameter));
  }
  for (const std::string& path : every_parameter_path(base.model)) {
    if (std::find(fit.names.begin(), fit.names.end(), path) == fit.names.end()) {
      provenance->preserved_parameter_paths.push_back(path);
    }
  }

  provenance->objective_definition =
      "sum over samples and declared outputs of ((predicted state - observed channel) / scale)^2, "
      "the model simulated from the declared initial state with the record's own commands held "
      "zero-order between samples. The scales are the study's; every residual figure here is "
      "in those scaled units";
  for (const identify::OutputMatch& match : request.outputs) {
    provenance->output_matches.push_back(match.channel + " -> " + match.state_name + " (scale "
                                         + scientific(match.scale) + ")");
  }
  provenance->command_channels = request.command_channels;

  provenance->objective = fit.objective;
  provenance->residual_rms = fit.residual_rms;
  provenance->residual_count = fit.residual_count;
  provenance->iterations_declared = fit.iterations_declared;
  provenance->iterations_run = fit.iterations_run;
  provenance->stop_reason = identify::to_string(fit.stop_reason);
  provenance->objective_improved = fit.objective_improved;
  provenance->initial_objective = fit.initial_objective;
  provenance->accepted_steps = fit.accepted_steps;
  provenance->last_step_norm = fit.convergence.last_step_norm;
  provenance->last_accepted_iteration = fit.convergence.last_accepted_iteration;
  provenance->gradient_infinity_norm = fit.convergence.gradient_infinity_norm;
  provenance->gradient_over_bound_span_infinity_norm =
      fit.convergence.gradient_over_bound_span_infinity_norm;
  provenance->jacobian_condition_number = fit.jacobian_condition_number;
  provenance->identifiability_ratio = fit.identifiability_ratio;
  provenance->uncertainty_is_estimable = fit.uncertainty_is_estimable;
  provenance->uncertainty_assumptions = fit.note;
  provenance->step_s = request.step_s;

  QuadrotorArtifact payload;
  payload.model = fit.fitted_model;
  payload.identity.origin = "fit";
  payload.identity.summary = fit.fitted_model.description;
  payload.identity.fit = provenance;

  std::ostringstream summary;
  summary << fit.names.size() << " parameter(s) from " << record.sample_count()
          << " sample(s); residual RMS " << scientific(fit.residual_rms) << " (scaled)";
  // What a reader most needs is not "it ran" but "did it move, and does the
  // record constrain what moved". Both go in the headline rather than in a
  // field somebody has to look up.
  if (!fit.objective_improved) {
    summary << "; THE OBJECTIVE DID NOT IMPROVE on the declared starting point";
  } else {
    summary << "; objective " << scientific(fit.initial_objective) << " -> "
            << scientific(fit.objective) << " over " << fit.accepted_steps << " accepted step(s)";
  }
  summary << "; sensitivity condition number " << scientific(fit.jacobian_condition_number);
  summary << (fit.uncertainty_is_estimable ? "; standard errors reported"
                                           : "; NO uncertainty (" + fit.note + ")");
  const auto resting = static_cast<int>(std::count(fit.at_bound.begin(), fit.at_bound.end(), true));
  if (resting > 0) {
    summary << "; " << resting
            << " parameter(s) RESTING ON A BOUND, which is not an interior "
               "estimate";
  }

  Artifact artifact;
  artifact.kind = "quadrotor";
  artifact.summary = summary.str();
  artifact.payload = std::move(payload);
  return artifact;
}

// --- identify.validate -----------------------------------------------------

Artifact validate_capability(const StageContext& context) {
  const QuadrotorArtifact& subject = quadrotor_input(context, "model");
  const auto& record = context.upstream_at("record").payload_as<data::Record>("measured_record");

  identify::ValidationRequest request;
  request.step_s = context.input->number_at("step_s");
  request.initial_extended_state =
      initial_state_for(context, subject.model, record, "identify.validate");
  request.caller_declares_different_data = context.input->bool_at("declare_different_data", false);

  const ValuePtr outputs = context.input->get("outputs");
  if (!outputs) {
    throw std::invalid_argument("identify.validate: `outputs` is required");
  }
  for (const ValuePtr& entry : outputs->as_list()) {
    request.outputs.push_back({entry->string_at("channel"), entry->string_at("state")});
  }
  const ValuePtr commands = context.input->get("command_channels");
  if (!commands) {
    throw std::invalid_argument("identify.validate: `command_channels` is required");
  }
  for (const ValuePtr& entry : commands->as_list()) {
    request.command_channels.push_back(entry->as_string());
  }

  // WHERE THE ESTIMATION IDENTITY COMES FROM, in the order that prefers what
  // was recorded over what was typed. A fitted model carries the digest of the
  // record it was fitted to; a caller retyping that digest beside it can retype
  // it wrong, and the disagreement is refused rather than resolved.
  const std::string declared_digest = context.input->string_at("estimation_record_sha256", "");
  const data::Record* estimation = nullptr;
  if (context.input->get("estimation_record") != nullptr) {
    estimation =
        &context.upstream_at("estimation_record").payload_as<data::Record>("measured_record");
    request.estimation_record = estimation;
  }

  std::string provenance_digest;
  std::string provenance_path;
  const FittedModelProvenance* recorded_fit = nullptr;
  if (subject.identity.is_fitted() && subject.identity.fit) {
    recorded_fit = subject.identity.fit.get();
    provenance_digest = recorded_fit->estimation_record_sha256;
    provenance_path = recorded_fit->estimation_record_path;
  }

  if (!provenance_digest.empty()) {
    if (!declared_digest.empty() && declared_digest != provenance_digest) {
      throw std::invalid_argument(
          "identify.validate: the study declares the estimation record's digest as '"
          + declared_digest + "', and the model's own fit provenance records '" + provenance_digest
          + "'. The model knows what it was fitted to; a caller who disagrees with it is "
            "describing a different run, and which one this is is not this stage's to decide");
    }
    // A DIGEST IS NOT ENOUGH HERE, for the same reason it is not enough to
    // establish independence: a window keeps the digest of the file it was cut
    // from, so the whole import and any window of it are indistinguishable by
    // digest alone. The lineage the fit recorded is compared entry by entry.
    if (estimation != nullptr) {
      const bool same_bytes = estimation->source_sha256 == provenance_digest;
      const bool same_cut =
          estimation->is_window == recorded_fit->estimation_record_is_window
          && estimation->window_start_s == recorded_fit->estimation_window_start_s
          && estimation->window_end_s == recorded_fit->estimation_window_end_s
          && static_cast<int>(estimation->sample_count()) == recorded_fit->estimation_sample_count;
      if (!same_bytes || !same_cut) {
        std::ostringstream message;
        message << "identify.validate: the record wired in as `estimation_record` is not the "
                   "record this model was fitted to. The fit recorded ";
        if (recorded_fit->estimation_record_is_window) {
          message << "a window [" << recorded_fit->estimation_window_start_s << ", "
                  << recorded_fit->estimation_window_end_s << ") s of ";
        }
        message << "'" << provenance_path << "' (sha256 " << provenance_digest << ") with "
                << recorded_fit->estimation_sample_count << " sample(s); this stage was given ";
        if (estimation->is_window) {
          message << "a window [" << estimation->window_start_s << ", " << estimation->window_end_s
                  << ") s of ";
        } else {
          message << "the whole of ";
        }
        message << "'" << estimation->source_path << "' (sha256 " << estimation->source_sha256
                << ") with " << estimation->sample_count()
                << " sample(s). Supplying the wrong record as the training data would make "
                   "every independence check below an answer to the wrong question";
        throw std::invalid_argument(message.str());
      }
    }
    request.estimation_record_sha256 = provenance_digest;
  } else {
    request.estimation_record_sha256 = declared_digest;
  }

  const identify::ValidationResult result =
      identify::validate_model(subject.model, record, request);

  std::ostringstream summary;
  summary << result.outputs.size() << " output(s) over " << result.sample_count << " sample(s); ";
  // The label first, in words, because it is the difference between a
  // diagnostic and a claim and a reader skimming must not have to hunt for it.
  summary << "separation: " << identify::to_string(result.separation);
  if (result.caller_declaration_was_contradicted) {
    summary << " (THE STUDY'S OWN DECLARATION WAS CONTRADICTED)";
  }
  for (const identify::ValidationOutput& out : result.outputs) {
    summary << "; " << out.channel << " RMSE " << scientific(out.rmse);
    if (out.fit_fraction_is_defined) {
      summary << ", fit " << std::fixed << std::setprecision(3) << out.fit_fraction;
      summary.unsetf(std::ios::floatfield);
    } else {
      summary << ", no fit fraction";
    }
  }

  Artifact artifact;
  artifact.kind = "validation";
  artifact.summary = summary.str();
  artifact.payload = ValidationArtifact{result, subject.identity, std::nullopt, std::nullopt};
  return artifact;
}

Artifact validate_vehicle_capability(const StageContext& context) {
  const auto& subject = context.upstream_at("model").payload_as<VehicleArtifact>("vehicle_model");
  if (!subject.model) {
    throw std::invalid_argument(
        "identify.validate.vehicle: the vehicle artifact does not carry a model");
  }
  const auto& record = context.upstream_at("record").payload_as<data::Record>("measured_record");

  identify::VehicleValidationRequest request;
  request.step_s = context.input->number_at("step_s");
  request.initial_extended_state =
      initial_vehicle_state_for(context, subject, record, "identify.validate.vehicle");
  request.environment = validation_environment_for(context);
  request.caller_declares_different_data = context.input->bool_at("declare_different_data", false);

  const ValuePtr outputs = context.input->get("outputs");
  if (!outputs) {
    throw std::invalid_argument("identify.validate.vehicle: outputs is required");
  }
  for (const ValuePtr& entry : outputs->as_list()) {
    request.outputs.push_back({entry->string_at("channel"), entry->string_at("state")});
  }

  const ValuePtr commands = context.input->get("command_channels");
  if (!commands) {
    throw std::invalid_argument("identify.validate.vehicle: command_channels is required");
  }
  for (const ValuePtr& entry : commands->as_list()) {
    request.command_channels.push_back(entry->as_string());
  }

  const std::string declared_digest = context.input->string_at("estimation_record_sha256", "");
  if (context.input->get("estimation_record") != nullptr) {
    request.estimation_record =
        &context.upstream_at("estimation_record").payload_as<data::Record>("measured_record");
  }
  request.estimation_record_sha256 = declared_digest;

  const identify::ValidationResult result =
      identify::validate_vehicle_model(*subject.model, record, request);
  std::optional<ValidationAcceptance> acceptance = validation_acceptance_from_input(context);
  if (acceptance && !acceptance->campaign_manifest_path.empty()) {
    const std::string manifest_path =
        context.resolve_input_path(acceptance->campaign_manifest_path);
    const std::string manifest =
        context.read_input(acceptance->campaign_manifest_path, 2U * 1024U * 1024U);
    const identify::FlightTestCampaign campaign = identify::parse_flight_test_campaign(manifest);
    const std::filesystem::path manifest_file(manifest_path);
    const std::filesystem::path package_root =
        manifest_file.has_parent_path() ? manifest_file.parent_path() : std::filesystem::path(".");
    (void)identify::verify_flight_test_campaign(campaign, package_root);
    // The campaign verifier reads directly from its package root so it can
    // enforce the package's own symlink and containment rules. Read the same
    // verified bytes through the run-scoped file ledger as well, so the run
    // manifest retains every controlled evidence input rather than only the
    // campaign manifest and flight record.
    constexpr std::size_t kMaximumCampaignFileBytes = 256U * 1024U * 1024U;
    for (const identify::FlightTestCampaignFile& file : campaign.files) {
      (void)context.read_input((package_root / file.relative_path).string(),
                               kMaximumCampaignFileBytes);
    }
    const auto flight_record = std::find_if(
        campaign.files.begin(),
        campaign.files.end(),
        [](const identify::FlightTestCampaignFile& file) { return file.role == "flight_record"; });
    if (flight_record == campaign.files.end() || record.source_sha256 != flight_record->sha256) {
      throw std::invalid_argument(
          "identify.validate.vehicle: acceptance.campaign_manifest flight_record does not "
          "match the validation record bytes");
    }
    acceptance->flight_test_evidence = campaign.evidence;
    acceptance->flight_test_evidence.campaign_manifest_sha256 = campaign.manifest_sha256;
    acceptance->flight_test_evidence.campaign_package_verified = true;
  }
  std::optional<identify::ValidationGate> gate;
  std::optional<identify::FlightTestGate> flight_test_gate;
  if (acceptance) {
    gate = identify::evaluate_validation_gate(result, acceptance->criteria);
    flight_test_gate = identify::evaluate_flight_test_gate(
        result, acceptance->criteria, acceptance->flight_test_evidence);
  }

  std::ostringstream summary;
  summary << subject.vehicle_kind << ": " << result.outputs.size() << " output(s) over "
          << result.sample_count
          << " sample(s); separation: " << identify::to_string(result.separation);
  if (result.caller_declaration_was_contradicted) {
    summary << " (THE STUDY'S OWN DECLARATION WAS CONTRADICTED)";
  }
  for (const identify::ValidationOutput& output : result.outputs) {
    summary << "; " << output.channel << " RMSE " << scientific(output.rmse);
    if (output.fit_fraction_is_defined) {
      summary << ", fit " << std::fixed << std::setprecision(3) << output.fit_fraction;
      summary.unsetf(std::ios::floatfield);
    } else {
      summary << ", no fit fraction";
    }
  }
  if (gate) {
    summary << "; acceptance gate: " << identify::to_string(gate->status) << " ("
            << gate->passed_count << "/" << gate->criteria_count << " criteria)";
  }
  if (flight_test_gate) {
    summary << "; flight-test evidence gate: " << identify::to_string(flight_test_gate->status);
  }

  Artifact artifact;
  artifact.kind = "validation";
  artifact.summary = summary.str();
  artifact.payload = ValidationArtifact{result, subject.identity, gate, flight_test_gate};
  return artifact;
}

// --- identify.static_fit ---------------------------------------------------
//
// The library API does the work; this is the study-facing shape of it. Every
// term, its power, its name and its unit are declared, because a coefficient
// whose meaning was chosen by software is not a measurement of anything.
Artifact static_fit_capability(const StageContext& context) {
  const auto& record = context.upstream_at("record").payload_as<data::Record>("measured_record");

  identify::StaticFitRequest request;
  request.response_channel = context.input->string_at("response");
  request.intercept = context.input->bool_at("intercept", false);
  request.intercept_name = context.input->string_at("intercept_name", "intercept");
  request.intercept_unit = context.input->string_at("intercept_unit", "");
  request.maximum_condition_number = context.input->number_at("maximum_condition_number", 1e8);

  const ValuePtr terms = context.input->get("terms");
  if (!terms) {
    throw std::invalid_argument("identify.static_fit: `terms` is required");
  }
  for (const ValuePtr& entry : terms->as_list()) {
    identify::Term term;
    term.channel = entry->string_at("channel");
    term.power = entry->number_at("power");
    term.name = entry->string_at("name");
    term.unit = entry->string_at("unit");
    request.terms.push_back(std::move(term));
  }

  const identify::StaticFit fit = identify::fit_static(record, request);

  std::ostringstream summary;
  summary << fit.coefficients.size() << " coefficient(s) from " << fit.sample_count
          << " sample(s); residual RMS " << scientific(fit.residual_rms);
  // Whether an uncertainty exists is part of the headline, not a detail: a
  // reader who skims the summary must not come away thinking one was reported
  // when none could be.
  summary << (fit.uncertainty_is_estimable ? "; standard errors reported"
                                           : "; NO uncertainty (" + fit.uncertainty_note + ")");

  Artifact artifact;
  artifact.kind = "static_fit";
  artifact.summary = summary.str();
  artifact.payload = fit;
  return artifact;
}

}  // namespace

void register_identify_capabilities(Registry& registry) {
  registry.add(Capability{
      "identify.static_fit",
      "Fit a response that is linear in declared terms — a bench map — reporting the range it "
      "was measured over and an uncertainty only where the data supports one",
      "static_fit",
      Capability::State::ImplementedUnvalidated,
      static_fit_capability,
      {"record",
       "response",
       "terms",
       "intercept",
       "intercept_name",
       "intercept_unit",
       "maximum_condition_number"}});

  registry.add(Capability{
      "identify.greybox",
      "Fit a declared subset of a multirotor's parameters to a measured record by simulating "
      "the nonlinear plant, refusing a parameter the data does not constrain",
      "quadrotor",
      Capability::State::ImplementedUnvalidated,
      greybox_capability,
      {"model",
       "record",
       "parameters",
       "outputs",
       "command_channels",
       "trim",
       "initial_extended_state",
       "initial_state_from_record",
       "step_s",
       "iterations",
       "identifiability_ratio"}});

  registry.add(Capability{
      "identify.validate",
      "Run an identified model on another record and report per-output error, fit fraction and "
      "residual structure, with separation from the training data classified on stated grounds "
      "rather than inferred from a digest",
      "validation",
      Capability::State::ImplementedUnvalidated,
      validate_capability,
      {"model",
       "record",
       "estimation_record",
       "estimation_record_sha256",
       "declare_different_data",
       "outputs",
       "command_channels",
       "trim",
       "initial_extended_state",
       "initial_state_from_record",
       "step_s"}});

  registry.add(Capability{
      "identify.validate.vehicle",
      "Run a built-in fixed-wing, multirotor or helicopter model against a measured record with "
      "explicit state, environment, timing and held-out-data declarations; report numerical "
      "prediction error without making an airworthiness claim",
      "validation",
      Capability::State::ImplementedUnvalidated,
      validate_vehicle_capability,
      {"model",
       "record",
       "estimation_record",
       "estimation_record_sha256",
       "declare_different_data",
       "outputs",
       "command_channels",
       "trim",
       "initial_extended_state",
       "initial_state_from_record",
       "step_s",
       "altitude_m",
       "delta_isa_k",
       "wind_ned_m_s",
       "acceptance"}});
}

}  // namespace galata::pipeline
