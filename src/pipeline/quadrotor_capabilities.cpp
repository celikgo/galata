// SPDX-License-Identifier: Apache-2.0
//
// Study adapters for the multirotor packages of RFC-0002: hover trim, the
// model-generic linearisation, and the named matrix export that closes the
// round trip with `model.linear.statespace`.
//
// The numerics are in galata::trim and galata::linearize. What lives here is
// the schema — which keys a stage may set, what they mean, and which unit each
// carries in its own name (ADR-0003) — and the evidence, because a matrix that
// reaches a file without the operating point it was taken about is a number
// with no routine behind it (charter rule 9).

#include "galata/linearize/extended.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/trim/hover.hpp"
#include "galata/units.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {

// --- schema helpers --------------------------------------------------------

// A three-component NED vector, written as a list of exactly three numbers.
// Absent means zero; the wrong length is an error rather than a pad or a
// truncation, because a two-entry wind is a caller who thinks this vehicle
// flies in a plane.
Eigen::Vector3d vector3_at(const StageContext& context, const std::string& key) {
  const ValuePtr value = context.input->get(key);
  if (!value || value->kind() == Value::Kind::Null) {
    return Eigen::Vector3d::Zero();
  }
  const std::vector<ValuePtr>& items = value->as_list();
  if (items.size() != 3) {
    std::ostringstream message;
    message << "'" << key << "' must be a list of exactly three numbers, north east down; "
            << items.size() << " were given";
    throw std::invalid_argument(message.str());
  }
  Eigen::Vector3d out;
  for (std::size_t i = 0; i < 3; ++i) {
    out(static_cast<Eigen::Index>(i)) = items[i]->as_number();
  }
  return out;
}

int positive_integer_at(const StageContext& context, const std::string& key, int fallback) {
  const double number = context.input->number_at(key, static_cast<double>(fallback));
  if (!std::isfinite(number) || number < 1.0 || number > std::numeric_limits<int>::max()
      || std::floor(number) != number) {
    throw std::invalid_argument("'" + key + "' must be a positive whole number");
  }
  return static_cast<int>(number);
}

std::ostringstream evidence_stream() {
  std::ostringstream out;
  // ADR-0004: the classic locale, and the shortest precision that survives a
  // text round trip. An evidence file a comma-decimal machine writes must be
  // one every other machine can read.
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  return out;
}

void write_vector(std::ostream& out, const std::string& key, const Eigen::VectorXd& values) {
  out << key << ": [";
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << values(i);
  }
  out << "]\n";
}

// --- trim.hover ------------------------------------------------------------

Artifact trim_hover_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("quadrotor");
  const auto& model = upstream.payload_as<model::Quadrotor>("quadrotor");

  trim::HoverTrimRequest request;
  request.altitude_m = context.input->number_at("altitude_m", 0.0);
  request.wind_ned_m_s = vector3_at(context, "wind_ned_m_s");
  request.ground_velocity_ned_m_s = vector3_at(context, "ground_velocity_ned_m_s");
  // Degrees at the boundary and nowhere below it, converted by units.hpp alone
  // (ADR-0003). `trim.level` states its flight path angle the same way.
  request.heading_rad = units::degrees_to_radians(context.input->number_at("heading_deg", 0.0));
  request.battery_state_of_charge = context.input->number_at("battery_state_of_charge", 1.0);
  request.residual_tolerance = context.input->number_at("tolerance", 1e-10);
  request.iterations = positive_integer_at(context, "iterations", 40);

  HoverTrimArtifact trimmed{trim::trim_hover(model, request), model};

  std::ostringstream summary;
  summary << std::fixed << std::setprecision(3) << "roll "
          << units::radians_to_degrees(trimmed.point.roll_rad) << " deg, pitch "
          << units::radians_to_degrees(trimmed.point.pitch_rad) << " deg, airspeed "
          << trimmed.point.airspeed_m_s << " m/s; smallest rotor margin "
          << (100.0 * trimmed.point.smallest_rotor_margin_fraction) << " %; residual "
          << std::scientific << std::setprecision(1) << trimmed.point.residual_norm << " after "
          << trimmed.point.newton_iterations << " iterations";

  Artifact artifact;
  artifact.kind = "hover_trim";
  artifact.summary = summary.str();
  artifact.payload = trimmed;
  return artifact;
}

// --- linearize.extended ----------------------------------------------------

linearize::OutputKind output_kind_from_name(const std::string& name) {
  if (name == "specific_force") {
    return linearize::OutputKind::BodySpecificForce;
  }
  if (name == "body_rates") {
    return linearize::OutputKind::BodyRates;
  }
  if (name == "position_ned") {
    return linearize::OutputKind::PositionNed;
  }
  if (name == "altitude") {
    return linearize::OutputKind::Altitude;
  }
  if (name == "ground_velocity_ned") {
    return linearize::OutputKind::GroundVelocityNed;
  }
  throw std::invalid_argument(
      "'outputs' contains '" + name
      + "'; it must be one of specific_force, body_rates, position_ned, altitude, "
        "ground_velocity_ned");
}

std::vector<linearize::OutputKind> outputs_from(const StageContext& context) {
  const ValuePtr value = context.input->get("outputs");
  if (!value || value->kind() == Value::Kind::Null) {
    // The full observation model. Chosen as the default rather than C = I
    // because this programme's sensors do not measure the states, and a caller
    // who says nothing should get the honest set rather than the flattering one.
    return {linearize::OutputKind::BodySpecificForce,
            linearize::OutputKind::BodyRates,
            linearize::OutputKind::PositionNed,
            linearize::OutputKind::Altitude,
            linearize::OutputKind::GroundVelocityNed};
  }
  std::vector<linearize::OutputKind> kinds;
  for (const ValuePtr& item : value->as_list()) {
    kinds.push_back(output_kind_from_name(item->as_string()));
  }
  if (kinds.empty()) {
    throw std::invalid_argument("'outputs' is an empty list; omit the key for the full set");
  }
  return kinds;
}

std::string evidence_for(const trim::HoverTrim& point,
                         const linearize::ExtendedLinearisation& linearisation,
                         const std::string& stage_id) {
  std::ostringstream out = evidence_stream();
  out << "schema: galata.operating-point.v1\n";
  out << "stage: \"" << stage_id << "\"\n";
  out << "# The point the matrices beside this file were taken about, and every\n";
  out << "# diagnostic of the taking. A matrix without this is a number with no\n";
  out << "# routine behind it.\n";
  out << "trim:\n";
  out << "  altitude_m: " << point.altitude_m << "\n";
  out << "  roll_rad: " << point.roll_rad << "\n";
  out << "  pitch_rad: " << point.pitch_rad << "\n";
  out << "  yaw_rad: " << point.yaw_rad << "\n";
  out << "  airspeed_m_s: " << point.airspeed_m_s << "\n";
  out << "  battery_state_of_charge: " << point.battery_state_of_charge << "\n";
  out << "  residual_norm: " << point.residual_norm << "\n";
  out << "  residual_tolerance: " << point.residual_tolerance << "\n";
  out << "  newton_iterations: " << point.newton_iterations << "\n";
  out << "  jacobian_condition_number: " << point.jacobian_condition_number << "\n";
  out << "  smallest_rotor_margin_fraction: " << point.smallest_rotor_margin_fraction << "\n";
  out << "  ";
  write_vector(out, "extended_state", point.extended_state);
  out << "  ";
  write_vector(out, "command_rad_s", point.command_rad_s);
  out << "linearisation:\n";
  out << "  equilibrium_residual_norm: " << linearisation.equilibrium_residual_norm << "\n";
  out << "  equilibrium_tolerance: " << linearisation.equilibrium_tolerance << "\n";
  out << "  worst_relative_truncation: " << linearisation.worst_relative_truncation << "\n";
  out << "  worst_relative_truncation_a: " << linearisation.worst_relative_truncation_a << "\n";
  out << "  worst_relative_truncation_b: " << linearisation.worst_relative_truncation_b << "\n";
  out << "  worst_relative_truncation_c: " << linearisation.worst_relative_truncation_c << "\n";
  out << "  worst_relative_truncation_d: " << linearisation.worst_relative_truncation_d << "\n";
  out << "  ";
  write_vector(out, "state_steps", linearisation.state_steps);
  out << "  ";
  write_vector(out, "input_steps", linearisation.input_steps);
  out << "  frozen_states:\n";
  if (linearisation.frozen_state_names.empty()) {
    out << "    []\n";
  }
  for (std::size_t i = 0; i < linearisation.frozen_state_names.size(); ++i) {
    // The rate that was declared away, reported so the declaration is auditable.
    out << "    - name: \"" << linearisation.frozen_state_names[i] << "\"\n";
    out << "      declared_zero_actual_rate: " << linearisation.frozen_state_rates[i] << "\n";
  }
  return out.str();
}

Artifact linearize_extended_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("trim");
  const auto& trimmed = upstream.payload_as<HoverTrimArtifact>("hover_trim");
  const model::Quadrotor& model = trimmed.model;
  const trim::HoverTrim& point = trimmed.point;

  const bool wind_inputs = context.input->bool_at("wind_inputs", true);
  const bool freeze_battery = context.input->bool_at("freeze_battery", true);

  linearize::ExtendedLinearisationOptions options;
  options.appended_state_names = model.extended_state_names();
  options.appended_state_names.erase(options.appended_state_names.begin(),
                                     options.appended_state_names.begin() + core::kStateSize);
  options.input_names = model.input_names();
  if (wind_inputs) {
    options.wind_input_offset = static_cast<int>(options.input_names.size());
    options.input_names.push_back("wind_north_m_s");
    options.input_names.push_back("wind_east_m_s");
    options.input_names.push_back("wind_down_m_s");
  }
  options.outputs = outputs_from(context);
  options.report_truncation_error = context.input->bool_at("report_truncation_error", true);
  options.equilibrium_tolerance = context.input->number_at("equilibrium_tolerance", 1e-10);
  if (freeze_battery && model.has_battery()) {
    // The battery index within the APPENDED block: one per rotor comes first.
    options.frozen_appended_states = {model.rotor_count()};
  }

  const int rotor_count = model.rotor_count();
  const linearize::ExtendedDynamics dynamics = [&](const Eigen::VectorXd& x,
                                                   const Eigen::VectorXd& u) {
    const Eigen::Vector3d wind =
        wind_inputs ? Eigen::Vector3d(u.segment<3>(rotor_count)) : Eigen::Vector3d::Zero();
    return model.derivative(x, u.head(rotor_count), wind);
  };

  // The nominal input is the trim's own: its steady rotor commands, and the
  // wind it was solved at. Neither is re-read from this stage, so the point the
  // matrices describe cannot drift from the point that was trimmed.
  Eigen::VectorXd input =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(options.input_names.size()));
  input.head(rotor_count) = point.command_rad_s;
  if (wind_inputs) {
    input.segment<3>(rotor_count) = point.wind_ned_m_s;
  }

  const linearize::ExtendedLinearisation linearisation =
      linearize::linearize_extended(dynamics, point.extended_state, input, options);

  std::ostringstream description;
  description << (model.description.empty() ? std::string("multirotor") : model.description)
              << " — linearised about " << point.altitude_m << " m, airspeed " << point.airspeed_m_s
              << " m/s";

  std::ostringstream summary;
  summary << linearisation.a.rows() << " states, " << linearisation.b.cols() << " inputs, "
          << linearisation.c.rows() << " outputs; equilibrium residual " << std::scientific
          << std::setprecision(1) << linearisation.equilibrium_residual_norm;
  if (options.report_truncation_error) {
    summary << ", worst truncation " << linearisation.worst_relative_truncation;
  } else {
    summary << ", truncation not estimated";
  }
  if (!linearisation.frozen_state_names.empty()) {
    summary << "; frozen " << linearisation.frozen_state_names.size() << " state(s)";
  }

  if (const ValuePtr evidence_path = context.input->get("evidence_path")) {
    context.write_output(evidence_path->as_string(),
                         evidence_for(point, linearisation, context.stage_id));
  }

  Artifact artifact;
  artifact.kind = "linear_system";
  artifact.summary = summary.str();
  artifact.payload = linearisation.to_linear_system(description.str(), model.citation);
  return artifact;
}

// --- model.linear.export ---------------------------------------------------

Artifact export_state_space(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("system");
  const auto& system = upstream.payload_as<model::LinearSystem>("linear_system");

  const std::string path = context.input->string_at("path");
  context.write_output(path, model::serialize_linear_system(system));

  std::ostringstream summary;
  summary << "wrote " << system.state_count() << " states, " << system.input_count() << " inputs, "
          << system.output_count() << " outputs to " << path;

  Artifact artifact;
  artifact.kind = "linear_system";
  artifact.summary = summary.str();
  // The system itself, unchanged, so an export can sit mid-chain rather than
  // only at the end of one.
  artifact.payload = system;
  artifact.linearization_evidence = upstream.linearization_evidence;
  return artifact;
}

}  // namespace

void register_quadrotor_capabilities(Registry& registry) {
  // ImplementedUnvalidated, for the reason `model.quadrotor` carries: the cases
  // behind these are exact invariants of the equations and a cross-check
  // against an independent implementation. Neither is a published reference.
  registry.add(Capability{
      "trim.hover",
      "Solve multirotor equilibrium — still-air hover, hover in a crosswind, or cruise as a "
      "relative equilibrium — for attitude and rotor speeds, reporting each rotor's margin",
      "hover_trim",
      Capability::State::ImplementedUnvalidated,
      trim_hover_capability,
      {"quadrotor",
       "altitude_m",
       "wind_ned_m_s",
       "ground_velocity_ned_m_s",
       "heading_deg",
       "battery_state_of_charge",
       "tolerance",
       "iterations"}});

  registry.add(Capability{
      "linearize.extended",
      "Linearise a multirotor about a hover trim on a local attitude-error chart, with named "
      "wind disturbance columns and a declared observation model",
      "linear_system",
      Capability::State::ImplementedUnvalidated,
      linearize_extended_capability,
      {"trim",
       "outputs",
       "wind_inputs",
       "freeze_battery",
       "report_truncation_error",
       "equilibrium_tolerance",
       "evidence_path"},
      {},
      {"evidence_path"}});

  registry.add(
      Capability{"model.linear.export",
                 "Write a linear model as the named-matrix YAML that model.linear.statespace reads",
                 "linear_system",
                 Capability::State::ImplementedUnvalidated,
                 export_state_space,
                 {"system", "path"},
                 {},
                 {"path"}});
}

}  // namespace galata::pipeline
