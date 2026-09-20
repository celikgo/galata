// SPDX-License-Identifier: Apache-2.0
//
// The helicopter vertical, at the capability boundary: load a model, trim it,
// linearise it, simulate it, and report the component build-up.
//
// Reference: the physics is in galata/model/helicopter.hpp and
// galata/model/rotor/rotor.hpp; the trim formulation is in
// galata/trim/problem.hpp. This file resolves study input, adds deterministic
// study scheduling and evidence calculations, and turns the result into an
// artefact with a summary line. It does not claim higher-fidelity physics.
//
// EVERY CAPABILITY HERE IS `ImplementedUnvalidated`, and that is not modesty.
// docs/VERIFICATION.md records agreement against a published reference and no
// published rotorcraft reference has been compared against: the Souxmar model is
// a DESIGN STUDY, and the cases behind these capabilities are exact invariants of
// the equations plus a cross-check against the design package's own independent
// hover computation. Neither is a published reference, so neither earns
// `Implemented`. models/souxmar-heli/PROVENANCE.md says the same in more detail.
//
// The optional gaussian_ou_v1 wind is a reproducible disturbance signal, not a
// claim about an atmospheric spectrum. Its recursion follows Uhlenbeck and
// Ornstein, "On the Theory of the Brownian Motion", Physical Review 36 (1930),
// 823-841. What this is NOT: a Dryden/von Karman turbulence model or a measured
// aircraft environment. It is supported only for positive airspeed and its
// statistics are accepted as a declared study budget, not as aircraft data.

#include "galata/analyze/response_metrics.hpp"
#include "galata/core/atmosphere.hpp"
#include "galata/core/quaternion.hpp"
#include "galata/core/sha256.hpp"
#include "galata/linearize/extended.hpp"
#include "galata/linearize/vehicle.hpp"
#include "galata/model/helicopter.hpp"
#include "galata/numerics/integration_method.hpp"
#include "galata/pipeline/registry.hpp"
#include "galata/sim/sampled_loop.hpp"
#include "galata/sim/schedule.hpp"
#include "galata/sim/sensor.hpp"
#include "galata/synth/control.hpp"
#include "galata/synth/discrete_control.hpp"
#include "galata/trim/problem.hpp"

#include "input_schedule_parse.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <ostream>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace galata::pipeline {
namespace {

std::string fixed(double value, int digits = 3) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(digits) << value;
  return out.str();
}

std::string scientific(double value, int digits = 1) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::scientific << std::setprecision(digits) << value;
  return out.str();
}

std::string json_string(const std::string& value) {
  std::string result = "\"";
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      result += '\\';
    }
    result += character;
  }
  result += '"';
  return result;
}

// Degrees are for the SUMMARY LINE ONLY, which is user-interface text. ADR-0003
// puts the conversion at the boundary, and a capability's printed summary is
// that boundary. The artefact itself carries radians.
constexpr double kDegreesPerRadian = 57.29577951308232;  // GALATA_SI_EXEMPT: UI display only

// ---------------------------------------------------------------------------
// Artefact payloads
// ---------------------------------------------------------------------------

struct HelicopterArtifact {
  std::shared_ptr<const model::HelicopterModel> model;
  std::string path;
  std::string sha256;
  std::map<std::string, double> parameter_overrides;
};

struct HelicopterTrimArtifact {
  std::shared_ptr<const model::HelicopterModel> model;
  trim::TrimResult result;
  model::Environment environment;
  // The COMMANDS that hold the trim. Equal to the solved actuator positions,
  // because an equilibrium is a statement about where the swashplate is and a
  // command that differs from it drives the actuator away from the trim.
  Eigen::VectorXd holding_controls;
  model::HelicopterModel::Breakdown breakdown;
  std::map<std::string, double> parameter_overrides;
};

struct HelicopterTrajectoryArtifact {
  std::shared_ptr<const model::HelicopterModel> model;
  std::vector<double> times_s;
  std::vector<Eigen::VectorXd> states;  // extended
  std::vector<Eigen::VectorXd> outputs;
  std::vector<Eigen::VectorXd> applied_controls;
  std::vector<std::string> state_names;
  std::vector<std::string> output_names;
  numerics::TerminationReason reason = numerics::TerminationReason::Completed;
  std::string termination_detail;
  double step_s = 0.0;
  int steps_taken = 0;
  // Deterministic records of scheduled failures. The manifest also retains
  // the source declarations; this list is the trajectory's applied event log.
  std::vector<std::string> failure_events;
  model::EnvelopeStatus worst_envelope;
  int envelope_departures = 0;
  bool closed_loop = false;
  std::vector<std::string> controller_measurement_names;
  std::vector<std::string> controller_state_names;
  std::vector<double> controller_times_s;
  std::vector<Eigen::VectorXd> controller_measurements;
  std::vector<Eigen::VectorXd> controller_references;
  std::vector<Eigen::VectorXd> controller_errors;
  std::vector<Eigen::VectorXd> controller_requested;
  std::vector<Eigen::VectorXd> controller_saturated;
  std::vector<Eigen::VectorXd> controller_applied;
  std::vector<Eigen::VectorXd> controller_states;
  std::vector<bool> controller_measurement_available;
  std::vector<bool> controller_measurement_stale;
  std::vector<std::string> sensor_provenance;
  std::vector<Eigen::Vector3d> wind_samples_ned_m_s;
  std::map<std::string, double> trim_reference_values;
  std::vector<std::string> initial_condition_provenance;
  std::string disturbance_provenance;
  std::string parameter_provenance;
};

struct HelicopterEnsembleMember {
  std::string id;
  std::uint64_t seed = 0;
  double mass_scale = 1.0;
  Eigen::Vector3d cg_offset_body_m = Eigen::Vector3d::Zero();
  bool completed = false;
  bool numerical_checks_passed = false;
  bool envelope_checks_passed = false;
  bool controller_requirements_passed = true;
  std::string controller_requirements_status = "not_requested";
  bool criteria_passed = false;
  int envelope_departures = 0;
  std::string execution_status;
  std::string failure_reason;
  std::string distribution_provenance;
  std::string termination;
  std::string manifest_path;
};

struct HelicopterEnsembleArtifact {
  std::vector<HelicopterEnsembleMember> members;
  bool parallel = false;
  std::string ordering_policy;
};

struct HelicopterResponseMetric {
  std::string signal;
  std::string unit;
  double reference = 0.0;
  double peak_error = 0.0;
  double final_error = 0.0;
  double rms_error = 0.0;
  double settling_time_s = std::numeric_limits<double>::infinity();  // s
  std::string settling_status;
  std::vector<std::string> segment_settling_statuses;
  std::vector<double> segment_settling_times_s;
  double overshoot_fraction = std::numeric_limits<double>::quiet_NaN();
  bool overshoot_applicable = false;
  double uncontrolled_peak_error = 0.0;
  double open_closed_peak_error_ratio = std::numeric_limits<double>::quiet_NaN();
  double open_closed_improvement_fraction = std::numeric_limits<double>::quiet_NaN();
};

struct HelicopterResponseArtifact {
  std::vector<HelicopterResponseMetric> metrics;
  std::map<std::string, double> requirements;
  double requested_minus_limited_peak = 0.0;   // rad
  double limited_minus_delayed_peak = 0.0;     // rad
  double delayed_minus_actual_peak = 0.0;      // rad
  double control_effort_peak = 0.0;            // rad from trim
  double control_effort_rms = 0.0;             // rad from trim
  double saturation_duration_s = 0.0;          // s
  double maximum_rotor_speed_excursion = 0.0;  // rad/s
  int controlled_envelope_departures = 0;
  int uncontrolled_envelope_departures = 0;
  bool matched_initial_condition = false;
  bool matched_disturbance = false;
  bool criteria_passed = false;
};

std::map<std::string, double*> helicopter_parameter_fields(model::HelicopterModel& heli) {
  std::map<std::string, double*> fields{
      {"mass.mass_kg", &heli.mass.mass_kg},
      {"mass.inertia_xx_kg_m2", &heli.mass.inertia_cg_body_kg_m2(0, 0)},
      {"mass.inertia_yy_kg_m2", &heli.mass.inertia_cg_body_kg_m2(1, 1)},
      {"mass.inertia_zz_kg_m2", &heli.mass.inertia_cg_body_kg_m2(2, 2)},
      {"main_rotor.radius_m", &heli.main_rotor.radius_m},
      {"main_rotor.chord_m", &heli.main_rotor.chord_m},
      {"main_rotor.inflow_time_constant_s", &heli.main_rotor.inflow_time_constant_s},
      {"tail_rotor.radius_m", &heli.tail_rotor.radius_m},
      {"tail_rotor.chord_m", &heli.tail_rotor.chord_m},
      {"tail_rotor.inflow_time_constant_s", &heli.tail_rotor.inflow_time_constant_s},
      {"airframe.flat_plate_area_m2", &heli.airframe.flat_plate_area_m2},
      {"drivetrain.reference_rotor_speed_rad_s", &heli.drivetrain.reference_rotor_speed_rad_s},
      {"drivetrain.governor_proportional_n_m_s", &heli.drivetrain.governor_proportional_n_m_s},
      {"drivetrain.governor_time_constant_s", &heli.drivetrain.governor_time_constant_s},
      {"drivetrain.maximum_engine_torque_n_m", &heli.drivetrain.maximum_engine_torque_n_m},
      {"drivetrain.accessory_torque_n_m", &heli.drivetrain.accessory_torque_n_m},
      {"tail_rotor_blockage_factor", &heli.tail_rotor_blockage_factor},
      {"pedal_to_tail_collective", &heli.pedal_to_tail_collective},
  };
  const auto control_names = heli.control_names();
  for (std::size_t index = 0; index < control_names.size(); ++index) {
    const std::string prefix =
        "actuators."
        + control_names[index].substr(
            0, control_names[index].size() - std::string("_command_rad").size());
    auto& limits = heli.actuators[index];
    fields.emplace(prefix + ".minimum_rad", &limits.minimum_rad);
    fields.emplace(prefix + ".maximum_rad", &limits.maximum_rad);
    fields.emplace(prefix + ".rate_limit_rad_s", &limits.rate_limit_rad_s);
    fields.emplace(prefix + ".time_constant_s", &limits.time_constant_s);
  }
  return fields;
}

std::map<std::string, double> apply_helicopter_parameter_overrides(model::HelicopterModel& heli,
                                                                   const ValuePtr& declared) {
  std::map<std::string, double> applied;
  if (!declared) {
    return applied;
  }
  if (declared->kind() != Value::Kind::Map) {
    throw std::invalid_argument(
        "model.helicopter parameter_overrides must be a map of named finite SI values");
  }
  const auto fields = helicopter_parameter_fields(heli);
  for (const auto& [name, value] : declared->as_map()) {
    const auto field = fields.find(name);
    if (field == fields.end()) {
      std::ostringstream message;
      message << "model.helicopter parameter_overrides names unknown parameter '" << name
              << "'. Available:";
      for (const auto& [known, ignored] : fields) {
        (void)ignored;
        message << " " << known;
      }
      throw std::invalid_argument(message.str());
    }
    if (value->kind() != Value::Kind::Number || !std::isfinite(value->as_number())) {
      throw std::invalid_argument("model.helicopter parameter_overrides['" + name
                                  + "'] must be a finite number in SI units");
    }
    *field->second = value->as_number();
    applied.emplace(name, value->as_number());
  }
  heli.validate();
  return applied;
}

std::map<std::string, double> helicopter_parameter_values(const model::HelicopterModel& heli) {
  model::HelicopterModel copy = heli;
  const auto fields = helicopter_parameter_fields(copy);
  std::map<std::string, double> values;
  for (const auto& [name, field] : fields) {
    values.emplace(name, *field);
  }
  return values;
}

std::string json_number(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return out.str();
}

void json_vector(std::ostream& out, const Eigen::VectorXd& values) {
  out << '[';
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    out << (index == 0 ? "" : ",") << json_number(values(index));
  }
  out << ']';
}

void json_strings(std::ostream& out, const std::vector<std::string>& values) {
  out << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    out << (index == 0 ? "" : ",") << json_string(values[index]);
  }
  out << ']';
}

std::string unit_for_channel(const std::string& name) {
  if (name.ends_with("_rad_s")) {
    return "rad/s";
  }
  if (name.ends_with("_rad")) {
    return "rad";
  }
  if (name.ends_with("_m_s")) {
    return "m/s";
  }
  if (name.ends_with("_n_m")) {
    return "N m";
  }
  if (name.ends_with("_m")) {
    return "m";
  }
  if (name.ends_with("_n")) {
    return "N";
  }
  if (name.ends_with("_w")) {
    return "W";
  }
  if (name.ends_with("_kg")) {
    return "kg";
  }
  if (name.ends_with("_kg_m2")) {
    return "kg m^2";
  }
  return "dimensionless";
}

std::string frame_for_channel(const std::string& name) {
  if (name.find("position_") == 0 || name == "altitude_m") {
    return name == "altitude_m" ? "NED-derived altitude" : "NED position";
  }
  if (name.find("velocity_") == 0) {
    return "body FRD velocity";
  }
  if (name.find("quaternion_") == 0 || name == "roll_rad" || name == "pitch_rad"
      || name == "yaw_rad") {
    return "body-to-NED Hamilton attitude";
  }
  if (name.find("rate_") != std::string::npos || name.find("_rate_") != std::string::npos) {
    return "body FRD angular rate";
  }
  if (name.find("command_") != std::string::npos || name.find("cyclic") != std::string::npos
      || name == "pedal_rad") {
    return "body actuator/control";
  }
  return "model-native";
}

struct FailureEvent {
  int step = 0;
  double time_s = 0.0;
  std::string component;
  double fraction = 0.0;
  int actuator_index = -1;
  std::string actuator_name;
  std::optional<double> position_rad;
};

double event_number(const std::map<std::string, ValuePtr>& fields,
                    const std::string& key,
                    const std::string& description) {
  const auto found = fields.find(key);
  if (found == fields.end()) {
    throw std::runtime_error("sim.helicopter: failure event " + description + " is missing '" + key
                             + "'");
  }
  if (found->second->kind() != Value::Kind::Number) {
    throw std::runtime_error("sim.helicopter: failure event " + description + "." + key
                             + " must be a number");
  }
  return found->second->as_number();
}

std::string event_text(const std::map<std::string, ValuePtr>& fields,
                       const std::string& key,
                       const std::string& description) {
  const auto found = fields.find(key);
  if (found == fields.end()) {
    throw std::runtime_error("sim.helicopter: failure event " + description + " is missing '" + key
                             + "'");
  }
  if (found->second->kind() != Value::Kind::String) {
    throw std::runtime_error("sim.helicopter: failure event " + description + "." + key
                             + " must be a string");
  }
  return found->second->as_string();
}

std::vector<FailureEvent> parse_failure_events(const StageContext& context,
                                               const model::HelicopterModel& heli,
                                               double step_s,
                                               int steps) {
  const ValuePtr declared = context.input->get("failure_events");
  if (declared == nullptr) {
    return {};
  }
  if (declared->kind() != Value::Kind::List) {
    throw std::runtime_error("sim.helicopter: failure_events must be a list of event maps");
  }

  std::vector<FailureEvent> events;
  const double horizon_s = static_cast<double>(steps) * step_s;
  const auto controls = heli.control_names();
  for (std::size_t i = 0; i < declared->as_list().size(); ++i) {
    const ValuePtr& entry = declared->as_list()[i];
    const std::string description = "#" + std::to_string(i);
    if (entry->kind() != Value::Kind::Map) {
      throw std::runtime_error("sim.helicopter: failure event " + description + " must be a map");
    }
    const auto& fields = entry->as_map();
    FailureEvent event;
    event.time_s = event_number(fields, "time_s", description);
    event.component = event_text(fields, "component", description);
    if (!std::isfinite(event.time_s) || event.time_s < 0.0 || event.time_s > horizon_s) {
      throw std::runtime_error("sim.helicopter: failure event " + description
                               + " time_s must lie in [0, " + fixed(horizon_s, 6) + "]");
    }
    const double lattice_step = event.time_s / step_s;
    const double nearest = std::round(lattice_step);
    if (std::fabs(lattice_step - nearest) > 1.0e-12) {
      throw std::runtime_error("sim.helicopter: failure event " + description
                               + " is at " + fixed(event.time_s, 9)
                               + " s, between integration steps. Failure events are applied at "
                                 "fixed-step boundaries; choose a time on the declared step lattice");
    }
    event.step = static_cast<int>(nearest);

    if (event.component == "tail_rotor" || event.component == "engine") {
      for (const auto& [key, value] : fields) {
        (void)value;
        if (key != "time_s" && key != "component" && key != "fraction") {
          throw std::runtime_error("sim.helicopter: failure event " + description
                                   + " has unknown key '" + key + "'");
        }
      }
      event.fraction = event_number(fields, "fraction", description);
      if (!std::isfinite(event.fraction) || event.fraction < 0.0 || event.fraction > 1.0) {
        throw std::runtime_error("sim.helicopter: failure event " + description
                                 + ".fraction must be finite and in [0, 1]");
      }
    } else if (event.component == "actuator_jam") {
      for (const auto& [key, value] : fields) {
        (void)value;
        if (key != "time_s" && key != "component" && key != "actuator" && key != "position_rad") {
          throw std::runtime_error("sim.helicopter: failure event " + description
                                   + " has unknown key '" + key + "'");
        }
      }
      event.actuator_name = event_text(fields, "actuator", description);
      const auto found = std::find(controls.begin(), controls.end(), event.actuator_name);
      if (found == controls.end()) {
        throw std::runtime_error("sim.helicopter: failure event " + description
                                 + " names actuator '" + event.actuator_name
                                 + "', but the model has no such control");
      }
      event.actuator_index = static_cast<int>(found - controls.begin());
      const auto position = fields.find("position_rad");
      if (position != fields.end()) {
        if (position->second->kind() != Value::Kind::Number) {
          throw std::runtime_error("sim.helicopter: failure event " + description
                                   + ".position_rad must be a number");
        }
        event.position_rad = position->second->as_number();
        if (!std::isfinite(*event.position_rad)) {
          throw std::runtime_error("sim.helicopter: failure event " + description
                                   + ".position_rad must be finite");
        }
      }
    } else {
      throw std::runtime_error("sim.helicopter: failure event " + description
                               + " component must be 'tail_rotor', 'engine' or 'actuator_jam'");
    }
    events.push_back(std::move(event));
  }

  std::stable_sort(
      events.begin(), events.end(), [](const FailureEvent& left, const FailureEvent& right) {
        return left.step < right.step;
      });
  return events;
}

std::string apply_failure_event(const FailureEvent& event,
                                model::HelicopterModel& heli,
                                Eigen::VectorXd& state) {
  if (event.component == "tail_rotor") {
    heli.failures.tail_rotor_effectiveness = event.fraction;
    return "t=" + fixed(event.time_s, 6) + " s: tail_rotor fraction=" + fixed(event.fraction, 6);
  }
  if (event.component == "engine") {
    heli.failures.engine_available_fraction = event.fraction;
    return "t=" + fixed(event.time_s, 6) + " s: engine fraction=" + fixed(event.fraction, 6);
  }

  const auto& limits = heli.actuators[static_cast<std::size_t>(event.actuator_index)];
  const double jam_position = event.position_rad.value_or(
      state(core::kStateSize + model::kCollectivePosition + event.actuator_index));
  if (jam_position < limits.minimum_rad || jam_position > limits.maximum_rad) {
    throw std::runtime_error(
        "sim.helicopter: actuator jam position is outside the actuator's declared travel");
  }
  state(core::kStateSize + model::kCollectivePosition + event.actuator_index) = jam_position;
  heli.failures.jammed_actuator_rad[static_cast<std::size_t>(event.actuator_index)] = jam_position;
  return "t=" + fixed(event.time_s, 6) + " s: actuator_jam " + event.actuator_name + " at "
         + fixed(jam_position, 6) + " rad";
}

model::HelicopterModel model_at_failure_step(const model::HelicopterModel& final_model,
                                             const std::vector<FailureEvent>& events,
                                             int step) {
  model::HelicopterModel model = final_model;
  model.failures = model::HelicopterFailures{};
  for (const auto& event : events) {
    if (event.step > step) {
      break;
    }
    if (event.component == "tail_rotor") {
      model.failures.tail_rotor_effectiveness = event.fraction;
    } else if (event.component == "engine") {
      model.failures.engine_available_fraction = event.fraction;
    } else {
      const auto index = static_cast<std::size_t>(event.actuator_index);
      const double position = event.position_rad.value_or(
          final_model.failures.jammed_actuator_rad[index].value_or(0.0));
      model.failures.jammed_actuator_rad[index] = position;
    }
  }
  return model;
}

void require_map_keys(const ValuePtr& value,
                      const std::vector<std::string>& allowed,
                      const std::string& description) {
  if (!value || value->kind() != Value::Kind::Map) {
    throw std::runtime_error(description + " must be a map");
  }
  for (const auto& [key, entry] : value->as_map()) {
    (void)entry;
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
      throw std::runtime_error(description + " has unknown key '" + key + "'");
    }
  }
}

Eigen::VectorXd value_vector(const ValuePtr& value,
                             Eigen::Index width,
                             const std::string& description,
                             const Eigen::VectorXd& fallback = {}) {
  if (!value) {
    if (fallback.size() == width) {
      return fallback;
    }
    return Eigen::VectorXd::Zero(width);
  }
  if (value->kind() != Value::Kind::List
      || value->as_list().size() != static_cast<std::size_t>(width)) {
    throw std::runtime_error(description + " must be a list of " + std::to_string(width)
                             + " numbers");
  }
  Eigen::VectorXd result(width);
  for (Eigen::Index i = 0; i < width; ++i) {
    const ValuePtr& entry = value->as_list()[static_cast<std::size_t>(i)];
    if (entry->kind() != Value::Kind::Number || !std::isfinite(entry->as_number())) {
      throw std::runtime_error(description + " must contain only finite numbers");
    }
    result(i) = entry->as_number();
  }
  return result;
}

struct InitialCondition {
  Eigen::VectorXd state;
  std::vector<std::string> provenance;
};

InitialCondition initial_condition_from(const StageContext& context,
                                        const model::HelicopterModel& heli,
                                        const Eigen::VectorXd& trim_state) {
  InitialCondition result{trim_state, {"initial state: declared trim"}};
  const ValuePtr perturbation = context.input->get("initial_state_perturbation");
  if (perturbation) {
    if (perturbation->kind() != Value::Kind::Map) {
      throw std::runtime_error(
          "helicopter simulation initial_state_perturbation must be a map of state names to "
          "SI increments; use attitude_perturbation_body_rad for attitude");
    }
    const auto names = heli.state_names();
    for (const auto& [name, value] : perturbation->as_map()) {
      if (name.rfind("quaternion_", 0) == 0) {
        throw std::runtime_error(
            "helicopter simulation initial_state_perturbation may not edit quaternion_* "
            "components; declare attitude_perturbation_body_rad as a body-frame rotation vector "
            "so the quaternion is normalised and the ADR-0002 convention is preserved");
      }
      const auto found = std::find(names.begin(), names.end(), name);
      if (found == names.end() || value->kind() != Value::Kind::Number
          || !std::isfinite(value->as_number())) {
        throw std::runtime_error(
            "helicopter simulation initial_state_perturbation must name "
            "finite non-quaternion helicopter states");
      }
      result.state(static_cast<Eigen::Index>(found - names.begin())) += value->as_number();
      result.provenance.push_back("initial perturbation " + name
                                  + " += " + fixed(value->as_number(), 9));
    }
  }

  const ValuePtr attitude = context.input->get("attitude_perturbation_body_rad");
  if (attitude) {
    const Eigen::VectorXd rotation =
        value_vector(attitude, 3, "helicopter simulation attitude_perturbation_body_rad");
    const core::Quaternion nominal(result.state(core::kQuaternionW),
                                   result.state(core::kQuaternionX),
                                   result.state(core::kQuaternionY),
                                   result.state(core::kQuaternionZ));
    const core::Quaternion perturbed =
        core::normalised(nominal * core::quaternion_from_rotation_vector(rotation));
    result.state(core::kQuaternionW) = perturbed.w();
    result.state(core::kQuaternionX) = perturbed.x();
    result.state(core::kQuaternionY) = perturbed.y();
    result.state(core::kQuaternionZ) = perturbed.z();
    result.provenance.push_back("attitude perturbation: body rotation vector ["
                                + fixed(rotation(0), 9) + ", " + fixed(rotation(1), 9) + ", "
                                + fixed(rotation(2), 9) + "] rad; quaternion normalised");
  }
  model::VehicleModel::project(result.state);
  return result;
}

struct WindEvent {
  int step = 0;
  double declared_time_s = 0.0;  // s
};

std::vector<WindEvent> wind_events(const sim::InputSchedule& schedule, double step_s, int steps) {
  std::vector<WindEvent> events;
  for (const double time_s : schedule.discontinuities()) {
    const double lattice = time_s / step_s;
    const double nearest = std::round(lattice);
    if (std::fabs(lattice - nearest) > 1.0e-9 * std::fmax(1.0, std::fabs(lattice))) {
      throw std::invalid_argument(
          "wind_schedule changes between integration steps; align a discontinuity to the "
          "declared step so ground velocity can be rebased at an explicit boundary");
    }
    if (nearest < 0.0 || nearest > static_cast<double>(steps)) {
      throw std::invalid_argument(
          "wind_schedule discontinuity lies outside the simulation horizon");
    }
    events.push_back({static_cast<int>(nearest), time_s});
  }
  return events;
}

void validate_schedule_horizon(const sim::InputSchedule& schedule,
                               double end_s,
                               const std::string& name) {
  if (!schedule.empty()) {
    const double tolerance = 1.0e-9 * std::fmax(1.0, std::fabs(end_s));
    const double read_at =
        std::fabs(schedule.last_time_s() - end_s) <= tolerance ? schedule.last_time_s() : end_s;
    try {
      (void)schedule.at(read_at);
    } catch (const std::exception& error) {
      throw std::invalid_argument(name + " does not cover the simulation horizon: " + error.what());
    }
  }
}

Eigen::Vector3d wind_at(const sim::InputSchedule& schedule,
                        double time_s,
                        const Eigen::Vector3d& fallback) {
  if (schedule.empty()) {
    return fallback;
  }
  const double read_at = std::clamp(time_s, schedule.first_time_s(), schedule.last_time_s());
  return Eigen::Vector3d(schedule.at(read_at));
}

std::string wind_provenance(const sim::InputSchedule& schedule) {
  if (schedule.empty()) {
    return "disturbance: constant trim wind";
  }
  return std::string("disturbance: declared wind_schedule, hold=")
         + (schedule.hold() == sim::HoldPolicy::Linear ? "linear" : "zero_order")
         + ", extrapolation="
         + (schedule.discontinuities().empty() ? "declared span" : "event-aligned span");
}

std::string turbulence_provenance(const StageContext& context) {
  const ValuePtr turbulence = context.input->get("turbulence");
  if (!turbulence) {
    return {};
  }
  const double seed = turbulence->number_at("seed");
  return "disturbance: gaussian_ou_v1 seeded turbulence, seed="
         + std::to_string(static_cast<std::uint64_t>(seed));
}

std::uint64_t turbulence_splitmix64(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

double turbulence_normal(std::uint64_t seed, int sample, int channel) {
  std::mt19937_64 generator(
      turbulence_splitmix64(seed ^ turbulence_splitmix64(static_cast<std::uint64_t>(sample + 1))
                            ^ static_cast<std::uint64_t>(channel + 1)));
  const double first = (static_cast<double>(generator() >> 11) + 0.5) / 9007199254740992.0;
  const double second = (static_cast<double>(generator() >> 11) + 0.5) / 9007199254740992.0;
  return std::sqrt(-2.0 * std::log(first)) * std::cos(6.2831853071795864769 * second);
}

sim::InputSchedule wind_schedule_from(const StageContext& context,
                                      const Eigen::VectorXd& initial_state,
                                      const model::Environment& trim_environment,
                                      double step_s,
                                      int steps) {
  const ValuePtr turbulence = context.input->get("turbulence");
  const ValuePtr declared_schedule = context.input->get("wind_schedule");
  if (turbulence && declared_schedule) {
    throw std::invalid_argument(
        "helicopter simulation declares both wind_schedule and turbulence; choose one "
        "disturbance source so the applied wind is unambiguous");
  }
  if (!turbulence) {
    return schedule_at(context, "wind_schedule", 3);
  }
  require_map_keys(turbulence,
                   {"model", "seed", "mean_ned_m_s", "stddev_ned_m_s", "correlation_time_s"},
                   "helicopter simulation turbulence");
  if (turbulence->string_at("model") != "gaussian_ou_v1") {
    throw std::invalid_argument("helicopter simulation turbulence.model must be 'gaussian_ou_v1'");
  }
  const double seed_number = turbulence->number_at("seed");
  if (!std::isfinite(seed_number) || seed_number < 0.0 || std::floor(seed_number) != seed_number
      || seed_number > 9007199254740991.0) {
    throw std::invalid_argument(
        "helicopter simulation turbulence.seed must be a non-negative exactly representable "
        "integer");
  }
  const Eigen::VectorXd mean = value_vector(
      turbulence->get("mean_ned_m_s"), 3, "helicopter simulation turbulence.mean_ned_m_s");
  const Eigen::VectorXd stddev = value_vector(
      turbulence->get("stddev_ned_m_s"), 3, "helicopter simulation turbulence.stddev_ned_m_s");
  const double correlation_time_s = turbulence->number_at("correlation_time_s");
  if (!(correlation_time_s > 0.0) || !std::isfinite(correlation_time_s)
      || (stddev.array() < 0.0).any()) {
    throw std::invalid_argument(
        "helicopter simulation turbulence requires a positive finite correlation_time_s and "
        "non-negative finite stddev_ned_m_s");
  }
  const core::State initial = model::VehicleModel::rigid_body_part(initial_state);
  if (!(core::airspeed(initial.velocity_body_m_s) > 0.0)) {
    throw std::invalid_argument(
        "helicopter simulation turbulence.gaussian_ou_v1 is unsupported at zero airspeed; "
        "use a declared constant/gust wind or a forward-flight trim");
  }

  const std::uint64_t seed = static_cast<std::uint64_t>(seed_number);
  const double rho = std::exp(-step_s / correlation_time_s);
  const double innovation = std::sqrt(std::max(0.0, 1.0 - rho * rho));
  Eigen::Vector3d value(mean);
  std::vector<double> times;
  std::vector<Eigen::VectorXd> values;
  times.reserve(static_cast<std::size_t>(steps) + 1U);
  values.reserve(static_cast<std::size_t>(steps) + 1U);
  for (int step = 0; step <= steps; ++step) {
    if (step > 0) {
      for (int channel = 0; channel < 3; ++channel) {
        value(channel) = mean(channel) + rho * (value(channel) - mean(channel))
                         + innovation * stddev(channel) * turbulence_normal(seed, step, channel);
      }
    }
    times.push_back(static_cast<double>(step) * step_s);
    values.push_back(value);
  }
  (void)trim_environment;
  return sim::InputSchedule(
      std::move(times), std::move(values), sim::HoldPolicy::ZeroOrder, sim::Extrapolation::Hold);
}

double mass_scale_from(const StageContext& context) {
  const double scale = context.input->number_at("mass_scale", 1.0);
  if (!(scale > 0.0) || !std::isfinite(scale)) {
    throw std::invalid_argument(
        "helicopter simulation mass_scale must be a positive finite design parameter");
  }
  return scale;
}

Eigen::Vector3d cg_offset_from(const StageContext& context) {
  const ValuePtr declared = context.input->get("cg_offset_body_m");
  if (!declared) {
    return Eigen::Vector3d::Zero();
  }
  const Eigen::VectorXd offset =
      value_vector(declared, 3, "helicopter simulation cg_offset_body_m");
  if (offset.norm() > 2.0) {
    throw std::invalid_argument(
        "helicopter simulation cg_offset_body_m must lie within the declared 2 m design range");
  }
  return Eigen::Vector3d(offset);
}

void apply_parameter_variation(model::HelicopterModel& simulation_model,
                               double mass_scale,
                               const Eigen::Vector3d& cg_offset_body_m) {
  simulation_model.mass.mass_kg *= mass_scale;
  if (!cg_offset_body_m.isZero()) {
    simulation_model.main_rotor.position_cg_to_hub_body_m -= cg_offset_body_m;
    simulation_model.tail_rotor.position_cg_to_hub_body_m -= cg_offset_body_m;
    simulation_model.airframe.cg_to_fuselage_reference_body_m -= cg_offset_body_m;
    simulation_model.airframe.cg_to_horizontal_tail_body_m -= cg_offset_body_m;
    simulation_model.airframe.cg_to_vertical_tail_body_m -= cg_offset_body_m;
  }
}

std::map<std::string, double> helicopter_named_values(const model::HelicopterModel& heli,
                                                      const Eigen::VectorXd& state,
                                                      const Eigen::VectorXd& controls,
                                                      const model::Environment& environment) {
  if (state.size() != heli.extended_state_size()) {
    throw std::invalid_argument("sim.helicopter.closed_loop: state width does not match model");
  }
  const core::State rigid = model::VehicleModel::rigid_body_part(state);
  const Eigen::VectorXd auxiliary = heli.auxiliary_part(state);
  const auto state_names = heli.state_names();
  const auto output_names = heli.output_names();
  const Eigen::VectorXd outputs = heli.outputs(rigid, auxiliary, controls, environment);
  const core::EulerAngles euler = core::euler_from_quaternion(rigid.attitude_body_to_ned);
  const std::vector<std::string> euler_names = {"position_north_m",
                                                "position_east_m",
                                                "position_down_m",
                                                "velocity_u_m_s",
                                                "velocity_v_m_s",
                                                "velocity_w_m_s",
                                                "roll_rad",
                                                "pitch_rad",
                                                "yaw_rad",
                                                "roll_rate_rad_s",
                                                "pitch_rate_rad_s",
                                                "yaw_rate_rad_s"};
  Eigen::VectorXd euler_state(12 + heli.auxiliary_state_count());
  euler_state.segment<3>(0) = rigid.position_ned_m;
  euler_state.segment<3>(3) = rigid.velocity_body_m_s;
  euler_state(6) = euler.roll_rad;
  euler_state(7) = euler.pitch_rad;
  euler_state(8) = euler.yaw_rad;
  euler_state.segment<3>(9) = rigid.angular_rate_body_rad_s;
  euler_state.tail(heli.auxiliary_state_count()) = auxiliary;

  std::map<std::string, double> named;
  for (std::size_t i = 0; i < state_names.size(); ++i) {
    named.emplace(state_names[i], state(static_cast<Eigen::Index>(i)));
  }
  for (std::size_t i = 0; i < euler_names.size(); ++i) {
    named[euler_names[i]] = euler_state(static_cast<Eigen::Index>(i));
  }
  for (int i = 0; i < heli.auxiliary_state_count(); ++i) {
    named[heli.state_names()[static_cast<std::size_t>(core::kStateSize + i)]] = auxiliary(i);
  }
  for (std::size_t i = 0; i < output_names.size(); ++i) {
    named[output_names[i]] = outputs(static_cast<Eigen::Index>(i));
  }
  return named;
}

std::vector<std::string> string_list(const ValuePtr& value, const std::string& description) {
  if (!value || value->kind() != Value::Kind::List || value->as_list().empty()) {
    throw std::invalid_argument(description + " must be a non-empty list of strings");
  }
  std::vector<std::string> result;
  result.reserve(value->as_list().size());
  for (const auto& item : value->as_list()) {
    if (item->kind() != Value::Kind::String || item->as_string().empty()) {
      throw std::invalid_argument(description + " must contain non-empty strings");
    }
    result.push_back(item->as_string());
  }
  return result;
}

std::map<std::string, double> finite_number_map(const ValuePtr& value,
                                                const std::string& description) {
  std::map<std::string, double> result;
  if (!value) {
    return result;
  }
  if (value->kind() != Value::Kind::Map) {
    throw std::invalid_argument(description + " must be a map of finite numbers");
  }
  for (const auto& [name, entry] : value->as_map()) {
    if (entry->kind() != Value::Kind::Number || !std::isfinite(entry->as_number())) {
      throw std::invalid_argument(description + " must contain only finite numbers");
    }
    result.emplace(name, entry->as_number());
  }
  return result;
}

std::string response_unit(const std::string& signal) {
  if (signal.size() >= 5 && signal.ends_with("_rad_s")) {
    return "rad/s";
  }
  if (signal.size() >= 4 && signal.ends_with("_rad")) {
    return "rad";
  }
  if (signal.size() >= 2 && signal.ends_with("_m")) {
    return "m";
  }
  if (signal.size() >= 2 && signal.ends_with("_n")) {
    return "N";
  }
  return "native";
}

std::string response_unit_suffix(const std::string& signal) {
  if (signal.ends_with("_rad_s")) {
    return "rad_s";
  }
  if (signal.ends_with("_rad")) {
    return "rad";
  }
  if (signal.ends_with("_m")) {
    return "m";
  }
  if (signal.ends_with("_n")) {
    return "n";
  }
  return "native";
}

struct ResponseRequirements {
  std::map<std::string, double> aggregate;
  std::map<std::string, std::map<std::string, double>> per_signal;
};

ResponseRequirements parse_response_requirements(const ValuePtr& aggregate_value,
                                                 const ValuePtr& signal_value,
                                                 const std::vector<std::string>& signals) {
  ResponseRequirements result;
  const std::set<std::string> aggregate_names = {
      "requested_minus_limited_peak_rad",
      "limited_minus_delayed_peak_rad",
      "delayed_minus_actual_peak_rad",
      "control_effort_peak_rad",
      "control_effort_rms_rad",
      "saturation_duration_s",
      "rotor_speed_excursion_rad_s",
      "validity_envelope_departures",
      // Explicit migration aliases from the pre-milestone schema. They are
      // retained only so historical studies remain executable.
      "tracking_error",
      "settling_time_s",
      "overshoot",
      "control_effort_rad",
  };
  if (aggregate_value) {
    if (aggregate_value->kind() != Value::Kind::Map) {
      throw std::invalid_argument(
          "analyze.helicopter_response requirements must be a map of named finite budgets");
    }
    for (const auto& [name, entry] : aggregate_value->as_map()) {
      if (!aggregate_names.contains(name)) {
        throw std::invalid_argument(
            "analyze.helicopter_response requirements contains unknown requirement '" + name + "'");
      }
      if (entry->kind() != Value::Kind::Number || !std::isfinite(entry->as_number())
          || entry->as_number() < 0.0) {
        throw std::invalid_argument("analyze.helicopter_response requirement '" + name
                                    + "' must be a finite non-negative number");
      }
      result.aggregate.emplace(name, entry->as_number());
    }
  }
  if (!signal_value) {
    return result;
  }
  if (signal_value->kind() != Value::Kind::Map) {
    throw std::invalid_argument(
        "analyze.helicopter_response signal_requirements must be a map keyed by signal name");
  }
  const std::set<std::string> requested(signals.begin(), signals.end());
  for (const auto& [signal, requirements] : signal_value->as_map()) {
    if (!requested.contains(signal)) {
      throw std::invalid_argument("analyze.helicopter_response signal_requirements names signal '"
                                  + signal + "' that is not in signals");
    }
    if (requirements->kind() != Value::Kind::Map) {
      throw std::invalid_argument("analyze.helicopter_response requirements for '" + signal
                                  + "' must be a map");
    }
    const std::string suffix = response_unit_suffix(signal);
    const std::set<std::string> allowed = {
        "peak_tracking_error_" + suffix,
        "final_tracking_error_" + suffix,
        "rms_tracking_error_" + suffix,
        "settling_band_" + suffix,
        "settling_dwell_s",
        "settling_time_s",
        "overshoot_fraction",
        "open_closed_peak_error_ratio",
        "open_closed_improvement_fraction",
    };
    for (const auto& [name, entry] : requirements->as_map()) {
      if (!allowed.contains(name)) {
        throw std::invalid_argument("analyze.helicopter_response signal '" + signal
                                    + "' contains unknown or unit-incompatible requirement '" + name
                                    + "'");
      }
      if (entry->kind() != Value::Kind::Number || !std::isfinite(entry->as_number())
          || entry->as_number() < 0.0) {
        throw std::invalid_argument("analyze.helicopter_response signal requirement '" + name
                                    + "' for '" + signal
                                    + "' must be a finite non-negative number");
      }
      result.per_signal[signal].emplace(name, entry->as_number());
    }
  }
  return result;
}

double trajectory_signal(const HelicopterTrajectoryArtifact& run,
                         std::size_t sample,
                         const std::string& signal) {
  const auto state = std::find(run.state_names.begin(), run.state_names.end(), signal);
  if (state != run.state_names.end()) {
    return run.states[sample](static_cast<Eigen::Index>(state - run.state_names.begin()));
  }
  const auto output = std::find(run.output_names.begin(), run.output_names.end(), signal);
  if (output != run.output_names.end()) {
    return run.outputs[sample](static_cast<Eigen::Index>(output - run.output_names.begin()));
  }
  const Eigen::VectorXd controls = run.applied_controls.size() == run.states.size()
                                       ? run.applied_controls[sample]
                                       : Eigen::VectorXd::Zero(run.model->control_count());
  const model::Environment environment = [&] {
    model::Environment value = model::Environment::at_geometric_altitude(0.0);
    if (run.wind_samples_ned_m_s.size() == run.times_s.size()) {
      value.wind_ned_m_s = run.wind_samples_ned_m_s[sample];
    }
    return value;
  }();
  const auto named = helicopter_named_values(*run.model, run.states[sample], controls, environment);
  const auto found = named.find(signal);
  if (found == named.end()) {
    throw std::invalid_argument("analyze.helicopter_response names unknown signal '" + signal
                                + "'");
  }
  return found->second;
}

Eigen::VectorXd named_vector(const std::map<std::string, double>& values,
                             const std::vector<std::string>& names,
                             const std::string& description) {
  Eigen::VectorXd result(static_cast<Eigen::Index>(names.size()));
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto found = values.find(names[i]);
    if (found == values.end()) {
      throw std::runtime_error(description + " names '" + names[i]
                               + "', but the helicopter has no named value with that name");
    }
    result(static_cast<Eigen::Index>(i)) = found->second;
  }
  return result;
}

struct PidLoopRuntime {
  std::string measurement;
  std::string control;
  int control_index = -1;
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double derivative_filter_s = 0.0;
  double integrator_limit = std::numeric_limits<double>::infinity();
  double anti_windup_gain = 0.0;
  double reference = 0.0;
  double integrator = 0.0;
  double filtered_derivative = 0.0;
  double previous_error = 0.0;
  double last_requested = 0.0;
  double last_saturated = 0.0;
  bool has_previous_error = false;
};

struct ClosedLoopControllerRuntime {
  enum class Kind { StateFeedback, Pid };

  Kind kind = Kind::StateFeedback;
  std::string missing_measurement = "refuse";
  std::vector<std::string> measurement_names;
  std::vector<std::string> controller_state_names;
  std::vector<double> references;
  Eigen::MatrixXd gain;
  Eigen::VectorXd trim_controls;
  double period_s = 0.0;
  std::vector<PidLoopRuntime> pid_loops;
  sim::InputSchedule reference_schedule;
  Eigen::VectorXd last_effective_measurement;
  bool have_last_effective_measurement = false;

  [[nodiscard]] sim::SampledTick update(int tick,
                                        double time_s,
                                        const std::map<std::string, double>& truth,
                                        const sim::SensorReading& reading) {
    (void)tick;
    (void)named_vector(truth, measurement_names, "sim.helicopter.closed_loop controller");
    if (!reference_schedule.empty()) {
      const Eigen::VectorXd scheduled = reference_schedule.at(time_s);
      references.resize(static_cast<std::size_t>(scheduled.size()));
      for (Eigen::Index i = 0; i < scheduled.size(); ++i) {
        references[static_cast<std::size_t>(i)] = scheduled(i);
      }
    }
    Eigen::VectorXd measurement = reading.values;
    if (!reading.available) {
      if (missing_measurement == "refuse") {
        throw std::runtime_error(
            "sim.helicopter.closed_loop: controller measurement is unavailable at t="
            + fixed(time_s, 6) + " s; choose missing_measurement: hold_last or trim_fallback "
              "to make that policy explicit");
      }
      if (missing_measurement == "hold_last" && have_last_effective_measurement) {
        measurement = last_effective_measurement;
      } else if (missing_measurement == "trim_fallback") {
        measurement = Eigen::Map<const Eigen::VectorXd>(
            references.data(), static_cast<Eigen::Index>(references.size()));
      } else if (missing_measurement == "hold_last") {
        measurement = Eigen::Map<const Eigen::VectorXd>(
            references.data(), static_cast<Eigen::Index>(references.size()));
      } else {
        throw std::runtime_error("sim.helicopter.closed_loop: unknown missing_measurement policy '"
                                 + missing_measurement + "'");
      }
    }
    if (measurement.size() != static_cast<Eigen::Index>(measurement_names.size())
        || !measurement.allFinite()) {
      throw std::runtime_error(
          "sim.helicopter.closed_loop: effective controller measurement is not finite");
    }
    last_effective_measurement = measurement;
    have_last_effective_measurement = true;

    sim::SampledTick record;
    record.measurement = measurement;
    record.measurement_available = reading.available;
    record.measurement_stale = reading.stale;
    record.requested_controls = trim_controls;
    record.controller_state =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(controller_state_names.size()));
    Eigen::VectorXd errors(static_cast<Eigen::Index>(references.size()));
    for (std::size_t i = 0; i < references.size(); ++i) {
      errors(static_cast<Eigen::Index>(i)) =
          references[i] - measurement(static_cast<Eigen::Index>(i));
    }
    if (kind == Kind::StateFeedback) {
      record.requested_controls = trim_controls + gain * errors;
    } else {
      for (std::size_t i = 0; i < pid_loops.size(); ++i) {
        PidLoopRuntime& loop = pid_loops[i];
        const double error = errors(static_cast<Eigen::Index>(i));
        const double derivative =
            loop.has_previous_error ? (error - loop.previous_error) / period_s : 0.0;
        const double alpha = loop.derivative_filter_s / (loop.derivative_filter_s + period_s);
        loop.filtered_derivative = alpha * loop.filtered_derivative + (1.0 - alpha) * derivative;
        const double integral_drive = loop.ki * error * period_s;
        const double back_calculation =
            loop.anti_windup_gain * (loop.last_saturated - loop.last_requested) * period_s;
        // Conditional integration handles the actuator limitation directly:
        // when the error-driven integral term would push an already-limited
        // request farther into saturation, freeze that drive and retain only
        // the declared back-calculation unwind. This is intentionally
        // separate from the finite integrator bound; a bound alone is not
        // anti-windup.
        const bool drives_further_into_saturation =
            std::fabs(loop.last_requested - loop.last_saturated) > 1.0e-12
            && (loop.last_requested - loop.last_saturated) * integral_drive > 0.0;
        loop.integrator +=
            (drives_further_into_saturation ? 0.0 : integral_drive) + back_calculation;
        loop.integrator =
            std::clamp(loop.integrator, -loop.integrator_limit, loop.integrator_limit);
        loop.previous_error = error;
        loop.has_previous_error = true;
      }
      record.requested_controls = trim_controls;
      for (std::size_t i = 0; i < pid_loops.size(); ++i) {
        const PidLoopRuntime& loop = pid_loops[i];
        const Eigen::Index control_index = static_cast<Eigen::Index>(loop.control_index);
        const double error = errors(static_cast<Eigen::Index>(i));
        record.requested_controls(control_index) +=
            loop.kp * error + loop.integrator + loop.kd * loop.filtered_derivative;
        record.controller_state(static_cast<Eigen::Index>(2 * i)) = loop.integrator;
        record.controller_state(static_cast<Eigen::Index>(2 * i + 1)) = loop.filtered_derivative;
      }
    }
    record.references = Eigen::Map<const Eigen::VectorXd>(
        references.data(), static_cast<Eigen::Index>(references.size()));
    record.errors = errors;
    return record;
  }
};

ClosedLoopControllerRuntime make_closed_loop_controller(const StageContext& context,
                                                        const model::HelicopterModel& heli,
                                                        const HelicopterTrimArtifact& trim,
                                                        double period_s) {
  const ValuePtr declared = context.input->get("controller");
  require_map_keys(declared,
                   {"type", "missing_measurement", "references", "loops"},
                   "sim.helicopter.closed_loop controller");
  const std::string type = declared->string_at("type");
  const std::string missing = declared->string_at("missing_measurement", "refuse");
  if (missing != "refuse" && missing != "hold_last" && missing != "trim_fallback") {
    throw std::runtime_error(
        "sim.helicopter.closed_loop controller.missing_measurement must be 'refuse', "
        "'hold_last' or 'trim_fallback'");
  }

  ClosedLoopControllerRuntime controller;
  controller.missing_measurement = missing;
  controller.trim_controls = trim.holding_controls;
  controller.period_s = period_s;
  const auto trim_values = helicopter_named_values(
      heli, trim.result.extended_state, trim.holding_controls, trim.environment);

  if (type == "state_feedback") {
    controller.kind = ClosedLoopControllerRuntime::Kind::StateFeedback;
    const Artifact& law_artifact = context.upstream_at("law");
    std::vector<std::string> input_names;
    if (law_artifact.kind == "control_law") {
      const auto& law = law_artifact.payload_as<synth::LqrDesign>("control_law");
      controller.gain = law.riccati.k;
      controller.measurement_names = law.plant.state_names;
      input_names = law.plant.input_names;
    } else if (law_artifact.kind == "sampled_control_law") {
      const auto& law = law_artifact.payload_as<synth::SampledLqrDesign>("sampled_control_law");
      if (law.sample_time_s != period_s) {
        throw std::runtime_error(
            "sim.helicopter.closed_loop: sampled LQR sample_time_s must exactly match "
            "controller_period_s");
      }
      controller.gain = law.riccati.k;
      controller.measurement_names = law.discretisation.system.state_names;
      input_names = law.discretisation.system.input_names;
    } else {
      throw std::runtime_error(
          "sim.helicopter.closed_loop: state_feedback requires a control_law from synth.lqr "
          "or sampled_control_law from synth.sampled_lqr");
    }
    if (input_names != heli.control_names()) {
      throw std::runtime_error(
          "sim.helicopter.closed_loop: the feedback law inputs must exactly match the "
          "helicopter control_names order");
    }
    if (controller.gain.rows() != heli.control_count()
        || controller.gain.cols()
               != static_cast<Eigen::Index>(controller.measurement_names.size())) {
      throw std::runtime_error(
          "sim.helicopter.closed_loop: feedback gain dimensions do not match its named "
          "measurement and helicopter control channels");
    }
    if (!controller.gain.allFinite()) {
      throw std::runtime_error("sim.helicopter.closed_loop: feedback gain must be finite");
    }
    std::set<std::string> unique_names(controller.measurement_names.begin(),
                                       controller.measurement_names.end());
    if (unique_names.size() != controller.measurement_names.size()) {
      throw std::runtime_error(
          "sim.helicopter.closed_loop: feedback law contains duplicate state names");
    }
    controller.references.resize(controller.measurement_names.size());
    for (std::size_t i = 0; i < controller.measurement_names.size(); ++i) {
      controller.references[i] = trim_values.at(controller.measurement_names[i]);
    }
    const ValuePtr references = declared->get("references");
    if (references) {
      if (references->kind() != Value::Kind::Map) {
        throw std::runtime_error(
            "sim.helicopter.closed_loop controller.references must be a map of named values");
      }
      for (const auto& [name, value] : references->as_map()) {
        const auto found = std::find(
            controller.measurement_names.begin(), controller.measurement_names.end(), name);
        if (found == controller.measurement_names.end() || value->kind() != Value::Kind::Number
            || !std::isfinite(value->as_number())) {
          throw std::runtime_error(
              "sim.helicopter.closed_loop controller.references must name finite controller "
              "measurements");
        }
        controller
            .references[static_cast<std::size_t>(found - controller.measurement_names.begin())] =
            value->as_number();
      }
    }
  } else if (type == "pid") {
    controller.kind = ClosedLoopControllerRuntime::Kind::Pid;
    const ValuePtr loops = declared->get("loops");
    if (!loops || loops->kind() != Value::Kind::List || loops->as_list().empty()) {
      throw std::runtime_error(
          "sim.helicopter.closed_loop controller.loops must contain at least one PID loop");
    }
    const auto control_names = heli.control_names();
    std::set<std::string> used_measurements;
    std::set<std::string> used_controls;
    for (std::size_t i = 0; i < loops->as_list().size(); ++i) {
      const ValuePtr& entry = loops->as_list()[i];
      const std::string description =
          "sim.helicopter.closed_loop controller.loops[" + std::to_string(i) + "]";
      require_map_keys(entry,
                       {"measurement",
                        "control",
                        "kp",
                        "ki",
                        "kd",
                        "derivative_filter_s",
                        "integrator_limit",
                        "anti_windup_gain",
                        "reference"},
                       description);
      PidLoopRuntime loop;
      loop.measurement = entry->string_at("measurement");
      loop.control = entry->string_at("control");
      if (!used_measurements.insert(loop.measurement).second) {
        throw std::runtime_error(description + " duplicates measurement '" + loop.measurement
                                 + "'");
      }
      if (!used_controls.insert(loop.control).second) {
        throw std::runtime_error(description + " duplicates control '" + loop.control + "'");
      }
      const auto control = std::find(control_names.begin(), control_names.end(), loop.control);
      if (control == control_names.end()) {
        throw std::runtime_error(description + " names unknown control '" + loop.control + "'");
      }
      loop.control_index = static_cast<int>(control - control_names.begin());
      loop.kp = entry->number_at("kp");
      loop.ki = entry->number_at("ki");
      loop.kd = entry->number_at("kd");
      loop.derivative_filter_s = entry->number_at("derivative_filter_s");
      loop.integrator_limit =
          entry->number_at("integrator_limit", std::numeric_limits<double>::infinity());
      loop.anti_windup_gain = entry->number_at("anti_windup_gain", 0.0);
      if (!std::isfinite(loop.kp) || !std::isfinite(loop.ki) || !std::isfinite(loop.kd)
          || !(loop.derivative_filter_s > 0.0) || !std::isfinite(loop.derivative_filter_s)
          || loop.integrator_limit < 0.0
          || (!std::isfinite(loop.integrator_limit)
              && loop.integrator_limit != std::numeric_limits<double>::infinity())
          || !std::isfinite(loop.anti_windup_gain) || loop.anti_windup_gain < 0.0) {
        throw std::runtime_error(description
                                 + " requires finite gains, a positive derivative_filter_s, "
                                   "a non-negative integrator_limit and a non-negative finite "
                                   "anti_windup_gain");
      }
      const auto trim_value = trim_values.find(loop.measurement);
      if (trim_value == trim_values.end()) {
        throw std::runtime_error(description + " names unknown measurement '" + loop.measurement
                                 + "'");
      }
      loop.reference = entry->number_at("reference", trim_value->second);
      if (!std::isfinite(loop.reference)) {
        throw std::runtime_error(description + ".reference must be finite");
      }
      loop.last_requested = trim.holding_controls(loop.control_index);
      loop.last_saturated = loop.last_requested;
      controller.measurement_names.push_back(loop.measurement);
      controller.references.push_back(loop.reference);
      controller.controller_state_names.push_back(loop.control + ".integrator");
      controller.controller_state_names.push_back(loop.control + ".filtered_derivative");
      controller.pid_loops.push_back(std::move(loop));
    }
  } else {
    throw std::runtime_error(
        "sim.helicopter.closed_loop controller.type must be 'state_feedback' or 'pid'");
  }
  return controller;
}

std::unique_ptr<sim::DeterministicSensor> make_closed_loop_sensor(
    const StageContext& context,
    const std::vector<std::string>& measurement_names,
    double step_s,
    std::vector<std::string>& provenance) {
  const ValuePtr declared = context.input->get("sensor");
  if (!declared) {
    provenance.push_back("measurement source: perfect named plant values");
    return nullptr;
  }
  require_map_keys(declared,
                   {"name",
                    "channels",
                    "seed",
                    "algorithm_version",
                    "sample_period_s",
                    "latency_s",
                    "bias",
                    "white_noise_stddev",
                    "quantization_step",
                    "saturation_min",
                    "saturation_max",
                    "dropout_samples",
                    "dropout_policy"},
                   "sim.helicopter.closed_loop sensor");
  const ValuePtr channels = declared->get("channels");
  if (!channels || channels->kind() != Value::Kind::List
      || channels->as_list().size() != measurement_names.size()) {
    throw std::runtime_error(
        "sim.helicopter.closed_loop sensor.channels must match the controller measurement count");
  }
  std::vector<std::string> channel_names;
  channel_names.reserve(measurement_names.size());
  for (const ValuePtr& channel : channels->as_list()) {
    if (channel->kind() != Value::Kind::String) {
      throw std::runtime_error("sim.helicopter.closed_loop sensor.channels must contain strings");
    }
    channel_names.push_back(channel->as_string());
  }
  if (channel_names != measurement_names) {
    throw std::runtime_error(
        "sim.helicopter.closed_loop sensor.channels must be the controller measurement names "
        "in the same order");
  }
  const ValuePtr seed = declared->get("seed");
  if (!seed || seed->kind() != Value::Kind::Number || !std::isfinite(seed->as_number())
      || seed->as_number() < 0.0 || std::floor(seed->as_number()) != seed->as_number()
      || seed->as_number() > 9007199254740991.0) {
    throw std::runtime_error(
        "sim.helicopter.closed_loop sensor.seed must be a non-negative integer exactly "
        "representable by the study number format");
  }
  sim::SensorConfiguration configuration;
  configuration.name = declared->string_at("name", "helicopter_sensor");
  configuration.channel_names = channel_names;
  configuration.seed = static_cast<std::uint64_t>(seed->as_number());
  configuration.algorithm_version =
      declared->string_at("algorithm_version", "mt19937_64_box_muller_v1");
  configuration.sample_period_s = declared->number_at("sample_period_s");
  configuration.latency_s = declared->number_at("latency_s", 0.0);
  configuration.bias = value_vector(declared->get("bias"),
                                    static_cast<Eigen::Index>(channel_names.size()),
                                    "sim.helicopter.closed_loop sensor.bias");
  configuration.white_noise_stddev =
      value_vector(declared->get("white_noise_stddev"),
                   static_cast<Eigen::Index>(channel_names.size()),
                   "sim.helicopter.closed_loop sensor.white_noise_stddev");
  configuration.quantization_step = declared->number_at("quantization_step", 0.0);
  configuration.saturation_min =
      declared->number_at("saturation_min", -std::numeric_limits<double>::infinity());
  configuration.saturation_max =
      declared->number_at("saturation_max", std::numeric_limits<double>::infinity());
  configuration.dropout_policy = declared->string_at("dropout_policy", "unavailable");
  const ValuePtr dropouts = declared->get("dropout_samples");
  if (dropouts) {
    if (dropouts->kind() != Value::Kind::List) {
      throw std::runtime_error(
          "sim.helicopter.closed_loop sensor.dropout_samples must be a list of integers");
    }
    for (const ValuePtr& entry : dropouts->as_list()) {
      if (entry->kind() != Value::Kind::Number || !std::isfinite(entry->as_number())
          || entry->as_number() < 0.0 || std::floor(entry->as_number()) != entry->as_number()
          || entry->as_number() > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::runtime_error(
            "sim.helicopter.closed_loop sensor.dropout_samples must contain non-negative "
            "integers");
      }
      configuration.dropout_samples.push_back(static_cast<int>(entry->as_number()));
    }
  }
  auto sensor = std::make_unique<sim::DeterministicSensor>(configuration, step_s);
  provenance.push_back("sensor name: " + configuration.name);
  provenance.push_back("sensor algorithm: " + configuration.algorithm_version);
  provenance.push_back("sensor seed: " + std::to_string(configuration.seed));
  provenance.push_back("sensor sample_period_s: " + fixed(configuration.sample_period_s, 9));
  provenance.push_back("sensor latency_s: " + fixed(configuration.latency_s, 9));
  for (const auto& stream : sensor->stream_ids()) {
    provenance.push_back("sensor stream: " + stream);
  }
  return sensor;
}

// ---------------------------------------------------------------------------
// model.helicopter
// ---------------------------------------------------------------------------

Artifact load_helicopter_capability(const StageContext& context) {
  const std::string declared = context.input->string_at("path");
  const std::string bytes = context.read_input(declared);
  auto model = std::make_shared<model::HelicopterModel>(model::parse_helicopter(bytes, declared));
  const auto parameter_overrides =
      apply_helicopter_parameter_overrides(*model, context.input->get("parameter_overrides"));

  Artifact artifact;
  artifact.kind = "helicopter";
  std::ostringstream summary;
  summary << model->extended_state_size() << " states, " << model->control_count()
          << " controls; main rotor R = " << fixed(model->main_rotor.radius_m, 2)
          << " m, sigma = " << fixed(model->main_rotor.solidity(), 4) << ", tail arm "
          << fixed(-model->tail_rotor.position_cg_to_hub_body_m.x(), 2) << " m — "
          << model->description();
  artifact.summary = summary.str();
  artifact.payload =
      HelicopterArtifact{std::move(model), declared, core::sha256(bytes), parameter_overrides};
  return artifact;
}

// ---------------------------------------------------------------------------
// trim.helicopter
// ---------------------------------------------------------------------------

Artifact trim_helicopter_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("helicopter");
  const auto& subject = upstream.payload_as<HelicopterArtifact>("helicopter");
  const model::HelicopterModel& heli = *subject.model;

  // Airspeed and altitude are DECLARED, not solved: they are the flight
  // condition the trim is asked about, not an output of it.
  const double airspeed = context.input->number_at("airspeed_m_s", 0.0);
  const double altitude = context.input->number_at("altitude_m", 0.0);
  const double delta_isa_k = context.input->number_at("delta_isa_k", 0.0);
  if (!(airspeed >= 0.0)) {
    throw std::runtime_error(
        "trim.helicopter: airspeed_m_s must be non-negative. A trim is stated in wind axes, and a "
        "negative airspeed is a heading, not a speed");
  }
  if (!std::isfinite(altitude)) {
    throw std::runtime_error("trim.helicopter: altitude_m must be finite");
  }
  if (!std::isfinite(delta_isa_k)) {
    throw std::runtime_error("trim.helicopter: delta_isa_k must be finite");
  }

  model::Environment environment;
  try {
    environment = model::Environment::at_geometric_altitude(altitude, delta_isa_k);
  } catch (const std::exception& error) {
    throw std::runtime_error("trim.helicopter: cannot construct the declared atmosphere at "
                             + fixed(altitude, 3) + " m: " + error.what());
  }

  // A declared steady wind, which the trim sees as a shift of the air-relative
  // velocity. Non-steady wind is not a trim condition: an equilibrium in a gust
  // does not exist.
  if (context.input->get("wind_ned_m_s") != nullptr) {
    const auto& list = context.input->get("wind_ned_m_s")->as_list();
    if (list.size() != 3) {
      throw std::runtime_error("trim.helicopter: wind_ned_m_s must be three numbers");
    }
    for (int i = 0; i < 3; ++i) {
      environment.wind_ned_m_s(i) = list[static_cast<std::size_t>(i)]->as_number();
    }
  }

  trim::TrimCondition condition;
  condition.environment = environment;
  condition.controls = Eigen::VectorXd::Zero(heli.control_count());
  core::State state;
  state.attitude_body_to_ned = core::identity_attitude();
  state.velocity_body_m_s = Eigen::Vector3d(airspeed, 0.0, 0.0);
  state.position_ned_m = Eigen::Vector3d(0.0, 0.0, -altitude);
  condition.extended_state =
      heli.join(state, heli.initial_auxiliary(condition.controls, environment));

  trim::TrimOptions options;
  options.iterations = context.input->integer_at("iterations", options.iterations);
  options.residual_tolerance =
      context.input->number_at("residual_tolerance", options.residual_tolerance);

  const trim::TrimProblem problem = trim::helicopter_trim_problem(heli);
  const trim::TrimResult result = trim::solve_trim(heli, problem, condition, options);

  // THE COMMANDS THAT HOLD THE TRIM. The trim solves actuator POSITIONS; the
  // commands that keep them there are the same numbers. Without this the
  // actuator rates are non-zero at the point and it is not an equilibrium of the
  // whole model — which linearize.vehicle refuses, correctly and confusingly.
  Eigen::VectorXd holding = result.controls;
  const Eigen::VectorXd auxiliary = heli.auxiliary_part(result.extended_state);
  for (int i = 0; i < heli.control_count(); ++i) {
    holding(i) = auxiliary(model::kCollectivePosition + i);
  }

  const auto breakdown = heli.breakdown(
      model::VehicleModel::rigid_body_part(result.extended_state), auxiliary, holding, environment);

  Artifact artifact;
  artifact.kind = "helicopter_trim";
  const auto value_of = [&](const std::string& name) {
    const auto it = std::find(result.unknown_names.begin(), result.unknown_names.end(), name);
    return it == result.unknown_names.end()
               ? 0.0
               : result.unknown_values(it - result.unknown_names.begin());
  };
  std::ostringstream summary;
  summary << "airspeed " << fixed(airspeed, 2) << " m/s; collective "
          << fixed(value_of("collective_rad") * kDegreesPerRadian, 2) << " deg, cyclic "
          << fixed(value_of("longitudinal_cyclic_rad") * kDegreesPerRadian, 2) << "/"
          << fixed(value_of("lateral_cyclic_rad") * kDegreesPerRadian, 2) << " deg, pedal "
          << fixed(value_of("pedal_rad") * kDegreesPerRadian, 2) << " deg; roll "
          << fixed(value_of("roll_rad") * kDegreesPerRadian, 2) << " deg, pitch "
          << fixed(value_of("pitch_rad") * kDegreesPerRadian, 2) << " deg; main thrust "
          << fixed(breakdown.main.thrust_n, 0) << " N, tail thrust "
          << fixed(breakdown.tail.thrust_n, 0) << " N, power "
          << fixed(breakdown.total_power_w / 1000.0, 1) << " kW; residual "
          << scientific(result.residual_norm) << " after " << result.iterations
          << " iterations, Jacobian rank " << result.jacobian_rank << "/" << result.unknown_count
          << ", condition " << scientific(result.jacobian_condition_number);
  summary << "; atmosphere altitude " << fixed(environment.atmospheric_altitude_m, 1)
          << " m, density " << fixed(environment.density_kg_m3, 4) << " kg/m^3";
  if (result.envelope.outside) {
    summary << "; ENVELOPE: " << result.envelope.reason;
  }
  artifact.summary = summary.str();
  artifact.payload = HelicopterTrimArtifact{subject.model,
                                            result,
                                            environment,
                                            std::move(holding),
                                            breakdown,
                                            subject.parameter_overrides};
  return artifact;
}

// ---------------------------------------------------------------------------
// linearize.vehicle
// ---------------------------------------------------------------------------

Artifact linearize_vehicle_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("trim");
  const auto& subject = upstream.payload_as<HelicopterTrimArtifact>("helicopter_trim");

  linearize::VehicleLinearisationOptions options;
  options.equilibrium_tolerance =
      context.input->number_at("equilibrium_tolerance", options.equilibrium_tolerance);

  const auto full = linearize::linearize_vehicle(*subject.model,
                                                 subject.result.extended_state,
                                                 subject.holding_controls,
                                                 subject.environment,
                                                 options);
  const bool reduce = context.input->bool_at("drop_position_and_heading", true);
  const auto linearisation = reduce ? full.reduced() : full;

  Artifact artifact;
  artifact.kind = "linear_system";
  std::ostringstream summary;
  summary << linearisation.a.rows() << " states, " << linearisation.b.cols()
          << " inputs; equilibrium residual " << scientific(full.equilibrium_residual)
          << ", worst column-relative truncation " << scientific(full.worst_relative_truncation);
  if (reduce) {
    summary << "; position and heading dropped by name";
  }
  if (!full.saturated_controls.empty()) {
    summary << "; SATURATED at the point:";
    for (const auto& name : full.saturated_controls) {
      summary << " " << name;
    }
    summary << " — the control derivative for each of those is a one-sided difference and is HALF "
               "its true value";
  }
  artifact.summary = summary.str();
  artifact.payload =
      linearisation.to_linear_system("Souxmar helicopter linearised about its declared trim");
  return artifact;
}

// ---------------------------------------------------------------------------
// sim.helicopter
// ---------------------------------------------------------------------------

Artifact simulate_helicopter_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("trim");
  const auto& subject = upstream.payload_as<HelicopterTrimArtifact>("helicopter_trim");
  const model::HelicopterModel& heli = *subject.model;

  const double step_s = context.input->number_at("step_s");
  const int steps = context.input->integer_at("steps", 0);
  const int stride = context.input->integer_at("sample_stride", 1);
  if (!(step_s > 0.0) || steps < 1) {
    throw std::runtime_error("sim.helicopter: step_s must be positive and steps at least one");
  }

  // The integration method is DECLARED. Fixed-step RK4 is the default and the
  // only one gated results use; an A-stable method is available for a model
  // whose flap or inflow lags make the explicit step impractical (ADR-0021).
  const std::string method_name = context.input->string_at("method", "rk4");
  numerics::IntegrationMethod method = numerics::IntegrationMethod::Rk4Fixed;
  if (method_name == "rk4") {
    method = numerics::IntegrationMethod::Rk4Fixed;
  } else if (method_name == "implicit_euler") {
    method = numerics::IntegrationMethod::ImplicitEulerFixed;
  } else if (method_name == "trapezoidal") {
    method = numerics::IntegrationMethod::TrapezoidalFixed;
  } else {
    throw std::runtime_error(
        "sim.helicopter: method must be 'rk4', 'implicit_euler' or "
        "'trapezoidal'; got '"
        + method_name + "'");
  }

  // A commanded step on one control, from the trim. Declared as a control NAME
  // and a size, so the study says what it is doing rather than indexing a
  // vector.
  Eigen::VectorXd commands = subject.holding_controls;
  std::string step_description = "no input step";
  if (context.input->get("step_control") != nullptr) {
    const std::string name = context.input->string_at("step_control");
    const double size = context.input->number_at("step_size_rad", 0.0);
    const auto names = heli.control_names();
    const auto it = std::find(names.begin(), names.end(), name);
    if (it == names.end()) {
      std::ostringstream message;
      message << "sim.helicopter: step_control '" << name
              << "' is not a control of this model. It "
                 "has {";
      for (std::size_t i = 0; i < names.size(); ++i) {
        message << (i ? ", " : "") << names[i];
      }
      message << "}";
      throw std::runtime_error(message.str());
    }
    commands(it - names.begin()) += size;
    step_description = name + " stepped by " + fixed(size * kDegreesPerRadian, 3) + " deg";
  }

  const InitialCondition initial =
      initial_condition_from(context, heli, subject.result.extended_state);
  const sim::InputSchedule wind_schedule =
      wind_schedule_from(context, initial.state, subject.environment, step_s, steps);
  validate_schedule_horizon(wind_schedule, static_cast<double>(steps) * step_s, "wind_schedule");
  const std::vector<WindEvent> wind_schedule_events =
      wind_schedule.empty() ? std::vector<WindEvent>{} : wind_events(wind_schedule, step_s, steps);

  numerics::IntegrationOptions options;
  options.method = method;
  options.step_s = step_s;
  options.step_count = steps;
  options.sample_stride = stride;
  options.newton_iterations = context.input->integer_at("newton_iterations", 3);

  // A run owns a mutable copy of the model when it has scheduled failures.
  // Healthy runs still use one integration call, preserving their historical
  // operation ordering and bit pattern. Event times are required to lie on
  // the fixed-step lattice; an event between steps is refused instead of
  // silently moving the failure in time.
  model::HelicopterModel simulation_model = *subject.model;
  const double mass_scale = mass_scale_from(context);
  const Eigen::Vector3d cg_offset_body_m = cg_offset_from(context);
  apply_parameter_variation(simulation_model, mass_scale, cg_offset_body_m);
  const auto derivative = [&](double, const Eigen::VectorXd& x) {
    return Eigen::VectorXd(simulation_model.derivative(x, commands, subject.environment));
  };
  const std::vector<FailureEvent> failure_events =
      parse_failure_events(context, heli, step_s, steps);

  numerics::IntegrationResult result;
  std::vector<std::string> applied_failure_events;
  if (failure_events.empty() && wind_schedule.empty() && initial.provenance.size() == 1) {
    result = numerics::integrate(derivative,
                                 initial.state,
                                 0.0,
                                 options,
                                 &model::VehicleModel::project,
                                 simulation_model.state_bounds());
  } else if (wind_schedule.empty()) {
    std::vector<double> all_times{0.0};
    std::vector<Eigen::VectorXd> all_states{initial.state};
    Eigen::VectorXd state = initial.state;
    int current_step = 0;
    bool completed = true;
    numerics::TerminationReason reason = numerics::TerminationReason::Completed;
    std::string detail;
    int steps_taken = 0;
    long long newton_iterations = 0;

    const auto integrate_to = [&](int target_step) {
      numerics::IntegrationOptions segment = options;
      segment.step_count = target_step - current_step;
      segment.sample_stride = 1;
      const auto segment_result = numerics::integrate(derivative,
                                                      state,
                                                      static_cast<double>(current_step) * step_s,
                                                      segment,
                                                      &model::VehicleModel::project,
                                                      simulation_model.state_bounds());
      newton_iterations += segment_result.newton_iterations_performed;
      for (std::size_t i = 1; i < segment_result.trajectory.states.size(); ++i) {
        all_times.push_back(segment_result.trajectory.times_s[i]);
        all_states.push_back(segment_result.trajectory.states[i]);
      }
      state = all_states.back();
      steps_taken = current_step + segment_result.steps_taken;
      if (!segment_result.completed()) {
        completed = false;
        reason = segment_result.reason;
        detail = segment_result.detail;
        return false;
      }
      current_step = target_step;
      steps_taken = current_step;
      return true;
    };

    std::size_t next_event = 0;
    while (next_event < failure_events.size() && completed) {
      const int target_step = failure_events[next_event].step;
      if (target_step < current_step || !integrate_to(target_step)) {
        break;
      }
      while (next_event < failure_events.size() && failure_events[next_event].step == target_step) {
        applied_failure_events.push_back(
            apply_failure_event(failure_events[next_event], simulation_model, state));
        all_states.back() = state;
        ++next_event;
      }
    }
    if (completed && current_step < steps) {
      (void)integrate_to(steps);
    }

    result.method = method;
    result.reason = reason;
    result.detail = detail;
    result.steps_taken = steps_taken;
    result.newton_iterations_performed = newton_iterations;
    result.termination_time_s = all_times.back();
    result.trajectory.step_s = step_s;
    result.trajectory.step_count = steps_taken;
    result.trajectory.sample_stride = stride;
    const auto has_event_at_step = [&failure_events](int step) {
      return std::any_of(failure_events.begin(),
                         failure_events.end(),
                         [step](const FailureEvent& event) { return event.step == step; });
    };
    for (std::size_t i = 0; i < all_states.size(); ++i) {
      const int step = static_cast<int>(i);
      if (i == 0 || i + 1 == all_states.size() || step % stride == 0 || has_event_at_step(step)) {
        result.trajectory.times_s.push_back(all_times[i]);
        result.trajectory.states.push_back(all_states[i]);
      }
    }
  } else {
    // A wind history is evaluated by the simulator, not by the model's force
    // routine. Zero-order changes are events: rebase the air-relative velocity
    // by -R^T delta-w before the next step so ground velocity is continuous.
    // Linear histories carry their declared derivative through Environment.
    Eigen::VectorXd state = initial.state;
    Eigen::Vector3d current_wind = Eigen::Vector3d(wind_schedule.at(0.0));
    std::size_t next_failure = 0;
    std::size_t next_wind = 0;
    std::vector<double> all_times{0.0};
    std::vector<Eigen::VectorXd> all_states{state};
    numerics::TerminationReason reason = numerics::TerminationReason::Completed;
    std::string detail;
    int steps_taken = 0;
    for (int step = 0; step <= steps; ++step) {
      const double time_s = static_cast<double>(step) * step_s;
      while (next_wind < wind_schedule_events.size()
             && wind_schedule_events[next_wind].step == step) {
        const Eigen::Vector3d next_wind_value =
            Eigen::Vector3d(wind_schedule.at(wind_schedule_events[next_wind].declared_time_s));
        const core::State at_event = core::State::from_vector(state.head<core::kStateSize>());
        if (wind_schedule.hold() == sim::HoldPolicy::ZeroOrder) {
          state.segment<3>(core::kVelocityU) -=
              core::dcm_body_from_ned(at_event.attitude_body_to_ned)
              * (next_wind_value - current_wind);
        }
        current_wind = next_wind_value;
        ++next_wind;
      }
      while (next_failure < failure_events.size() && failure_events[next_failure].step == step) {
        applied_failure_events.push_back(
            apply_failure_event(failure_events[next_failure], simulation_model, state));
        ++next_failure;
      }
      // The boundary state at this timestamp is the state after all events at
      // the boundary. Without replacing the previously stored sample, the
      // trajectory would pair the post-event wind with the pre-event state,
      // making a physically continuous wind step appear discontinuous in the
      // exported record.
      all_states.back() = state;
      if (step == steps) {
        break;
      }
      const auto scheduled_derivative = [&](double stage_time, const Eigen::VectorXd& current) {
        model::Environment environment = subject.environment;
        environment.wind_ned_m_s = wind_schedule.hold() == sim::HoldPolicy::Linear
                                       ? wind_at(wind_schedule, stage_time, current_wind)
                                       : current_wind;
        if (wind_schedule.hold() == sim::HoldPolicy::Linear) {
          const double read_at =
              std::clamp(std::nextafter(stage_time, -std::numeric_limits<double>::infinity()),
                         wind_schedule.first_time_s(),
                         wind_schedule.last_time_s());
          environment.wind_rate_ned_m_s2 = wind_schedule.rate_at(read_at);
        } else {
          environment.wind_rate_ned_m_s2.setZero();
        }
        return Eigen::VectorXd(simulation_model.derivative(current, commands, environment));
      };
      numerics::IntegrationOptions one_step;
      one_step.method = method;
      one_step.step_s = step_s;
      one_step.step_count = 1;
      one_step.sample_stride = 1;
      one_step.newton_iterations = options.newton_iterations;
      const auto integrated = numerics::integrate(scheduled_derivative,
                                                  state,
                                                  time_s,
                                                  one_step,
                                                  &model::VehicleModel::project,
                                                  simulation_model.state_bounds());
      if (!integrated.completed()) {
        reason = integrated.reason;
        detail = integrated.detail;
        steps_taken = step + integrated.steps_taken;
        break;
      }
      state = integrated.trajectory.states.back();
      all_times.push_back(time_s + step_s);
      all_states.push_back(state);
      steps_taken = step + 1;
    }
    result.method = method;
    result.reason = reason;
    result.detail = detail;
    result.steps_taken = steps_taken;
    result.termination_time_s = all_times.back();
    result.trajectory.step_s = step_s;
    result.trajectory.step_count = steps_taken;
    result.trajectory.sample_stride = stride;
    for (std::size_t i = 0; i < all_states.size(); ++i) {
      if (i == 0 || i + 1 == all_states.size() || (static_cast<int>(i) % stride == 0)) {
        result.trajectory.times_s.push_back(all_times[i]);
        result.trajectory.states.push_back(all_states[i]);
      }
    }
  }

  HelicopterTrajectoryArtifact trajectory;
  trajectory.model = std::make_shared<const model::HelicopterModel>(simulation_model);
  trajectory.times_s = result.trajectory.times_s;
  trajectory.states = result.trajectory.states;
  trajectory.state_names = heli.state_names();
  trajectory.output_names = heli.output_names();
  trajectory.reason = result.reason;
  trajectory.termination_detail = result.detail;
  trajectory.step_s = step_s;
  trajectory.steps_taken = result.steps_taken;
  trajectory.failure_events = std::move(applied_failure_events);
  trajectory.trim_reference_values = helicopter_named_values(
      heli, subject.result.extended_state, subject.holding_controls, subject.environment);
  trajectory.initial_condition_provenance = initial.provenance;
  trajectory.disturbance_provenance = context.input->get("turbulence")
                                          ? turbulence_provenance(context)
                                          : wind_provenance(wind_schedule);
  trajectory.parameter_provenance =
      "mass_scale=" + fixed(mass_scale, 9) + ", cg_offset_body_m=[" + fixed(cg_offset_body_m(0), 9)
      + ", " + fixed(cg_offset_body_m(1), 9) + ", " + fixed(cg_offset_body_m(2), 9) + "]";
  for (const double time_s : trajectory.times_s) {
    trajectory.wind_samples_ned_m_s.push_back(
        wind_at(wind_schedule, time_s, subject.environment.wind_ned_m_s));
  }

  for (std::size_t i = 0; i < trajectory.states.size(); ++i) {
    const auto& state = trajectory.states[i];
    const int step = static_cast<int>(std::llround(trajectory.times_s[i] / step_s));
    const model::HelicopterModel output_model =
        model_at_failure_step(simulation_model, failure_events, step);
    const core::State rigid = model::VehicleModel::rigid_body_part(state);
    const Eigen::VectorXd auxiliary = output_model.auxiliary_part(state);
    trajectory.outputs.push_back(output_model.outputs(rigid, auxiliary, commands, [&] {
      model::Environment environment = subject.environment;
      environment.wind_ned_m_s = trajectory.wind_samples_ned_m_s[i];
      return environment;
    }()));
    const auto envelope = output_model.envelope(rigid, auxiliary, commands, [&] {
      model::Environment environment = subject.environment;
      environment.wind_ned_m_s = trajectory.wind_samples_ned_m_s[i];
      return environment;
    }());
    if (envelope.outside) {
      ++trajectory.envelope_departures;
      if (envelope.worst_departure > trajectory.worst_envelope.worst_departure) {
        trajectory.worst_envelope = envelope;
      }
    }
  }

  Artifact artifact;
  artifact.kind = "helicopter_trajectory";
  std::ostringstream summary;
  summary << trajectory.states.size() << " samples over "
          << fixed(static_cast<double>(result.steps_taken) * step_s, 3) << " s at "
          << numerics::to_string(method) << ", step " << scientific(step_s) << " s; "
          << step_description;
  if (numerics::method_is_implicit(method)) {
    summary << "; " << result.newton_iterations_performed << " Newton iterations";
  }
  // THE TERMINATION REASON REACHES THE SUMMARY, THE REPORT AND THE MANIFEST.
  // A diverged run that says "completed" is the failure this carries.
  if (!result.completed()) {
    summary << "; REFUSED: " << numerics::to_string(result.reason) << " — " << result.detail;
  } else {
    summary << "; completed";
  }
  if (trajectory.envelope_departures > 0) {
    summary << "; OUTSIDE THE DECLARED ENVELOPE at " << trajectory.envelope_departures << " of "
            << trajectory.states.size() << " samples: " << trajectory.worst_envelope.reason;
  }
  if (!trajectory.failure_events.empty()) {
    summary << "; applied " << trajectory.failure_events.size() << " scheduled failure event(s)";
    for (const auto& event : trajectory.failure_events) {
      summary << " [" << event << "]";
    }
  }
  artifact.summary = summary.str();
  artifact.payload = std::move(trajectory);
  return artifact;
}

// ---------------------------------------------------------------------------
// sim.helicopter.closed_loop
// ---------------------------------------------------------------------------

Artifact simulate_helicopter_closed_loop_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("trim");
  const auto& subject = upstream.payload_as<HelicopterTrimArtifact>("helicopter_trim");
  const model::HelicopterModel& heli = *subject.model;

  const double step_s = context.input->number_at("step_s");
  const int steps = context.input->integer_at("steps", 0);
  const int stride = context.input->integer_at("sample_stride", 1);
  const double period_s = context.input->number_at("controller_period_s");
  const int delay_periods = context.input->integer_at("delay_periods", 0);
  if (!(step_s > 0.0) || !std::isfinite(step_s) || steps < 1 || stride < 1 || !(period_s > 0.0)
      || !std::isfinite(period_s) || delay_periods < 0) {
    throw std::runtime_error(
        "sim.helicopter.closed_loop: step_s, controller_period_s and positive steps/stride "
        "are required; delay_periods must be non-negative");
  }
  const std::string method = context.input->string_at("method", "rk4");
  if (method != "rk4") {
    throw std::runtime_error(
        "sim.helicopter.closed_loop: only fixed-step rk4 is supported because controller and "
        "sensor callbacks are defined on that deterministic lattice");
  }

  auto controller = std::make_shared<ClosedLoopControllerRuntime>(
      make_closed_loop_controller(context, heli, subject, period_s));
  const InitialCondition initial =
      initial_condition_from(context, heli, subject.result.extended_state);
  const sim::InputSchedule wind_schedule =
      wind_schedule_from(context, initial.state, subject.environment, step_s, steps);
  validate_schedule_horizon(wind_schedule, static_cast<double>(steps) * step_s, "wind_schedule");
  const std::vector<WindEvent> wind_schedule_events =
      wind_schedule.empty() ? std::vector<WindEvent>{} : wind_events(wind_schedule, step_s, steps);
  controller->reference_schedule =
      schedule_at(context, "reference_schedule", static_cast<int>(controller->references.size()));
  validate_schedule_horizon(
      controller->reference_schedule, static_cast<double>(steps) * step_s, "reference_schedule");
  if (!controller->reference_schedule.empty()
      && controller->reference_schedule.first_time_s() > 0.0) {
    throw std::invalid_argument(
        "reference_schedule must declare a value at t = 0 so the initial controller reference "
        "is unambiguous");
  }
  std::vector<std::string> sensor_provenance;
  auto sensor = std::shared_ptr<sim::DeterministicSensor>(
      make_closed_loop_sensor(context, controller->measurement_names, step_s, sensor_provenance));
  const std::vector<FailureEvent> failure_events =
      parse_failure_events(context, heli, step_s, steps);

  model::HelicopterModel simulation_model = *subject.model;
  const double mass_scale = mass_scale_from(context);
  const Eigen::Vector3d cg_offset_body_m = cg_offset_from(context);
  apply_parameter_variation(simulation_model, mass_scale, cg_offset_body_m);
  std::size_t next_event = 0;
  std::size_t next_wind_event = 0;
  Eigen::Vector3d current_wind = wind_schedule.empty() ? subject.environment.wind_ned_m_s
                                                       : Eigen::Vector3d(wind_schedule.at(0.0));
  std::map<std::string, double> latest_truth;
  sim::SensorReading latest_reading;
  Eigen::VectorXd last_applied = subject.holding_controls;
  std::vector<std::string> applied_failure_events;
  const auto update_measurement = [&](int step, double time_s, Eigen::VectorXd& state) {
    while (next_wind_event < wind_schedule_events.size()
           && wind_schedule_events[next_wind_event].step == step) {
      const Eigen::Vector3d next_wind_value =
          Eigen::Vector3d(wind_schedule.at(wind_schedule_events[next_wind_event].declared_time_s));
      const core::State at_event = core::State::from_vector(state.head<core::kStateSize>());
      if (wind_schedule.hold() == sim::HoldPolicy::ZeroOrder) {
        state.segment<3>(core::kVelocityU) -= core::dcm_body_from_ned(at_event.attitude_body_to_ned)
                                              * (next_wind_value - current_wind);
      }
      current_wind = next_wind_value;
      ++next_wind_event;
    }
    while (next_event < failure_events.size() && failure_events[next_event].step == step) {
      applied_failure_events.push_back(
          apply_failure_event(failure_events[next_event], simulation_model, state));
      ++next_event;
    }
    model::Environment measurement_environment = subject.environment;
    measurement_environment.wind_ned_m_s = current_wind;
    if (!wind_schedule.empty() && wind_schedule.hold() == sim::HoldPolicy::Linear) {
      measurement_environment.wind_ned_m_s = wind_at(wind_schedule, time_s, current_wind);
    }
    latest_truth =
        helicopter_named_values(simulation_model, state, last_applied, measurement_environment);
    const Eigen::VectorXd truth = named_vector(
        latest_truth, controller->measurement_names, "sim.helicopter.closed_loop sensor");
    if (sensor) {
      latest_reading = sensor->sample_if_due(step, time_s, truth);
    } else {
      latest_reading.available = true;
      latest_reading.stale = false;
      latest_reading.sample_index = step;
      latest_reading.time_s = time_s;
      latest_reading.values = truth;
    }
  };

  sim::SampledLoopOptions options;
  options.step_s = step_s;
  options.controller_period_s = period_s;
  options.steps = steps;
  options.sample_stride = stride;
  options.delay_periods = delay_periods;
  options.initial_state = initial.state;
  options.trim_controls = subject.holding_controls;
  options.derivative =
      [&](double time_s, const Eigen::VectorXd& state, const Eigen::VectorXd& controls) {
        model::Environment environment = subject.environment;
        environment.wind_ned_m_s =
            wind_schedule.empty() || wind_schedule.hold() == sim::HoldPolicy::ZeroOrder
                ? current_wind
                : wind_at(wind_schedule, time_s, current_wind);
        if (!wind_schedule.empty() && wind_schedule.hold() == sim::HoldPolicy::Linear) {
          const double read_at =
              std::clamp(std::nextafter(time_s, -std::numeric_limits<double>::infinity()),
                         wind_schedule.first_time_s(),
                         wind_schedule.last_time_s());
          environment.wind_rate_ned_m_s2 = wind_schedule.rate_at(read_at);
        } else {
          environment.wind_rate_ned_m_s2.setZero();
        }
        return Eigen::VectorXd(simulation_model.derivative(state, controls, environment));
      };
  options.projection = &model::VehicleModel::project;
  options.state_bounds = simulation_model.state_bounds();
  options.on_boundary = [&](int step, double time_s, Eigen::VectorXd& state) {
    update_measurement(step, time_s, state);
  };
  options.on_tick = [&](int tick, double time_s, const Eigen::VectorXd& state) {
    (void)state;
    return controller->update(tick, time_s, latest_truth, latest_reading);
  };
  options.saturate = [&](const Eigen::VectorXd& requested) {
    Eigen::VectorXd saturated = requested;
    for (int i = 0; i < heli.control_count(); ++i) {
      const auto& limits = heli.actuators[static_cast<std::size_t>(i)];
      saturated(i) = std::clamp(saturated(i), limits.minimum_rad, limits.maximum_rad);
    }
    if (controller->kind == ClosedLoopControllerRuntime::Kind::Pid) {
      for (auto& loop : controller->pid_loops) {
        loop.last_requested = requested(loop.control_index);
        loop.last_saturated = saturated(loop.control_index);
      }
    }
    return saturated;
  };
  options.on_command_applied = [&](const Eigen::VectorXd& applied) { last_applied = applied; };

  const sim::SampledLoopResult result = sim::run_sampled_loop(options);
  HelicopterTrajectoryArtifact trajectory;
  trajectory.model = std::make_shared<const model::HelicopterModel>(simulation_model);
  trajectory.closed_loop = true;
  trajectory.times_s = result.integration.trajectory.times_s;
  trajectory.states = result.integration.trajectory.states;
  trajectory.state_names = heli.state_names();
  trajectory.output_names = heli.output_names();
  trajectory.reason = result.integration.reason;
  trajectory.termination_detail = result.integration.detail;
  trajectory.step_s = step_s;
  trajectory.steps_taken = result.integration.steps_taken;
  trajectory.failure_events = std::move(applied_failure_events);
  trajectory.trim_reference_values = helicopter_named_values(
      heli, subject.result.extended_state, subject.holding_controls, subject.environment);
  trajectory.initial_condition_provenance = initial.provenance;
  trajectory.disturbance_provenance = context.input->get("turbulence")
                                          ? turbulence_provenance(context)
                                          : wind_provenance(wind_schedule);
  trajectory.parameter_provenance =
      "mass_scale=" + fixed(mass_scale, 9) + ", cg_offset_body_m=[" + fixed(cg_offset_body_m(0), 9)
      + ", " + fixed(cg_offset_body_m(1), 9) + ", " + fixed(cg_offset_body_m(2), 9) + "]";
  trajectory.applied_controls = result.held_controls;
  trajectory.controller_measurement_names = controller->measurement_names;
  trajectory.controller_state_names = controller->controller_state_names;
  trajectory.sensor_provenance = std::move(sensor_provenance);
  for (const auto& tick : result.ticks) {
    trajectory.controller_times_s.push_back(tick.time_s);
    trajectory.controller_measurements.push_back(tick.measurement);
    trajectory.controller_references.push_back(tick.references);
    trajectory.controller_errors.push_back(tick.errors);
    trajectory.controller_requested.push_back(tick.requested_controls);
    trajectory.controller_saturated.push_back(tick.saturated_controls);
    trajectory.controller_applied.push_back(tick.applied_controls);
    trajectory.controller_states.push_back(tick.controller_state);
    trajectory.controller_measurement_available.push_back(tick.measurement_available);
    trajectory.controller_measurement_stale.push_back(tick.measurement_stale);
  }

  for (const double time_s : trajectory.times_s) {
    trajectory.wind_samples_ned_m_s.push_back(
        wind_at(wind_schedule, time_s, subject.environment.wind_ned_m_s));
  }

  for (std::size_t i = 0; i < trajectory.states.size(); ++i) {
    const auto& state = trajectory.states[i];
    const int step = static_cast<int>(std::llround(trajectory.times_s[i] / step_s));
    const model::HelicopterModel output_model =
        model_at_failure_step(simulation_model, failure_events, step);
    const core::State rigid = model::VehicleModel::rigid_body_part(state);
    const Eigen::VectorXd auxiliary = output_model.auxiliary_part(state);
    const Eigen::VectorXd& output_controls =
        trajectory.applied_controls.size() == trajectory.states.size()
            ? trajectory.applied_controls[i]
            : last_applied;
    model::Environment output_environment = subject.environment;
    output_environment.wind_ned_m_s = trajectory.wind_samples_ned_m_s[i];
    trajectory.outputs.push_back(
        output_model.outputs(rigid, auxiliary, output_controls, output_environment));
    const auto envelope =
        output_model.envelope(rigid, auxiliary, output_controls, output_environment);
    if (envelope.outside) {
      ++trajectory.envelope_departures;
      if (envelope.worst_departure > trajectory.worst_envelope.worst_departure) {
        trajectory.worst_envelope = envelope;
      }
    }
  }

  Artifact artifact;
  artifact.kind = "helicopter_trajectory";
  std::ostringstream summary;
  summary << trajectory.states.size() << " samples over "
          << fixed(static_cast<double>(result.integration.steps_taken) * step_s, 3)
          << " s at fixed-step rk4; " << result.ticks.size() << " controller ticks every "
          << fixed(period_s, 6) << " s, delay " << delay_periods << " period(s), "
          << (controller->kind == ClosedLoopControllerRuntime::Kind::Pid ? "PID" : "state feedback")
          << "; " << (sensor ? "deterministic named sensor" : "perfect named measurement");
  if (!result.integration.completed()) {
    summary << "; REFUSED: " << numerics::to_string(result.integration.reason) << " — "
            << result.integration.detail;
  } else {
    summary << "; completed";
  }
  if (!trajectory.failure_events.empty()) {
    summary << "; applied " << trajectory.failure_events.size() << " scheduled failure event(s)";
  }
  if (trajectory.envelope_departures > 0) {
    summary << "; OUTSIDE THE DECLARED ENVELOPE at " << trajectory.envelope_departures
            << " samples: " << trajectory.worst_envelope.reason;
  }
  artifact.summary = summary.str();
  artifact.payload = std::move(trajectory);
  return artifact;
}

// ---------------------------------------------------------------------------
// report.helicopter_csv
// ---------------------------------------------------------------------------

Artifact report_helicopter_csv_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("trajectory");
  const auto& run = upstream.payload_as<HelicopterTrajectoryArtifact>("helicopter_trajectory");
  const std::string path = context.input->string_at("path");

  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "time_s";
  for (const auto& name : run.state_names) {
    out << "," << name;
  }
  for (const auto& name : run.output_names) {
    out << ",output_" << name;
  }
  if (run.wind_samples_ned_m_s.size() == run.times_s.size()) {
    out << ",wind_north_m_s,wind_east_m_s,wind_down_m_s";
  }
  const bool has_held_controls =
      run.closed_loop && run.applied_controls.size() == run.times_s.size();
  if (has_held_controls) {
    for (const auto& name : run.model->control_names()) {
      out << ",applied_" << name;
    }
  }
  out << "\n";
  out << std::setprecision(17);
  for (std::size_t i = 0; i < run.times_s.size(); ++i) {
    out << run.times_s[i];
    for (Eigen::Index j = 0; j < run.states[i].size(); ++j) {
      out << "," << run.states[i](j);
    }
    for (Eigen::Index j = 0; j < run.outputs[i].size(); ++j) {
      out << "," << run.outputs[i](j);
    }
    if (run.wind_samples_ned_m_s.size() == run.times_s.size()) {
      out << "," << run.wind_samples_ned_m_s[i](0) << "," << run.wind_samples_ned_m_s[i](1) << ","
          << run.wind_samples_ned_m_s[i](2);
    }
    if (has_held_controls) {
      for (Eigen::Index j = 0; j < run.applied_controls[i].size(); ++j) {
        out << "," << run.applied_controls[i](j);
      }
    }
    out << "\n";
  }
  context.write_output(path, out.str());

  std::string event_path;
  if (!run.failure_events.empty()) {
    event_path = path + ".events.txt";
    std::ostringstream events;
    events << "# Applied failure events\n";
    for (const auto& event : run.failure_events) {
      events << event << "\n";
    }
    context.write_output(event_path, events.str());
  }

  std::string controller_path;
  if (run.closed_loop) {
    controller_path = path + ".controller.csv";
    std::ostringstream controller;
    controller.imbue(std::locale::classic());
    controller << "time_s";
    for (const auto& name : run.controller_measurement_names) {
      controller << ",measurement_" << name;
    }
    for (const auto& name : run.controller_measurement_names) {
      controller << ",reference_" << name;
    }
    for (const auto& name : run.controller_measurement_names) {
      controller << ",error_" << name;
    }
    for (const auto& name : run.model->control_names()) {
      controller << ",requested_" << name << ",saturated_" << name << ",applied_" << name;
    }
    for (const auto& name : run.controller_state_names) {
      controller << ",controller_state_" << name;
    }
    controller << ",measurement_available,measurement_stale\n";
    controller << std::setprecision(17);
    for (std::size_t i = 0; i < run.controller_times_s.size(); ++i) {
      controller << run.controller_times_s[i];
      for (Eigen::Index j = 0; j < run.controller_measurements[i].size(); ++j) {
        controller << "," << run.controller_measurements[i](j);
      }
      for (Eigen::Index j = 0; j < run.controller_references[i].size(); ++j) {
        controller << "," << run.controller_references[i](j);
      }
      for (Eigen::Index j = 0; j < run.controller_errors[i].size(); ++j) {
        controller << "," << run.controller_errors[i](j);
      }
      for (Eigen::Index j = 0; j < run.controller_requested[i].size(); ++j) {
        controller << "," << run.controller_requested[i](j) << "," << run.controller_saturated[i](j)
                   << "," << run.controller_applied[i](j);
      }
      for (Eigen::Index j = 0; j < run.controller_states[i].size(); ++j) {
        controller << "," << run.controller_states[i](j);
      }
      controller << "," << (run.controller_measurement_available[i] ? 1 : 0) << ","
                 << (run.controller_measurement_stale[i] ? 1 : 0) << "\n";
    }
    context.write_output(controller_path, controller.str());
  }

  Artifact artifact;
  artifact.kind = "report";
  std::ostringstream summary;
  summary << "wrote " << path << " (" << run.times_s.size() << " rows, "
          << run.state_names.size() + run.output_names.size() << " channels)";
  if (run.reason != numerics::TerminationReason::Completed) {
    summary << "; the run it records did NOT complete: " << run.termination_detail;
  }
  if (!event_path.empty()) {
    summary << "; applied failure events in " << event_path;
  }
  if (!controller_path.empty()) {
    summary << "; controller evidence in " << controller_path;
  }
  artifact.summary = summary.str();
  artifact.payload = std::string(path);
  return artifact;
}

// ---------------------------------------------------------------------------
// analyze.helicopter_response
// ---------------------------------------------------------------------------

Artifact analyze_helicopter_response_capability(const StageContext& context) {
  const auto& open = context.upstream_at("open_loop")
                         .payload_as<HelicopterTrajectoryArtifact>("helicopter_trajectory");
  const auto& closed = context.upstream_at("closed_loop")
                           .payload_as<HelicopterTrajectoryArtifact>("helicopter_trajectory");
  bool matched_time_lattice = open.times_s.size() == closed.times_s.size();
  if (matched_time_lattice) {
    for (std::size_t i = 0; i < open.times_s.size(); ++i) {
      matched_time_lattice =
          matched_time_lattice && std::fabs(open.times_s[i] - closed.times_s[i]) <= 1.0e-12;
    }
  }
  if (!matched_time_lattice || open.states.front().size() != closed.states.front().size()
      || open.states.size() != closed.states.size()) {
    throw std::invalid_argument(
        "analyze.helicopter_response requires open_loop and closed_loop on the same time lattice");
  }
  HelicopterResponseArtifact response;
  response.requirements = finite_number_map(context.input->get("requirements"),
                                            "analyze.helicopter_response requirements");
  const std::vector<std::string> signals =
      string_list(context.input->get("signals"), "analyze.helicopter_response signals");
  const std::map<std::string, double> references =
      finite_number_map(context.input->get("reference"), "analyze.helicopter_response reference");

  response.matched_initial_condition =
      (open.states.front() - closed.states.front()).norm() <= 1.0e-12;
  response.matched_disturbance =
      open.wind_samples_ned_m_s.size() == closed.wind_samples_ned_m_s.size();
  if (response.matched_disturbance) {
    for (std::size_t i = 0; i < open.wind_samples_ned_m_s.size(); ++i) {
      response.matched_disturbance =
          response.matched_disturbance
          && (open.wind_samples_ned_m_s[i] - closed.wind_samples_ned_m_s[i]).norm() <= 1.0e-12;
    }
  }
  if (!response.matched_initial_condition || !response.matched_disturbance) {
    throw std::invalid_argument(
        "analyze.helicopter_response requires identical initial conditions and disturbance "
        "histories for the controlled/uncontrolled comparison");
  }

  const ResponseRequirements parsed_requirements = parse_response_requirements(
      context.input->get("requirements"), context.input->get("signal_requirements"), signals);
  response.requirements = parsed_requirements.aggregate;
  const auto aggregate_budget =
      [&parsed_requirements](const std::string& name) -> std::optional<double> {
    const auto found = parsed_requirements.aggregate.find(name);
    return found == parsed_requirements.aggregate.end() ? std::nullopt
                                                        : std::optional<double>(found->second);
  };
  for (const std::string& signal : signals) {
    const auto explicit_reference = references.find(signal);
    const auto measurement = std::find(closed.controller_measurement_names.begin(),
                                       closed.controller_measurement_names.end(),
                                       signal);
    const std::optional<std::size_t> scheduled_index =
        measurement == closed.controller_measurement_names.end()
            ? std::nullopt
            : std::optional<std::size_t>(static_cast<std::size_t>(
                  measurement - closed.controller_measurement_names.begin()));
    const auto reference_at = [&](std::size_t sample) {
      if (explicit_reference != references.end()) {
        return explicit_reference->second;
      }
      if (scheduled_index && !closed.controller_references.empty()) {
        std::size_t tick = 0;
        while (tick + 1 < closed.controller_times_s.size()
               && closed.controller_times_s[tick + 1] <= open.times_s[sample] + 1.0e-12) {
          ++tick;
        }
        return closed.controller_references[tick](static_cast<Eigen::Index>(*scheduled_index));
      }
      const auto trim_reference = open.trim_reference_values.find(signal);
      if (trim_reference != open.trim_reference_values.end()) {
        return trim_reference->second;
      }
      throw std::invalid_argument("analyze.helicopter_response has no trim reference for signal '"
                                  + signal + "'; declare reference explicitly");
    };
    std::vector<double> reference_values(closed.times_s.size());
    std::vector<double> controlled_values(closed.times_s.size());
    std::vector<double> uncontrolled_values(closed.times_s.size());
    for (std::size_t i = 0; i < closed.times_s.size(); ++i) {
      reference_values[i] = reference_at(i);
      controlled_values[i] = trajectory_signal(closed, i, signal);
      uncontrolled_values[i] = trajectory_signal(open, i, signal);
    }
    HelicopterResponseMetric metric;
    metric.signal = signal;
    metric.unit = response_unit(signal);
    const auto signal_requirements = parsed_requirements.per_signal.find(signal);
    const std::string suffix = response_unit_suffix(signal);
    const auto signal_budget = [&](const std::string& name) -> std::optional<double> {
      if (signal_requirements != parsed_requirements.per_signal.end()) {
        const auto found = signal_requirements->second.find(name);
        if (found != signal_requirements->second.end()) {
          return found->second;
        }
      }
      return std::nullopt;
    };
    const auto settling_band = signal_budget("settling_band_" + suffix)
                                   .value_or(aggregate_budget("tracking_error").value_or(-1.0));
    if (!(settling_band >= 0.0)) {
      throw std::invalid_argument("analyze.helicopter_response signal '" + signal
                                  + "' must declare settling_band_" + suffix
                                  + " (or the legacy tracking_error migration alias)");
    }
    const double settling_dwell = signal_budget("settling_dwell_s").value_or(0.0);
    metric.settling_time_s = 0.0;

    struct SegmentStart {
      std::size_t index = 0;
      bool reference_step = false;
    };

    std::vector<SegmentStart> segment_starts{{0, false}};
    const auto is_discontinuity = [](const std::vector<double>& values, std::size_t i) {
      const double change = std::fabs(values[i] - values[i - 1]);
      const double previous_change = i > 1 ? std::fabs(values[i - 1] - values[i - 2]) : 0.0;
      const double next_change = i + 1 < values.size() ? std::fabs(values[i + 1] - values[i]) : 0.0;
      return change > 1.0e-9 && previous_change <= 1.0e-9 && next_change <= 1.0e-9;
    };
    for (std::size_t i = 1; i < reference_values.size(); ++i) {
      // A held reference step is a one-sample discontinuity. A ramp changes
      // every sample and is therefore analyzed as tracking, not as a series
      // of fictitious settling events.
      const bool reference_step = is_discontinuity(reference_values, i);
      const bool wind_step =
          i < closed.wind_samples_ned_m_s.size() && i < open.wind_samples_ned_m_s.size()
          && (open.wind_samples_ned_m_s[i] - open.wind_samples_ned_m_s[i - 1]).norm() > 1.0e-9
          && (i == 1
              || (open.wind_samples_ned_m_s[i - 1] - open.wind_samples_ned_m_s[i - 2]).norm()
                     <= 1.0e-9)
          && (i + 1 >= open.wind_samples_ned_m_s.size()
              || (open.wind_samples_ned_m_s[i + 1] - open.wind_samples_ned_m_s[i]).norm()
                     <= 1.0e-9);
      if (reference_step || wind_step) {
        const std::size_t start =
            wind_step && !reference_step && i + 1 < reference_values.size() ? i + 1 : i;
        if (segment_starts.back().index == start) {
          segment_starts.back().reference_step =
              segment_starts.back().reference_step || reference_step;
        } else {
          segment_starts.push_back({start, reference_step});
        }
      }
    }
    for (std::size_t segment = 0; segment < segment_starts.size(); ++segment) {
      const std::size_t start = segment_starts[segment].index;
      const std::size_t end = segment + 1 < segment_starts.size()
                                  ? segment_starts[segment + 1].index - 1
                                  : reference_values.size() - 1;
      std::vector<double> segment_times(
          closed.times_s.begin() + static_cast<std::ptrdiff_t>(start),
          closed.times_s.begin() + static_cast<std::ptrdiff_t>(end + 1));
      std::vector<double> segment_references(
          reference_values.begin() + static_cast<std::ptrdiff_t>(start),
          reference_values.begin() + static_cast<std::ptrdiff_t>(end + 1));
      std::vector<double> segment_controlled(
          controlled_values.begin() + static_cast<std::ptrdiff_t>(start),
          controlled_values.begin() + static_cast<std::ptrdiff_t>(end + 1));
      const bool has_step = segment_starts[segment].reference_step;
      analyze::ResponseMetricOptions options;
      options.event_time_s = segment_times.front();
      options.settling_band = settling_band;
      options.settling_dwell_s = settling_dwell;
      options.has_reference_step = has_step;
      options.reference_before = has_step ? reference_values[start - 1] : reference_values[start];
      options.reference_after = reference_values[start];
      const auto segment_result = analyze::analyze_response_segment(
          segment_times, segment_references, segment_controlled, options);
      metric.peak_error = std::max(metric.peak_error, segment_result.peak_tracking_error);
      if (segment == segment_starts.size() - 1) {
        metric.final_error = segment_result.final_tracking_error;
      }
      if (segment_result.settling_status == analyze::SettlingStatus::DemonstratedRecovery) {
        metric.settling_time_s =
            std::max(metric.settling_time_s, segment_result.settling_duration_s);
      }
      metric.segment_settling_statuses.emplace_back(
          analyze::to_string(segment_result.settling_status));
      metric.segment_settling_times_s.push_back(segment_result.settling_duration_s);
      if (segment_result.overshoot_applicable) {
        metric.overshoot_applicable = true;
        metric.overshoot_fraction =
            std::isnan(metric.overshoot_fraction)
                ? segment_result.overshoot_fraction
                : std::max(metric.overshoot_fraction, segment_result.overshoot_fraction);
      }
    }
    double sum_squared = 0.0;
    for (std::size_t i = 0; i < controlled_values.size(); ++i) {
      const double error = controlled_values[i] - reference_values[i];
      const double uncontrolled = uncontrolled_values[i] - reference_values[i];
      sum_squared += error * error;
      metric.uncontrolled_peak_error =
          std::max(metric.uncontrolled_peak_error, std::fabs(uncontrolled));
    }
    metric.rms_error = std::sqrt(sum_squared / static_cast<double>(controlled_values.size()));
    metric.reference = reference_values.back();
    // A reference can change again before the preceding segment settles. All
    // segment results remain visible; the declared settling requirement is
    // evaluated against the final segment, which is the only segment with a
    // complete post-event observation window.
    metric.settling_status = metric.segment_settling_statuses.back();
    metric.settling_time_s = metric.segment_settling_times_s.back();
    if (metric.uncontrolled_peak_error > 0.0) {
      metric.open_closed_peak_error_ratio = metric.peak_error / metric.uncontrolled_peak_error;
      metric.open_closed_improvement_fraction = 1.0 - metric.open_closed_peak_error_ratio;
    }
    if (signal_requirements == parsed_requirements.per_signal.end()
        && !aggregate_budget("tracking_error").has_value()) {
      throw std::invalid_argument("analyze.helicopter_response signal '" + signal
                                  + "' has no declared performance requirements");
    }
    response.metrics.push_back(metric);
  }

  if (closed.closed_loop && !closed.controller_requested.empty()) {
    const double tick_period = closed.controller_times_s.size() > 1
                                   ? closed.controller_times_s[1] - closed.controller_times_s[0]
                                   : closed.step_s;
    int saturated_ticks = 0;
    double squared_effort = 0.0;
    for (std::size_t i = 0; i < closed.controller_requested.size(); ++i) {
      const Eigen::VectorXd requested_minus_limited =
          closed.controller_requested[i] - closed.controller_saturated[i];
      const Eigen::VectorXd limited_minus_delayed =
          closed.controller_saturated[i] - closed.controller_applied[i];
      response.requested_minus_limited_peak = std::max(
          response.requested_minus_limited_peak, requested_minus_limited.cwiseAbs().maxCoeff());
      response.limited_minus_delayed_peak = std::max(response.limited_minus_delayed_peak,
                                                     limited_minus_delayed.cwiseAbs().maxCoeff());
      const Eigen::VectorXd effort =
          closed.controller_applied[i] - closed.controller_applied.front();
      response.control_effort_peak =
          std::max(response.control_effort_peak, effort.cwiseAbs().maxCoeff());
      squared_effort +=
          effort.squaredNorm() / static_cast<double>(std::max<Eigen::Index>(1, effort.size()));
      if (requested_minus_limited.cwiseAbs().maxCoeff() > 1.0e-12) {
        ++saturated_ticks;
      }
      const auto actual_name = [](const std::string& command) -> std::string {
        if (command.ends_with("_command_rad")) {
          return command.substr(0, command.size() - std::string("_command_rad").size()) + "_rad";
        }
        return {};
      };
      const std::size_t trajectory_sample =
          std::min(closed.states.size() - 1,
                   static_cast<std::size_t>(std::lower_bound(closed.times_s.begin(),
                                                             closed.times_s.end(),
                                                             closed.controller_times_s[i])
                                            - closed.times_s.begin()));
      for (std::size_t control = 0;
           control < static_cast<std::size_t>(closed.controller_applied[i].size());
           ++control) {
        const std::string state_name = actual_name(closed.model->control_names()[control]);
        const auto actual =
            std::find(closed.state_names.begin(), closed.state_names.end(), state_name);
        if (actual != closed.state_names.end()) {
          response.delayed_minus_actual_peak = std::max(
              response.delayed_minus_actual_peak,
              std::fabs(closed.controller_applied[i](static_cast<Eigen::Index>(control))
                        - closed.states[trajectory_sample](
                            static_cast<Eigen::Index>(actual - closed.state_names.begin()))));
        }
      }
    }
    response.saturation_duration_s = static_cast<double>(saturated_ticks) * tick_period;
    response.control_effort_rms =
        std::sqrt(squared_effort / static_cast<double>(closed.controller_applied.size()));
  }
  const auto rotor =
      std::find(closed.output_names.begin(), closed.output_names.end(), "main_rotor_speed_rad_s");
  if (rotor != closed.output_names.end()) {
    const double initial_speed = closed.outputs.front()(rotor - closed.output_names.begin());
    for (const auto& output : closed.outputs) {
      response.maximum_rotor_speed_excursion =
          std::max(response.maximum_rotor_speed_excursion,
                   std::fabs(output(rotor - closed.output_names.begin()) - initial_speed));
    }
  }
  response.controlled_envelope_departures = closed.envelope_departures;
  response.uncontrolled_envelope_departures = open.envelope_departures;

  response.criteria_passed = true;
  const auto budget_exceeded = [&response](const std::string& name, double measured) {
    const auto budget = response.requirements.find(name);
    return budget != response.requirements.end() && measured > budget->second;
  };
  for (const auto& metric : response.metrics) {
    const auto found = parsed_requirements.per_signal.find(metric.signal);
    const auto budget = [&](const std::string& key,
                            const std::string& legacy) -> std::optional<double> {
      if (found != parsed_requirements.per_signal.end()) {
        const auto signal_budget = found->second.find(key);
        if (signal_budget != found->second.end()) {
          return signal_budget->second;
        }
      }
      return aggregate_budget(legacy);
    };
    const std::string suffix = response_unit_suffix(metric.signal);
    const auto peak_budget = budget("peak_tracking_error_" + suffix, "tracking_error");
    const auto final_budget = budget("final_tracking_error_" + suffix, "final_tracking_error");
    const auto rms_budget = budget("rms_tracking_error_" + suffix, "rms_tracking_error");
    response.criteria_passed = response.criteria_passed
                               && (!peak_budget || metric.peak_error <= *peak_budget)
                               && (!final_budget || metric.final_error <= *final_budget)
                               && (!rms_budget || metric.rms_error <= *rms_budget);
    const auto settling_budget = budget("settling_time_s", "settling_time_s");
    if (settling_budget) {
      const bool within_budget = metric.settling_status == "already_within_band"
                                 || (metric.settling_status == "demonstrated_recovery"
                                     && metric.settling_time_s <= *settling_budget);
      response.criteria_passed = response.criteria_passed && within_budget;
    }
    const auto overshoot_budget = budget("overshoot_fraction", "overshoot");
    if (overshoot_budget && metric.overshoot_applicable) {
      response.criteria_passed =
          response.criteria_passed && metric.overshoot_fraction <= *overshoot_budget;
    }
    const auto ratio_budget =
        budget("open_closed_peak_error_ratio", "open_closed_peak_error_ratio");
    if (ratio_budget && std::isfinite(metric.open_closed_peak_error_ratio)) {
      response.criteria_passed =
          response.criteria_passed && metric.open_closed_peak_error_ratio <= *ratio_budget;
    }
    const auto improvement_budget =
        budget("open_closed_improvement_fraction", "open_closed_improvement_fraction");
    if (improvement_budget && std::isfinite(metric.open_closed_improvement_fraction)) {
      response.criteria_passed = response.criteria_passed
                                 && metric.open_closed_improvement_fraction >= *improvement_budget;
    }
  }
  response.criteria_passed =
      response.criteria_passed
      && !budget_exceeded("requested_minus_limited_peak_rad", response.requested_minus_limited_peak)
      && !budget_exceeded("limited_minus_delayed_peak_rad", response.limited_minus_delayed_peak)
      && !budget_exceeded("delayed_minus_actual_peak_rad", response.delayed_minus_actual_peak)
      && !budget_exceeded("control_effort_peak_rad", response.control_effort_peak)
      && !budget_exceeded("control_effort_rms_rad", response.control_effort_rms)
      && !budget_exceeded("control_effort_rad", response.control_effort_peak)
      && !budget_exceeded("saturation_duration_s", response.saturation_duration_s)
      && !budget_exceeded("rotor_speed_excursion_rad_s", response.maximum_rotor_speed_excursion)
      && (response.requirements.count("validity_envelope_departures") == 0
          || response.controlled_envelope_departures
                 <= response.requirements.at("validity_envelope_departures"));

  Artifact artifact;
  artifact.kind = "helicopter_response";
  std::ostringstream summary;
  summary << "matched open/closed-loop study; " << signals.size() << " response signal(s)";
  for (const auto& metric : response.metrics) {
    summary << "; " << metric.signal << " peak " << scientific(metric.peak_error) << " "
            << metric.unit << ", final " << scientific(metric.final_error) << " " << metric.unit
            << ", RMS " << scientific(metric.rms_error) << " " << metric.unit << ", settling ";
    if (std::isfinite(metric.settling_time_s)) {
      summary << fixed(metric.settling_time_s, 3) << " s (" << metric.settling_status << ")";
    } else {
      summary << metric.settling_status;
    }
  }
  summary << "; requested-limited peak " << scientific(response.requested_minus_limited_peak)
          << ", limited-delayed peak " << scientific(response.limited_minus_delayed_peak)
          << ", delayed-actual peak " << scientific(response.delayed_minus_actual_peak)
          << ", control-effort peak/RMS " << scientific(response.control_effort_peak) << "/"
          << scientific(response.control_effort_rms) << ", saturation duration "
          << fixed(response.saturation_duration_s, 3) << " s"
          << ", rotor-speed excursion " << scientific(response.maximum_rotor_speed_excursion)
          << ", envelope departures controlled/uncontrolled "
          << response.controlled_envelope_departures << "/"
          << response.uncontrolled_envelope_departures << "; engineering criteria "
          << (response.criteria_passed ? "PASS" : "FAIL");
  artifact.summary = summary.str();
  artifact.payload = std::move(response);
  return artifact;
}

Artifact ensemble_helicopter_capability(const StageContext& context) {
  const auto& trim =
      context.upstream_at("trim").payload_as<HelicopterTrimArtifact>("helicopter_trim");
  (void)trim;
  const ValuePtr members = context.input->get("members");
  if (!members || members->kind() != Value::Kind::List || members->as_list().empty()) {
    throw std::invalid_argument(
        "study.helicopter_ensemble members must be a non-empty declaration-order list");
  }
  const double step_s = context.input->number_at("step_s");
  const int steps = context.input->integer_at("steps", 0);
  const int sample_stride = context.input->integer_at("sample_stride", 1);
  if (!(step_s > 0.0) || !std::isfinite(step_s) || steps < 1 || sample_stride < 1) {
    throw std::invalid_argument(
        "study.helicopter_ensemble requires positive step_s, steps and sample_stride");
  }
  const bool parallel = context.input->bool_at("parallel", false);
  const int worker_count = context.input->integer_at(
      "worker_count", parallel ? static_cast<int>(members->as_list().size()) : 1);
  if (worker_count < 1) {
    throw std::invalid_argument("study.helicopter_ensemble worker_count must be positive");
  }
  const std::string manifest_prefix = context.input->string_at("manifest_prefix", "member");
  if (manifest_prefix.empty()) {
    throw std::invalid_argument("study.helicopter_ensemble manifest_prefix must be non-empty");
  }
  const int criteria_limit = context.input->integer_at("criteria_max_envelope_departures", 0);
  if (criteria_limit < 0) {
    throw std::invalid_argument(
        "study.helicopter_ensemble criteria_max_envelope_departures must be non-negative");
  }
  const ValuePtr parameter_distribution = context.input->get("parameter_distribution");
  double mass_mean = 1.0;
  double mass_stddev = 0.0;
  double mass_minimum = 0.1;
  double mass_maximum = 10.0;
  std::string distribution_provenance = "manual member values; not a Monte Carlo sample";
  if (parameter_distribution) {
    if (parameter_distribution->kind() != Value::Kind::Map) {
      throw std::invalid_argument("study.helicopter_ensemble parameter_distribution must be a map");
    }
    const ValuePtr mass = parameter_distribution->get("mass_scale");
    if (mass) {
      require_map_keys(mass,
                       {"distribution", "mean", "stddev", "minimum", "maximum"},
                       "study.helicopter_ensemble parameter_distribution.mass_scale");
      if (mass->string_at("distribution") != "normal_box_muller_v1") {
        throw std::invalid_argument(
            "study.helicopter_ensemble mass_scale distribution must be "
            "'normal_box_muller_v1'");
      }
      mass_mean = mass->number_at("mean");
      mass_stddev = mass->number_at("stddev");
      mass_minimum = mass->number_at("minimum");
      mass_maximum = mass->number_at("maximum");
      if (!std::isfinite(mass_mean) || !std::isfinite(mass_stddev) || mass_stddev < 0.0
          || !std::isfinite(mass_minimum) || !std::isfinite(mass_maximum) || !(mass_minimum > 0.0)
          || mass_maximum < mass_minimum) {
        throw std::invalid_argument(
            "study.helicopter_ensemble mass_scale distribution has invalid bounds or moments");
      }
      distribution_provenance = "mass_scale ~ normal_box_muller_v1(mean=" + fixed(mass_mean, 9)
                                + ", stddev=" + fixed(mass_stddev, 9) + ", truncated=["
                                + fixed(mass_minimum, 9) + ", " + fixed(mass_maximum, 9)
                                + "]); independent streams by member seed";
    }
  }

  struct MemberInput {
    std::string id;
    std::uint64_t seed = 0;
    double mass_scale = 1.0;
    Eigen::Vector3d cg_offset_body_m = Eigen::Vector3d::Zero();
    ValuePtr perturbation;
    std::string distribution_provenance;
    ValuePtr closed_loop;
  };

  std::vector<MemberInput> declared;
  declared.reserve(members->as_list().size());
  std::set<std::string> ids;
  for (std::size_t i = 0; i < members->as_list().size(); ++i) {
    const ValuePtr& entry = members->as_list()[i];
    require_map_keys(entry,
                     {"id", "seed", "mass_scale", "cg_offset_body_m", "initial_state_perturbation"},
                     "study.helicopter_ensemble members[]");
    const std::string id = entry->string_at("id");
    if (!ids.insert(id).second) {
      throw std::invalid_argument("study.helicopter_ensemble member ids must be unique");
    }
    const double seed_number = entry->number_at("seed");
    if (!std::isfinite(seed_number) || seed_number < 0.0 || std::floor(seed_number) != seed_number
        || seed_number > 9007199254740991.0) {
      throw std::invalid_argument(
          "study.helicopter_ensemble member seeds must be non-negative exactly representable "
          "integers");
    }
    const double mass_scale = entry->number_at("mass_scale", 1.0);
    if (!(mass_scale > 0.0) || !std::isfinite(mass_scale)) {
      throw std::invalid_argument(
          "study.helicopter_ensemble member mass_scale must be positive and finite");
    }
    const Eigen::Vector3d cg_offset = [&]() -> Eigen::Vector3d {
      const ValuePtr value = entry->get("cg_offset_body_m");
      if (!value) {
        return Eigen::Vector3d::Zero();
      }
      const Eigen::VectorXd parsed =
          value_vector(value, 3, "study.helicopter_ensemble member cg_offset_body_m");
      if (parsed.norm() > 2.0) {
        throw std::invalid_argument(
            "study.helicopter_ensemble member cg_offset_body_m must lie within the declared "
            "2 m design range");
      }
      return Eigen::Vector3d(parsed);
    }();
    double sampled_mass_scale = mass_scale;
    std::string member_distribution = distribution_provenance;
    if (parameter_distribution && parameter_distribution->get("mass_scale")) {
      sampled_mass_scale = std::clamp(
          mass_mean
              + mass_stddev
                    * turbulence_normal(
                        static_cast<std::uint64_t>(seed_number), static_cast<int>(i), 101),
          mass_minimum,
          mass_maximum);
      member_distribution = distribution_provenance;
    }
    declared.push_back({id,
                        static_cast<std::uint64_t>(seed_number),
                        sampled_mass_scale,
                        cg_offset,
                        entry->get("initial_state_perturbation"),
                        member_distribution,
                        context.input->get("closed_loop")});
  }

  const ValuePtr turbulence = context.input->get("turbulence");
  if (turbulence) {
    require_map_keys(turbulence,
                     {"model", "seed", "mean_ned_m_s", "stddev_ned_m_s", "correlation_time_s"},
                     "study.helicopter_ensemble turbulence");
    if (turbulence->string_at("model") != "gaussian_ou_v1") {
      throw std::invalid_argument(
          "study.helicopter_ensemble turbulence.model must be 'gaussian_ou_v1'");
    }
  }

  const auto run_member = [&](const MemberInput& member) {
    std::map<std::string, ValuePtr> input;
    input.emplace("trim", context.input->get("trim"));
    input.emplace("step_s", Value::number(step_s));
    input.emplace("steps", Value::number(static_cast<double>(steps)));
    input.emplace("sample_stride", Value::number(static_cast<double>(sample_stride)));
    input.emplace("mass_scale", Value::number(member.mass_scale));
    if (!member.cg_offset_body_m.isZero()) {
      input.emplace("cg_offset_body_m",
                    Value::list({Value::number(member.cg_offset_body_m(0)),
                                 Value::number(member.cg_offset_body_m(1)),
                                 Value::number(member.cg_offset_body_m(2))}));
    }
    if (member.perturbation) {
      input.emplace("initial_state_perturbation", member.perturbation);
    }
    if (turbulence) {
      std::map<std::string, ValuePtr> fields = turbulence->as_map();
      fields["seed"] = Value::number(static_cast<double>(member.seed));
      input.emplace("turbulence", Value::map(std::move(fields)));
    }
    StageContext member_context = context;
    member_context.input = Value::map(input);
    member_context.stage_id = context.stage_id + "." + member.id;
    HelicopterEnsembleMember record;
    record.id = member.id;
    record.seed = member.seed;
    record.mass_scale = member.mass_scale;
    record.cg_offset_body_m = member.cg_offset_body_m;
    record.distribution_provenance = member.distribution_provenance;
    try {
      Artifact run = simulate_helicopter_capability(member_context);
      const Artifact* run_for_checks = &run;
      std::optional<Artifact> response_run;
      if (member.closed_loop) {
        if (member.closed_loop->kind() != Value::Kind::Map) {
          throw std::invalid_argument("study.helicopter_ensemble closed_loop must be a map");
        }
        std::map<std::string, ValuePtr> closed_input = input;
        for (const auto& [key, value] : member.closed_loop->as_map()) {
          closed_input[key] = value;
        }
        closed_input["trim"] = context.input->get("trim");
        closed_input["step_s"] = Value::number(step_s);
        closed_input["steps"] = Value::number(static_cast<double>(steps));
        closed_input["sample_stride"] = Value::number(static_cast<double>(sample_stride));
        closed_input["mass_scale"] = Value::number(member.mass_scale);
        StageContext closed_context = context;
        closed_context.input = Value::map(std::move(closed_input));
        closed_context.stage_id = context.stage_id + "." + member.id + ".closed_loop";
        response_run = simulate_helicopter_closed_loop_capability(closed_context);
        run_for_checks = &*response_run;
      }
      const auto& trajectory =
          run_for_checks->payload_as<HelicopterTrajectoryArtifact>("helicopter_trajectory");
      record.completed = trajectory.reason == numerics::TerminationReason::Completed;
      record.numerical_checks_passed = trajectory.reason == numerics::TerminationReason::Completed
                                       && trajectory.steps_taken == steps;
      record.envelope_departures = trajectory.envelope_departures;
      record.envelope_checks_passed = record.envelope_departures <= criteria_limit;
      record.controller_requirements_passed = true;
      record.controller_requirements_status = "not_requested";
      if (member.closed_loop && context.input->get("response")) {
        try {
          const ValuePtr response = context.input->get("response");
          require_map_keys(response,
                           {"signals", "reference", "requirements", "signal_requirements"},
                           "study.helicopter_ensemble response");
          std::map<std::string, ValuePtr> response_input;
          response_input.emplace("open_loop", Value::stage_reference(member.id + ".open"));
          response_input.emplace("closed_loop", Value::stage_reference(member.id + ".closed"));
          for (const char* key : {"signals", "reference", "requirements", "signal_requirements"}) {
            if (const ValuePtr value = response->get(key)) {
              response_input.emplace(key, value);
            }
          }
          StageContext response_context = context;
          response_context.input = Value::map(std::move(response_input));
          response_context.stage_id = context.stage_id + "." + member.id + ".response";
          response_context.upstream[member.id + ".open"] = run;
          response_context.upstream[member.id + ".closed"] = *response_run;
          const Artifact response_artifact =
              analyze_helicopter_response_capability(response_context);
          const auto& response_payload =
              response_artifact.payload_as<HelicopterResponseArtifact>("helicopter_response");
          record.controller_requirements_passed = response_payload.criteria_passed;
          record.controller_requirements_status =
              record.controller_requirements_passed ? "passed" : "failed";
          if (!record.controller_requirements_passed) {
            record.failure_reason = "controller-performance requirements failed";
          }
        } catch (const std::exception& error) {
          record.controller_requirements_passed = false;
          record.controller_requirements_status = "unsupported_or_missing_metrics";
          record.failure_reason = error.what();
        }
      }
      record.criteria_passed = record.numerical_checks_passed && record.envelope_checks_passed;
      record.criteria_passed = record.criteria_passed && record.controller_requirements_passed;
      record.execution_status = record.completed ? "completed" : "diverged";
      record.termination = record.completed ? "completed" : numerics::to_string(trajectory.reason);
      if (!record.completed) {
        record.failure_reason = trajectory.termination_detail;
      } else if (!record.envelope_checks_passed) {
        record.failure_reason = "validity-envelope departure budget exceeded";
      }
    } catch (const std::exception& error) {
      record.completed = false;
      record.numerical_checks_passed = false;
      record.envelope_checks_passed = false;
      record.controller_requirements_passed = false;
      record.controller_requirements_status = "execution_failed";
      record.criteria_passed = false;
      record.execution_status = "refused";
      record.failure_reason = error.what();
      record.termination = std::string("refused: ") + error.what();
    }
    return record;
  };

  std::vector<HelicopterEnsembleMember> records(declared.size());
  if (parallel) {
    // The declaration order is the aggregation order. A fixed worker limit
    // changes scheduling only; it never changes member seeds or stream
    // derivation.
    for (std::size_t first = 0; first < declared.size();
         first += static_cast<std::size_t>(worker_count)) {
      const std::size_t last =
          std::min(declared.size(), first + static_cast<std::size_t>(worker_count));
      std::vector<std::future<HelicopterEnsembleMember>> futures;
      futures.reserve(last - first);
      for (std::size_t i = first; i < last; ++i) {
        futures.push_back(std::async(std::launch::async, run_member, declared[i]));
      }
      for (std::size_t i = first; i < last; ++i) {
        records[i] = futures[i - first].get();
      }
    }
  } else {
    for (std::size_t i = 0; i < declared.size(); ++i) {
      records[i] = run_member(declared[i]);
    }
  }

  std::ostringstream aggregate;
  aggregate.imbue(std::locale::classic());
  aggregate << "{\n  \"ordering\": \"declaration_order\",\n  \"parallel\": "
            << (parallel ? "true" : "false") << ",\n  \"worker_count\": " << worker_count
            << ",\n  \"random_algorithm\": \"splitmix64_mt19937_64_box_muller_v1\",\n"
            << "  \"distribution\": " << json_string(distribution_provenance)
            << ",\n  \"members\": [\n";
  for (std::size_t i = 0; i < records.size(); ++i) {
    auto& record = records[i];
    record.manifest_path = manifest_prefix + "-" + std::to_string(i) + ".json";
    std::ostringstream manifest;
    manifest.imbue(std::locale::classic());
    manifest << "{\n  \"id\": " << json_string(record.id) << ",\n  \"seed\": " << record.seed
             << ",\n  \"random_algorithm\": \"splitmix64_mt19937_64_box_muller_v1\",\n"
                "  \"mass_scale\": "
             << std::setprecision(17) << record.mass_scale << ",\n  \"cg_offset_body_m\": ["
             << record.cg_offset_body_m(0) << ", " << record.cg_offset_body_m(1) << ", "
             << record.cg_offset_body_m(2) << "]"
             << ",\n  \"solver\": \"rk4_fixed\",\n  \"step_s\": " << step_s
             << ",\n  \"steps\": " << steps << ",\n  \"model\": \"" << trim.model->description()
             << "\",\n  \"completion\": " << (record.completed ? "true" : "false")
             << ",\n  \"execution_status\": " << json_string(record.execution_status)
             << ",\n  \"numerical_checks_passed\": "
             << (record.numerical_checks_passed ? "true" : "false")
             << ",\n  \"envelope_checks_passed\": "
             << (record.envelope_checks_passed ? "true" : "false")
             << ",\n  \"controller_requirements_passed\": "
             << (record.controller_requirements_passed ? "true" : "false")
             << ",\n  \"controller_requirements_status\": "
             << json_string(record.controller_requirements_status)
             << ",\n  \"criteria_passed\": " << (record.criteria_passed ? "true" : "false")
             << ",\n  \"envelope_departures\": " << record.envelope_departures
             << ",\n  \"distribution\": " << json_string(record.distribution_provenance)
             << ",\n  \"termination\": " << json_string(record.termination)
             << ",\n  \"failure_reason\": " << json_string(record.failure_reason) << "\n}\n";
    context.write_output(record.manifest_path, manifest.str());
    aggregate << "    {\"id\": " << json_string(record.id)
              << ", \"manifest\": " << json_string(record.manifest_path)
              << ", \"completed\": " << (record.completed ? "true" : "false")
              << ", \"execution_status\": " << json_string(record.execution_status)
              << ", \"numerical_checks_passed\": "
              << (record.numerical_checks_passed ? "true" : "false")
              << ", \"envelope_checks_passed\": "
              << (record.envelope_checks_passed ? "true" : "false")
              << ", \"controller_requirements_passed\": "
              << (record.controller_requirements_passed ? "true" : "false")
              << ", \"controller_requirements_status\": "
              << json_string(record.controller_requirements_status)
              << ", \"criteria_passed\": " << (record.criteria_passed ? "true" : "false")
              << ", \"failure_reason\": " << json_string(record.failure_reason) << "}"
              << (i + 1 == records.size() ? "\n" : ",\n");
  }
  aggregate << "  ]\n}\n";
  const std::string aggregate_path = context.input->string_at("aggregate_path", "ensemble.json");
  context.write_output(aggregate_path, aggregate.str());

  HelicopterEnsembleArtifact ensemble;
  ensemble.members = std::move(records);
  ensemble.parallel = parallel;
  ensemble.ordering_policy = "declaration_order; continuation is not used; members are independent";
  Artifact artifact;
  artifact.kind = "helicopter_ensemble";
  const int completed_count = static_cast<int>(
      std::count_if(ensemble.members.begin(), ensemble.members.end(), [](const auto& member) {
        return member.completed;
      }));
  const int numerical_count = static_cast<int>(
      std::count_if(ensemble.members.begin(), ensemble.members.end(), [](const auto& member) {
        return member.numerical_checks_passed;
      }));
  const int envelope_count = static_cast<int>(
      std::count_if(ensemble.members.begin(), ensemble.members.end(), [](const auto& member) {
        return member.envelope_checks_passed;
      }));
  const int criteria_count = static_cast<int>(
      std::count_if(ensemble.members.begin(), ensemble.members.end(), [](const auto& member) {
        return member.criteria_passed;
      }));
  std::ostringstream summary;
  summary << ensemble.members.size() << " independent helicopter members in declaration order; "
          << completed_count << " completed, " << numerical_count << " numerical checks passed, "
          << envelope_count << " envelope checks passed, " << criteria_count
          << " engineering criteria passed; aggregate in " << aggregate_path << "; execution "
          << (parallel ? "parallel" : "serial");
  artifact.summary = summary.str();
  artifact.payload = std::move(ensemble);
  return artifact;
}

// ---------------------------------------------------------------------------
// analyze.nonlinear_agreement
// ---------------------------------------------------------------------------

struct AgreementArtifact {
  linearize::NonlinearAgreement agreement;
  std::string perturbed_state;
  double horizon_s = 0.0;
  double step_s = 0.0;
};

Artifact nonlinear_agreement_capability(const StageContext& context) {
  const Artifact& trim_upstream = context.upstream_at("trim");
  const auto& subject = trim_upstream.payload_as<HelicopterTrimArtifact>("helicopter_trim");

  linearize::VehicleLinearisationOptions linear_options;
  const auto linearisation = linearize::linearize_vehicle(*subject.model,
                                                          subject.result.extended_state,
                                                          subject.holding_controls,
                                                          subject.environment,
                                                          linear_options);

  // The perturbation direction is declared by STATE NAME, so the study says
  // which coordinate it is disturbing rather than indexing a vector whose layout
  // it would have to know.
  const std::string state_name = context.input->string_at("perturb_state");
  const auto it =
      std::find(linearisation.state_names.begin(), linearisation.state_names.end(), state_name);
  if (it == linearisation.state_names.end()) {
    std::ostringstream message;
    message << "analyze.nonlinear_agreement: '" << state_name
            << "' is not a state of this linearisation. It has {";
    for (std::size_t i = 0; i < linearisation.state_names.size(); ++i) {
      message << (i ? ", " : "") << linearisation.state_names[i];
    }
    message << "}";
    throw std::runtime_error(message.str());
  }
  Eigen::VectorXd direction = Eigen::VectorXd::Zero(linearisation.a.rows());
  direction(it - linearisation.state_names.begin()) = 1.0;

  std::vector<double> epsilons;
  const ValuePtr declared = context.input->get("perturbations");
  if (declared == nullptr) {
    throw std::runtime_error(
        "analyze.nonlinear_agreement: 'perturbations' is required and needs at least two entries. "
        "The measurement is an ORDER, and an order needs two points — a single perturbation size "
        "cannot distinguish a correct linearisation from one with a sign error");
  }
  for (const auto& entry : declared->as_list()) {
    epsilons.push_back(entry->as_number());
  }

  const double horizon_s = context.input->number_at("horizon_s");
  const double step_s = context.input->number_at("step_s");

  const auto agreement = linearize::nonlinear_agreement(
      *subject.model, linearisation, subject.environment, direction, epsilons, horizon_s, step_s);

  Artifact artifact;
  artifact.kind = "nonlinear_agreement";
  std::ostringstream summary;
  summary << "perturbed " << state_name << " over " << epsilons.size() << " sizes, horizon "
          << fixed(horizon_s, 3) << " s; observed order";
  for (const double order : agreement.observed_orders) {
    summary << " " << fixed(order, 4);
  }
  summary << " (2 is exact for a correct linearisation; near 1 indicates a first-order error or a "
             "non-differentiable term)";
  artifact.summary = summary.str();
  artifact.payload = AgreementArtifact{agreement, state_name, horizon_s, step_s};
  return artifact;
}

void write_agreement_section(std::ostream& out, const AgreementArtifact& subject) {
  out << "Perturbation applied to `" << subject.perturbed_state << "`, integrated for "
      << fixed(subject.horizon_s, 3) << " s at a step of " << scientific(subject.step_s, 3)
      << " s.\n\n";
  out << "| Perturbation | Discrepancy (dimensionless) | Observed order |\n|---|---|---|\n";
  for (std::size_t i = 0; i < subject.agreement.epsilons.size(); ++i) {
    out << "| " << scientific(subject.agreement.epsilons[i], 3) << " | "
        << scientific(subject.agreement.discrepancies[i], 4) << " | ";
    if (i == 0) {
      out << "—";
    } else {
      out << fixed(subject.agreement.observed_orders[i - 1], 4);
    }
    out << " |\n";
  }
  out << "\nThe discrepancy is the norm of (nonlinear trajectory difference) minus (linear "
         "prediction), with each state divided by the trim point's own magnitude so the norm is "
         "dimensionless. A correct linearisation's error falls as the SQUARE of the perturbation, "
         "so the observed order should sit at 2. An order near 1, or a discrepancy that stops "
         "falling, indicates a first-order error — a wrong sign, a missing term, a point that is "
         "not an equilibrium — or a term that is not differentiable at the trim point.\n\n";
}

void write_response_section(std::ostream& out, const HelicopterResponseArtifact& response) {
  out << "The controlled and uncontrolled trajectories share the same initial state and "
         "disturbance history. Requirements were declared before the controller run.\n\n";
  out << "| Requirement | Declared budget |\n|---|---:|\n";
  for (const auto& [name, value] : response.requirements) {
    out << "| " << name << " | " << scientific(value, 4) << " |\n";
  }
  out << "\n| Signal | Unit | Reference | Peak error | Final error | RMS error | Settling | "
         "Overshoot | Open/closed ratio | Improvement | Uncontrolled peak error |\n"
         "|---|---|---:|---:|---:|---:|---|---:|---:|---:|---:|\n";
  for (const auto& metric : response.metrics) {
    out << "| " << metric.signal << " | " << metric.unit << " | " << scientific(metric.reference, 4)
        << " | " << scientific(metric.peak_error, 4) << " | " << scientific(metric.final_error, 4)
        << " | " << scientific(metric.rms_error, 4) << " | ";
    if (std::isfinite(metric.settling_time_s)) {
      out << fixed(metric.settling_time_s, 4) << " s (" << metric.settling_status << ")";
    } else {
      out << metric.settling_status;
    }
    out << " | ";
    if (metric.overshoot_applicable) {
      out << scientific(metric.overshoot_fraction, 4);
    } else {
      out << "inapplicable";
    }
    out << " | " << scientific(metric.open_closed_peak_error_ratio, 4) << " | "
        << scientific(metric.open_closed_improvement_fraction, 4) << " | "
        << scientific(metric.uncontrolled_peak_error, 4) << " |\n";
    out << "| `" << metric.signal << "` settling segments | — | — | — | — | — | ";
    for (std::size_t segment = 0; segment < metric.segment_settling_statuses.size(); ++segment) {
      if (segment > 0) {
        out << "; ";
      }
      out << segment << ":" << metric.segment_settling_statuses[segment];
      if (segment < metric.segment_settling_times_s.size()
          && std::isfinite(metric.segment_settling_times_s[segment])) {
        out << " @ " << fixed(metric.segment_settling_times_s[segment], 4) << " s";
      }
    }
    out << " | — | — | — | — |\n";
  }
  out << "\nActuator requested-minus-limited peak: "
      << scientific(response.requested_minus_limited_peak, 4)
      << " rad. Limited-minus-delayed peak: " << scientific(response.limited_minus_delayed_peak, 4)
      << " rad. Delayed-command-minus-actual peak: "
      << scientific(response.delayed_minus_actual_peak, 4)
      << " rad. Control effort peak/RMS from trim: " << scientific(response.control_effort_peak, 4)
      << "/" << scientific(response.control_effort_rms, 4)
      << " rad. Saturation duration: " << fixed(response.saturation_duration_s, 4)
      << " s. Rotor-speed excursion: " << scientific(response.maximum_rotor_speed_excursion, 4)
      << " rad/s. Envelope departures (controlled/uncontrolled): "
      << response.controlled_envelope_departures << "/" << response.uncontrolled_envelope_departures
      << ". Engineering criteria: **" << (response.criteria_passed ? "PASS" : "FAIL") << "**.\n\n";
}

Artifact write_json_output(const StageContext& context,
                           const std::string& path,
                           const std::string& bytes,
                           const std::string& summary) {
  context.write_output(path, bytes);
  Artifact result;
  result.kind = "report";
  result.summary = summary;
  result.payload = context.resolve_output_path(path);
  return result;
}

Artifact report_helicopter_schema_json_capability(const StageContext& context) {
  const auto& subject =
      context.upstream_at("helicopter").payload_as<HelicopterArtifact>("helicopter");
  const auto states = subject.model->state_names();
  const auto controls = subject.model->control_names();
  const auto outputs = subject.model->output_names();
  const auto parameters = helicopter_parameter_values(*subject.model);
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "{\"schema\":\"galata.helicopter.schema.v1\",\"model\":{\"path\":"
      << json_string(subject.path) << ",\"sha256\":" << json_string(subject.sha256)
      << ",\"description\":" << json_string(subject.model->description())
      << ",\"citation\":" << json_string(subject.model->citation)
      << "},\"conventions\":{"
         "\"attitude\":\"Hamilton scalar-first body-to-NED\","
         "\"position\":\"NED metres\",\"velocity\":\"body FRD m/s\","
         "\"controls\":\"body actuator commands in radians\"},\"states\":[";
  for (std::size_t index = 0; index < states.size(); ++index) {
    if (index != 0) {
      out << ',';
    }
    out << "{\"name\":" << json_string(states[index])
        << ",\"unit\":" << json_string(unit_for_channel(states[index]))
        << ",\"frame\":" << json_string(frame_for_channel(states[index])) << '}';
  }
  out << "],\"controls\":[";
  for (std::size_t index = 0; index < controls.size(); ++index) {
    if (index != 0) {
      out << ',';
    }
    out << "{\"name\":" << json_string(controls[index])
        << ",\"unit\":" << json_string(unit_for_channel(controls[index]))
        << ",\"frame\":" << json_string(frame_for_channel(controls[index])) << '}';
  }
  out << "],\"outputs\":[";
  for (std::size_t index = 0; index < outputs.size(); ++index) {
    if (index != 0) {
      out << ',';
    }
    out << "{\"name\":" << json_string(outputs[index])
        << ",\"unit\":" << json_string(unit_for_channel(outputs[index]))
        << ",\"frame\":" << json_string(frame_for_channel(outputs[index])) << '}';
  }
  out << "],\"parameters\":[";
  std::size_t index = 0;
  for (const auto& [name, value] : parameters) {
    if (index++ != 0) {
      out << ',';
    }
    out << "{\"name\":" << json_string(name) << ",\"value\":" << json_number(value)
        << ",\"unit\":" << json_string(unit_for_channel(name)) << '}';
  }
  out << "],\"parameter_overrides\":{";
  index = 0;
  for (const auto& [name, value] : subject.parameter_overrides) {
    if (index++ != 0) {
      out << ',';
    }
    out << json_string(name) << ':' << json_number(value);
  }
  out << "}}\n";
  return write_json_output(
      context, context.input->string_at("path"), out.str(), "wrote structured helicopter schema");
}

Artifact report_helicopter_trim_json_capability(const StageContext& context) {
  const auto& trim =
      context.upstream_at("trim").payload_as<HelicopterTrimArtifact>("helicopter_trim");
  const auto state_names = trim.model->state_names();
  const auto control_names = trim.model->control_names();
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "{\"schema\":\"galata.helicopter.trim.v1\",\"state_names\":";
  json_strings(out, state_names);
  out << ",\"state_values\":";
  json_vector(out, trim.result.extended_state);
  out << ",\"controls\":[";
  for (std::size_t index = 0; index < control_names.size(); ++index) {
    if (index != 0) {
      out << ',';
    }
    out << "{\"name\":" << json_string(control_names[index])
        << ",\"value\":" << json_number(trim.holding_controls(static_cast<Eigen::Index>(index)))
        << ",\"unit\":\"rad\"}";
  }
  out << "],\"unknowns\":[";
  for (std::size_t index = 0; index < trim.result.unknown_names.size(); ++index) {
    if (index != 0) {
      out << ',';
    }
    out << "{\"name\":" << json_string(trim.result.unknown_names[index]) << ",\"value\":"
        << json_number(trim.result.unknown_values(static_cast<Eigen::Index>(index)))
        << ",\"unit\":" << json_string(unit_for_channel(trim.result.unknown_names[index])) << '}';
  }
  out << "],\"condition\":{\"altitude_m\":" << json_number(trim.environment.atmospheric_altitude_m)
      << ",\"delta_isa_k\":" << json_number(trim.environment.delta_isa_k)
      << ",\"density_kg_m3\":" << json_number(trim.environment.density_kg_m3)
      << ",\"airspeed_m_s\":" << json_number(trim.result.extended_state(3))
      << "},\"diagnostics\":{\"converged\":" << (trim.result.converged ? "true" : "false")
      << ",\"residual_norm\":" << json_number(trim.result.residual_norm)
      << ",\"residual_tolerance\":" << json_number(trim.result.residual_tolerance)
      << ",\"iterations\":" << trim.result.iterations
      << ",\"jacobian_rank\":" << trim.result.jacobian_rank
      << ",\"unknown_count\":" << trim.result.unknown_count
      << ",\"jacobian_condition_number\":" << json_number(trim.result.jacobian_condition_number)
      << ",\"envelope_outside\":" << (trim.result.envelope.outside ? "true" : "false")
      << ",\"envelope_reason\":" << json_string(trim.result.envelope.reason)
      << "},"
         "\"parameter_overrides\":{";
  std::size_t index = 0;
  for (const auto& [name, value] : trim.parameter_overrides) {
    if (index++ != 0) {
      out << ',';
    }
    out << json_string(name) << ':' << json_number(value);
  }
  out << "},\"breakdown\":{\"main_thrust_n\":" << json_number(trim.breakdown.main.thrust_n)
      << ",\"tail_thrust_n\":" << json_number(trim.breakdown.tail.thrust_n)
      << ",\"total_power_w\":" << json_number(trim.breakdown.total_power_w)
      << ",\"rotor_speed_rad_s\":" << json_number(trim.breakdown.rotor_speed_rad_s) << "}}\n";
  return write_json_output(
      context, context.input->string_at("path"), out.str(), "wrote structured helicopter trim");
}

Artifact report_helicopter_response_json_capability(const StageContext& context) {
  const auto& response =
      context.upstream_at("response").payload_as<HelicopterResponseArtifact>("helicopter_response");
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "{\"schema\":\"galata.helicopter.response.v1\",\"criteria_passed\":"
      << (response.criteria_passed ? "true" : "false") << ",\"matched_initial_condition\":"
      << (response.matched_initial_condition ? "true" : "false")
      << ",\"matched_disturbance\":" << (response.matched_disturbance ? "true" : "false")
      << ",\"requirements\":{";
  std::size_t index = 0;
  for (const auto& [name, value] : response.requirements) {
    if (index++ != 0) {
      out << ',';
    }
    out << json_string(name) << ':' << json_number(value);
  }
  out << "},\"metrics\":[";
  for (std::size_t metric_index = 0; metric_index < response.metrics.size(); ++metric_index) {
    const auto& metric = response.metrics[metric_index];
    if (metric_index != 0) {
      out << ',';
    }
    out << "{\"signal\":" << json_string(metric.signal) << ",\"unit\":" << json_string(metric.unit)
        << ",\"reference\":" << json_number(metric.reference)
        << ",\"peak_error\":" << json_number(metric.peak_error)
        << ",\"final_error\":" << json_number(metric.final_error)
        << ",\"rms_error\":" << json_number(metric.rms_error)
        << ",\"settling_time_s\":" << json_number(metric.settling_time_s)
        << ",\"settling_status\":" << json_string(metric.settling_status)
        << ",\"overshoot_applicable\":" << (metric.overshoot_applicable ? "true" : "false")
        << ",\"overshoot_fraction\":" << json_number(metric.overshoot_fraction)
        << ",\"open_closed_peak_error_ratio\":" << json_number(metric.open_closed_peak_error_ratio)
        << ",\"open_closed_improvement_fraction\":"
        << json_number(metric.open_closed_improvement_fraction)
        << ",\"uncontrolled_peak_error\":" << json_number(metric.uncontrolled_peak_error)
        << ",\"segment_settling_statuses\":";
    json_strings(out, metric.segment_settling_statuses);
    out << ",\"segment_settling_times_s\":[";
    for (std::size_t segment = 0; segment < metric.segment_settling_times_s.size(); ++segment) {
      out << (segment == 0 ? "" : ",") << json_number(metric.segment_settling_times_s[segment]);
    }
    out << "]}";
  }
  out << "],\"actuator\":{\"requested_minus_limited_peak_rad\":"
      << json_number(response.requested_minus_limited_peak)
      << ",\"limited_minus_delayed_peak_rad\":" << json_number(response.limited_minus_delayed_peak)
      << ",\"delayed_minus_actual_peak_rad\":" << json_number(response.delayed_minus_actual_peak)
      << ",\"control_effort_peak_rad\":" << json_number(response.control_effort_peak)
      << ",\"control_effort_rms_rad\":" << json_number(response.control_effort_rms)
      << ",\"saturation_duration_s\":" << json_number(response.saturation_duration_s)
      << ",\"rotor_speed_excursion_rad_s\":" << json_number(response.maximum_rotor_speed_excursion)
      << ",\"controlled_envelope_departures\":" << response.controlled_envelope_departures
      << ",\"uncontrolled_envelope_departures\":" << response.uncontrolled_envelope_departures
      << "}}\n";
  return write_json_output(context,
                           context.input->string_at("path"),
                           out.str(),
                           "wrote structured helicopter response metrics");
}

void write_ensemble_section(std::ostream& out, const HelicopterEnsembleArtifact& ensemble) {
  out << "Members execute independently in " << ensemble.ordering_policy << ". Execution was "
      << (ensemble.parallel ? "parallel" : "serial") << ".\n\n";
  out << "| Member | Seed | Mass scale | CG offset body (m) | Execution | Numerical | Envelope | "
         "Controller | Criteria | Envelope departures | Failure reason | Manifest |\n"
         "|---|---:|---:|---|---|---|---|---|---|---:|---|---|\n";
  for (const auto& member : ensemble.members) {
    out << "| " << member.id << " | " << member.seed << " | " << fixed(member.mass_scale, 6)
        << " | [" << fixed(member.cg_offset_body_m(0), 4) << ", "
        << fixed(member.cg_offset_body_m(1), 4) << ", " << fixed(member.cg_offset_body_m(2), 4)
        << "] | " << member.execution_status << " | "
        << (member.numerical_checks_passed ? "pass" : "fail") << " | "
        << (member.envelope_checks_passed ? "pass" : "fail") << " | "
        << member.controller_requirements_status << " | "
        << (member.criteria_passed ? "pass" : "fail/excluded") << " | "
        << member.envelope_departures << " | " << member.failure_reason << " | "
        << member.manifest_path << " |\n";
  }
  out << "\nSuccessful completion is reported separately from passing the declared engineering "
         "criteria.\n\n";
}

// ---------------------------------------------------------------------------
// Report sections
// ---------------------------------------------------------------------------

void write_trim_section(std::ostream& out, const HelicopterTrimArtifact& trim) {
  const model::HelicopterModel& heli = *trim.model;
  out << "**" << heli.description() << "**\n\n";
  out << "Source: " << heli.citation << "\n\n";

  out << "Atmosphere: geometric altitude " << fixed(trim.environment.atmospheric_altitude_m, 3)
      << " m, ISA temperature offset " << fixed(trim.environment.delta_isa_k, 3) << " K, density "
      << fixed(trim.environment.density_kg_m3, 6) << " kg/m^3, pressure "
      << fixed(trim.environment.pressure_pa, 1) << " Pa, temperature "
      << fixed(trim.environment.temperature_k, 3)
      << " K. The atmosphere is frozen at this trim condition during downstream simulation; "
         "`altitude_m` in the trajectory remains height above the NED origin.\n\n";

  out << "| Quantity | Value | Unit |\n|---|---|---|\n";
  for (std::size_t i = 0; i < trim.result.unknown_names.size(); ++i) {
    const std::string& name = trim.result.unknown_names[i];
    const double value = trim.result.unknown_values(static_cast<Eigen::Index>(i));
    out << "| " << name << " | " << fixed(value, 6) << " | ";
    // The unit is read off the name, which is why every name carries one.
    out << (name.find("_rad") != std::string::npos ? "rad" : "dimensionless") << " |\n";
  }
  out << "\n";

  out << "| Component | Thrust (N) | Torque (N m) | Power (kW) |\n|---|---|---|---|\n";
  out << "| Main rotor | " << fixed(trim.breakdown.main.thrust_n, 1) << " | "
      << fixed(trim.breakdown.main.torque_n_m, 1) << " | "
      << fixed(trim.breakdown.main.power_w / 1000.0, 2) << " |\n";
  out << "| Tail rotor | " << fixed(trim.breakdown.tail.thrust_n, 1) << " | "
      << fixed(trim.breakdown.tail.torque_n_m, 2) << " | "
      << fixed(trim.breakdown.tail.power_w / 1000.0, 2) << " |\n\n";

  out << "Main-rotor C_T/sigma " << fixed(trim.breakdown.main.thrust_coefficient_solidity, 5)
      << ", induced velocity " << fixed(trim.breakdown.main.induced_velocity_m_s, 4)
      << " m/s, advance ratio " << fixed(trim.breakdown.main.advance_ratio, 5) << ". Rotor speed "
      << fixed(trim.breakdown.rotor_speed_rad_s, 4) << " rad/s, engine torque "
      << fixed(trim.breakdown.engine_torque_n_m, 1) << " N m.\n\n";

  // THE ANTI-TORQUE BALANCE, stated as a number rather than asserted in prose.
  out << "Yaw-moment residual at the trim: "
      << scientific(trim.breakdown.anti_torque_residual_n_m, 3)
      << " N m. That residual vanishing IS the anti-torque balance: the tail rotor's thrust at its "
      << "moment arm against the main rotor's shaft torque.\n\n";

  out << "Solved in " << trim.result.iterations << " iterations to a residual norm of "
      << scientific(trim.result.residual_norm, 3) << " against a budget of "
      << scientific(trim.result.residual_tolerance, 3) << ". Jacobian rank "
      << trim.result.jacobian_rank << " of " << trim.result.unknown_count << ", condition number "
      << scientific(trim.result.jacobian_condition_number, 3) << ".\n\n";

  if (trim.result.envelope.outside) {
    out << "**Outside the declared envelope.** " << trim.result.envelope.reason << "\n\n";
  }
  out << "_This model is a preliminary design study, not a measured aircraft. No published "
         "rotorcraft reference has been compared against; see "
         "`models/souxmar-heli/PROVENANCE.md`._\n\n";
}

void write_trajectory_section(std::ostream& out, const HelicopterTrajectoryArtifact& run) {
  out << run.times_s.size() << " samples over "
      << fixed(static_cast<double>(run.steps_taken) * run.step_s, 3) << " s at a step of "
      << scientific(run.step_s, 3) << " s.\n\n";

  if (run.closed_loop) {
    out << "Closed-loop execution: " << run.controller_times_s.size()
        << " controller ticks with deterministic sample/hold ordering. Controller measurement "
           "channels are "
        << (run.controller_measurement_names.empty() ? "none" : "named below") << ".\n\n";
    if (!run.sensor_provenance.empty()) {
      out << "Measurement provenance:\n\n";
      for (const auto& item : run.sensor_provenance) {
        out << "- " << item << "\n";
      }
      out << "\n";
    }
  }

  // THE TERMINATION REASON IS IN THE REPORT, NOT ONLY IN THE CLI LINE. A reader
  // who has the document and not the terminal must still be able to tell a
  // completed run from a refused one.
  if (run.reason == numerics::TerminationReason::Completed) {
    out << "Termination: **completed**.\n\n";
  } else {
    out << "Termination: **" << numerics::to_string(run.reason) << "** — " << run.termination_detail
        << "\n\nThe trajectory recorded above ends at the last state that satisfied its declared "
           "bounds. It is NOT a completed run and must not be read as one.\n\n";
  }
  if (run.envelope_departures > 0) {
    out << "**Outside the declared envelope at " << run.envelope_departures << " of "
        << run.times_s.size() << " samples.** Worst: " << run.worst_envelope.reason << "\n\n";
  }
  if (!run.failure_events.empty()) {
    out << "Applied failure events, in deterministic declaration order at each time:\n\n";
    for (const auto& event : run.failure_events) {
      out << "- " << event << "\n";
    }
    out << "\n";
  }
  if (!run.initial_condition_provenance.empty()) {
    out << "Initial condition:\n\n";
    for (const auto& item : run.initial_condition_provenance) {
      out << "- " << item << "\n";
    }
    out << "\n";
  }
  out << run.disturbance_provenance << ".\n\n";
  if (!run.parameter_provenance.empty()) {
    out << "Parameter variation: " << run.parameter_provenance << ".\n\n";
  }

  if (!run.times_s.empty()) {
    out << "| Output | Initial | Final |\n|---|---|---|\n";
    for (std::size_t j = 0; j < run.output_names.size(); ++j) {
      out << "| " << run.output_names[j] << " | "
          << fixed(run.outputs.front()(static_cast<Eigen::Index>(j)), 5) << " | "
          << fixed(run.outputs.back()(static_cast<Eigen::Index>(j)), 5) << " |\n";
    }
    out << "\n";
  }
}

}  // namespace

bool write_helicopter_section(std::ostream& out, const Artifact& artifact) {
  if (artifact.kind == "helicopter_trim") {
    write_trim_section(out, artifact.payload_as<HelicopterTrimArtifact>("helicopter_trim"));
    return true;
  }
  if (artifact.kind == "helicopter_trajectory") {
    write_trajectory_section(
        out, artifact.payload_as<HelicopterTrajectoryArtifact>("helicopter_trajectory"));
    return true;
  }
  if (artifact.kind == "nonlinear_agreement") {
    write_agreement_section(out, artifact.payload_as<AgreementArtifact>("nonlinear_agreement"));
    return true;
  }
  if (artifact.kind == "helicopter_response") {
    write_response_section(out,
                           artifact.payload_as<HelicopterResponseArtifact>("helicopter_response"));
    return true;
  }
  if (artifact.kind == "helicopter_ensemble") {
    write_ensemble_section(out,
                           artifact.payload_as<HelicopterEnsembleArtifact>("helicopter_ensemble"));
    return true;
  }
  if (artifact.kind == "helicopter") {
    const auto& subject = artifact.payload_as<HelicopterArtifact>("helicopter");
    out << "Loaded from `" << subject.path << "`.\n\n"
        << subject.model->description() << "\n\nSource: " << subject.model->citation << "\n\n";
    return true;
  }
  return false;
}

namespace {}  // namespace

void register_helicopter_capabilities(Registry& registry) {
  registry.add(Capability{
      "model.helicopter",
      "Load a Level-1 single-main-rotor helicopter — oriented main and tail rotors with "
      "momentum-theory inflow and quasi-static flapping, component fuselage and empennage "
      "aerodynamics, a governed rotor-speed state and four limited actuators",
      "helicopter",
      Capability::State::ImplementedUnvalidated,
      load_helicopter_capability,
      {"path", "parameter_overrides"},
      {"path"}});

  registry.add(Capability{
      "trim.helicopter",
      "Solve helicopter equilibrium for attitude, collective, both cyclics and pedal — and for "
      "the rotor inflow states, which have their own equilibrium — reporting the Jacobian's rank "
      "and condition number",
      "helicopter_trim",
      Capability::State::ImplementedUnvalidated,
      trim_helicopter_capability,
      {"helicopter",
       "airspeed_m_s",
       "altitude_m",
       "delta_isa_k",
       "wind_ned_m_s",
       "iterations",
       "residual_tolerance"}});

  registry.add(Capability{
      "linearize.vehicle",
      "Linearise any vehicle model about a declared trim by central differences in Euler "
      "coordinates, refusing a point whose dynamic residual exceeds its budget and naming any "
      "control that was at a stop",
      "linear_system",
      Capability::State::ImplementedUnvalidated,
      linearize_vehicle_capability,
      {"trim", "equilibrium_tolerance", "drop_position_and_heading"}});

  registry.add(Capability{
      "sim.helicopter",
      "Integrate the nonlinear helicopter from a trim under a declared control step, with a "
      "declared integration method and per-state magnitude bounds that refuse a divergence "
      "instead of reporting it as a completed run",
      "helicopter_trajectory",
      Capability::State::ImplementedUnvalidated,
      simulate_helicopter_capability,
      {"trim",
       "step_s",
       "steps",
       "sample_stride",
       "method",
       "newton_iterations",
       "step_control",
       "step_size_rad",
       "failure_events",
       "initial_state_perturbation",
       "attitude_perturbation_body_rad",
       "wind_schedule",
       "turbulence",
       "mass_scale",
       "cg_offset_body_m"}});

  registry.add(Capability{
      "sim.helicopter.closed_loop",
      "Execute a nonlinear helicopter VehicleModel with deterministic sampled measurements, "
      "controller updates, zero-order hold, whole-period delay, actuator saturation and "
      "scheduled failures",
      "helicopter_trajectory",
      Capability::State::ImplementedUnvalidated,
      simulate_helicopter_closed_loop_capability,
      {"trim",
       "law",
       "controller",
       "sensor",
       "step_s",
       "steps",
       "sample_stride",
       "controller_period_s",
       "delay_periods",
       "method",
       "failure_events",
       "initial_state_perturbation",
       "attitude_perturbation_body_rad",
       "reference_schedule",
       "wind_schedule",
       "turbulence",
       "mass_scale",
       "cg_offset_body_m"}});

  registry.add(Capability{
      "study.helicopter_ensemble",
      "Execute independent helicopter parameter members with seeded disturbances, per-member "
      "manifests, deterministic declaration order and optional parallel execution",
      "helicopter_ensemble",
      Capability::State::ImplementedUnvalidated,
      ensemble_helicopter_capability,
      {"trim",
       "members",
       "step_s",
       "steps",
       "sample_stride",
       "parallel",
       "worker_count",
       "turbulence",
       "parameter_distribution",
       "closed_loop",
       "response",
       "criteria_max_envelope_departures",
       "aggregate_path",
       "manifest_prefix"},
      {},
      {"aggregate_path"}});

  registry.add(Capability{
      "analyze.helicopter_response",
      "Compare matched open- and closed-loop helicopter trajectories with declared tracking, "
      "settling, actuator, saturation, rotor-speed and envelope budgets",
      "helicopter_response",
      Capability::State::ImplementedUnvalidated,
      analyze_helicopter_response_capability,
      {"open_loop", "closed_loop", "signals", "reference", "requirements", "signal_requirements"}});

  registry.add(Capability{
      "analyze.nonlinear_agreement",
      "Measure the ORDER at which a linearisation's error falls with perturbation size — two is "
      "exact for a correct one — rather than its magnitude at a single size, which cannot "
      "distinguish a correct linearisation from one with a sign error",
      "nonlinear_agreement",
      Capability::State::ImplementedUnvalidated,
      nonlinear_agreement_capability,
      {"trim", "perturb_state", "perturbations", "horizon_s", "step_s"}});

  registry.add(Capability{
      "report.helicopter_csv",
      "Export a helicopter trajectory as CSV with every state and output named, carrying the "
      "run's termination reason so a refused run cannot be read as a completed one",
      "report",
      Capability::State::ImplementedUnvalidated,
      report_helicopter_csv_capability,
      {"trajectory", "path"},
      {},
      {"path"}});

  registry.add(Capability{
      "report.helicopter_schema_json",
      "Write versioned machine-readable helicopter vocabulary, units, frames and parameter values",
      "report",
      Capability::State::ImplementedUnvalidated,
      report_helicopter_schema_json_capability,
      {"helicopter", "path"},
      {},
      {"path"}});

  registry.add(Capability{
      "report.helicopter_trim_json",
      "Write versioned machine-readable helicopter trim values and numerical diagnostics",
      "report",
      Capability::State::ImplementedUnvalidated,
      report_helicopter_trim_json_capability,
      {"trim", "path"},
      {},
      {"path"}});

  registry.add(Capability{
      "report.helicopter_response_json",
      "Write versioned machine-readable helicopter response metrics and acceptance verdict",
      "report",
      Capability::State::ImplementedUnvalidated,
      report_helicopter_response_json_capability,
      {"response", "path"},
      {},
      {"path"}});
}

}  // namespace galata::pipeline
