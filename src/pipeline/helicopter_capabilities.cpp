// SPDX-License-Identifier: Apache-2.0
//
// The helicopter vertical, at the capability boundary: load a model, trim it,
// linearise it, simulate it, and report the component build-up.
//
// Reference: the physics is in galata/model/helicopter.hpp and
// galata/model/rotor/rotor.hpp; the trim formulation is in
// galata/trim/problem.hpp. This file adds no numerics — it resolves study input,
// calls those, and turns the result into an artefact with a summary line.
//
// EVERY CAPABILITY HERE IS `ImplementedUnvalidated`, and that is not modesty.
// docs/VERIFICATION.md records agreement against a published reference and no
// published rotorcraft reference has been compared against: the Souxmar model is
// a DESIGN STUDY, and the cases behind these capabilities are exact invariants of
// the equations plus a cross-check against the design package's own independent
// hover computation. Neither is a published reference, so neither earns
// `Implemented`. models/souxmar-heli/PROVENANCE.md says the same in more detail.

#include "galata/core/atmosphere.hpp"
#include "galata/linearize/vehicle.hpp"
#include "galata/model/helicopter.hpp"
#include "galata/numerics/integration_method.hpp"
#include "galata/pipeline/registry.hpp"
#include "galata/trim/problem.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <ostream>
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
};

struct HelicopterTrajectoryArtifact {
  std::shared_ptr<const model::HelicopterModel> model;
  std::vector<double> times_s;
  std::vector<Eigen::VectorXd> states;  // extended
  std::vector<Eigen::VectorXd> outputs;
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
};

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

// ---------------------------------------------------------------------------
// model.helicopter
// ---------------------------------------------------------------------------

Artifact load_helicopter_capability(const StageContext& context) {
  const std::string declared = context.input->string_at("path");
  const std::string bytes = context.read_input(declared);
  auto model = std::make_shared<model::HelicopterModel>(model::parse_helicopter(bytes, declared));

  Artifact artifact;
  artifact.kind = "helicopter";
  std::ostringstream summary;
  summary << model->extended_state_size() << " states, " << model->control_count()
          << " controls; main rotor R = " << fixed(model->main_rotor.radius_m, 2)
          << " m, sigma = " << fixed(model->main_rotor.solidity(), 4) << ", tail arm "
          << fixed(-model->tail_rotor.position_cg_to_hub_body_m.x(), 2) << " m — "
          << model->description();
  artifact.summary = summary.str();
  artifact.payload = HelicopterArtifact{std::move(model), declared, {}};
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
  artifact.payload =
      HelicopterTrimArtifact{subject.model, result, environment, std::move(holding), breakdown};
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
  const auto derivative = [&](double, const Eigen::VectorXd& x) {
    return Eigen::VectorXd(simulation_model.derivative(x, commands, subject.environment));
  };
  const std::vector<FailureEvent> failure_events =
      parse_failure_events(context, heli, step_s, steps);

  numerics::IntegrationResult result;
  std::vector<std::string> applied_failure_events;
  if (failure_events.empty()) {
    result = numerics::integrate(derivative,
                                 subject.result.extended_state,
                                 0.0,
                                 options,
                                 &model::VehicleModel::project,
                                 simulation_model.state_bounds());
  } else {
    std::vector<double> all_times{0.0};
    std::vector<Eigen::VectorXd> all_states{subject.result.extended_state};
    Eigen::VectorXd state = subject.result.extended_state;
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
  }

  HelicopterTrajectoryArtifact trajectory;
  trajectory.model = subject.model;
  trajectory.times_s = result.trajectory.times_s;
  trajectory.states = result.trajectory.states;
  trajectory.state_names = heli.state_names();
  trajectory.output_names = heli.output_names();
  trajectory.reason = result.reason;
  trajectory.termination_detail = result.detail;
  trajectory.step_s = step_s;
  trajectory.steps_taken = result.steps_taken;
  trajectory.failure_events = std::move(applied_failure_events);

  for (const auto& state : trajectory.states) {
    const core::State rigid = model::VehicleModel::rigid_body_part(state);
    const Eigen::VectorXd auxiliary = simulation_model.auxiliary_part(state);
    trajectory.outputs.push_back(
        simulation_model.outputs(rigid, auxiliary, commands, subject.environment));
    const auto envelope =
        simulation_model.envelope(rigid, auxiliary, commands, subject.environment);
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
  artifact.summary = summary.str();
  artifact.payload = std::string(path);
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
      {"path"},
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
       "failure_events"}});

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
}

}  // namespace galata::pipeline
