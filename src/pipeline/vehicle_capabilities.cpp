// SPDX-License-Identifier: Apache-2.0
// Shared, type-erased vehicle capabilities.  The family-specific capability
// names remain compatibility adapters; this file is the common path used by
// new studies and by the cross-vehicle Python client.

#include "galata/core/sha256.hpp"
#include "galata/linearize/vehicle.hpp"
#include "galata/model/vehicle_adapters.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/sim/vehicle_execution.hpp"
#include "galata/synth/control.hpp"
#include "galata/trim/hover.hpp"
#include "galata/trim/level.hpp"
#include "galata/trim/problem.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace galata::pipeline {
namespace {

std::string quote(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      out << '\\';
    }
    out << character;
  }
  out << '"';
  return out.str();
}

std::string number(double value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return out.str();
}

void strings(std::ostream& out, const std::vector<std::string>& values) {
  out << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      out << ',';
    }
    out << quote(values[index]);
  }
  out << ']';
}

void metadata(std::ostream& out, const std::vector<model::ChannelMetadata>& values) {
  out << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    const auto& channel = values[index];
    if (index != 0) {
      out << ',';
    }
    out << "{\"name\":" << quote(channel.name) << ",\"unit\":" << quote(channel.unit)
        << ",\"frame\":" << quote(channel.frame)
        << ",\"lower_bound\":" << (channel.has_lower_bound ? number(channel.lower_bound) : "null")
        << ",\"upper_bound\":" << (channel.has_upper_bound ? number(channel.upper_bound) : "null")
        << '}';
  }
  out << ']';
}

void parameters(std::ostream& out, const VehicleArtifact& vehicle) {
  out << '[';
  bool first = true;
  const auto add = [&](const std::string& name, double value, const std::string& unit) {
    if (!first)
      out << ',';
    first = false;
    out << "{\"name\":" << quote(name) << ",\"value\":" << number(value)
        << ",\"unit\":" << quote(unit) << '}';
  };
  if (vehicle.vehicle_kind == "fixed-wing") {
    const auto& aircraft =
        dynamic_cast<const model::FixedWingVehicleModel&>(*vehicle.model).source_model();
    add("mass.mass_kg", aircraft.mass.mass_kg, "kg");
    add("geometry.wing_area_m2", aircraft.geometry.wing_area_m2, "m^2");
    add("geometry.wing_span_m", aircraft.geometry.wing_span_m, "m");
    add("geometry.mean_aerodynamic_chord_m", aircraft.geometry.mean_aerodynamic_chord_m, "m");
    add("aero.reference_alpha_rad", aircraft.aero.reference_alpha_rad, "rad");
    add("aero.lift_alpha", aircraft.aero.lift_alpha, "1/rad");
    add("aero.drag_alpha", aircraft.aero.drag_alpha, "1/rad");
    add("aero.pitching_moment_alpha", aircraft.aero.pitching_moment_alpha, "1/rad");
    add("aero.pitching_moment_elevator", aircraft.aero.pitching_moment_elevator, "1/rad");
  } else if (vehicle.vehicle_kind == "multirotor") {
    const auto& quad =
        dynamic_cast<const model::MultirotorVehicleModel&>(*vehicle.model).source_model();
    add("mass.mass_kg", quad.mass.mass_kg, "kg");
    for (std::size_t index = 0; index < quad.rotors.size(); ++index) {
      const auto& rotor = quad.rotors[index];
      const std::string prefix = "rotors[" + std::to_string(index) + "].";
      add(prefix + "thrust_coefficient_n_s2", rotor.thrust_coefficient_n_s2, "N s^2");
      add(prefix + "torque_coefficient_n_m_s2", rotor.torque_coefficient_n_m_s2, "N m s^2");
      add(prefix + "speed_time_constant_s", rotor.speed_time_constant_s, "s");
      add(prefix + "maximum_speed_rad_s", rotor.maximum_speed_rad_s, "rad/s");
    }
  }
  out << ']';
}

std::string detect_kind(const std::string& declared, const std::string& bytes) {
  if (declared != "auto") {
    return declared;
  }
  if (bytes.find("main_rotor:") != std::string::npos
      || bytes.find("drivetrain:") != std::string::npos) {
    return "helicopter";
  }
  if (bytes.find("rotors:") != std::string::npos) {
    return "multirotor";
  }
  return "fixed-wing";
}

std::map<std::string, double> apply_fixed_overrides(model::Aircraft& aircraft,
                                                    const ValuePtr& declared) {
  std::map<std::string, double> applied;
  if (!declared) {
    return applied;
  }
  std::map<std::string, double*> targets = {
      {"mass.mass_kg", &aircraft.mass.mass_kg},
      {"geometry.wing_area_m2", &aircraft.geometry.wing_area_m2},
      {"geometry.wing_span_m", &aircraft.geometry.wing_span_m},
      {"geometry.mean_aerodynamic_chord_m", &aircraft.geometry.mean_aerodynamic_chord_m},
      {"thrust_incidence_rad", &aircraft.thrust_incidence_rad},
      {"aero.reference_alpha_rad", &aircraft.aero.reference_alpha_rad},
      {"aero.lift_alpha", &aircraft.aero.lift_alpha},
      {"aero.drag_alpha", &aircraft.aero.drag_alpha},
      {"aero.pitching_moment_alpha", &aircraft.aero.pitching_moment_alpha},
      {"aero.pitching_moment_elevator", &aircraft.aero.pitching_moment_elevator},
      {"aero.lift_elevator", &aircraft.aero.lift_elevator},
      {"aero.drag_elevator", &aircraft.aero.drag_elevator}};
  const auto& entries = declared->as_map();
  for (const auto& [name, value] : entries) {
    const auto found = targets.find(name);
    if (found == targets.end()) {
      throw std::invalid_argument("model.vehicle: unsupported fixed-wing parameter '" + name
                                  + "'; inspect the schema for the declared vocabulary");
    }
    if (value->kind() != Value::Kind::Number || !std::isfinite(value->as_number())) {
      throw std::invalid_argument("parameter override '" + name + "' must be a finite number");
    }
    *found->second = value->as_number();
    applied[name] = value->as_number();
  }
  aircraft.validate();
  return applied;
}

std::map<std::string, double> apply_quad_overrides(model::Quadrotor& quadrotor,
                                                   const ValuePtr& declared) {
  std::map<std::string, double> applied;
  if (!declared) {
    return applied;
  }
  for (const auto& [name, value] : declared->as_map()) {
    if (value->kind() != Value::Kind::Number || !std::isfinite(value->as_number())) {
      throw std::invalid_argument("parameter override '" + name + "' must be a finite number");
    }
    const std::string prefix = "rotors[";
    if (name.rfind(prefix, 0) != 0) {
      if (name == "mass.mass_kg") {
        quadrotor.mass.mass_kg = value->as_number();
        applied[name] = value->as_number();
        continue;
      }
      throw std::invalid_argument("model.vehicle: unsupported multirotor parameter '" + name
                                  + "'; rotor overrides use rotors[i].field");
    }
    const std::size_t close = name.find(']');
    if (close == std::string::npos || close + 2 >= name.size() || name[close + 1] != '.') {
      throw std::invalid_argument("model.vehicle: malformed multirotor parameter '" + name + "'");
    }
    const int index = std::stoi(name.substr(prefix.size(), close - prefix.size()));
    if (index < 0 || index >= quadrotor.rotor_count()) {
      throw std::invalid_argument("model.vehicle: rotor parameter index is outside the model");
    }
    auto& rotor = quadrotor.rotors[static_cast<std::size_t>(index)];
    const std::string field = name.substr(close + 2);
    double* target = nullptr;
    if (field == "thrust_coefficient_n_s2") {
      target = &rotor.thrust_coefficient_n_s2;
    } else if (field == "torque_coefficient_n_m_s2") {
      target = &rotor.torque_coefficient_n_m_s2;
    } else if (field == "speed_time_constant_s") {
      target = &rotor.speed_time_constant_s;
    } else if (field == "maximum_speed_rad_s") {
      target = &rotor.maximum_speed_rad_s;
    } else {
      throw std::invalid_argument("model.vehicle: unsupported rotor parameter '" + field + "'");
    }
    *target = value->as_number();
    applied[name] = value->as_number();
  }
  quadrotor.validate();
  return applied;
}

Artifact load_vehicle(const StageContext& context) {
  const std::string declared_path = context.input->string_at("path");
  const std::string bytes = context.read_input(declared_path);
  const std::string kind = detect_kind(context.input->string_at("kind", "auto"), bytes);
  std::shared_ptr<const model::VehicleModel> adapter;
  std::map<std::string, double> overrides;
  if (kind == "fixed-wing") {
    model::Aircraft aircraft = model::parse_aircraft(bytes, declared_path);
    overrides = apply_fixed_overrides(aircraft, context.input->get("parameter_overrides"));
    adapter = std::make_shared<model::FixedWingVehicleModel>(std::move(aircraft));
  } else if (kind == "multirotor") {
    model::Quadrotor quadrotor = model::parse_quadrotor(bytes, declared_path);
    overrides = apply_quad_overrides(quadrotor, context.input->get("parameter_overrides"));
    adapter = std::make_shared<model::MultirotorVehicleModel>(std::move(quadrotor));
  } else if (kind == "helicopter") {
    if (context.input->get("parameter_overrides")) {
      throw std::invalid_argument(
          "model.vehicle: helicopter overrides must use the established model.helicopter "
          "capability");
    }
    adapter = std::make_shared<model::HelicopterVehicleAdapter>(
        model::parse_helicopter(bytes, declared_path));
  } else {
    throw std::invalid_argument(
        "model.vehicle: kind must be fixed-wing, multirotor, helicopter or auto");
  }
  adapter->validate_vocabulary();
  VehicleArtifact payload;
  payload.model = std::move(adapter);
  payload.vehicle_kind = kind;
  payload.identity.origin = "file";
  payload.identity.path = declared_path;
  payload.identity.sha256 = core::sha256(bytes);
  payload.identity.summary = payload.model->description();
  payload.parameter_overrides = std::move(overrides);
  Artifact result;
  result.kind = "vehicle_model";
  result.summary = kind + ": " + payload.model->description() + "; "
                   + std::to_string(payload.model->extended_state_size()) + " states, "
                   + std::to_string(payload.model->control_count()) + " controls";
  result.payload = std::move(payload);
  return result;
}

Artifact trim_vehicle(const StageContext& context) {
  const auto& subject = context.upstream_at("vehicle").payload_as<VehicleArtifact>("vehicle_model");
  const double altitude = context.input->number_at("altitude_m", 0.0);
  const double delta_isa = context.input->number_at("delta_isa_k", 0.0);
  model::Environment environment = model::Environment::at_geometric_altitude(altitude, delta_isa);
  if (const auto wind = context.input->get("wind_ned_m_s")) {
    if (wind->kind() != Value::Kind::List || wind->as_list().size() != 3) {
      throw std::invalid_argument("trim.vehicle wind_ned_m_s must contain three numbers");
    }
    for (int axis = 0; axis < 3; ++axis) {
      environment.wind_ned_m_s(axis) = wind->as_list()[static_cast<std::size_t>(axis)]->as_number();
    }
  }

  Eigen::VectorXd state;
  Eigen::VectorXd controls;
  double residual = 0.0;
  double tolerance = context.input->number_at("tolerance", 1.0e-8);
  if (subject.vehicle_kind == "fixed-wing") {
    const auto* adapter = dynamic_cast<const model::FixedWingVehicleModel*>(subject.model.get());
    if (!adapter) {
      throw std::runtime_error("trim.vehicle: fixed-wing artifact has the wrong adapter type");
    }
    trim::LevelTrimRequest request;
    request.altitude_m = altitude;
    request.airspeed_m_s = context.input->number_at("airspeed_m_s");
    request.delta_isa_k = delta_isa;
    request.residual_tolerance = tolerance;
    const trim::TrimPoint point = trim::trim_level(adapter->source_model(), request);
    state = point.state.to_vector();
    controls = point.controls.to_vector();
    residual = point.residual_norm;
  } else if (subject.vehicle_kind == "multirotor") {
    const auto* adapter = dynamic_cast<const model::MultirotorVehicleModel*>(subject.model.get());
    if (!adapter) {
      throw std::runtime_error("trim.vehicle: multirotor artifact has the wrong adapter type");
    }
    trim::HoverTrimRequest request;
    request.altitude_m = altitude;
    request.wind_ned_m_s = environment.wind_ned_m_s;
    request.ground_velocity_ned_m_s = Eigen::Vector3d::Zero();
    request.heading_rad = context.input->number_at("heading_rad", 0.0);
    request.residual_tolerance = tolerance;
    const trim::HoverTrim point = trim::trim_hover(adapter->source_model(), request);
    state = point.extended_state;
    controls = point.command_rad_s;
    residual = point.residual_norm;
  } else if (subject.vehicle_kind == "helicopter") {
    const auto* adapter = dynamic_cast<const model::HelicopterVehicleAdapter*>(subject.model.get());
    if (!adapter) {
      throw std::runtime_error("trim.vehicle: helicopter artifact has the wrong adapter type");
    }
    trim::TrimCondition condition;
    condition.environment = environment;
    condition.controls = Eigen::VectorXd::Zero(adapter->control_count());
    core::State rigid;
    rigid.position_ned_m = Eigen::Vector3d(0.0, 0.0, -altitude);
    rigid.velocity_body_m_s =
        Eigen::Vector3d(context.input->number_at("airspeed_m_s", 0.0), 0.0, 0.0);
    rigid.attitude_body_to_ned = core::identity_attitude();
    condition.extended_state = adapter->join(
        rigid, adapter->source_model().initial_auxiliary(condition.controls, environment));
    trim::TrimOptions options;
    options.iterations = context.input->integer_at("iterations", options.iterations);
    options.residual_tolerance = tolerance;
    const trim::TrimResult solved =
        trim::solve_trim(adapter->source_model(),
                         trim::helicopter_trim_problem(adapter->source_model()),
                         condition,
                         options);
    state = solved.extended_state;
    const Eigen::VectorXd auxiliary = adapter->auxiliary_part(state);
    controls = solved.controls;
    for (int index = 0; index < adapter->control_count(); ++index) {
      controls(index) = auxiliary(model::kCollectivePosition + index);
    }
    residual = solved.residual_norm;
  } else {
    throw std::runtime_error("trim.vehicle: no trim declaration for kind '" + subject.vehicle_kind
                             + "'");
  }
  VehicleTrimArtifact payload;
  payload.model = subject.model;
  payload.vehicle_kind = subject.vehicle_kind;
  payload.environment = environment;
  payload.extended_state = state;
  payload.controls = controls;
  payload.residual_norm = residual;
  payload.residual_tolerance = tolerance;
  payload.parameter_overrides = subject.parameter_overrides;
  Artifact result;
  result.kind = "vehicle_trim";
  result.summary = subject.vehicle_kind + " trim; residual " + number(residual);
  result.payload = std::move(payload);
  return result;
}

Eigen::VectorXd euler_chart(const Eigen::VectorXd& extended) {
  const core::State state = core::State::from_vector(extended.head<core::kStateSize>());
  const auto angles = core::euler_from_quaternion(state.attitude_body_to_ned);
  Eigen::VectorXd chart(extended.size() - 1);
  chart.segment<3>(0) = state.position_ned_m;
  chart.segment<3>(3) = state.velocity_body_m_s;
  chart(6) = angles.roll_rad;
  chart(7) = angles.pitch_rad;
  chart(8) = angles.yaw_rad;
  chart.segment<3>(9) = state.angular_rate_body_rad_s;
  if (extended.size() > core::kStateSize) {
    chart.segment(12, extended.size() - core::kStateSize) =
        extended.segment(core::kStateSize, extended.size() - core::kStateSize);
  }
  return chart;
}

std::vector<std::string> chart_names(const model::VehicleModel& model) {
  std::vector<std::string> names = {"position_north_m",
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
  const auto states = model.state_names();
  for (int index = 0; index < model.auxiliary_state_count(); ++index) {
    names.push_back(states[static_cast<std::size_t>(core::kStateSize + index)]);
  }
  return names;
}

std::string shared_signal_unit(const std::string& signal) {
  if (signal == "roll_rad" || signal == "pitch_rad" || signal == "yaw_rad") {
    return "rad";
  }
  if (signal == "roll_rate_rad_s" || signal == "pitch_rate_rad_s" || signal == "yaw_rate_rad_s") {
    return "rad/s";
  }
  return "model-declared";
}

Eigen::VectorXd state_from_chart(const model::VehicleModel& model, const Eigen::VectorXd& chart) {
  if (chart.size() != model.extended_state_size() - 1) {
    throw std::invalid_argument("sim.vehicle: chart perturbation has the wrong length");
  }
  core::State state;
  state.position_ned_m = chart.segment<3>(0);
  state.velocity_body_m_s = chart.segment<3>(3);
  state.attitude_body_to_ned = core::quaternion_from_euler({chart(6), chart(7), chart(8)});
  state.angular_rate_body_rad_s = chart.segment<3>(9);
  Eigen::VectorXd auxiliary = Eigen::VectorXd::Zero(model.auxiliary_state_count());
  if (auxiliary.size() > 0) {
    auxiliary = chart.segment(12, auxiliary.size());
  }
  return model.join(state, auxiliary);
}

Artifact linearize_shared(const StageContext& context) {
  const auto& trim = context.upstream_at("trim").payload_as<VehicleTrimArtifact>("vehicle_trim");
  linearize::VehicleLinearisationOptions options;
  options.equilibrium_tolerance = context.input->number_at("equilibrium_tolerance", 1.0e-6);
  options.estimate_truncation_error = context.input->bool_at("report_truncation_error", true);
  auto linear = linearize::linearize_vehicle(
      *trim.model, trim.extended_state, trim.controls, trim.environment, options);
  if (context.input->bool_at("drop_position_and_heading", false)) {
    linear = linear.reduced();
  }
  Artifact result;
  result.kind = "linear_system";
  result.summary = std::to_string(linear.a.rows()) + " states, " + std::to_string(linear.b.cols())
                   + " controls; shared VehicleModel path";
  result.payload = linear.to_linear_system(trim.model->description());
  return result;
}

Artifact simulate_shared(const StageContext& context) {
  const auto& trim = context.upstream_at("trim").payload_as<VehicleTrimArtifact>("vehicle_trim");
  const double step = context.input->number_at("step_s");
  const int steps = context.input->integer_at("steps", 0);
  const int stride = context.input->integer_at("sample_stride", 1);
  Eigen::VectorXd initial = trim.extended_state;
  if (const auto declared = context.input->get("initial_chart_perturbation")) {
    const auto& entries = declared->as_list();
    Eigen::VectorXd perturbation(static_cast<Eigen::Index>(entries.size()));
    for (std::size_t index = 0; index < entries.size(); ++index) {
      perturbation(static_cast<Eigen::Index>(index)) = entries[index]->as_number();
    }
    Eigen::VectorXd chart = euler_chart(initial);
    if (perturbation.size() != chart.size()) {
      throw std::invalid_argument(
          "sim.vehicle initial_chart_perturbation must match the shared chart");
    }
    initial = state_from_chart(*trim.model, chart + perturbation);
  }

  std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> controller;
  if (const auto law_value = context.input->get("law")) {
    const auto& law = context.upstream_at("law").payload_as<synth::LqrDesign>("control_law");
    const Eigen::VectorXd reference = euler_chart(trim.extended_state);
    const auto metadata = trim.model->control_metadata();
    const auto full_names = chart_names(*trim.model);
    std::vector<int> selected_indices;
    for (const auto& name : law.plant.state_names) {
      const auto found = std::find(full_names.begin(), full_names.end(), name);
      if (found == full_names.end()) {
        throw std::invalid_argument("sim.vehicle: controller state '" + name
                                    + "' is not in the vehicle chart");
      }
      selected_indices.push_back(static_cast<int>(found - full_names.begin()));
    }
    controller = [law, reference, metadata, selected_indices, trim_controls = trim.controls](
                     double, const Eigen::VectorXd& state) {
      const Eigen::VectorXd full_error = euler_chart(state) - reference;
      Eigen::VectorXd error(static_cast<Eigen::Index>(selected_indices.size()));
      for (std::size_t index = 0; index < selected_indices.size(); ++index) {
        error(static_cast<Eigen::Index>(index)) = full_error(selected_indices[index]);
      }
      Eigen::VectorXd command =
          law.plant.input_count() == 0 ? Eigen::VectorXd{} : law.riccati.k * error;
      command = trim_controls - command;
      for (Eigen::Index index = 0; index < command.size(); ++index) {
        const auto& channel = metadata[static_cast<std::size_t>(index)];
        if (channel.has_lower_bound) {
          command(index) = std::max(command(index), channel.lower_bound);
        }
        if (channel.has_upper_bound) {
          command(index) = std::min(command(index), channel.upper_bound);
        }
      }
      return command;
    };
  } else {
    controller = [controls = trim.controls](double, const Eigen::VectorXd&) { return controls; };
  }

  sim::VehicleExecutionOptions options;
  options.step_s = step;
  options.steps = steps;
  options.sample_stride = stride;
  options.initial_state = initial;
  options.trim_controls = trim.controls;
  options.environment = trim.environment;
  options.controls = std::move(controller);
  const sim::VehicleExecutionResult execution = sim::execute_vehicle(*trim.model, options);
  VehicleRunArtifact payload;
  payload.model = trim.model;
  payload.vehicle_kind = trim.vehicle_kind;
  payload.result = execution;
  Artifact result;
  result.kind = "vehicle_trajectory";
  result.summary = trim.vehicle_kind + " shared execution; "
                   + std::to_string(execution.integration.trajectory.states.size()) + " samples";
  result.payload = std::move(payload);
  return result;
}

Artifact report_schema(const StageContext& context) {
  const auto& vehicle = context.upstream_at("vehicle").payload_as<VehicleArtifact>("vehicle_model");
  const auto& model = *vehicle.model;
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "{\"schema\":\"galata.vehicle.schema.v1\",\"kind\":" << quote(vehicle.vehicle_kind)
      << ",\"description\":" << quote(model.description()) << ",\"parameters\":";
  parameters(out, vehicle);
  out << ",\"state_metadata\":";
  metadata(out, model.state_metadata());
  out << ",\"states\":";
  metadata(out, model.state_metadata());
  out << ",\"control_metadata\":";
  metadata(out, model.control_metadata());
  out << ",\"controls\":";
  metadata(out, model.control_metadata());
  out << ",\"output_metadata\":";
  metadata(out, model.output_metadata());
  out << ",\"outputs\":";
  metadata(out, model.output_metadata());
  out << ",\"supported_operations\":";
  strings(out, model.supported_operations());
  out << ",\"identity\":{\"path\":" << quote(vehicle.identity.path)
      << ",\"sha256\":" << quote(vehicle.identity.sha256) << ",\"parameter_overrides\":{";
  bool first_override = true;
  for (const auto& [name, value] : vehicle.parameter_overrides) {
    if (!first_override)
      out << ',';
    first_override = false;
    out << quote(name) << ':' << number(value);
  }
  out << "}}}\n";
  const std::string path = context.input->string_at("path");
  context.write_output(path, out.str());
  Artifact result;
  result.kind = "report";
  result.summary = "wrote shared vehicle schema";
  result.payload = context.resolve_output_path(path);
  return result;
}

Artifact report_trim(const StageContext& context) {
  const auto& trim = context.upstream_at("trim").payload_as<VehicleTrimArtifact>("vehicle_trim");
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "{\"schema\":\"galata.vehicle.trim.v1\",\"kind\":" << quote(trim.vehicle_kind)
      << ",\"state_names\":";
  strings(out, trim.model->state_names());
  out << ",\"control_names\":";
  strings(out, trim.model->control_names());
  out << ",\"extended_state\":[";
  for (Eigen::Index i = 0; i < trim.extended_state.size(); ++i) {
    out << (i == 0 ? "" : ",") << trim.extended_state(i);
  }
  out << "],\"controls\":[";
  for (Eigen::Index i = 0; i < trim.controls.size(); ++i) {
    out << (i == 0 ? "" : ",") << trim.controls(i);
  }
  out << "],\"residual_norm\":" << trim.residual_norm
      << ",\"residual_tolerance\":" << trim.residual_tolerance << ",\"parameter_overrides\":{";
  bool first_override = true;
  for (const auto& [name, value] : trim.parameter_overrides) {
    if (!first_override)
      out << ',';
    first_override = false;
    out << quote(name) << ':' << number(value);
  }
  out << "}}\n";
  const std::string path = context.input->string_at("path");
  context.write_output(path, out.str());
  Artifact result;
  result.kind = "report";
  result.summary = "wrote shared vehicle trim";
  result.payload = context.resolve_output_path(path);
  return result;
}

Artifact report_vehicle_csv(const StageContext& context) {
  const auto& run =
      context.upstream_at("trajectory").payload_as<VehicleRunArtifact>("vehicle_trajectory");
  const auto& states = run.result.integration.trajectory.states;
  const auto& times = run.result.integration.trajectory.times_s;
  const auto state_names = run.model->state_names();
  const auto control_names = run.model->control_names();
  const auto output_names = run.model->output_names();
  if (states.size() != times.size() || run.result.controls.size() != states.size()
      || run.result.outputs.size() != states.size()) {
    throw std::runtime_error("report.vehicle_csv: trajectory metadata lengths differ");
  }
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << "time_s";
  for (const auto& name : state_names)
    out << ",state:" << name;
  for (const auto& name : control_names)
    out << ",control:" << name;
  for (const auto& name : output_names)
    out << ",output:" << name;
  out << '\n';
  for (std::size_t row = 0; row < states.size(); ++row) {
    out << times[row];
    for (Eigen::Index i = 0; i < states[row].size(); ++i)
      out << ',' << states[row](i);
    for (Eigen::Index i = 0; i < run.result.controls[row].size(); ++i)
      out << ',' << run.result.controls[row](i);
    for (Eigen::Index i = 0; i < run.result.outputs[row].size(); ++i)
      out << ',' << run.result.outputs[row](i);
    out << '\n';
  }
  const std::string path = context.input->string_at("path");
  context.write_output(path, out.str());
  Artifact result;
  result.kind = "report";
  result.summary = "wrote shared vehicle trajectory";
  result.payload = context.resolve_output_path(path);
  return result;
}

Artifact report_vehicle_response(const StageContext& context) {
  const auto& open =
      context.upstream_at("open").payload_as<VehicleRunArtifact>("vehicle_trajectory");
  const auto& closed =
      context.upstream_at("closed").payload_as<VehicleRunArtifact>("vehicle_trajectory");
  const auto& times_open = open.result.integration.trajectory.times_s;
  const auto& times_closed = closed.result.integration.trajectory.times_s;
  if (times_open != times_closed || open.result.outputs.size() != closed.result.outputs.size()
      || open.result.outputs.empty()) {
    throw std::invalid_argument(
        "report.vehicle_response_json: open and closed runs require identical non-empty time "
        "grids");
  }
  std::map<std::string, std::map<std::string, double>> requirements;
  if (const auto declared = context.input->get("signal_requirements")) {
    if (declared->kind() != Value::Kind::Map) {
      throw std::invalid_argument(
          "report.vehicle_response_json signal_requirements must be a map keyed by signal");
    }
    const std::set<std::string> allowed = {"peak_tracking_error_rad",
                                           "final_tracking_error_rad",
                                           "rms_tracking_error_rad",
                                           "settling_band_rad",
                                           "settling_dwell_s",
                                           "settling_time_s",
                                           "peak_tracking_error_rad_s",
                                           "final_tracking_error_rad_s",
                                           "rms_tracking_error_rad_s",
                                           "settling_band_rad_s",
                                           "peak_tracking_error_m",
                                           "final_tracking_error_m",
                                           "rms_tracking_error_m",
                                           "settling_band_m"};
    for (const auto& [signal, signal_value] : declared->as_map()) {
      if (signal_value->kind() != Value::Kind::Map) {
        throw std::invalid_argument("report.vehicle_response_json requirements for '" + signal
                                    + "' must be a map");
      }
      for (const auto& [name, value] : signal_value->as_map()) {
        if (!allowed.contains(name)) {
          throw std::invalid_argument("report.vehicle_response_json contains unknown requirement '"
                                      + name + "'");
        }
        if (value->kind() != Value::Kind::Number || !std::isfinite(value->as_number())
            || value->as_number() < 0.0) {
          throw std::invalid_argument("report.vehicle_response_json requirement '" + name
                                      + "' must be finite and non-negative");
        }
        requirements[signal][name] = value->as_number();
      }
    }
  }
  bool criteria_passed = !requirements.empty();
  bool criteria_evaluated = !requirements.empty();
  std::vector<std::string> requested;
  if (const auto signals = context.input->get("signals")) {
    for (const auto& item : signals->as_list())
      requested.push_back(item->as_string());
  } else {
    requested = closed.model->output_names();
  }
  const auto names = closed.model->output_names();
  const auto chart = chart_names(*closed.model);
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "{\"schema\":\"galata.vehicle.response.v1\",\"criteria_status\":\"not_assessed\","
      << "\"criteria_passed\":false,\"metrics\":[";
  bool first = true;
  for (const auto& signal : requested) {
    const auto found = std::find(names.begin(), names.end(), signal);
    const auto chart_found = std::find(chart.begin(), chart.end(), signal);
    if (found == names.end() && chart_found == chart.end()) {
      throw std::invalid_argument("report.vehicle_response_json: unknown signal '" + signal + "'");
    }
    const bool from_chart = found == names.end();
    const Eigen::Index column =
        static_cast<Eigen::Index>(from_chart ? chart_found - chart.begin() : found - names.begin());
    const auto value_at = [&](const VehicleRunArtifact& run, std::size_t row) {
      if (!from_chart) {
        return run.result.outputs[row](column);
      }
      return euler_chart(run.result.integration.trajectory.states[row])(column);
    };
    const double reference = value_at(closed, 0);
    double peak = 0.0;
    double sum_squared = 0.0;
    for (std::size_t row = 0; row < closed.result.outputs.size(); ++row) {
      const double error = value_at(closed, row) - reference;
      peak = std::max(peak, std::fabs(error));
      sum_squared += error * error;
    }
    const double final_error = value_at(closed, closed.result.outputs.size() - 1) - reference;
    const double rms = std::sqrt(sum_squared / static_cast<double>(closed.result.outputs.size()));
    const auto requirement_values = requirements.find(signal);
    const auto requirement = [&](const std::string& name) {
      if (requirement_values == requirements.end())
        return std::numeric_limits<double>::quiet_NaN();
      const auto found_requirement = requirement_values->second.find(name);
      return found_requirement == requirement_values->second.end()
                 ? std::numeric_limits<double>::quiet_NaN()
                 : found_requirement->second;
    };
    const std::string suffix = shared_signal_unit(signal) == "m"
                                   ? "_m"
                                   : (shared_signal_unit(signal) == "rad/s" ? "_rad_s" : "_rad");
    const double settling_band = requirement("settling_band" + suffix);
    const double settling_dwell = requirement("settling_dwell_s");
    std::string settling_status = "not_assessed";
    double settling_time = std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(settling_band)) {
      const double sample_period =
          times_closed.size() > 1 ? times_closed[1] - times_closed[0] : 0.0;
      const std::size_t dwell_samples =
          sample_period > 0.0 && std::isfinite(settling_dwell)
              ? std::max<std::size_t>(
                    1, static_cast<std::size_t>(std::ceil(settling_dwell / sample_period)))
              : 1;
      const bool initially_inside = std::fabs(value_at(closed, 0) - reference) <= settling_band;
      if (initially_inside) {
        settling_status = "already_within_band";
        settling_time = 0.0;
      } else {
        for (std::size_t start = 0; start + dwell_samples <= closed.result.outputs.size();
             ++start) {
          bool inside = true;
          for (std::size_t sample = start; sample < start + dwell_samples; ++sample) {
            inside = inside && std::fabs(value_at(closed, sample) - reference) <= settling_band;
          }
          if (inside) {
            settling_status = "demonstrated_recovery";
            settling_time = times_closed[start] - times_closed.front();
            break;
          }
        }
        if (!std::isfinite(settling_time))
          settling_status = "not_settled_within_observation_window";
      }
    }
    const auto check = [&](const std::string& name, double measured) {
      const double limit = requirement(name);
      return !std::isfinite(limit) || measured <= limit;
    };
    const bool signal_passed =
        check("peak_tracking_error" + suffix, peak)
        && check("final_tracking_error" + suffix, std::fabs(final_error))
        && check("rms_tracking_error" + suffix, rms)
        && (!std::isfinite(requirement("settling_time_s"))
            || (std::isfinite(settling_time) && settling_time <= requirement("settling_time_s")));
    criteria_passed = criteria_passed && signal_passed;
    if (!first)
      out << ',';
    first = false;
    out << "{\"signal\":" << quote(signal) << ",\"unit\":" << quote(shared_signal_unit(signal))
        << ",\"peak_tracking_error\":" << peak
        << ",\"final_tracking_error\":" << std::fabs(final_error)
        << ",\"rms_tracking_error\":" << rms << ",\"settling_status\":" << quote(settling_status)
        << ",\"settling_time_s\":"
        << (std::isfinite(settling_time) ? number(settling_time) : "null")
        << ",\"criteria_passed\":" << (signal_passed ? "true" : "false") << "}";
  }
  out << "],\"requirements\":{},\"criteria_status\":"
      << quote(criteria_evaluated ? (criteria_passed ? "pass" : "fail") : "not_assessed")
      << ",\"criteria_passed\":" << (criteria_evaluated && criteria_passed ? "true" : "false")
      << ",\"requirements_present\":" << (criteria_evaluated ? "true" : "false")
      << ",\"execution_completed\":true}\n";
  const std::string path = context.input->string_at("path");
  context.write_output(path, out.str());
  Artifact result;
  result.kind = "report";
  result.summary = "wrote shared vehicle response metrics";
  result.payload = context.resolve_output_path(path);
  return result;
}

}  // namespace

void register_vehicle_capabilities(Registry& registry) {
  using State = Capability::State;
  registry.add({"model.vehicle",
                "Load a fixed-wing, multirotor or helicopter through the shared VehicleModel seam",
                "vehicle_model",
                State::ImplementedUnvalidated,
                load_vehicle,
                {"path", "kind", "parameter_overrides"},
                {"path"},
                {},
                {{"path", 8 * 1024 * 1024}}});
  registry.add(
      {"trim.vehicle",
       "Execute the declared family-specific trim problem and return a shared operating point",
       "vehicle_trim",
       State::ImplementedUnvalidated,
       trim_vehicle,
       {"vehicle",
        "airspeed_m_s",
        "altitude_m",
        "delta_isa_k",
        "wind_ned_m_s",
        "heading_rad",
        "tolerance",
        "iterations"}});
  registry.add(
      {"linearize.shared",
       "Linearise any built-in vehicle adapter through the common named VehicleModel service",
       "linear_system",
       State::ImplementedUnvalidated,
       linearize_shared,
       {"trim", "drop_position_and_heading", "report_truncation_error", "equilibrium_tolerance"}});
  registry.add({"sim.vehicle",
                "Execute fixed-step RK4, projection, envelope and named outputs through the shared "
                "vehicle service",
                "vehicle_trajectory",
                State::ImplementedUnvalidated,
                simulate_shared,
                {"trim", "law", "step_s", "steps", "sample_stride", "initial_chart_perturbation"}});
  registry.add(
      {"report.vehicle_schema_json",
       "Write versioned shared model metadata, units, frames, bounds and supported operations",
       "report",
       State::ImplementedUnvalidated,
       report_schema,
       {"vehicle", "path"},
       {},
       {"path"}});
  registry.add(
      {"report.vehicle_trim_json",
       "Write a structured shared trim result with state, controls and residual diagnostics",
       "report",
       State::ImplementedUnvalidated,
       report_trim,
       {"trim", "path"},
       {},
       {"path"}});
  registry.add({"report.vehicle_csv",
                "Write a shared trajectory with named state, control and output channels",
                "report",
                State::ImplementedUnvalidated,
                report_vehicle_csv,
                {"trajectory", "path"},
                {},
                {"path"}});
  registry.add({"report.vehicle_response_json",
                "Write structured open/closed response metrics on a shared vehicle trajectory",
                "report",
                State::ImplementedUnvalidated,
                report_vehicle_response,
                {"open", "closed", "signals", "signal_requirements", "path"},
                {},
                {"path"}});
}

}  // namespace galata::pipeline
