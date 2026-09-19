// SPDX-License-Identifier: Apache-2.0

#include "galata/model/vehicle_adapters.hpp"

#include "galata/core/atmosphere.hpp"
#include "galata/core/frames.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace galata::model {
namespace {

core::AtmosphereState atmosphere_from(const Environment& environment) {
  core::AtmosphereState atmosphere;
  atmosphere.temperature_k = environment.temperature_k;
  atmosphere.pressure_pa = environment.pressure_pa;
  atmosphere.density_kg_m3 = environment.density_kg_m3;
  atmosphere.speed_of_sound_m_s = environment.speed_of_sound_m_s;
  atmosphere.geometric_altitude_m = environment.atmospheric_altitude_m;
  atmosphere.delta_isa_k = environment.delta_isa_k;
  return atmosphere;
}

std::vector<ChannelMetadata> make_control_metadata(const std::vector<std::string>& names,
                                                   const std::vector<double>& lower,
                                                   const std::vector<double>& upper,
                                                   const std::string& unit) {
  if (names.size() != lower.size() || names.size() != upper.size()) {
    throw std::logic_error("vehicle adapter: control metadata dimensions differ");
  }
  std::vector<ChannelMetadata> result;
  result.reserve(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    ChannelMetadata channel;
    channel.name = names[i];
    channel.unit = unit;
    channel.frame = "body-FRD";
    channel.has_lower_bound = true;
    channel.has_upper_bound = true;
    channel.lower_bound = lower[i];
    channel.upper_bound = upper[i];
    result.push_back(channel);
  }
  return result;
}

}  // namespace

std::string FixedWingVehicleModel::description() const {
  return aircraft_.description.empty() ? std::string("fixed-wing aircraft") : aircraft_.description;
}

std::vector<std::string> FixedWingVehicleModel::state_names() const {
  return {"position_north_m", "position_east_m", "position_down_m", "velocity_u_m_s",
          "velocity_v_m_s", "velocity_w_m_s", "quaternion_w", "quaternion_x", "quaternion_y",
          "quaternion_z", "roll_rate_rad_s", "pitch_rate_rad_s", "yaw_rate_rad_s"};
}

std::vector<std::string> FixedWingVehicleModel::control_names() const {
  return {"elevator_rad", "aileron_rad", "rudder_rad", "thrust_n"};
}

std::vector<std::string> FixedWingVehicleModel::output_names() const {
  return state_names();
}

sim::MassProperties FixedWingVehicleModel::mass_properties(const Eigen::VectorXd&) const {
  return aircraft_.mass;
}

sim::Wrench FixedWingVehicleModel::wrench(const core::State& state,
                                          const Eigen::VectorXd& auxiliary,
                                          const Eigen::VectorXd& controls,
                                          const Environment& environment) const {
  if (auxiliary.size() != 0) {
    throw std::invalid_argument("FixedWingVehicleModel: auxiliary state must be empty");
  }
  const Controls converted = Controls::from_vector(controls);
  const core::AtmosphereState atmosphere = atmosphere_from(environment);
  if (aircraft_.aero.pitching_moment_alpha_dot == 0.0) {
    return aircraft_.wrench(state, converted, atmosphere, 0.0);
  }
  // Preserve the two-pass alpha-dot ordering of Aircraft::derivative while
  // still exposing one wrench to VehicleModel::derivative.  Translational
  // acceleration is independent of alpha-dot by Aircraft::validate(), so the
  // intermediate rigid-body rate is the same rate the legacy path used.
  const sim::Wrench without_lag = aircraft_.wrench(state, converted, atmosphere, 0.0);
  const core::StateVector first = sim::rigid_body_derivative(
      state, aircraft_.mass, without_lag, environment.gravity_ned_m_s2);
  const double u = state.velocity_body_m_s.x();
  const double w = state.velocity_body_m_s.z();
  const double denominator = u * u + w * w;
  const double alpha_dot = denominator > 0.0
                               ? (u * first(core::kVelocityW) - w * first(core::kVelocityU))
                                     / denominator
                               : 0.0;
  return aircraft_.wrench(state, converted, atmosphere, alpha_dot);
}

Eigen::VectorXd FixedWingVehicleModel::auxiliary_derivative(const core::State&,
                                                            const Eigen::VectorXd& auxiliary,
                                                            const Eigen::VectorXd&,
                                                            const Environment&) const {
  if (auxiliary.size() != 0) {
    throw std::invalid_argument("FixedWingVehicleModel: auxiliary state must be empty");
  }
  return Eigen::VectorXd(0);
}

EnvelopeStatus FixedWingVehicleModel::envelope(const core::State& state,
                                               const Eigen::VectorXd& auxiliary,
                                               const Eigen::VectorXd& controls,
                                               const Environment& environment) const {
  const auto warning = aircraft_.envelope(state, atmosphere_from(environment));
  EnvelopeStatus result;
  result.outside = warning.outside_advisory_envelope;
  result.worst_departure = std::max(warning.alpha_departure_rad,
                                    warning.mach_departure);
  if (warning.outside_advisory_envelope) {
    result.reason = "fixed-wing alpha/Mach advisory envelope";
  }
  (void)auxiliary;
  (void)controls;
  return result;
}

std::vector<std::string> FixedWingVehicleModel::supported_operations() const {
  return {"inspect", "evaluate", "trim.level", "simulate", "linearize", "synth.lqr"};
}

std::string MultirotorVehicleModel::description() const {
  return quadrotor_.description.empty() ? std::string("multirotor") : quadrotor_.description;
}

std::vector<std::string> MultirotorVehicleModel::state_names() const {
  std::vector<std::string> names = {
      "position_north_m", "position_east_m", "position_down_m", "velocity_u_m_s",
      "velocity_v_m_s", "velocity_w_m_s", "quaternion_w", "quaternion_x",
      "quaternion_y", "quaternion_z", "roll_rate_rad_s", "pitch_rate_rad_s",
      "yaw_rate_rad_s"};
  for (int index = 0; index < quadrotor_.rotor_count(); ++index) {
    names.push_back("rotor_speed_" + std::to_string(index) + "_rad_s");
  }
  if (quadrotor_.has_battery()) {
    names.emplace_back("battery_soc");
  }
  return names;
}

std::vector<std::string> MultirotorVehicleModel::control_names() const {
  return quadrotor_.input_names();
}

std::vector<std::string> MultirotorVehicleModel::output_names() const {
  return state_names();
}

int MultirotorVehicleModel::auxiliary_state_count() const {
  return quadrotor_.extended_state_size() - core::kStateSize;
}

sim::MassProperties MultirotorVehicleModel::mass_properties(
    const Eigen::VectorXd&) const {
  return quadrotor_.mass;
}

sim::Wrench MultirotorVehicleModel::wrench(const core::State& state,
                                           const Eigen::VectorXd& auxiliary,
                                           const Eigen::VectorXd& controls,
                                           const Environment&) const {
  if (auxiliary.size() != auxiliary_state_count()) {
    throw std::invalid_argument("MultirotorVehicleModel: auxiliary state has the wrong length");
  }
  (void)controls;
  return quadrotor_.wrench(state,
                           auxiliary.head(quadrotor_.rotor_count()));
}

Eigen::VectorXd MultirotorVehicleModel::auxiliary_derivative(
    const core::State& state,
    const Eigen::VectorXd& auxiliary,
    const Eigen::VectorXd& controls,
    const Environment& environment) const {
  if (auxiliary.size() != auxiliary_state_count()) {
    throw std::invalid_argument("MultirotorVehicleModel: auxiliary state has the wrong length");
  }
  (void)state;
  (void)environment;
  Eigen::VectorXd result = Eigen::VectorXd::Zero(auxiliary_state_count());
  const Eigen::VectorXd speeds = auxiliary.head(quadrotor_.rotor_count());
  const double state_of_charge = quadrotor_.has_battery()
                                     ? auxiliary(quadrotor_.battery_state_index()
                                                 - core::kStateSize)
                                     : 1.0;
  const double drawn_shaft_w = quadrotor_.has_battery()
                                   ? quadrotor_.shaft_power_w(speeds)
                                   : 0.0;
  for (int index = 0; index < quadrotor_.rotor_count(); ++index) {
    const Rotor& rotor = quadrotor_.rotors[static_cast<std::size_t>(index)];
    const double ceiling = std::max(
        quadrotor_.speed_ceiling_rad_s(index, state_of_charge, drawn_shaft_w),
        rotor.minimum_speed_rad_s);
    const double target = std::clamp(controls(index), rotor.minimum_speed_rad_s, ceiling);
    result(index) = (target - speeds(index)) / rotor.speed_time_constant_s;
  }
  if (quadrotor_.has_battery()) {
    double electrical_w = drawn_shaft_w;
    if (quadrotor_.battery->sag == Battery::SagModel::Resistive) {
      electrical_w = drawn_shaft_w / quadrotor_.battery->motor_and_esc_efficiency
                     + quadrotor_.battery->auxiliary_load_w;
    }
    result(quadrotor_.battery_state_index() - core::kStateSize) =
        -electrical_w / quadrotor_.battery->energy_j;
  }
  return result;
}

EnvelopeStatus MultirotorVehicleModel::envelope(const core::State& state,
                                                const Eigen::VectorXd& auxiliary,
                                                const Eigen::VectorXd& controls,
                                                const Environment&) const {
  EnvelopeStatus result;
  const double airspeed = state.velocity_body_m_s.norm();
  if (!std::isfinite(airspeed) || !state.to_vector().allFinite() || !auxiliary.allFinite()
      || !controls.allFinite()) {
    result.outside = true;
    result.worst_departure = std::numeric_limits<double>::infinity();
    result.reason = "multirotor non-finite state or command";
  }
  return result;
}

std::vector<ChannelMetadata> MultirotorVehicleModel::control_metadata() const {
  std::vector<double> lower;
  std::vector<double> upper;
  lower.reserve(quadrotor_.rotors.size());
  upper.reserve(quadrotor_.rotors.size());
  for (const auto& rotor : quadrotor_.rotors) {
    lower.push_back(rotor.minimum_speed_rad_s);
    upper.push_back(rotor.maximum_speed_rad_s);
  }
  return make_control_metadata(control_names(), lower, upper, "rad/s");
}

std::vector<std::string> MultirotorVehicleModel::supported_operations() const {
  return {"inspect", "evaluate", "trim.hover", "simulate", "linearize", "synth.lqr",
          "wind_schedule"};
}

std::vector<ChannelMetadata> HelicopterVehicleAdapter::control_metadata() const {
  std::vector<double> lower;
  std::vector<double> upper;
  for (const auto& actuator : helicopter_.actuators) {
    lower.push_back(actuator.minimum_rad);
    upper.push_back(actuator.maximum_rad);
  }
  return make_control_metadata(control_names(), lower, upper, "rad");
}

std::vector<std::string> HelicopterVehicleAdapter::supported_operations() const {
  return {"inspect", "evaluate", "trim.helicopter", "simulate", "linearize", "synth.lqr",
          "sampled_control", "wind_schedule", "failure_schedule"};
}

}  // namespace galata::model
