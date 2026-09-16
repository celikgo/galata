// SPDX-License-Identifier: Apache-2.0
//
// Composition of the extended-state derivative, and the vocabulary checks.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 2.
//
// Validity envelope and known error behaviour are the implementing model's.
// This file adds no physics: it resolves gravity, calls the shared kernel, and
// adds the wind back to the position rate. Each of those is exactly once.

#include "galata/model/vehicle.hpp"

#include "galata/core/quaternion.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

namespace galata::model {
namespace {

// Standard gravity, 3rd CGPM 1901. Not a unit conversion: it is the defining
// value of the acceleration the numerical core already works in.
constexpr double kStandardGravityM_S2 = 9.80665;

// ISA sea level, U.S. Standard Atmosphere 1976. These are the same values
// core/atmosphere.cpp computes at h = 0; they are repeated here only as the
// default for a caller who has not asked for an altitude, and a test asserts
// they agree with the atmosphere model rather than trusting that they do.
constexpr double kSeaLevelDensityKg_M3 = 1.225;
constexpr double kSeaLevelSpeedOfSoundM_S = 340.294;
constexpr double kSeaLevelPressurePa = 101325.0;
constexpr double kSeaLevelTemperatureK = 288.15;

void require_finite(double value, const char* what) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string("Environment: ") + what + " must be finite");
  }
}

}  // namespace

Environment Environment::sea_level_still_air() noexcept {
  Environment environment;
  environment.gravity_ned_m_s2 = Eigen::Vector3d(0.0, 0.0, kStandardGravityM_S2);
  environment.density_kg_m3 = kSeaLevelDensityKg_M3;
  environment.speed_of_sound_m_s = kSeaLevelSpeedOfSoundM_S;
  environment.pressure_pa = kSeaLevelPressurePa;
  environment.temperature_k = kSeaLevelTemperatureK;
  return environment;
}

void Environment::validate() const {
  if (!gravity_ned_m_s2.allFinite() || !wind_ned_m_s.allFinite()
      || !wind_rate_ned_m_s2.allFinite()) {
    throw std::invalid_argument("Environment: gravity, wind and wind rate must be finite");
  }
  require_finite(density_kg_m3, "density_kg_m3");
  require_finite(speed_of_sound_m_s, "speed_of_sound_m_s");
  require_finite(pressure_pa, "pressure_pa");
  require_finite(temperature_k, "temperature_k");
  if (!(density_kg_m3 > 0.0)) {
    throw std::invalid_argument(
        "Environment: density_kg_m3 must be positive. A zero density silently zeroes every "
        "aerodynamic force and produces a smooth, plausible, wrong trajectory");
  }
  if (!(speed_of_sound_m_s > 0.0) || !(pressure_pa > 0.0) || !(temperature_k > 0.0)) {
    throw std::invalid_argument(
        "Environment: speed of sound, pressure and temperature must be positive");
  }
}

Eigen::VectorXd VehicleModel::outputs(const core::State& state,
                                      const Eigen::VectorXd& auxiliary,
                                      const Eigen::VectorXd& /*controls*/,
                                      const Environment& /*environment*/) const {
  return join(state, auxiliary);
}

core::State VehicleModel::rigid_body_part(const Eigen::VectorXd& extended_state) {
  if (extended_state.size() < core::kStateSize) {
    throw std::invalid_argument("VehicleModel: extended state is shorter than the rigid-body state");
  }
  return core::State::from_vector(core::StateVector(extended_state.head<core::kStateSize>()));
}

Eigen::VectorXd VehicleModel::auxiliary_part(const Eigen::VectorXd& extended_state) const {
  const int count = auxiliary_state_count();
  if (extended_state.size() != extended_state_size()) {
    throw std::invalid_argument("VehicleModel: extended state has "
                                + std::to_string(extended_state.size()) + " entries, expected "
                                + std::to_string(extended_state_size()));
  }
  return extended_state.segment(core::kStateSize, count);
}

Eigen::VectorXd VehicleModel::join(const core::State& state,
                                   const Eigen::VectorXd& auxiliary) const {
  const int count = auxiliary_state_count();
  if (auxiliary.size() != count) {
    throw std::invalid_argument("VehicleModel: auxiliary state has "
                                + std::to_string(auxiliary.size()) + " entries, expected "
                                + std::to_string(count));
  }
  Eigen::VectorXd extended(extended_state_size());
  extended.head<core::kStateSize>() = state.to_vector();
  if (count > 0) {
    extended.segment(core::kStateSize, count) = auxiliary;
  }
  return extended;
}

void VehicleModel::project(Eigen::VectorXd& extended_state) {
  if (extended_state.size() < core::kStateSize) {
    throw std::invalid_argument("VehicleModel::project: state is shorter than the rigid-body state");
  }
  const Eigen::Vector4d wxyz = extended_state.segment<4>(core::kQuaternionW);
  const double norm = wxyz.norm();
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    throw std::runtime_error("VehicleModel::project: attitude quaternion has zero or non-finite norm");
  }
  extended_state.segment<4>(core::kQuaternionW) = wxyz / norm;
}

Eigen::VectorXd VehicleModel::derivative(const Eigen::VectorXd& extended_state,
                                         const Eigen::VectorXd& controls,
                                         const Environment& environment) const {
  if (extended_state.size() != extended_state_size()) {
    throw std::invalid_argument("VehicleModel::derivative: state has "
                                + std::to_string(extended_state.size()) + " entries, expected "
                                + std::to_string(extended_state_size()));
  }
  if (controls.size() != control_count()) {
    throw std::invalid_argument("VehicleModel::derivative: controls have "
                                + std::to_string(controls.size()) + " entries, expected "
                                + std::to_string(control_count()));
  }
  if (!extended_state.allFinite()) {
    throw std::runtime_error("VehicleModel::derivative: the state is not finite");
  }
  if (!controls.allFinite()) {
    throw std::runtime_error("VehicleModel::derivative: the controls are not finite");
  }

  const core::State state = rigid_body_part(extended_state);
  const Eigen::VectorXd auxiliary = auxiliary_part(extended_state);

  const sim::MassProperties mass = mass_properties(auxiliary);
  const sim::Wrench applied = wrench(state, auxiliary, controls, environment);
  if (!applied.force_body_n.allFinite() || !applied.moment_cg_body_n_m.allFinite()) {
    throw std::runtime_error("VehicleModel::derivative: the model returned a non-finite wrench");
  }

  Eigen::VectorXd rate(extended_state_size());
  rate.head<core::kStateSize>() =
      sim::rigid_body_derivative(state, mass, applied, environment.gravity_ned_m_s2);

  // THE POSITION RATE IS THE GROUND VELOCITY, AND THE STATE VELOCITY IS NOT.
  //
  // ADR-0002's velocity is air-relative, so the kernel's position rate is the
  // air-relative position rate and the wind has to be added back. state.hpp
  // makes that the caller's responsibility; this is the caller, and this is the
  // only place it happens.
  rate.segment<3>(core::kPositionNorth) += environment.wind_ned_m_s;

  // AND THE AIR-RELATIVE ACCELERATION CARRIES THE WIND'S OWN DERIVATIVE.
  //
  // d(v_air)/dt = a_inertial - dw/dt, with dw/dt resolved into body axes. For a
  // steady wind this term is zero and nothing changes, which is why every
  // existing result is unaffected. For a wind that CHANGES it is the difference
  // between a gust the aircraft flies through and a step the integrator eats as
  // a ground-velocity error.
  if (!environment.wind_rate_ned_m_s2.isZero()) {
    const Eigen::Matrix3d body_from_ned =
        core::dcm_body_from_ned(state.attitude_body_to_ned);
    rate.segment<3>(core::kVelocityU) -= body_from_ned * environment.wind_rate_ned_m_s2;
  }

  const int count = auxiliary_state_count();
  if (count > 0) {
    const Eigen::VectorXd auxiliary_rate =
        auxiliary_derivative(state, auxiliary, controls, environment);
    if (auxiliary_rate.size() != count) {
      throw std::runtime_error("VehicleModel::derivative: auxiliary_derivative returned "
                               + std::to_string(auxiliary_rate.size()) + " entries, expected "
                               + std::to_string(count));
    }
    if (!auxiliary_rate.allFinite()) {
      throw std::runtime_error(
          "VehicleModel::derivative: the model returned a non-finite auxiliary rate");
    }
    rate.segment(core::kStateSize, count) = auxiliary_rate;
  }
  return rate;
}

void VehicleModel::validate_vocabulary() const {
  const auto states = state_names();
  const auto controls = control_names();
  const auto outputs_named = output_names();

  if (static_cast<int>(states.size()) != extended_state_size()) {
    throw std::invalid_argument("VehicleModel: state_names() has " + std::to_string(states.size())
                                + " entries but the extended state has "
                                + std::to_string(extended_state_size()));
  }
  if (auxiliary_state_count() < 0) {
    throw std::invalid_argument("VehicleModel: auxiliary_state_count() is negative");
  }

  // A DUPLICATED OR EMPTY NAME IS A MODEL BUG, NOT A COSMETIC ONE. Trim resolves
  // its unknowns by name, a linearisation labels its rows by name, and a report
  // column is identified by name. Two states called "omega" make a trim unknown
  // ambiguous and the solver picks whichever the lookup reached first.
  const auto check = [](const std::vector<std::string>& names, const char* what) {
    std::set<std::string> seen;
    for (const auto& name : names) {
      if (name.empty()) {
        throw std::invalid_argument(std::string("VehicleModel: an entry of ") + what + " is empty");
      }
      if (!seen.insert(name).second) {
        throw std::invalid_argument(std::string("VehicleModel: ") + what + " repeats the name '"
                                    + name + "'; names identify trim unknowns and matrix rows, so "
                                    "a repeat makes both ambiguous");
      }
    }
  };
  check(states, "state_names()");
  check(controls, "control_names()");
  check(outputs_named, "output_names()");
}

}  // namespace galata::model
