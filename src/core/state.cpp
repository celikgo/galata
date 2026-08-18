// SPDX-License-Identifier: Apache-2.0
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 2 — the rigid-body state and
//   the definitions of V, alpha and beta.
//   B. Etkin and L. D. Reid, "Dynamics of Flight: Stability and Control",
//   3rd ed., Wiley, 1996, chapter 4 — the same definitions, and the flight-
//   path angle relation.
//
// Normative convention statement: docs/adr/0002-state-and-frame-conventions.md.

#include "galata/core/state.hpp"

#include "galata/core/quaternion.hpp"

#include <cmath>

namespace galata::core {

StateVector State::to_vector() const noexcept {
  StateVector x;
  x(kPositionNorth) = position_ned_m.x();
  x(kPositionEast) = position_ned_m.y();
  x(kPositionDown) = position_ned_m.z();
  x(kVelocityU) = velocity_body_m_s.x();
  x(kVelocityV) = velocity_body_m_s.y();
  x(kVelocityW) = velocity_body_m_s.z();
  x(kQuaternionW) = attitude_body_to_ned.w();
  x(kQuaternionX) = attitude_body_to_ned.x();
  x(kQuaternionY) = attitude_body_to_ned.y();
  x(kQuaternionZ) = attitude_body_to_ned.z();
  x(kRateP) = angular_rate_body_rad_s.x();
  x(kRateQ) = angular_rate_body_rad_s.y();
  x(kRateR) = angular_rate_body_rad_s.z();
  return x;
}

State State::from_vector(const StateVector& x) noexcept {
  State state;
  state.position_ned_m = Eigen::Vector3d(x(kPositionNorth), x(kPositionEast), x(kPositionDown));
  state.velocity_body_m_s = Eigen::Vector3d(x(kVelocityU), x(kVelocityV), x(kVelocityW));
  state.attitude_body_to_ned =
      Quaternion(x(kQuaternionW), x(kQuaternionX), x(kQuaternionY), x(kQuaternionZ));
  state.angular_rate_body_rad_s = Eigen::Vector3d(x(kRateP), x(kRateQ), x(kRateR));
  return state;
}

void State::renormalise_attitude() noexcept {
  attitude_body_to_ned = normalised(attitude_body_to_ned);
}

double airspeed(const Eigen::Vector3d& velocity_body_m_s) noexcept {
  return velocity_body_m_s.norm();
}

double angle_of_attack(const Eigen::Vector3d& velocity_body_m_s) noexcept {
  const double u = velocity_body_m_s.x();
  const double w = velocity_body_m_s.z();
  if (u == 0.0 && w == 0.0) {
    return 0.0;
  }
  return std::atan2(w, u);
}

double sideslip_angle(const Eigen::Vector3d& velocity_body_m_s) noexcept {
  const double speed = velocity_body_m_s.norm();
  if (speed == 0.0) {
    return 0.0;
  }
  // Clamped because v/V can exceed 1 by an ulp when the velocity is almost
  // purely lateral, and asin of 1+eps is NaN.
  const double ratio = velocity_body_m_s.y() / speed;
  return std::asin(std::fmin(1.0, std::fmax(-1.0, ratio)));
}

WindAxisVelocity to_wind_axes(const Eigen::Vector3d& velocity_body_m_s) noexcept {
  WindAxisVelocity wind;
  wind.airspeed_m_s = velocity_body_m_s.norm();
  wind.alpha_rad = angle_of_attack(velocity_body_m_s);
  wind.beta_rad = sideslip_angle(velocity_body_m_s);
  return wind;
}

Eigen::Vector3d from_wind_axes(const WindAxisVelocity& wind) noexcept {
  const double cos_alpha = std::cos(wind.alpha_rad);
  const double sin_alpha = std::sin(wind.alpha_rad);
  const double cos_beta = std::cos(wind.beta_rad);
  const double sin_beta = std::sin(wind.beta_rad);
  return Eigen::Vector3d(wind.airspeed_m_s * cos_alpha * cos_beta,
                         wind.airspeed_m_s * sin_beta,
                         wind.airspeed_m_s * sin_alpha * cos_beta);
}

Eigen::Vector3d velocity_ned(const State& state) noexcept {
  return dcm_ned_from_body(state.attitude_body_to_ned) * state.velocity_body_m_s;
}

double flight_path_angle(const State& state) noexcept {
  const Eigen::Vector3d v_ned = velocity_ned(state);
  const double speed = v_ned.norm();
  if (speed == 0.0) {
    return 0.0;
  }
  // z is DOWN, so a climb is negative p_d-dot and gamma is positive.
  // gamma = asin(-v_down / V).
  const double ratio = -v_ned.z() / speed;
  return std::asin(std::fmin(1.0, std::fmax(-1.0, ratio)));
}

}  // namespace galata::core
