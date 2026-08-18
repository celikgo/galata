// SPDX-License-Identifier: Apache-2.0
//
// The rigid-body state vector and the quantities derived from it.
//
// Ordering, normative, from ADR-0002:
//
//   x = [ p_n p_e p_d   position,     NED,          m
//         u   v   w     velocity,     body axes,    m/s
//         q_w q_x q_y q_z  attitude,  body -> NED,  dimensionless
//         p   q   r ]   angular rate, body axes,    rad/s
//
// Thirteen components, in that order. This is the row and column order of every
// A and B matrix galata produces, so it is a compatibility surface: changing it
// changes the meaning of every exported state-space model.
//
// The velocity is AIR-RELATIVE — the aircraft's velocity with respect to the
// local air mass, not with respect to the ground. In still air the two are
// equal and the distinction is invisible; in wind or turbulence they are not,
// and the aerodynamic forces depend on this one while the position derivative
// depends on the other. Whoever integrates position is responsible for adding
// the wind field back in.
//
// What this is NOT: this is a rigid-body state. It carries no structural modes,
// no fuel slosh, no rotor or propeller dynamics, and no engine state. A model
// needing those carries them alongside, not inside.

#ifndef GALATA_CORE_STATE_HPP
#define GALATA_CORE_STATE_HPP

#include "galata/core/quaternion.hpp"

#include <Eigen/Core>

namespace galata::core {

// Number of components in the packed state vector.
inline constexpr int kStateSize = 13;

// Named indices into the packed vector. Using these rather than literals is
// what makes a reordering a compile error somewhere rather than a silent
// permutation of an exported A matrix.
enum StateIndex : int {
  kPositionNorth = 0,  // m
  kPositionEast = 1,   // m
  kPositionDown = 2,   // m
  kVelocityU = 3,      // m/s, body x
  kVelocityV = 4,      // m/s, body y
  kVelocityW = 5,      // m/s, body z
  kQuaternionW = 6,    // dimensionless
  kQuaternionX = 7,    // dimensionless
  kQuaternionY = 8,    // dimensionless
  kQuaternionZ = 9,    // dimensionless
  kRateP = 10,         // rad/s, body x (roll rate)
  kRateQ = 11,         // rad/s, body y (pitch rate)
  kRateR = 12,         // rad/s, body z (yaw rate)
};

using StateVector = Eigen::Matrix<double, kStateSize, 1>;

struct State {
  Eigen::Vector3d position_ned_m = Eigen::Vector3d::Zero();           // m
  Eigen::Vector3d velocity_body_m_s = Eigen::Vector3d::Zero();        // m/s, air-relative
  Quaternion attitude_body_to_ned = Quaternion(1.0, 0.0, 0.0, 0.0);   // dimensionless
  Eigen::Vector3d angular_rate_body_rad_s = Eigen::Vector3d::Zero();  // rad/s

  [[nodiscard]] StateVector to_vector() const noexcept;
  [[nodiscard]] static State from_vector(const StateVector& x) noexcept;

  // q <- q / ||q||. Applied every integrator step (ADR-0004).
  void renormalise_attitude() noexcept;
};

// Air-relative velocity expressed in wind axes. The natural coordinates for
// stating a trim condition: "trim at Mach 0.6, wings level" is a statement
// about V, alpha and beta, not about u, v and w.
struct WindAxisVelocity {
  double airspeed_m_s = 0.0;  // V, m/s, always >= 0
  double alpha_rad = 0.0;     // angle of attack, rad
  double beta_rad = 0.0;      // sideslip angle, rad
};

// V = ||(u, v, w)||.
[[nodiscard]] double airspeed(const Eigen::Vector3d& velocity_body_m_s) noexcept;

// alpha = atan2(w, u). Defined for u <= 0 as well; atan2 rather than atan(w/u)
// so that rearward flight is represented rather than folded into the forward
// half-plane.
//
// Undefined when u and w are both exactly zero — a purely sideways velocity has
// no angle of attack. Returns 0 there, which is a convention, not an answer.
[[nodiscard]] double angle_of_attack(const Eigen::Vector3d& velocity_body_m_s) noexcept;

// beta = asin(v / V).
//
// Undefined at V = 0. Returns 0 there, which is again a convention: at zero
// airspeed there is no sideslip angle, and every aerodynamic force built on it
// is zero regardless.
[[nodiscard]] double sideslip_angle(const Eigen::Vector3d& velocity_body_m_s) noexcept;

// All three at once, avoiding the repeated norm.
[[nodiscard]] WindAxisVelocity to_wind_axes(const Eigen::Vector3d& velocity_body_m_s) noexcept;

// (u, v, w) = V (cos a cos b, sin b, sin a cos b). Exact inverse of
// to_wind_axes for V > 0; the round trip in both directions is tested.
[[nodiscard]] Eigen::Vector3d from_wind_axes(const WindAxisVelocity& wind) noexcept;

// Flight-path angle gamma, the inclination of the velocity vector above the
// horizon, in radians. Derived from the NED velocity, so it accounts for
// attitude and aerodynamic angles together rather than approximating
// gamma = theta - alpha, which holds only in wings-level symmetric flight.
[[nodiscard]] double flight_path_angle(const State& state) noexcept;

// Velocity in NED axes: R_ned<-body * (u, v, w). Air-relative, as above.
[[nodiscard]] Eigen::Vector3d velocity_ned(const State& state) noexcept;

}  // namespace galata::core

#endif  // GALATA_CORE_STATE_HPP
