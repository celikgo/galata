// SPDX-License-Identifier: Apache-2.0
//
// Nonlinear six-degree-of-freedom rigid-body dynamics.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation: Dynamics, Controls Design, and Autonomous Systems", 3rd ed.,
//   Wiley, 2016, chapter 2 — the flat-Earth six-degree-of-freedom equations.
//   B. Etkin and L. D. Reid, "Dynamics of Flight: Stability and Control",
//   3rd ed., Wiley, 1996, chapter 4.
//   H. Goldstein, C. P. Poole and J. L. Safko, "Classical Mechanics", 3rd ed.,
//   Addison Wesley, 2002 — Euler's equations and torque-free motion.
//
// Conventions are ADR-0002. The equations are written about the centre of
// gravity, and the reasoning is ADR-0006.
//
//   m (u_dot + q w - r v) = X
//   m (v_dot + r u - p w) = Y
//   m (w_dot + p v - q u) = Z
//   [I] {p_dot q_dot r_dot}^T + {p q r}^T x ([I] {p q r}^T) = {L M N}^T
//
// The inertia tensor is general. I_xz is NOT assumed zero: it is not zero for
// any real aircraft, and assuming it away is the difference between a correct
// Dutch roll and a plausible one.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not round-Earth. The navigation frame is treated as inertial: no Earth
//   rotation, no transport rate, no Coriolis or centrifugal terms. Earth's
//   rotation is 7.292e-5 rad/s, so the omitted Coriolis acceleration is about
//   2*Omega*V — roughly 0.03 m/s^2 at 200 m/s, or 0.3% of g. Over a 30-second
//   manoeuvre that is a few metres of cross-track position and is invisible in
//   a stability analysis. Over a transoceanic flight or a ballistic trajectory
//   it is not, and this is the wrong model for those.
//
// * Not flexible. The airframe is rigid: no structural modes, no aeroelastic
//   coupling, no fuel slosh. For a large transport the first symmetric bending
//   mode can sit close enough to the short period to matter for a control law
//   with high bandwidth, and this model will not show that interaction at all.
//
// * Not variable-mass in the momentum sense. Mass properties may be supplied
//   per call and may change between calls, but the equations do not carry the
//   momentum flux of the departing mass, nor the acceleration of the CG
//   relative to the airframe. For fuel burn on flight-dynamics timescales both
//   are negligible; for a rocket neither is. See ADR-0006.
//
// * Not a model of anything aerodynamic. This takes a wrench and produces a
//   state derivative. Where the wrench comes from is the aircraft model's
//   problem.

#ifndef GALATA_SIM_RIGID_BODY_HPP
#define GALATA_SIM_RIGID_BODY_HPP

#include "galata/core/state.hpp"

#include <Eigen/Core>

namespace galata::sim {

struct MassProperties {
  double mass_kg = 0.0;  // kg

  // Inertia tensor about the CENTRE OF GRAVITY, in body axes, kg m^2.
  //
  // Full tensor, including products of inertia. In the aerospace sign
  // convention the off-diagonal entries of this matrix are the NEGATIVE
  // products of inertia: the matrix is
  //
  //     [  Ixx  -Ixy  -Ixz ]
  //     [ -Ixy   Iyy  -Iyz ]
  //     [ -Ixz  -Iyz   Izz ]
  //
  // so a source quoting "I_xz = 1200 kg m^2" contributes -1200 here. This
  // catches people, and it is the sign that decides whether roll-yaw coupling
  // has the right handedness.
  Eigen::Matrix3d inertia_cg_body_kg_m2 = Eigen::Matrix3d::Zero();

  // Throws std::invalid_argument unless the mass is positive and the inertia
  // tensor is symmetric and positive definite. Checked rather than assumed
  // because an asymmetric or indefinite tensor produces a solve that succeeds
  // and returns nonsense.
  void validate() const;
};

// Forces and moments in body axes. The moment is about the CG (ADR-0006).
struct Wrench {
  Eigen::Vector3d force_body_n = Eigen::Vector3d::Zero();        // N
  Eigen::Vector3d moment_cg_body_n_m = Eigen::Vector3d::Zero();  // N m, about the CG

  Wrench& operator+=(const Wrench& other) noexcept;
};

[[nodiscard]] Wrench operator+(Wrench left, const Wrench& right) noexcept;

// Move a wrench's reference point to the CG.
//
//   M_cg = M_ref + r_ref_to_cg_reversed x F,  with the offset taken as the
//   vector FROM the CG TO the application point.
//
// The argument is named for its direction because getting it backwards flips
// the sign of every moment contribution, and a sign-flipped pitching moment
// still trims — at the wrong elevator, with the wrong static margin.
[[nodiscard]] Wrench moved_to_cg(const Wrench& wrench_about_point,
                                 const Eigen::Vector3d& cg_to_point_body_m) noexcept;

// Gravity in body axes: R_body<-ned * gravity_ned.
[[nodiscard]] Eigen::Vector3d gravity_body(const core::Quaternion& attitude_body_to_ned,
                                           const Eigen::Vector3d& gravity_ned_m_s2) noexcept;

// dx/dt for the thirteen-component state, in the state vector's order.
//
// `applied` excludes gravity, which is passed separately because it needs the
// attitude to be resolved into body axes and because keeping it separate makes
// a zero-gravity torque-free validation case a one-argument change rather than
// a special case in the caller.
//
// The quaternion derivative is the exact kinematic one; the integrator is
// responsible for renormalising after each completed step.
[[nodiscard]] core::StateVector rigid_body_derivative(const core::State& state,
                                                      const MassProperties& mass,
                                                      const Wrench& applied,
                                                      const Eigen::Vector3d& gravity_ned_m_s2);

// Rotational kinetic energy, 0.5 * omega^T I omega, in joules.
//
// Conserved exactly under torque-free motion. Its drift under numerical
// integration is a direct measure of integrator error and is what the
// validation suite reports.
[[nodiscard]] double rotational_kinetic_energy(const core::State& state,
                                               const MassProperties& mass) noexcept;

// Angular momentum about the CG, in body axes, kg m^2/s.
//
// The VECTOR is constant in the NED frame under torque-free motion, not in the
// body frame — in body axes it rotates. Its MAGNITUDE is constant in both.
[[nodiscard]] Eigen::Vector3d angular_momentum_body(const core::State& state,
                                                    const MassProperties& mass) noexcept;

// The same vector resolved in NED, which is the frame it is constant in.
[[nodiscard]] Eigen::Vector3d angular_momentum_ned(const core::State& state,
                                                   const MassProperties& mass) noexcept;

}  // namespace galata::sim

#endif  // GALATA_SIM_RIGID_BODY_HPP
