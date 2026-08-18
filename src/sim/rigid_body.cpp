// SPDX-License-Identifier: Apache-2.0
//
// Nonlinear six-degree-of-freedom rigid-body dynamics, flat non-rotating Earth.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 2.
//   H. Goldstein, C. P. Poole and J. L. Safko, "Classical Mechanics", 3rd ed.,
//   Addison Wesley, 2002 — Euler's equations.
//
// Conventions: ADR-0002. Reference point: ADR-0006. Validity envelope and the
// direction and magnitude of the omitted terms: the header's
// "WHAT THIS IS NOT" block.

#include "galata/sim/rigid_body.hpp"

#include "galata/core/quaternion.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace galata::sim {

void MassProperties::validate() const {
  if (!(mass_kg > 0.0)) {
    std::ostringstream message;
    message << "MassProperties: mass is " << mass_kg << " kg, must be positive and finite";
    throw std::invalid_argument(message.str());
  }

  const Eigen::Matrix3d& inertia = inertia_cg_body_kg_m2;

  // Symmetry first: an asymmetric tensor is almost always a transcription slip
  // (a product of inertia entered in one place and not its mirror), and every
  // check after this one assumes symmetry.
  const double asymmetry = (inertia - inertia.transpose()).cwiseAbs().maxCoeff();
  const double scale = inertia.cwiseAbs().maxCoeff();
  if (asymmetry > 1e-9 * (scale + 1.0)) {
    std::ostringstream message;
    message << "MassProperties: inertia tensor is not symmetric (worst asymmetry " << asymmetry
            << " kg m^2). A product of inertia was probably entered on one side only.";
    throw std::invalid_argument(message.str());
  }

  // Positive definiteness. An indefinite tensor is not a physical rigid body,
  // and the solve below would succeed on one anyway and return nonsense.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(inertia);
  if (solver.info() != Eigen::Success) {
    throw std::invalid_argument("MassProperties: inertia tensor eigendecomposition failed");
  }
  const double smallest = solver.eigenvalues().minCoeff();
  if (!(smallest > 0.0)) {
    std::ostringstream message;
    message << "MassProperties: inertia tensor is not positive definite (smallest principal "
               "moment "
            << smallest << " kg m^2)";
    throw std::invalid_argument(message.str());
  }

  // The triangle inequality on principal moments. Every rigid body satisfies
  // I_a + I_b >= I_c for each pairing; a set that violates it cannot be
  // realised by any distribution of mass, however the numbers were obtained.
  // Worth checking because it catches the common case of a moment quoted about
  // the wrong axis, which passes both tests above.
  const Eigen::Vector3d principal = solver.eigenvalues();
  const double tolerance = 1e-9 * (principal.maxCoeff() + 1.0);
  const bool triangle_holds = (principal(0) + principal(1) >= principal(2) - tolerance)
                              && (principal(0) + principal(2) >= principal(1) - tolerance)
                              && (principal(1) + principal(2) >= principal(0) - tolerance);
  if (!triangle_holds) {
    std::ostringstream message;
    message << "MassProperties: principal moments of inertia (" << principal(0) << ", "
            << principal(1) << ", " << principal(2)
            << " kg m^2) violate the triangle inequality, so no mass distribution produces "
               "them. A moment is probably quoted about the wrong axis.";
    throw std::invalid_argument(message.str());
  }
}

Wrench& Wrench::operator+=(const Wrench& other) noexcept {
  force_body_n += other.force_body_n;
  moment_cg_body_n_m += other.moment_cg_body_n_m;
  return *this;
}

Wrench operator+(Wrench left, const Wrench& right) noexcept {
  left += right;
  return left;
}

Wrench moved_to_cg(const Wrench& wrench_about_point,
                   const Eigen::Vector3d& cg_to_point_body_m) noexcept {
  Wrench moved;
  moved.force_body_n = wrench_about_point.force_body_n;
  // The force acts at the point; its moment about the CG is r x F with r
  // pointing from the CG to the point.
  moved.moment_cg_body_n_m = wrench_about_point.moment_cg_body_n_m
                             + cg_to_point_body_m.cross(wrench_about_point.force_body_n);
  return moved;
}

Eigen::Vector3d gravity_body(const core::Quaternion& attitude_body_to_ned,
                             const Eigen::Vector3d& gravity_ned_m_s2) noexcept {
  return core::dcm_body_from_ned(attitude_body_to_ned) * gravity_ned_m_s2;
}

core::StateVector rigid_body_derivative(const core::State& state,
                                        const MassProperties& mass,
                                        const Wrench& applied,
                                        const Eigen::Vector3d& gravity_ned_m_s2) {
  const Eigen::Vector3d& velocity = state.velocity_body_m_s;
  const Eigen::Vector3d& omega = state.angular_rate_body_rad_s;

  // Position: the NED velocity is the body velocity rotated out.
  const Eigen::Vector3d position_rate =
      core::dcm_ned_from_body(state.attitude_body_to_ned) * velocity;

  // Translation. The -omega x v term is the transport term that appears because
  // the velocity components are expressed in a rotating frame; it is the
  // (q w - r v) group in the scalar form.
  const Eigen::Vector3d specific_force = applied.force_body_n / mass.mass_kg;
  const Eigen::Vector3d velocity_rate = specific_force
                                        + gravity_body(state.attitude_body_to_ned, gravity_ned_m_s2)
                                        - omega.cross(velocity);

  // Attitude kinematics.
  const Eigen::Vector4d quaternion_rate =
      core::quaternion_derivative(state.attitude_body_to_ned, omega);

  // Rotation, Euler's equations with a general inertia tensor.
  //
  // Solved rather than inverted. For a well-conditioned 3x3 the difference is
  // slight, but an aircraft with a large product of inertia relative to its
  // moments can be poorly conditioned, and an explicit inverse loses accuracy
  // exactly there.
  //
  // LLT rather than LDLT. Cholesky without pivoting is the right factorisation
  // for a symmetric POSITIVE DEFINITE system, and validate() has established
  // that the inertia tensor is exactly that — an inertia tensor is never
  // indefinite or semi-definite, which is the case LDLT's pivoting exists to
  // handle. LLT is also the cheaper of the two, and it runs on every RK4 stage
  // of every step.
  const Eigen::Vector3d angular_momentum = mass.inertia_cg_body_kg_m2 * omega;
  const Eigen::Vector3d net_moment = applied.moment_cg_body_n_m - omega.cross(angular_momentum);
  const Eigen::Vector3d angular_rate_rate = mass.inertia_cg_body_kg_m2.llt().solve(net_moment);

  core::StateVector derivative;
  derivative(core::kPositionNorth) = position_rate.x();
  derivative(core::kPositionEast) = position_rate.y();
  derivative(core::kPositionDown) = position_rate.z();
  derivative(core::kVelocityU) = velocity_rate.x();
  derivative(core::kVelocityV) = velocity_rate.y();
  derivative(core::kVelocityW) = velocity_rate.z();
  derivative(core::kQuaternionW) = quaternion_rate(0);
  derivative(core::kQuaternionX) = quaternion_rate(1);
  derivative(core::kQuaternionY) = quaternion_rate(2);
  derivative(core::kQuaternionZ) = quaternion_rate(3);
  derivative(core::kRateP) = angular_rate_rate.x();
  derivative(core::kRateQ) = angular_rate_rate.y();
  derivative(core::kRateR) = angular_rate_rate.z();
  return derivative;
}

double rotational_kinetic_energy(const core::State& state, const MassProperties& mass) noexcept {
  const Eigen::Vector3d& omega = state.angular_rate_body_rad_s;
  return 0.5 * omega.dot(mass.inertia_cg_body_kg_m2 * omega);
}

Eigen::Vector3d angular_momentum_body(const core::State& state,
                                      const MassProperties& mass) noexcept {
  return mass.inertia_cg_body_kg_m2 * state.angular_rate_body_rad_s;
}

Eigen::Vector3d angular_momentum_ned(const core::State& state,
                                     const MassProperties& mass) noexcept {
  return core::dcm_ned_from_body(state.attitude_body_to_ned) * angular_momentum_body(state, mass);
}

}  // namespace galata::sim
