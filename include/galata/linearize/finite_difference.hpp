// SPDX-License-Identifier: Apache-2.0
//
// Linearisation of a nonlinear aircraft model about a trim point, by central
// differences.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 3 — numerical linearisation
//   about a trim point.
//   B. Etkin and L. D. Reid, "Dynamics of Flight: Stability and Control",
//   3rd ed., Wiley, 1996, chapters 4 and 5.
//
// THE COORDINATES ARE EULER ANGLES, NOT THE QUATERNION, and this is the one
// design decision in the file that matters.
//
// ADR-0002 fixes the integrated state as a quaternion, precisely so the
// dynamics carry no singularity. But a quaternion cannot be the coordinate
// system for a LINEARISATION: it has four components and three degrees of
// freedom, so perturbing all four explores a direction — growing the norm —
// that the dynamics do not have. The resulting 13-by-13 Jacobian is singular
// by construction, its eigenvalues include a spurious mode, and every
// participation factor is polluted.
//
// So the perturbation is done in twelve coordinates, with the attitude as
// Euler angles:
//
//   x = [ p_n p_e p_d   u v w   phi theta psi   p q r ]
//
// The model is still integrated as a quaternion; the conversion happens inside
// the difference quotient. Euler angles remain derived output, exactly as
// ADR-0002 requires — they are the chart this linearisation is written in, not
// the state the simulation carries.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not valid at a trim near 90 degrees of pitch. The Euler chart is singular
//   there, and so is this. `chart_conditioning` reports |cos(theta)| so the
//   caller can see it coming; below about 0.1 the result should not be
//   believed. A vertical-climb trim needs a different chart, not a smaller
//   step.
//
// * Not exact, and the error is reported rather than assumed. Every entry
//   carries a Richardson truncation estimate. What that estimate CANNOT see is
//   a discontinuity inside the perturbation window — a rate limit, a table
//   breakpoint, a saturation — where the difference quotient returns an
//   average slope the model never has and the estimate looks perfectly
//   healthy.
//
// * Only as good as the trim. Linearising about a point that is not an
//   equilibrium gives a state-space model with a spurious constant term that
//   the A matrix cannot represent, so the model is simply wrong. This is why
//   trim_level throws rather than returning a best effort.
//
// * C is the identity over the retained states and D is zero. Output
//   selection — load factor, flight-path angle, a sensor at a station — is not
//   implemented, and a caller wanting those must form them itself.

#ifndef GALATA_LINEARIZE_FINITE_DIFFERENCE_HPP
#define GALATA_LINEARIZE_FINITE_DIFFERENCE_HPP

#include "galata/model/aircraft.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/numerics/jacobian.hpp"
#include "galata/trim/level.hpp"

#include <string>
#include <vector>

namespace galata::linearize {

// The twelve linearisation coordinates, in order.
enum EulerStateIndex : int {
  kPositionNorth = 0,
  kPositionEast = 1,
  kPositionDown = 2,
  kVelocityU = 3,
  kVelocityV = 4,
  kVelocityW = 5,
  kRoll = 6,   // phi
  kPitch = 7,  // theta
  kYaw = 8,    // psi
  kRateP = 9,
  kRateQ = 10,
  kRateR = 11,
};

inline constexpr int kEulerStateSize = 12;

// Names matching the indices above. These are what the modal classifier reads,
// so they are part of the contract rather than a display detail.
[[nodiscard]] std::vector<std::string> euler_state_names();
[[nodiscard]] std::vector<std::string> control_names();

// Conventional reduced sets. Selecting a subset is exact only when the
// coupling into it is zero, which for a symmetric aircraft at a wings-level
// trim it is — and `neglected_coupling` measures rather than assumes that.
[[nodiscard]] std::vector<int> longitudinal_states();  // u, w, q, theta
[[nodiscard]] std::vector<int> lateral_states();       // v, p, r, phi

struct LinearisationOptions {
  // Perturbation sizes. The defaults use a relative step with per-component
  // absolute floors, because an aircraft state mixes metres, metres per second
  // and radians and one floor cannot serve all three.
  numerics::JacobianOptions state_jacobian;
  numerics::JacobianOptions control_jacobian;

  // Empty keeps all twelve states. Otherwise, indices from EulerStateIndex.
  std::vector<int> state_subset;

  bool report_truncation_error = true;
};

struct Linearisation {
  Eigen::MatrixXd a;
  Eigen::MatrixXd b;
  Eigen::MatrixXd c;  // identity over the retained states
  Eigen::MatrixXd d;  // zero

  std::vector<std::string> state_names;
  std::vector<std::string> input_names;

  // The perturbation actually used per state and per control. Reported because
  // the step is a choice and a reader checking a suspicious entry needs to
  // know the window it was measured over.
  Eigen::VectorXd state_steps;
  Eigen::VectorXd control_steps;

  // Richardson estimate of the truncation error, entry by entry.
  Eigen::MatrixXd a_truncation;
  Eigen::MatrixXd b_truncation;
  double worst_relative_truncation = 0.0;

  // |cos(theta)| at the trim. The Euler chart degrades as this approaches zero.
  double chart_conditioning = 1.0;

  // When a subset was taken, the largest entry of the full Jacobian that
  // couples a retained state to a discarded one, relative to the largest
  // retained entry.
  //
  // Zero means the reduction is exact. It is reported rather than assumed
  // because "the lateral and longitudinal axes decouple" is true at a
  // wings-level symmetric trim and false in a turn, and the difference is not
  // visible in the reduced matrix.
  double neglected_coupling = 0.0;

  // The trim this was taken about, carried along so a result cannot be
  // separated from the condition it describes (charter rule 9).
  double trim_altitude_m = 0.0;
  double trim_airspeed_m_s = 0.0;
  double trim_alpha_rad = 0.0;
  double trim_residual_norm = 0.0;

  // Packaged for the analysis layer.
  [[nodiscard]] model::LinearSystem to_linear_system(const std::string& description,
                                                     const std::string& citation) const;
};

[[nodiscard]] Linearisation linearize_finite_difference(const model::Aircraft& aircraft,
                                                        const trim::TrimPoint& trim,
                                                        const LinearisationOptions& options = {});

}  // namespace galata::linearize

#endif  // GALATA_LINEARIZE_FINITE_DIFFERENCE_HPP
