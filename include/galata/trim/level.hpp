// SPDX-License-Identifier: Apache-2.0
//
// Straight-line trim: wings level, no sideslip, constant speed, at a given
// flight-path angle.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 3 — trim as a constrained
//   algebraic problem and the choice of unknowns.
//   B. Etkin and L. D. Reid, "Dynamics of Flight: Stability and Control",
//   3rd ed., Wiley, 1996, chapter 2.
//
// THE PROBLEM. Three unknowns — angle of attack, elevator, thrust — and three
// equations: the two body-axis translational accelerations and the pitching
// acceleration must all vanish. The lateral equations are satisfied identically
// by the symmetry of the condition (no sideslip, wings level, no lateral
// control), so they are not part of the system; including them would make it
// rectangular and Newton would not apply.
//
// The kinematic constraint is theta = alpha + gamma: wings level with no
// sideslip, the velocity vector lies in the plane of symmetry at gamma to the
// horizon, and alpha is the angle between the body x-axis and that vector.
//
// FAILURE IS LOUD. If the residual is above tolerance this throws. It does not
// return a best-effort answer with a small number attached, because that number
// gets dropped somewhere downstream and a linearisation about a point that is
// not a trim produces a state-space model that is plausible and wrong.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * No control limits. A trim needing 40 degrees of elevator is found and
//   returned; whether the elevator stops at 25 is not modelled here, and the
//   returned deflections must be checked against the real ones.
//
// * No thrust limits, and no engine model. The solve treats thrust as a free
//   variable in newtons. A trim requiring more thrust than the engine has is
//   reported as a perfectly good trim.
//
// * Not a turn, a pull-up or a steady sideslip. Those are different constraint
//   sets with different unknowns, and each is its own capability.
//
// * The trim is INSTANTANEOUS. At a non-zero flight-path angle the altitude is
//   changing, so the atmosphere is changing, so the condition is not actually
//   steady. What is solved is the equilibrium at the stated altitude; over a
//   long climb it drifts. This is the standard idealisation and it is stated
//   rather than assumed.

#ifndef GALATA_TRIM_LEVEL_HPP
#define GALATA_TRIM_LEVEL_HPP

#include "galata/core/atmosphere.hpp"
#include "galata/core/state.hpp"
#include "galata/model/aircraft.hpp"

#include <string>
#include <vector>

namespace galata::trim {

struct LevelTrimRequest {
  double altitude_m = 0.0;  // m, geometric

  // Exactly one of these must be set. Mach is resolved against the speed of
  // sound at the requested altitude.
  double airspeed_m_s = 0.0;
  double mach = 0.0;

  // Flight-path angle. Zero is level flight; positive is a climb.
  double flight_path_angle_rad = 0.0;  // rad

  double delta_isa_k = 0.0;  // K

  // Initial guess. Left at their defaults, the angle of attack starts at the
  // model's reference condition and the thrust at an estimate of the drag,
  // which is close enough for Newton on every conventional aircraft.
  bool use_default_guess = true;
  double initial_alpha_rad = 0.0;
  double initial_elevator_rad = 0.0;
  double initial_thrust_n = 0.0;

  // The residual norm below which the answer is a trim. The residual is in
  // units of acceleration: m/s^2 for the two force equations and rad/s^2 for
  // the moment equation, so this is a small acceleration rather than a
  // dimensionless quantity.
  double residual_tolerance = 1e-10;
};

struct TrimPoint {
  core::State state;
  model::Controls controls;
  core::AtmosphereState atmosphere;

  double alpha_rad = 0.0;
  double flight_path_angle_rad = 0.0;
  double pitch_attitude_rad = 0.0;
  double airspeed_m_s = 0.0;
  double mach = 0.0;
  double dynamic_pressure_pa = 0.0;

  // Charter rule 9: a number arrives with the evidence for believing it.
  double residual_norm = 0.0;

  // Condition number of the trim Jacobian.
  //
  // Read it alongside the residual, not on its own. For this problem it is
  // typically around 1e5 and that is a UNITS artefact rather than a physical
  // near-singularity: an angle of order 0.04 rad and a thrust of order 10,000 N
  // sit in the same unknown vector, so the Jacobian's columns differ in scale
  // by five orders before any aircraft is involved.
  //
  // What it is genuinely useful for is the case where it is large AND the
  // residual will not come down. That combination means a control has no
  // authority over any residual at this condition, and no amount of iterating
  // will fix it.
  double jacobian_condition_number = 0.0;
  std::vector<double> residual_history;
  model::EnvelopeWarning envelope;

  // The lift coefficient the trim required. Useful because it is the one trim
  // output that can be checked by hand: C_L = W / (q S) in level flight.
  double lift_coefficient = 0.0;
};

// Solves the trim. Throws std::runtime_error if the residual is above
// tolerance, with the residual, the condition number and the iteration history
// in the message.
[[nodiscard]] TrimPoint trim_level(const model::Aircraft& aircraft,
                                   const LevelTrimRequest& request);

}  // namespace galata::trim

#endif  // GALATA_TRIM_LEVEL_HPP
