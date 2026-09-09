// SPDX-License-Identifier: Apache-2.0
//
// Equilibrium for a multirotor: still-air hover, hover in a crosswind, and
// cruise as a relative equilibrium with a nonzero position rate.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 3 — trim as the solution of the
//   steady-state equations, which is the formulation used here.
//   R. Mahony, V. Kumar and P. Corke, "Multirotor Aerial Vehicles: Modeling,
//   Estimation, and Control of Quadrotor", IEEE Robotics & Automation
//   Magazine, vol. 19, no. 3, 2012 — the hover condition and the tilt a
//   multirotor needs to hold an airspeed.
//
// WHY THIS EXISTS SEPARATELY FROM trim_level. `galata/trim/level.hpp` solves
// angle of attack, elevator and thrust at POSITIVE airspeed, and it cannot
// express hover: its unknowns do not exist for a vehicle with no elevator, and
// its equations divide by an airspeed that is zero. This is the same idea — set
// the dynamic accelerations to zero and solve — over a different unknown set.
//
// WHAT IS SOLVED, AND WHAT IS DECLARED.
//
//   unknowns  = [ roll, pitch, omega_0 .. omega_{n-1} ]
//   equations = [ vdot (3), omegadot (3) ]
//
// Yaw is DECLARED, not solved. A multirotor in still air is in equilibrium at
// any heading, so heading is an input to the problem and not an output of it;
// solving for it would be solving a singular system and reporting whichever
// answer the line search wandered into.
//
// The position rate is NOT required to be zero. That is what makes cruise a
// relative equilibrium: the vehicle translates at a constant ground velocity
// while every DYNAMIC state — body velocity, attitude, body rate, rotor speed —
// is stationary. Requiring the position rate to vanish would admit only hover.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not a solver for an over-actuated vehicle. Six equations need six
//   unknowns, which for two attitude angles means exactly four rotors. A
//   hexarotor is not more of the same problem: it is an ALLOCATION problem
//   with a two-dimensional null space, and a Newton solve on it returns
//   whichever point the iteration happened to reach. This refuses instead.
//
// * Not a battery equilibrium. State of charge is FROZEN at a declared value
//   and its derivative is excluded from the residual. A powered battery has no
//   zero-energy-derivative equilibrium — it is always discharging — so there is
//   no trim to find, and pretending otherwise would make every trim infeasible
//   for a reason that has nothing to do with flight.
//
// * Not constrained. Newton finds the equilibrium and only then are the rotor
//   speed limits checked. A trim needing more rotor speed than the vehicle has
//   is REFUSED rather than returned with a note, which is the same choice
//   trim_level makes and for the same reason: a best effort that is reported as
//   a trim gets linearised, and a linearisation about a non-equilibrium carries
//   a constant term the A matrix cannot represent.
//
// * Not a claim that the equilibrium is stable, reachable, or unique. It is a
//   root of the steady-state equations. Whether the vehicle can get there, and
//   what happens when it is disturbed, is what the linearisation is for.

#ifndef GALATA_TRIM_HOVER_HPP
#define GALATA_TRIM_HOVER_HPP

#include "galata/core/state.hpp"
#include "galata/model/quadrotor.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::trim {

struct HoverTrimRequest {
  // Sets the NED down component of the returned position to its negative, and
  // nothing else. This plant's gravity does not vary with height and it carries
  // no atmosphere, so no residual and no Jacobian entry depends on it — but the
  // trim is a state that downstream code reads, and an altitude observation
  // reads exactly this component. A plant that later grows an atmosphere finds
  // the field already here and already in the state.
  double altitude_m = 0.0;  // m

  // Wind velocity in NED — the velocity of the air mass, so the air-relative
  // velocity is the ground velocity minus this.
  Eigen::Vector3d wind_ned_m_s = Eigen::Vector3d::Zero();  // m/s

  // Ground velocity to hold. Zero is hover; nonzero is cruise as a relative
  // equilibrium.
  Eigen::Vector3d ground_velocity_ned_m_s = Eigen::Vector3d::Zero();  // m/s

  // Declared, not solved. See the header.
  double heading_rad = 0.0;  // rad, positive nose to the right of north

  // Frozen for the whole solve, and excluded from the residual.
  double battery_state_of_charge = 1.0;  // dimensionless, 0 to 1

  // Acceleration-norm budget: m/s^2 on the three force equations and rad/s^2 on
  // the three moment equations.
  double residual_tolerance = 1e-10;

  // Fixed, not a maximum. ADR-0004 forbids a tolerance-based early exit.
  int iterations = 40;
};

struct HoverTrim {
  // The equilibrium, in the model's extended-state layout. North and east are
  // zero — arbitrary at this equilibrium — and down is minus the declared
  // altitude.
  Eigen::VectorXd extended_state;

  // As declared, repeated here so a caller reading the answer alone does not
  // have to negate a position component to recover it.
  double altitude_m = 0.0;  // m

  // Steady rotor commands. At equilibrium these equal the rotor speeds, and
  // both are reported because a caller that confuses them gets a plausible
  // trajectory that starts with a transient.
  Eigen::VectorXd command_rad_s;  // rad/s

  double roll_rad = 0.0;   // rad
  double pitch_rad = 0.0;  // rad
  double yaw_rad = 0.0;    // rad, as declared

  // As declared, carried with the answer so a linearisation about this point
  // uses the SAME wind the trim was solved at. A linearisation that reads its
  // nominal wind from its own stage input can be handed a different one, and
  // would then take its wind columns about a point that is not this trim.
  Eigen::Vector3d wind_ned_m_s = Eigen::Vector3d::Zero();             // m/s
  Eigen::Vector3d ground_velocity_ned_m_s = Eigen::Vector3d::Zero();  // m/s

  Eigen::Vector3d air_relative_velocity_body_m_s = Eigen::Vector3d::Zero();  // m/s
  double airspeed_m_s = 0.0;                                                 // m/s
  double battery_state_of_charge = 1.0;

  // TRUE when the model carries a battery, and therefore when the state of
  // charge above is a real extended-state coordinate that was HELD at its
  // declared value rather than solved for. The trim residual excludes that row
  // — a powered pack has no zero-energy-derivative equilibrium — so this point
  // is an equilibrium in the six dynamic coordinates and is NOT one in the
  // battery coordinate. A reader who does not know the row was excluded would
  // read this as an equilibrium in a coordinate the solver never balanced,
  // which is why the freeze is recorded here and exported rather than left to
  // be inferred from the presence of a battery block in the model file.
  bool battery_state_of_charge_frozen = false;

  // Charter rule 9: the evidence travels with the number.
  double residual_norm = 0.0;
  double residual_tolerance = 1e-10;
  int newton_iterations = 0;
  double jacobian_condition_number = 0.0;
  std::vector<double> residual_history;

  // Per rotor, (ceiling - omega) / ceiling. Negative would mean the trim needs
  // more than the rotor has, which is refused before this is returned, so every
  // entry here is non-negative by construction.
  std::vector<double> rotor_margin_fraction;

  // The smallest of the above, which is the one that decides feasibility.
  double smallest_rotor_margin_fraction = 0.0;
};

// Solves the equilibrium.
//
// Throws std::invalid_argument when the problem is not square — see the
// header's first exclusion — and std::runtime_error when no trim was found or
// when the trim found needs a rotor speed the vehicle does not have. The
// message carries the residual, the condition number and, for an infeasible
// trim, which rotor exceeded its ceiling and by how much.
[[nodiscard]] HoverTrim trim_hover(const model::Quadrotor& model, const HoverTrimRequest& request);

}  // namespace galata::trim

#endif  // GALATA_TRIM_HOVER_HPP
