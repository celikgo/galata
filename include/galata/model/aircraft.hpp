// SPDX-License-Identifier: Apache-2.0
//
// A nonlinear aircraft model built from a non-dimensional derivative set.
//
// Reference:
//   B. Etkin and L. D. Reid, "Dynamics of Flight: Stability and Control",
//   3rd ed., Wiley, 1996, chapters 3 and 4 — the coefficient buildup, the
//   non-dimensionalisation of the angular rates, and the wind-to-body force
//   transformation.
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 2.
//
// The coefficient buildup, with `da = alpha - alpha_ref`:
//
//   C_L = C_L_ref + C_L_alpha da + C_L_q qhat + C_L_de de
//   C_D = C_D_ref + C_D_alpha da + C_D_de de
//   C_Y = C_Y_beta beta + C_Y_da da_ail + C_Y_dr dr
//   C_l = C_l_beta beta + C_l_p phat + C_l_r rhat + C_l_da da_ail + C_l_dr dr
//   C_m = C_m_ref + C_m_alpha da + C_m_q qhat + C_m_alphadot ahat + C_m_de de
//   C_n = C_n_beta beta + C_n_p phat + C_n_r rhat + C_n_da da_ail + C_n_dr dr
//
// with the rates non-dimensionalised as ADR-0002 and Etkin define them:
// phat = p b / 2V, qhat = q c / 2V, rhat = r b / 2V, ahat = alphadot c / 2V.
//
// Lift and drag are in WIND axes and are rotated into body axes through
// dcm_body_from_wind, so the sign convention is the one ADR-0002 already
// fixes rather than a second one written out here.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * NOT A GLOBAL MODEL. This is a first-order expansion about one reference
//   condition. It is a good model within a few degrees of `reference_alpha`
//   and at speeds near the reference, and it is worthless outside that. It has
//   no stall: lift goes on rising linearly with angle of attack for ever,
//   which is not merely inaccurate but qualitatively wrong. `envelope_warning`
//   reports how far a query has strayed; nothing stops you ignoring it.
//
// * No Mach dependence. Every derivative is fixed at its reference-Mach value.
//   For the subsonic approach conditions this form is used at, that is
//   defensible; through transonic it is not, and there is no compressibility
//   correction here of any kind.
//
// * No propulsion model. Thrust is an input in newtons along a fixed thrust
//   line. There is no engine lag, no throttle-to-thrust map, no altitude or
//   Mach lapse. A trim solves for the thrust it needs and says nothing about
//   whether the engine could produce it.
//
// * No ground effect, no landing gear, no flaps as a variable, no fuel burn.
//   A configuration change means a different derivative set.
//
// * ALPHA-DOT FORCE DERIVATIVES ARE REJECTED, not approximated. A model with
//   C_L_alphadot or C_D_alphadot is implicit — the vertical acceleration would
//   depend on alphadot, which depends on the vertical acceleration — and
//   solving that is not implemented. `validate()` refuses such a model rather
//   than silently dropping the term. C_m_alphadot is supported and is NOT
//   dropped: it is the downwash-lag term, it is small, and dropping it moves
//   the short-period damping by more than the precision of a published
//   reference (a test measures exactly that).

#ifndef GALATA_MODEL_AIRCRAFT_HPP
#define GALATA_MODEL_AIRCRAFT_HPP

#include "galata/core/atmosphere.hpp"
#include "galata/core/state.hpp"
#include "galata/sim/rigid_body.hpp"

#include <string>

namespace galata::model {

struct Geometry {
  double wing_area_m2 = 0.0;              // S,     m^2
  double wing_span_m = 0.0;               // b,     m
  double mean_aerodynamic_chord_m = 0.0;  // c-bar, m
};

// Non-dimensional derivatives, all per radian, about a reference condition.
struct AeroDerivatives {
  // The condition the expansion is about.
  double reference_alpha_rad = 0.0;  // rad
  double reference_mach = 0.0;       // dimensionless

  // Longitudinal, wind axes for the force coefficients.
  double lift_ref = 0.0;             // C_L at the reference condition
  double drag_ref = 0.0;             // C_D at the reference condition
  double pitching_moment_ref = 0.0;  // C_m at the reference condition
  double lift_alpha = 0.0;           // 1/rad
  double drag_alpha = 0.0;           // 1/rad
  double pitching_moment_alpha = 0.0;
  double lift_pitch_rate = 0.0;             // per qhat
  double pitching_moment_pitch_rate = 0.0;  // per qhat
  double lift_alpha_dot = 0.0;              // per ahat — must be zero, see header
  double drag_alpha_dot = 0.0;              // per ahat — must be zero, see header
  double pitching_moment_alpha_dot = 0.0;   // per ahat
  double lift_elevator = 0.0;
  double drag_elevator = 0.0;
  double pitching_moment_elevator = 0.0;

  // Lateral-directional, body axes.
  double side_force_beta = 0.0;
  double rolling_moment_beta = 0.0;
  double yawing_moment_beta = 0.0;
  double rolling_moment_roll_rate = 0.0;  // per phat
  double yawing_moment_roll_rate = 0.0;
  double rolling_moment_yaw_rate = 0.0;  // per rhat
  double yawing_moment_yaw_rate = 0.0;
  double side_force_aileron = 0.0;
  double rolling_moment_aileron = 0.0;
  double yawing_moment_aileron = 0.0;
  double side_force_rudder = 0.0;
  double rolling_moment_rudder = 0.0;
  double yawing_moment_rudder = 0.0;
};

// Rotate a stability-axis lateral-directional derivative set into body axes.
//
// Published derivative sets very often give the lateral-directional
// derivatives in STABILITY axes — rotated from body by the trim angle of
// attack — while the equations of motion are written in body axes. The
// difference looks negligible and is not.
//
// The rotation is about the y-axis, which body and stability share, so the
// side force is unchanged. The rolling and yawing moments MIX:
//
//   C_l_body = C_l_stab cos(a) - C_n_stab sin(a)
//   C_n_body = C_l_stab sin(a) + C_n_stab cos(a)
//
// and the rate derivatives pick up a second mixing through
// phat_stab = phat cos(a) + rhat sin(a), rhat_stab = -phat sin(a) + rhat cos(a).
//
// At a trim angle of attack of only 2.2 degrees the cos(a) factor is 0.9993,
// which invites the conclusion that the whole rotation is a 0.07% effect. It
// is not. C_l_beta is typically several times C_n_beta, so the CROSS term
// dominates: for the NT-33A at 2.2 degrees, C_n_beta moves by 10%, and the
// Dutch roll damping with it. This function exists because that mistake was
// made here first and cost a 35% error in Dutch-roll zeta.
[[nodiscard]] AeroDerivatives lateral_stability_to_body(const AeroDerivatives& stability_axes,
                                                        double alpha_rad);

struct Controls {
  double elevator_rad = 0.0;  // positive trailing edge down (nose-down moment)
  double aileron_rad = 0.0;
  double rudder_rad = 0.0;
  double thrust_n = 0.0;  // N, along the thrust line

  [[nodiscard]] Eigen::VectorXd to_vector() const;
  [[nodiscard]] static Controls from_vector(const Eigen::VectorXd& u);
  static constexpr int kSize = 4;
};

// How far outside its reference condition a query has strayed. Every result
// carries one, because a first-order model gives a confident answer at any
// angle of attack you ask for and the only defence is that it says so.
struct EnvelopeWarning {
  double alpha_departure_rad = 0.0;  // |alpha - reference_alpha|
  double mach_departure = 0.0;
  // True when the departure exceeds the advisory limits below.
  bool outside_advisory_envelope = false;

  // Advisory only. A first-order lift curve is good to a few degrees; beyond
  // roughly ten it is being asked to model a stall it does not have.
  static constexpr double kAdvisoryAlphaLimitRad = 0.17453292519943295;  // 10 deg
  static constexpr double kAdvisoryMachLimit = 0.15;
};

class Aircraft {
 public:
  Geometry geometry;
  AeroDerivatives aero;
  sim::MassProperties mass;

  // Thrust line inclination relative to the body x-axis, positive nose-up.
  double thrust_incidence_rad = 0.0;  // rad
  // Offset from the CG to the point the aerodynamic moments are referenced to.
  Eigen::Vector3d cg_to_aero_reference_m = Eigen::Vector3d::Zero();  // m

  std::string description;
  std::string citation;

  // Throws std::invalid_argument on a model that cannot be simulated. This
  // includes an alpha-dot force derivative, which would make the model
  // implicit — see the header's "WHAT THIS IS NOT".
  void validate() const;

  // Aerodynamic and propulsive wrench about the CG, body axes.
  //
  // `alpha_dot_rad_s` feeds the downwash-lag term. Callers that do not have it
  // pass zero, which is correct at a trim point and is what the trim solver
  // does.
  [[nodiscard]] sim::Wrench wrench(const core::State& state,
                                   const Controls& controls,
                                   const core::AtmosphereState& atmosphere,
                                   double alpha_dot_rad_s) const;

  // dx/dt for the thirteen-component state.
  //
  // Computes the atmosphere from the state's altitude, so the model is a pure
  // function of (state, controls) — which is what the trim solver and the
  // linearisation both need.
  //
  // The alpha-dot term is handled without an implicit solve: the translational
  // accelerations do not depend on alphadot (validate() guarantees it), so
  // they are computed first, alphadot is formed from them, and only then does
  // the pitching moment use it.
  [[nodiscard]] core::StateVector derivative(const core::State& state,
                                             const Controls& controls,
                                             double delta_isa_k = 0.0) const;

  [[nodiscard]] EnvelopeWarning envelope(const core::State& state,
                                         const core::AtmosphereState& atmosphere) const;
};

// Reads an aircraft from a YAML file. Units in the file are SI, per ADR-0003.
[[nodiscard]] Aircraft load_aircraft(const std::string& path);

}  // namespace galata::model

#endif  // GALATA_MODEL_AIRCRAFT_HPP
