// SPDX-License-Identifier: Apache-2.0
//
// A rotor as a force-and-moment producer with an ORIENTATION and a TIP-PATH
// PLANE — the two things whose absence made a helicopter inexpressible.
//
// Reference:
//   G. D. Padfield, "Helicopter Flight Dynamics: The Theory and Application of
//   Flying Qualities and Simulation Modelling", 2nd ed., Blackwell, 2007,
//   chapter 3 — the Level-1 disc-tilt rotor formulation this file implements,
//   the multi-blade coordinates, and the quasi-static flapping approximation.
//   W. Johnson, "Helicopter Theory", Princeton University Press, 1980,
//   chapters 2 and 4 — momentum theory, the induced-velocity solution in axial
//   and forward flight, and blade flapping in multi-blade coordinates.
//   J. G. Leishman, "Principles of Helicopter Aerodynamics", 2nd ed.,
//   Cambridge University Press, 2006, chapters 2 and 3 — the Glauert forward-
//   flight inflow relation and the blade-element/momentum thrust integral.
//   H. Glauert, "A General Theory of the Autogyro", ARC R&M 1111, 1926 — the
//   forward-flight induced-velocity relation named for him below.
//   P. D. Talbot, B. E. Tinling, W. A. Decker and R. T. N. Chen, "A
//   Mathematical Model of a Single Main Rotor Helicopter for Piloted
//   Simulation", NASA TM-84281, 1982 — the component build-up this rotor is
//   sized to sit inside.
//
// WHAT THIS REPLACES, and why the replacement is structural rather than a
// refinement. `galata/model/quadrotor.hpp` gives every rotor the body-axis
// force (0, 0, -k_T omega^2). That has no direction, so a TAIL rotor — which
// must push along body y — becomes a downward thrust with a pitching moment,
// and a main rotor with CYCLIC — which tilts its thrust vector to translate —
// has identically zero longitudinal and lateral force. Neither is an
// approximation that is missing; both are inexpressible. Here the thrust acts
// along the tip-path-plane normal, resolved through the hub's own rotation into
// body axes, and both follow.
//
// THE MODEL, in the order it is evaluated:
//
//   1. Air-relative velocity at the hub, including the body rate's contribution
//      omega_body x r_hub, resolved into HUB axes.
//   2. Advance ratio mu and axial inflow ratio lambda_c from that velocity.
//   3. Thrust and inflow from closed-coupled blade-element momentum theory:
//        C_T = (a*sigma/2) [ (theta_0/3)(1 + 3mu^2/2) + (mu/2)theta_tw/...
//                            - (lambda_c + lambda_i)/2 ]      (blade element)
//        lambda_i = C_T / (2 sqrt(mu^2 + (lambda_c + lambda_i)^2))  (momentum)
//      solved together by a FIXED number of fixed-point iterations. Fixed, not
//      converged-to-tolerance: a tolerance exit is an exit read off the last
//      bits of the state, and ADR-0004 forbids it.
//   4. Tip-path-plane tilt a_1s, b_1s from the quasi-static flapping response
//      to cyclic, to mu, and to the body rates.
//   5. The wrench: thrust along the tip-path-plane normal, torque about the
//      shaft, hub moment from flapping stiffness, all rotated hub -> body and
//      moved to the CG.
//
// SIGN AND AXIS CONVENTIONS, normative for this file:
//   * Hub axes are FRD like the body: x forward, y right, z DOWN. Thrust is
//     along -z_hub for a rotor whose disc is horizontal, which is why an
//     untilted main rotor reproduces the old (0, 0, -T).
//   * `hub_to_body` transforms a vector's COMPONENTS from hub axes to body
//     axes. For a main rotor with forward shaft tilt it is a rotation about y;
//     for a tail rotor it is the rotation that puts -z_hub along +/-y_body.
//   * a_1s is longitudinal flapping, POSITIVE aft (disc tilted back, nose-up
//     thrust vector). b_1s is lateral flapping, POSITIVE toward the advancing
//     side. These are Padfield's conventions and getting either backwards
//     produces a helicopter that trims at the wrong stick and flies apart.
//   * `spin_about_shaft` is +1 when the rotor's angular velocity points along
//     +z_hub (down). The reaction torque on the airframe is opposite.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * NOT BLADE ELEMENT IN THE SPANWISE SENSE. The thrust integral is closed in
//   closed form over a uniform-inflow disc with a linear lift curve and a tip
//   loss factor. There is no spanwise station loop, no aerofoil table, no
//   radial inflow variation and no reverse-flow region. Above about mu = 0.35
//   the reverse-flow disc on the retreating side is no longer negligible and
//   this OVER-PREDICTS thrust; `envelope()` reports mu and says so.
//
// * NOT STALLED, AND NOT COMPRESSIBLE. The lift curve is linear in angle of
//   attack for ever. Retreating-blade stall and advancing-tip compressibility
//   are the two effects that actually limit a helicopter's forward speed, and
//   neither is here. A thrust computed above the declared C_T/sigma ceiling is
//   OPTIMISTIC — the real rotor would have stalled and produced less.
//
// * NOT DIFFERENTIABLE AT EXACTLY ZERO AIRSPEED, and this is a property of the
//   physics rather than of this implementation. The advance ratio is
//   mu = |V_inplane| / (Omega R), and |V| has no derivative at V = 0: approach
//   it from either side and the one-sided slope is +/-1. Every term carrying an
//   odd power of mu — the flap-back 2 mu (4/3 theta_0 - lambda) most of all —
//   inherits that kink.
//
//   The consequence is specific and worth stating, because it surprises people.
//   A finite-difference Jacobian at exactly zero airspeed returns
//   (|+h| - |-h|)/2h = 0 for d(mu)/dV, so the LINEARISATION sees no flap-back
//   response to a speed perturbation while the nonlinear model has one
//   proportional to |V|. The linearisation is therefore first-order accurate in
//   hover and second-order accurate everywhere else. Measured, not asserted:
//   `HelicopterLinearisation.NonlinearAndLinearAgreeToSecondOrderInForwardFlight`
//   observes order 2.004 at 5 m/s and above, and
//   `...IsOnlyFirstOrderInExactHover` observes 1.003 at V = 0 and exists to make
//   the degradation visible rather than to hide it.
//
//   This is not a defect to be fixed by smoothing mu. A smoothed advance ratio
//   would make the Jacobian look better and the MODEL worse, and the hover
//   linearisation would then be a good approximation to the wrong aircraft.
//
// * NOT A VORTEX-RING MODEL. The momentum-theory inflow solution is invalid in
//   the vortex-ring state, roughly -2 < lambda_c/lambda_h < 0, which is a
//   descent at between about one half and one and a half times the hover
//   induced velocity. There the real rotor loses thrust and becomes unsteady;
//   this one goes on producing a smooth answer. The answer is not merely
//   inaccurate there, it is QUALITATIVELY WRONG, and `envelope()` reports the
//   condition rather than the code refusing it, because a trajectory that
//   clips the region for one step is different from one that sits in it.
//
// * NOT DYNAMIC FLAPPING. The tip-path plane responds to cyclic and to rate
//   through a quasi-static relation: the flap regressing mode is assumed fast
//   relative to the body modes and its transient is not carried. That is the
//   standard Level-1 assumption and it is good to roughly 10 rad/s of body
//   bandwidth; a high-gain control law with bandwidth approaching the flap
//   frequency will not see the phase lag that actually limits it, and will
//   therefore look MORE stable here than in flight. `RotorFlapDynamics` is the
//   extension point and is deliberately not implemented rather than faked.
//
// * NO GROUND EFFECT, NO WAKE, NO INTERFERENCE. Thrust within about one rotor
//   diameter of the ground is UNDER-PREDICTED. Main-rotor downwash on the tail
//   rotor and empennage is not modelled here; the helicopter model applies a
//   declared blockage factor and says so.
//
// * NOT A DRIVETRAIN. This produces the torque the rotor DEMANDS. Where that
//   torque comes from, and what the rotor speed does in response, belongs to
//   the engine and governor.

#ifndef GALATA_MODEL_ROTOR_ROTOR_HPP
#define GALATA_MODEL_ROTOR_ROTOR_HPP

#include "galata/sim/rigid_body.hpp"

#include <Eigen/Core>

#include <string>

namespace galata::model::rotor {

// Geometry, inertia and aerodynamic constants of one rotor.
//
// Every field carries its unit. Nothing here is a coefficient fitted to a
// particular vehicle: these are measurable properties of a rotor.
struct RotorGeometry {
  std::string name;  // "main" or "tail"; reaches the report and the state names

  // Hub position relative to the CG, BODY axes, metres.
  Eigen::Vector3d position_cg_to_hub_body_m = Eigen::Vector3d::Zero();  // m

  // Components hub -> body. Identity is a disc whose normal is body -z.
  Eigen::Matrix3d hub_to_body = Eigen::Matrix3d::Identity();

  // +1 when the rotor's angular velocity points along +z_hub (downwards).
  int spin_about_shaft = 1;  // +1 or -1, dimensionless

  double radius_m = 0.0;            // R, m
  double chord_m = 0.0;             // c, m, equivalent constant chord
  int blade_count = 0;              // N_b
  double lift_curve_slope = 0.0;    // a, 1/rad
  double profile_drag_coefficient = 0.0;  // C_d0, dimensionless

  // Induced power factor kappa. Momentum theory's ideal induced power is a
  // LOWER BOUND: a real rotor loses to non-uniform inflow, tip losses beyond
  // the B factor, swirl and finite blade count. kappa is the empirical
  // multiplier that covers them, conventionally 1.10-1.20 for a helicopter main
  // rotor and higher for a heavily loaded tail rotor. Defaulting to 1.0 means
  // "ideal", which is a declared modelling choice and a visible one: a model
  // that leaves it there UNDER-PREDICTS power, and by roughly the 15% the
  // literature puts on it.
  double induced_power_factor = 1.0;  // kappa, dimensionless, >= 1
  double blade_twist_rad = 0.0;     // theta_tw, rad, linear, root to tip (negative = washout)
  double tip_loss_factor = 0.97;    // B, dimensionless, in (0, 1]

  // Flapping. `hinge_offset_m` is the flap hinge's radial station; the
  // equivalent flap stiffness supplies the hub moment a hingeless rotor
  // produces and an articulated one does not.
  double hinge_offset_m = 0.0;              // e, m
  double flap_stiffness_n_m_rad = 0.0;      // K_beta, N m/rad
  double blade_flap_inertia_kg_m2 = 0.0;    // I_beta, kg m^2, one blade about the hinge

  // Polar moment about the shaft, whole rotor, for the rotor-speed state and
  // the gyroscopic coupling into the airframe.
  double polar_inertia_kg_m2 = 0.0;  // I_R, kg m^2

  // Dynamic inflow lag. Zero means quasi-static inflow (no lag state).
  double inflow_time_constant_s = 0.0;  // tau_lambda, s

  // Declared operating envelope. Advisory: `envelope()` reports departures,
  // nothing refuses them.
  double maximum_thrust_coefficient_solidity = 0.12;  // C_T/sigma ceiling, dimensionless
  double maximum_advance_ratio = 0.35;                // mu ceiling, dimensionless

  // Solidity sigma = N_b c / (pi R). Derived rather than declared, so a model
  // cannot state a solidity that disagrees with its own chord and blade count.
  [[nodiscard]] double solidity() const;
  [[nodiscard]] double disc_area_m2() const;

  // Throws std::invalid_argument on a non-finite field, a non-positive radius,
  // chord, blade count or lift-curve slope, a tip loss factor outside (0, 1],
  // a spin that is not +/-1, a negative hinge offset or a hub_to_body that is
  // not a proper rotation (orthonormal, determinant +1). The rotation is
  // CHECKED because a near-rotation silently scales every force it resolves.
  void validate() const;
};

// The three swashplate inputs, in the rotor's own terms.
//
// For a tail rotor only `collective_rad` is meaningful and the two cyclics are
// zero: a tail rotor has no swashplate. The helicopter model enforces that;
// this struct does not, because a tilting-rotor vehicle might use all three.
struct RotorControls {
  double collective_rad = 0.0;             // theta_0, rad, blade root pitch
  double longitudinal_cyclic_rad = 0.0;    // theta_1s, rad, positive tilts the disc forward
  double lateral_cyclic_rad = 0.0;         // theta_1c, rad, positive tilts the disc right
};

// The rotor's own dynamic states, carried in the vehicle's auxiliary state.
struct RotorState {
  double speed_rad_s = 0.0;   // Omega, rad/s, magnitude (sign is `spin_about_shaft`)
  double inflow_ratio = 0.0;  // lambda_i, dimensionless, induced inflow / (Omega R)
};

// Everything the rotor computed, not only the wrench. Reported because a
// helicopter's trim is read in these terms and because the envelope check needs
// them.
struct RotorSolution {
  sim::Wrench wrench;  // in BODY axes, moment about the CG

  double thrust_n = 0.0;               // T, N, along the tip-path-plane normal
  double torque_n_m = 0.0;             // Q, N m, shaft torque DEMANDED (positive)
  double power_w = 0.0;                // P = Q * Omega, W
  double thrust_coefficient = 0.0;     // C_T, dimensionless
  double torque_coefficient = 0.0;     // C_Q, dimensionless
  double induced_inflow_ratio = 0.0;   // lambda_i, dimensionless, the one thrust was built on
  // The value the momentum balance settles at for this condition. Equal to
  // `induced_inflow_ratio` for a rotor with no lag; the target it is relaxing
  // towards for a rotor with one.
  double quasi_static_inflow_ratio = 0.0;  // dimensionless
  double axial_inflow_ratio = 0.0;     // lambda_c, dimensionless, positive climbing
  double advance_ratio = 0.0;          // mu, dimensionless
  double longitudinal_flap_rad = 0.0;  // a_1s, rad, positive aft
  double lateral_flap_rad = 0.0;       // b_1s, rad, positive toward the advancing side
  double induced_velocity_m_s = 0.0;   // v_i, m/s
  double thrust_coefficient_solidity = 0.0;  // C_T/sigma, dimensionless

  // Rate of the inflow state, for a rotor with a dynamic-inflow lag. Zero when
  // `inflow_time_constant_s` is zero.
  double inflow_rate_per_s = 0.0;  // 1/s
};

// The one function that replaces `Vector3d(0, 0, -thrust_n)`.
//
// `velocity_hub_body_m_s` is the AIR-RELATIVE velocity of the hub in BODY
// axes, already including the body rate's contribution. The caller computes it
// because it needs the whole-body state to do so and the rotor does not have it.
//
// Throws std::invalid_argument on a non-finite input or a non-positive density;
// std::runtime_error if the inflow solution fails to produce a finite value,
// which is the vortex-ring region's usual symptom.
[[nodiscard]] RotorSolution solve_rotor(const RotorGeometry& geometry,
                                        const RotorState& state,
                                        const RotorControls& controls,
                                        const Eigen::Vector3d& velocity_hub_body_m_s,
                                        const Eigen::Vector3d& body_rate_rad_s,
                                        double density_kg_m3);

// Hover induced velocity from momentum theory alone: v_h = sqrt(T / (2 rho A)).
//
// Exposed because it is the closed-form answer every hover check is made
// against, and because a caller sizing a rotor wants it without a full solve.
[[nodiscard]] double hover_induced_velocity_m_s(const RotorGeometry& geometry,
                                                double thrust_n,
                                                double density_kg_m3);

// Angular momentum of the spinning rotor about the CG, in body axes.
//
// THE GYROSCOPIC TERM THE MULTIROTOR MODEL DOCUMENTS AS ABSENT. A main rotor's
// angular momentum is a large fraction of the airframe's, so a pitch rate
// produces a roll moment and vice versa. Omitting it does not merely lose
// accuracy: it removes the cross-coupling that defines helicopter handling.
[[nodiscard]] Eigen::Vector3d rotor_angular_momentum_body(const RotorGeometry& geometry,
                                                          const RotorState& state);

// The gyroscopic moment on the airframe: -(omega_body x h_rotor).
[[nodiscard]] Eigen::Vector3d gyroscopic_moment_body(const RotorGeometry& geometry,
                                                     const RotorState& state,
                                                     const Eigen::Vector3d& body_rate_rad_s);

// A hub rotation for a rotor whose thrust acts along a named body direction.
//
// Convenience with a purpose: writing a tail rotor's 3x3 by hand is exactly the
// kind of thing that gets transposed, and a transposed rotation produces a
// helicopter that yaws the wrong way and still trims.
//   `main_rotor_hub(forward_tilt_rad, lateral_tilt_rad)` — thrust near body -z
//   `tail_rotor_hub(thrust_towards_starboard)`           — thrust along +/-y
[[nodiscard]] Eigen::Matrix3d main_rotor_hub(double forward_tilt_rad, double lateral_tilt_rad);
[[nodiscard]] Eigen::Matrix3d tail_rotor_hub(bool thrust_towards_starboard);

}  // namespace galata::model::rotor

#endif  // GALATA_MODEL_ROTOR_ROTOR_HPP
