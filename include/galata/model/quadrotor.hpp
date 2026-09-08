// SPDX-License-Identifier: Apache-2.0
//
// Nonlinear multirotor plant: rigid body, rotors with first-order speed lag,
// per-axis translational and angular drag, and an optional battery.
//
// Reference:
//   R. Mahony, V. Kumar and P. Corke, "Multirotor Aerial Vehicles: Modeling,
//   Estimation, and Control of Quadrotor", IEEE Robotics & Automation
//   Magazine, vol. 19, no. 3, pp. 20-32, 2012 — the standard rigid-body plus
//   rotor-thrust formulation this file implements.
//   P. Pounds, R. Mahony and P. Corke, "Modelling and control of a large
//   quadrotor robot", Control Engineering Practice, vol. 18, no. 7,
//   pp. 691-699, 2010 — the quadratic thrust and reaction-torque laws.
//   S. Bouabdallah and R. Siegwart, "Full control of a quadrotor",
//   IEEE/RSJ IROS, pp. 153-158, 2007 — the first-order rotor-speed lag.
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 2 — the six-degree-of-freedom
//   equations, which this model does not restate: it calls the shared kernel.
//
// Conventions are ADR-0002. The equations are written about the centre of
// gravity through `galata/sim/rigid_body.hpp`, per ADR-0006; there is no second
// six-degree-of-freedom implementation here and there must never be one.
//
// THE STATE IS THIRTEEN COMPONENTS PLUS THE MODEL'S OWN, AND THE ORDER MATTERS.
//
// ADR-0002 fixes thirteen rigid-body components and fixes their order. It does
// not forbid a model from carrying more, and `galata/core/state.hpp` says
// directly that a model needing rotor dynamics "carries them alongside, not
// inside". This model does exactly that. Its extended state is
//
//   x_ext = [ x_13   the ADR-0002 rigid-body state, unchanged and first
//             omega_0 .. omega_{n-1}   rotor speeds, rad/s, in rotor order
//             soc ]                    battery state of charge, if present
//
// Appending after index 12 permutes nothing, so every A and B matrix already
// exported by this project keeps its meaning. RFC-0002's acceptance section
// records why this needed no successor to ADR-0002.
//
// THE VELOCITY IN THE STATE IS AIR-RELATIVE, AND THE POSITION RATE IS NOT.
//
// ADR-0002's velocity is the velocity with respect to the local air mass, so
// the drag terms below act on it directly with no wind subtraction. The
// position derivative is a different question: the shared kernel rotates the
// air-relative velocity into NED, which is the air-relative position rate, and
// `state.hpp` makes the caller responsible for adding the wind field back.
// `derivative()` does that and is the only place it happens.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not an aerodynamic model of a rotor. Thrust is k_T * omega^2 and reaction
//   torque is k_Q * omega^2, both static and both independent of inflow. Blade
//   element and momentum theory both make thrust fall as axial inflow rises,
//   so this model OVER-PREDICTS thrust in climb and in fast forward flight,
//   and it has no vortex-ring state at all — in fast descent it is not merely
//   inaccurate but qualitatively wrong. It also has no ground effect, so it
//   UNDER-PREDICTS thrust within roughly one rotor diameter of the ground.
//   There is no blade flapping, no rotor-to-rotor interaction and no gyroscopic
//   term from the rotor discs.
//
// * Not an airframe aerodynamic model. Drag is a per-axis linear plus
//   quadratic fit on the air-relative body velocity: no lift, no side force,
//   no angle-of-attack dependence and no coupling between axes. The fit is
//   only as good as the airspeed range it was identified over, and outside
//   that range the quadratic term dominates and the error grows with the
//   square of the airspeed. `drag_linear / drag_quadratic` per axis is the
//   airspeed at which the two contribute equally and is the honest scale
//   below which a linearisation about hover means anything.
//
// * Not an ESC or motor model. The rotor responds to a commanded SPEED in
//   rad/s through one first-order lag. Mapping a PWM, DShot or normalised
//   throttle command to a rotor speed is out of scope, deliberately: that
//   mapping is where an airframe's identification effort actually goes, and
//   pretending to it here would invite a user to trust it. There is no motor
//   electrical model, no current limit and no torque saturation.
//
// * Not a battery model in any electrochemical sense. The optional block is a
//   quasi-static open-circuit voltage falling linearly with state of charge,
//   a constant internal resistance, and a speed ceiling derived from the
//   resulting terminal voltage. No thermal behaviour, no cell imbalance, no
//   rate-dependent capacity, no ageing. Real cells sag more under load and
//   recover when unloaded, so this OVER-ESTIMATES available speed late in a
//   discharge.
//
// * NOT A TIME-VARYING WIND. `derivative()` takes the wind as STEADY. The
//   air-relative velocity obeys d(v_air)/dt = a - dw/dt, and the second term
//   is not carried here. For a constant wind it is zero and nothing is lost.
//   For a wind that CHANGES, the consequence is specific and easy to miss:
//   the ground velocity is continuous across a wind change, because no force
//   acts at the instant the air mass changes speed, so the air-relative
//   velocity must jump by exactly minus the wind change. A caller that steps
//   the wind without re-basing the state injects the whole wind increment as a
//   ground-velocity error, silently and permanently. The caller owns that
//   re-basing, for the same reason state.hpp makes the caller own adding the
//   wind back to the position rate — and
//   `QuadrotorCrossImplementation.ReproducesTheSouxmarOpenLoopTrajectory` does
//   it explicitly at the one wind step in its fixture. Gusts and turbulence
//   are therefore out of scope until a wind model owns the derivative term.
//
// * Not rigid-body-exact for a real airframe. No structural modes, no
//   vibration, no slung load, no articulated payload.
//
// * Not validated against any real aircraft. The shipped parameter set is an
//   independent implementation's nominal set, not a measurement, and no
//   published quadrotor reference anchors it. A completed run of this model is
//   evidence about the equations, never about an aircraft. RFC-0002's
//   acceptance section records that decision and its reason.

#ifndef GALATA_MODEL_QUADROTOR_HPP
#define GALATA_MODEL_QUADROTOR_HPP

#include "galata/core/state.hpp"
#include "galata/sim/rigid_body.hpp"

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

namespace galata::model {

// One rotor, positioned relative to the centre of gravity in body FRD axes.
struct Rotor {
  // Position of the rotor hub relative to the CG, body axes.
  Eigen::Vector3d position_cg_to_hub_body_m = Eigen::Vector3d::Zero();  // m

  // Sign of the rotor's angular-velocity vector along the BODY Z AXIS, which
  // points down. +1 is a rotor whose angular velocity points downward, seen
  // from above as clockwise; -1 is the opposite.
  //
  // The reaction moment this produces on the airframe about body z is
  //
  //     N_i = -spin * k_Q * omega^2
  //
  // with the minus sign because the airframe receives the reaction to the
  // torque the motor applies to the rotor. A rotor spinning one way yaws the
  // body the other way, and getting this backwards produces a model that
  // hovers, trims and linearises with the yaw axis inverted.
  int spin_about_body_z = 1;  // +1 or -1, dimensionless

  double thrust_coefficient_n_s2 = 0.0;    // N s^2,   T = k_T omega^2
  double torque_coefficient_n_m_s2 = 0.0;  // N m s^2, Q = k_Q omega^2
  double speed_time_constant_s = 0.0;      // s, first-order lag to the command
  double minimum_speed_rad_s = 0.0;        // rad/s
  double maximum_speed_rad_s = 0.0;        // rad/s
};

// Quasi-static battery. Omit it and the plant is exactly the fixed-voltage one.
struct Battery {
  // Usable energy at full charge, in JOULES. A datasheet quotes watt-hours;
  // the conversion is exact (1 W h = 3600 J) and happens once, at transcription
  // time, recorded in the model's provenance file — never here. ADR-0003.
  double energy_j = 0.0;                 // J
  double full_voltage_v = 0.0;           // V, open-circuit at state of charge 1
  double empty_voltage_v = 0.0;          // V, open-circuit at state of charge 0
  double internal_resistance_ohm = 0.0;  // ohm
  // Rotor speed available at full_voltage_v. The ceiling scales with terminal
  // voltage, which is the only way the battery reaches the dynamics at all.
  double speed_at_full_voltage_rad_s = 0.0;  // rad/s
};

// Commanded rotor speeds, one per rotor, in rotor order.
struct RotorCommand {
  Eigen::VectorXd speed_rad_s;  // rad/s
};

class Quadrotor {
 public:
  sim::MassProperties mass;
  std::vector<Rotor> rotors;

  // Per-axis drag on the air-relative body velocity, body axes. The force on
  // axis k is -(linear_k * v_k + quadratic_k * v_k * |v_k|), so the quadratic
  // term opposes motion in either direction rather than always acting one way.
  Eigen::Vector3d drag_linear_n_s_m = Eigen::Vector3d::Zero();       // N s/m
  Eigen::Vector3d drag_quadratic_n_s2_m2 = Eigen::Vector3d::Zero();  // N s^2/m^2

  // Per-axis angular drag on the body rates: M = -angular_drag .* omega.
  Eigen::Vector3d angular_drag_n_m_s = Eigen::Vector3d::Zero();  // N m s

  std::optional<Battery> battery;

  std::string description;
  std::string citation;

  // Throws std::invalid_argument on a model that cannot be simulated: any
  // non-finite field, no rotors, a non-positive or non-finite time constant, a
  // negative coefficient, a speed range that is empty or negative, a spin that
  // is neither +1 nor -1, or mass properties that fail their own validation.
  //
  // Call after direct C++ edits. derivative() does not repeat this in the
  // integration hot loop.
  void validate() const;

  [[nodiscard]] int rotor_count() const noexcept {
    return static_cast<int>(rotors.size());
  }

  // Layout of the extended state, as described at the top of this file.
  [[nodiscard]] int rotor_state_offset() const noexcept {
    return core::kStateSize;
  }

  [[nodiscard]] bool has_battery() const noexcept {
    return battery.has_value();
  }

  [[nodiscard]] int battery_state_index() const;  // throws when there is no battery

  [[nodiscard]] int extended_state_size() const noexcept {
    return core::kStateSize + rotor_count() + (has_battery() ? 1 : 0);
  }

  // Names of the extended state, in order. These reach exported matrices and
  // the modal classifier, so they are contract, not display.
  [[nodiscard]] std::vector<std::string> extended_state_names() const;
  [[nodiscard]] std::vector<std::string> input_names() const;

  // Rotor speed the plant can actually reach, given the battery's terminal
  // voltage at `state_of_charge`. Without a battery this is the rotor's own
  // maximum. The current draw is not modelled, so the terminal voltage used
  // here is the open-circuit voltage; the internal resistance enters only
  // through `terminal_voltage_v`.
  [[nodiscard]] double speed_ceiling_rad_s(int rotor_index, double state_of_charge) const;

  // Open-circuit voltage falling linearly from full to empty.
  [[nodiscard]] double open_circuit_voltage_v(double state_of_charge) const;

  // Open-circuit voltage less the resistive drop at `current_a`.
  [[nodiscard]] double terminal_voltage_v(double state_of_charge, double current_a) const;

  // Total thrust and the moment about the CG produced by the rotors alone,
  // body axes. Separated from `wrench` so a test can check the rotor terms
  // without the drag terms confounding them.
  [[nodiscard]] sim::Wrench rotor_wrench(const Eigen::VectorXd& rotor_speed_rad_s) const;

  // Rotor wrench plus drag. Gravity is NOT included: the shared kernel takes it
  // separately, because it needs the attitude to resolve it into body axes.
  [[nodiscard]] sim::Wrench wrench(const core::State& state,
                                   const Eigen::VectorXd& rotor_speed_rad_s) const;

  // d/dt of the extended state, in the order this file documents.
  //
  // `wind_ned_m_s` is the wind velocity in NED. It reaches the derivative in
  // exactly one place — the position rate, which is the ground velocity — and
  // it does so because the state's own velocity is air-relative. Drag needs no
  // wind subtraction for the same reason.
  //
  // Throws std::invalid_argument when the extended state or the command is the
  // wrong length, so a caller that has miscounted its own rotors finds out
  // here rather than in a plausible trajectory.
  [[nodiscard]] Eigen::VectorXd derivative(
      const Eigen::VectorXd& extended_state,
      const Eigen::VectorXd& command_rad_s,
      const Eigen::Vector3d& wind_ned_m_s = Eigen::Vector3d::Zero()) const;

  // Rotor speed at which `count` equal rotors carry the weight, sqrt(m g / (n k_T)).
  // Defined only when every rotor shares one thrust coefficient; throws otherwise,
  // because an "average" hover speed for mismatched rotors is not a hover.
  [[nodiscard]] double hover_speed_rad_s(double gravity_m_s2) const;

  // Projection applied after every completed integrator step: renormalise the
  // attitude quaternion (ADR-0002) and clamp each rotor speed to its own
  // range and to the battery ceiling. Deterministic and branch-free in the
  // sense ADR-0004 requires — clamping is not a tolerance-based early exit.
  void project(Eigen::VectorXd& extended_state) const;
};

// Reads a quadrotor from a YAML file. Units in the file are SI, per ADR-0003.
//
// UNKNOWN KEYS ARE ERRORS, unlike `load_aircraft`. A model file is the one
// place where a typo produces a plant that loads, flies and is quietly wrong,
// so every map in the document is checked against its allowed key set.
[[nodiscard]] Quadrotor load_quadrotor(const std::string& path);

// Parse exactly these bytes; the pipeline records their digest before parsing.
[[nodiscard]] Quadrotor parse_quadrotor(const std::string& bytes,
                                        const std::string& source_name = "model");

}  // namespace galata::model

#endif  // GALATA_MODEL_QUADROTOR_HPP
