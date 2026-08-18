// SPDX-License-Identifier: Apache-2.0
//
// Straight-line trim by Newton's method on a square residual.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 3.
//
// Validity envelope and what is not modelled: the header's "WHAT THIS IS NOT".

#include "galata/trim/level.hpp"

#include "galata/core/quaternion.hpp"
#include "galata/numerics/newton.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace galata::trim {
namespace {

// Builds the flight state implied by (alpha, altitude, speed, gamma) under the
// straight-line constraints: wings level, no sideslip, no angular rates,
// theta = alpha + gamma.
core::State state_from_unknowns(double alpha_rad,
                                double altitude_m,
                                double airspeed_m_s,
                                double gamma_rad) {
  core::State state;
  state.position_ned_m = Eigen::Vector3d(0.0, 0.0, -altitude_m);
  state.velocity_body_m_s = core::from_wind_axes({airspeed_m_s, alpha_rad, 0.0});
  state.attitude_body_to_ned = core::quaternion_from_euler({0.0, alpha_rad + gamma_rad, 0.0});
  state.angular_rate_body_rad_s = Eigen::Vector3d::Zero();
  return state;
}

}  // namespace

TrimPoint trim_level(const model::Aircraft& aircraft, const LevelTrimRequest& request) {
  aircraft.validate();

  const bool has_airspeed = request.airspeed_m_s > 0.0;
  const bool has_mach = request.mach > 0.0;
  if (has_airspeed == has_mach) {
    throw std::invalid_argument(
        "trim_level: set exactly one of airspeed_m_s and mach. Setting both is ambiguous and "
        "setting neither leaves the condition undefined.");
  }

  const core::AtmosphereState atmosphere = core::isa(request.altitude_m, request.delta_isa_k);
  const double airspeed =
      has_airspeed ? request.airspeed_m_s : request.mach * atmosphere.speed_of_sound_m_s;
  if (!(airspeed > 0.0)) {
    throw std::invalid_argument("trim_level: the resolved airspeed is not positive");
  }

  const double dynamic_pressure = 0.5 * atmosphere.density_kg_m3 * airspeed * airspeed;
  const double reference_force = dynamic_pressure * aircraft.geometry.wing_area_m2;

  // The residual: the two body-axis translational accelerations and the
  // pitching acceleration, as functions of [alpha, elevator, thrust].
  const galata::numerics::VectorFunction residual =
      [&](const Eigen::VectorXd& unknowns) -> Eigen::VectorXd {
    const core::State state = state_from_unknowns(
        unknowns(0), request.altitude_m, airspeed, request.flight_path_angle_rad);
    model::Controls controls;
    controls.elevator_rad = unknowns(1);
    controls.thrust_n = unknowns(2);

    const core::StateVector rate = aircraft.derivative(state, controls, request.delta_isa_k);
    Eigen::VectorXd out(3);
    out(0) = rate(core::kVelocityU);
    out(1) = rate(core::kVelocityW);
    out(2) = rate(core::kRateQ);
    return out;
  };

  Eigen::VectorXd guess(3);
  if (request.use_default_guess) {
    // Angle of attack at the model's own reference condition, no elevator, and
    // thrust equal to the drag the reference condition would produce. For a
    // conventional aircraft that is within a degree or two of the answer.
    guess << aircraft.aero.reference_alpha_rad, 0.0, aircraft.aero.drag_ref * reference_force;
  } else {
    guess << request.initial_alpha_rad, request.initial_elevator_rad, request.initial_thrust_n;
  }

  galata::numerics::NewtonOptions options;
  options.residual_tolerance = request.residual_tolerance;
  // The unknowns have wildly different magnitudes — an angle of order 0.04 rad
  // and a thrust of order 10,000 N — so one absolute floor cannot be right for
  // all three. The relative step carries most of the work; these floors only
  // matter when an unknown passes through zero, which the elevator does.
  options.jacobian.absolute_step_per_component = Eigen::VectorXd(3);
  options.jacobian.absolute_step_per_component << 1e-6, 1e-6, 1e-1;

  const galata::numerics::NewtonResult solved =
      galata::numerics::solve_newton(residual, guess, options);

  if (!solved.converged) {
    std::ostringstream message;
    message << std::scientific << std::setprecision(6);
    message << "trim_level: did not converge. Residual norm " << solved.residual_norm
            << " exceeds the tolerance " << request.residual_tolerance << ".\n";
    message << "  Residual: u_dot = " << solved.residual(0)
            << " m/s^2, w_dot = " << solved.residual(1) << " m/s^2, q_dot = " << solved.residual(2)
            << " rad/s^2\n";
    message << "  Jacobian condition number: " << solved.jacobian_condition_number << "\n";
    message << "  Residual history:";
    for (const double value : solved.residual_history) {
      message << " " << value;
    }
    message << "\n  No trim is returned. A best-effort answer would be linearised about a "
               "point that is not an equilibrium, and the resulting state-space model would be "
               "plausible and wrong.";
    throw std::runtime_error(message.str());
  }

  TrimPoint trim;
  trim.state = state_from_unknowns(
      solved.solution(0), request.altitude_m, airspeed, request.flight_path_angle_rad);
  trim.controls.elevator_rad = solved.solution(1);
  trim.controls.thrust_n = solved.solution(2);
  trim.atmosphere = atmosphere;
  trim.alpha_rad = solved.solution(0);
  trim.flight_path_angle_rad = request.flight_path_angle_rad;
  trim.pitch_attitude_rad = solved.solution(0) + request.flight_path_angle_rad;
  trim.airspeed_m_s = airspeed;
  trim.mach =
      (atmosphere.speed_of_sound_m_s > 0.0) ? airspeed / atmosphere.speed_of_sound_m_s : 0.0;
  trim.dynamic_pressure_pa = dynamic_pressure;
  trim.residual_norm = solved.residual_norm;
  trim.jacobian_condition_number = solved.jacobian_condition_number;
  trim.residual_history = solved.residual_history;
  trim.envelope = aircraft.envelope(trim.state, atmosphere);

  // Reported because it is the one trim output a reader can check by hand:
  // in level flight lift equals weight, so C_L = W / (q S).
  const double delta_alpha = trim.alpha_rad - aircraft.aero.reference_alpha_rad;
  trim.lift_coefficient = aircraft.aero.lift_ref + aircraft.aero.lift_alpha * delta_alpha
                          + aircraft.aero.lift_elevator * trim.controls.elevator_rad;
  return trim;
}

}  // namespace galata::trim
