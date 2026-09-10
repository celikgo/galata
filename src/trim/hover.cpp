// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the multirotor equilibrium declared in
// include/galata/trim/hover.hpp.

#include "galata/trim/hover.hpp"

#include "galata/core/constants.hpp"
#include "galata/core/frames.hpp"
#include "galata/core/quaternion.hpp"
#include "galata/numerics/newton.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace galata::trim {
namespace {

// Two attitude angles plus one speed per rotor. Yaw is declared, so it is not
// among them.
constexpr int kAttitudeUnknowns = 2;

// Three force and three moment equations.
constexpr int kEquations = 6;

core::Quaternion attitude_from_angles(double roll_rad, double pitch_rad, double yaw_rad) {
  core::EulerAngles euler;
  euler.roll_rad = roll_rad;
  euler.pitch_rad = pitch_rad;
  euler.yaw_rad = yaw_rad;
  return core::quaternion_from_euler(euler);
}

// Builds the extended state the unknowns describe. Body rates are zero: a
// non-rotating equilibrium is the only kind this solves for, and a coordinated
// turn would need the turn rate among the declared inputs rather than among the
// unknowns.
Eigen::VectorXd assemble(const model::Quadrotor& model,
                         const HoverTrimRequest& request,
                         const Eigen::VectorXd& unknowns) {
  const double roll_rad = unknowns(0);
  const double pitch_rad = unknowns(1);
  const core::Quaternion attitude = attitude_from_angles(roll_rad, pitch_rad, request.heading_rad);

  const Eigen::Vector3d air_relative_ned = request.ground_velocity_ned_m_s - request.wind_ned_m_s;
  const Eigen::Vector3d air_relative_body =
      core::dcm_ned_from_body(attitude).transpose() * air_relative_ned;

  Eigen::VectorXd extended = Eigen::VectorXd::Zero(model.extended_state_size());
  // Horizontal position is arbitrary at an equilibrium of this plant — nothing
  // in the dynamics reads it — so north and east stay at the origin. The DOWN
  // component is not arbitrary in the same way: the request declares an
  // altitude, ADR-0002's down axis points down, and a trim that reported zero
  // there would hand every downstream consumer a state at sea level whatever
  // the caller asked for. `Altitude` is an output of the linearisation and the
  // NED down state is the only thing it reads, so leaving this at the origin
  // makes the altitude channel silently wrong rather than merely unset.
  //
  // This plant's dynamics still do not read it — gravity does not vary with
  // height here and there is no atmosphere — so setting it changes no residual
  // and no Jacobian entry. It changes what the answer SAYS, which is the point.
  extended(core::kPositionDown) = -request.altitude_m;
  extended(core::kVelocityU) = air_relative_body.x();
  extended(core::kVelocityV) = air_relative_body.y();
  extended(core::kVelocityW) = air_relative_body.z();
  extended(core::kQuaternionW) = attitude.w();
  extended(core::kQuaternionX) = attitude.x();
  extended(core::kQuaternionY) = attitude.y();
  extended(core::kQuaternionZ) = attitude.z();
  for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
    extended(model.rotor_state_offset() + rotor) = unknowns(kAttitudeUnknowns + rotor);
  }
  if (model.has_battery()) {
    extended(model.battery_state_index()) = request.battery_state_of_charge;
  }
  return extended;
}

}  // namespace

HoverTrim trim_hover(const model::Quadrotor& model, const HoverTrimRequest& request) {
  model.validate();

  const int rotor_count = model.rotor_count();
  const int unknown_count = kAttitudeUnknowns + rotor_count;
  if (unknown_count != kEquations) {
    std::ostringstream message;
    message << "trim_hover: this solve needs exactly " << (kEquations - kAttitudeUnknowns)
            << " rotors, and this model has " << rotor_count
            << ". With more rotors the equilibrium is an allocation problem with a null "
               "space, not a root; Newton would return whichever point it reached. Refused "
               "rather than answered.";
    throw std::invalid_argument(message.str());
  }
  if (!std::isfinite(request.battery_state_of_charge) || request.battery_state_of_charge < 0.0
      || request.battery_state_of_charge > 1.0) {
    throw std::invalid_argument("trim_hover: battery_state_of_charge must be in [0, 1]");
  }
  if (!request.wind_ned_m_s.allFinite() || !request.ground_velocity_ned_m_s.allFinite()
      || !std::isfinite(request.heading_rad) || !std::isfinite(request.altitude_m)
      || !std::isfinite(request.residual_tolerance)) {
    throw std::invalid_argument("trim_hover: the request carries a non-finite value");
  }
  if (request.residual_tolerance <= 0.0) {
    throw std::invalid_argument(
        "trim_hover: residual_tolerance must be positive; a budget of zero or less is a gate "
        "no equilibrium can pass rather than an exact one");
  }
  if (request.iterations <= 0) {
    throw std::invalid_argument("trim_hover: iterations must be positive");
  }

  // The residual is the six DYNAMIC accelerations. Position rate is excluded
  // deliberately — see the header on relative equilibria — and so is the
  // battery, which has no equilibrium to find.
  const numerics::VectorFunction residual = [&](const Eigen::VectorXd& unknowns) {
    const Eigen::VectorXd extended = assemble(model, request, unknowns);
    Eigen::VectorXd command(rotor_count);
    for (int rotor = 0; rotor < rotor_count; ++rotor) {
      command(rotor) = unknowns(kAttitudeUnknowns + rotor);
    }
    const Eigen::VectorXd rates = model.derivative(extended, command, request.wind_ned_m_s);

    Eigen::VectorXd out(kEquations);
    out.head<3>() = rates.segment<3>(core::kVelocityU);
    out.tail<3>() = rates.segment<3>(core::kRateP);
    return out;
  };

  // The initial guess is the still-air hover: level, every rotor carrying its
  // share of the weight. For every condition this solve admits that is inside
  // Newton's basin, because a multirotor's tilt at any airspeed it can hold is
  // small.
  //
  // The share is a share of FORCE, not a shared speed. Asking each rotor for
  // m g / n gives sqrt(m g / (n k_T_i)) per rotor, which is the same number for
  // every rotor of a homogeneous vehicle and a different one for each rotor of a
  // vehicle whose thrust coefficients differ. The earlier form asked the model
  // for one vehicle-wide hover speed, which is undefined when the coefficients
  // differ and threw — so a model with four measured rotors could not be trimmed
  // at all, though the solve below has always carried one unknown per rotor and
  // needed no vehicle-wide speed to run. The restriction was in the starting
  // point, never in the equations.
  //
  // This is a guess and not an equilibrium: equal thrust balances the force and
  // leaves a residual couple whenever the rotors differ. Newton removes it.
  Eigen::VectorXd guess = Eigen::VectorXd::Zero(unknown_count);
  for (int rotor = 0; rotor < rotor_count; ++rotor) {
    guess(kAttitudeUnknowns + rotor) = model.hover_speed_rad_s(rotor, core::kStandardGravity);
  }

  numerics::NewtonOptions options;
  options.iterations = request.iterations;
  options.residual_tolerance = request.residual_tolerance;
  const numerics::NewtonResult solved = numerics::solve_newton(residual, guess, options);

  if (!solved.converged) {
    std::ostringstream message;
    message << "trim_hover: no equilibrium found. Residual norm " << solved.residual_norm
            << " exceeds the budget " << request.residual_tolerance << " after "
            << request.iterations << " iterations; Jacobian condition number "
            << solved.jacobian_condition_number
            << ". A large condition number here means a rotor has no authority over any "
               "residual at this condition.";
    throw std::runtime_error(message.str());
  }

  HoverTrim trim;
  trim.extended_state = assemble(model, request, solved.solution);
  trim.altitude_m = request.altitude_m;
  trim.wind_ned_m_s = request.wind_ned_m_s;
  trim.ground_velocity_ned_m_s = request.ground_velocity_ned_m_s;
  trim.command_rad_s = solved.solution.tail(rotor_count);
  trim.roll_rad = solved.solution(0);
  trim.pitch_rad = solved.solution(1);
  trim.yaw_rad = request.heading_rad;
  trim.air_relative_velocity_body_m_s = trim.extended_state.segment<3>(core::kVelocityU);
  trim.airspeed_m_s = trim.air_relative_velocity_body_m_s.norm();
  trim.battery_state_of_charge = request.battery_state_of_charge;
  // Held, not solved — and the residual above never saw that row. Recorded so
  // the exported evidence can say so outright.
  trim.battery_state_of_charge_frozen = model.has_battery();
  trim.residual_norm = solved.residual_norm;
  trim.residual_tolerance = request.residual_tolerance;
  trim.newton_iterations = request.iterations;
  trim.jacobian_condition_number = solved.jacobian_condition_number;
  trim.residual_history = solved.residual_history;

  // Feasibility is checked AFTER the solve, because Newton is unaware of
  // constraints and a trim that needs more rotor than the vehicle has is still
  // a root of the equations.
  trim.rotor_margin_fraction.reserve(static_cast<std::size_t>(rotor_count));
  double smallest = 1.0;
  for (int rotor = 0; rotor < rotor_count; ++rotor) {
    const double ceiling = model.speed_ceiling_rad_s(rotor, request.battery_state_of_charge);
    const double speed = trim.command_rad_s(rotor);
    const double floor = model.rotors[static_cast<std::size_t>(rotor)].minimum_speed_rad_s;
    if (speed > ceiling || speed < floor) {
      std::ostringstream message;
      message << "trim_hover: the equilibrium is infeasible. Rotor " << rotor << " needs " << speed
              << " rad/s, outside its range [" << floor << ", " << ceiling
              << "] rad/s at state of charge " << request.battery_state_of_charge
              << ". Refused rather than returned: a best effort reported as a trim gets "
                 "linearised, and a linearisation about a non-equilibrium carries a constant "
                 "term the A matrix cannot represent.";
      throw std::runtime_error(message.str());
    }
    const double margin = (ceiling - speed) / ceiling;
    trim.rotor_margin_fraction.push_back(margin);
    smallest = std::fmin(smallest, margin);
  }
  trim.smallest_rotor_margin_fraction = smallest;

  return trim;
}

}  // namespace galata::trim
