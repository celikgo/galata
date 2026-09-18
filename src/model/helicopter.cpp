// SPDX-License-Identifier: Apache-2.0
//
// Level-1 single-main-rotor helicopter: the component build-up.
//
// Reference:
//   G. D. Padfield, "Helicopter Flight Dynamics", 2nd ed., Blackwell, 2007,
//   chapter 3.
//   P. D. Talbot et al., "A Mathematical Model of a Single Main Rotor
//   Helicopter for Piloted Simulation", NASA TM-84281, 1982.
//   R. K. Heffley and M. A. Mnich, "Minimum-Complexity Helicopter Simulation
//   Math Model", NASA CR-177476, 1988.
//   J. G. Leishman, "Principles of Helicopter Aerodynamics", 2nd ed.,
//   Cambridge, 2006, chapter 5.
//
// Validity envelope and known error direction are in the header's
// "WHAT THIS IS NOT" block, in full. In summary: no ground effect (thrust
// UNDER-predicted near the ground), no stall or compressibility (thrust and
// control power OVER-predicted at high C_T/sigma and mu), no autorotative
// energy exchange (rotor decays FASTER than the real aircraft with the engine
// failed), no dynamic flapping (a high-bandwidth law looks MORE stable here
// than in flight).

#include "galata/model/helicopter.hpp"

#include "galata/core/quaternion.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace galata::model {
namespace {

std::string number(double value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(6) << value;
  return out.str();
}

// Saturate to [low, high]. Written out rather than std::clamp so that a NaN
// argument returns the low limit rather than propagating; a NaN command must
// not become a NaN actuator position and take the whole trajectory with it.
double saturate(double value, double low, double high) {
  if (!std::isfinite(value)) {
    return low;
  }
  return value < low ? low : (value > high ? high : value);
}

// A lifting surface's coefficient, linear to the stall angle and HELD beyond
// it. Held, not extrapolated: a linear surface goes on lifting for ever, and
// through transition a helicopter's horizontal tail really does reach its
// stall. Holding is still wrong — a stalled surface loses lift — but it is
// wrong in the CONSERVATIVE direction, and the header says so.
double surface_coefficient(double slope, double incidence_rad, double stall_rad) {
  if (!(stall_rad > 0.0)) {
    return slope * incidence_rad;
  }
  const double limited = saturate(incidence_rad, -stall_rad, stall_rad);
  return slope * limited;
}

}  // namespace

bool HelicopterFailures::any() const {
  if (tail_rotor_effectiveness != 1.0 || engine_available_fraction != 1.0) {
    return true;
  }
  for (const auto& jam : jammed_actuator_rad) {
    if (jam.has_value()) {
      return true;
    }
  }
  return false;
}

std::string HelicopterModel::description() const {
  return description_text.empty() ? std::string("Level-1 single-main-rotor helicopter")
                                  : description_text;
}

int HelicopterModel::auxiliary_state_count() const {
  return kHelicopterAuxCount;
}

std::vector<std::string> HelicopterModel::state_names() const {
  return {"position_north_m",   "position_east_m",
          "position_down_m",    "velocity_u_m_s",
          "velocity_v_m_s",     "velocity_w_m_s",
          "quaternion_w",       "quaternion_x",
          "quaternion_y",       "quaternion_z",
          "roll_rate_rad_s",    "pitch_rate_rad_s",
          "yaw_rate_rad_s",     "main_rotor_speed_rad_s",
          "main_inflow_ratio",  "tail_inflow_ratio",
          "collective_rad",     "longitudinal_cyclic_rad",
          "lateral_cyclic_rad", "pedal_rad",
          "engine_torque_n_m"};
}

std::vector<std::string> HelicopterModel::control_names() const {
  return {"collective_command_rad",
          "longitudinal_cyclic_command_rad",
          "lateral_cyclic_command_rad",
          "pedal_command_rad"};
}

std::vector<std::string> HelicopterModel::output_names() const {
  return {"position_north_m",
          "position_east_m",
          "altitude_m",
          "velocity_u_m_s",
          "velocity_v_m_s",
          "velocity_w_m_s",
          "roll_rad",
          "pitch_rad",
          "yaw_rad",
          "roll_rate_rad_s",
          "pitch_rate_rad_s",
          "yaw_rate_rad_s",
          "airspeed_m_s",
          "main_rotor_speed_rad_s",
          "main_thrust_n",
          "tail_thrust_n",
          "total_power_w",
          "atmospheric_altitude_m"};
}

sim::MassProperties HelicopterModel::mass_properties(const Eigen::VectorXd& /*auxiliary*/) const {
  return mass;
}

Eigen::VectorXd HelicopterModel::initial_auxiliary(const Eigen::VectorXd& controls,
                                                   const Environment& environment) const {
  Eigen::VectorXd auxiliary = Eigen::VectorXd::Zero(kHelicopterAuxCount);
  auxiliary(kMainRotorSpeed) = drivetrain.reference_rotor_speed_rad_s;

  // Inflow seeded at the hover momentum value for the aircraft's own weight,
  // which is the right scale in every condition and saves the first few steps
  // of a trim from chasing it.
  const double weight_n = mass.mass_kg * environment.gravity_ned_m_s2.norm();
  const double tip_speed = main_rotor.radius_m * drivetrain.reference_rotor_speed_rad_s;
  if (tip_speed > 0.0) {
    auxiliary(kMainInflowRatio) =
        rotor::hover_induced_velocity_m_s(main_rotor, weight_n, environment.density_kg_m3)
        / tip_speed;
  }
  const double tail_tip_speed =
      tail_rotor.radius_m * drivetrain.reference_rotor_speed_rad_s * drivetrain.tail_gear_ratio;
  if (tail_tip_speed > 0.0) {
    auxiliary(kTailInflowRatio) = 0.0;
  }
  // Engine torque seeded at the torque a hover needs, so a trim does not have to
  // walk it up from zero through a rotor-speed excursion.
  const double tip = main_rotor.radius_m * drivetrain.reference_rotor_speed_rad_s;
  if (tip > 0.0) {
    const double hover_power =
        weight_n
        * rotor::hover_induced_velocity_m_s(main_rotor, weight_n, environment.density_kg_m3)
        * main_rotor.induced_power_factor;
    auxiliary(kEngineTorque) = saturate(hover_power / drivetrain.reference_rotor_speed_rad_s
                                            / std::max(drivetrain.transmission_efficiency, 1.0e-6),
                                        drivetrain.minimum_engine_torque_n_m,
                                        drivetrain.maximum_engine_torque_n_m);
  }
  if (controls.size() == kHelicopterControlCount) {
    for (int i = 0; i < kHelicopterControlCount; ++i) {
      auxiliary(kCollectivePosition + i) =
          saturate(controls(i),
                   actuators[static_cast<std::size_t>(i)].minimum_rad,
                   actuators[static_cast<std::size_t>(i)].maximum_rad);
    }
  }
  return auxiliary;
}

HelicopterModel::Breakdown HelicopterModel::breakdown(const core::State& state,
                                                      const Eigen::VectorXd& auxiliary,
                                                      const Eigen::VectorXd& /*controls*/,
                                                      const Environment& environment) const {
  Breakdown out;
  const double omega = auxiliary(kMainRotorSpeed);
  out.rotor_speed_rad_s = omega;

  const Eigen::Vector3d rate = state.angular_rate_body_rad_s;
  const Eigen::Vector3d velocity = state.velocity_body_m_s;

  // ---- main rotor -----------------------------------------------------
  //
  // The hub's air-relative velocity includes the body rate's contribution
  // omega x r. A rotor 2.5 m above the CG at 0.3 rad/s of pitch rate sees
  // 0.75 m/s of extra forward velocity, which is not negligible at hover.
  const Eigen::Vector3d main_hub_velocity =
      velocity + rate.cross(main_rotor.position_cg_to_hub_body_m);
  rotor::RotorState main_state;
  main_state.speed_rad_s = omega;
  main_state.inflow_ratio = auxiliary(kMainInflowRatio);
  rotor::RotorControls main_controls;
  main_controls.collective_rad = auxiliary(kCollectivePosition);
  main_controls.longitudinal_cyclic_rad = auxiliary(kLongitudinalCyclicPosition);
  main_controls.lateral_cyclic_rad = auxiliary(kLateralCyclicPosition);
  out.main = rotor::solve_rotor(
      main_rotor, main_state, main_controls, main_hub_velocity, rate, environment.density_kg_m3);

  // ---- tail rotor -----------------------------------------------------
  const Eigen::Vector3d tail_hub_velocity =
      velocity + rate.cross(tail_rotor.position_cg_to_hub_body_m);
  rotor::RotorState tail_state;
  tail_state.speed_rad_s = omega * drivetrain.tail_gear_ratio;
  tail_state.inflow_ratio = auxiliary(kTailInflowRatio);
  rotor::RotorControls tail_controls;
  tail_controls.collective_rad = pedal_to_tail_collective * auxiliary(kPedalPosition);
  out.tail = rotor::solve_rotor(
      tail_rotor, tail_state, tail_controls, tail_hub_velocity, rate, environment.density_kg_m3);

  // Blockage and the failure multiplier scale the tail's FORCE and the moment
  // that force makes, but not its shaft torque: a blocked or damaged tail rotor
  // still costs the drivetrain what it costs.
  const double tail_scale = tail_rotor_blockage_factor * failures.tail_rotor_effectiveness;
  out.tail.thrust_n *= tail_scale;
  out.tail.wrench.force_body_n *= tail_scale;
  // The tail's whole wrench came from its thrust, so scaling the force scales
  // the moment with it. The shaft torque term is the only part that does not,
  // and it is carried separately in `torque_n_m` rather than in this wrench.
  out.tail.wrench.moment_cg_body_n_m *= tail_scale;

  // ---- fuselage -------------------------------------------------------
  //
  // Drag along the air-relative velocity vector, sized by the equivalent flat
  // plate area: D = q f, directed OPPOSITE the velocity. Optional incidence
  // dependence adds a normal force and a pitching moment.
  const double speed = velocity.norm();
  const double dynamic_pressure = 0.5 * environment.density_kg_m3 * speed * speed;
  if (speed > 1.0e-6) {
    const Eigen::Vector3d drag_direction = -velocity / speed;
    out.fuselage.force_body_n = dynamic_pressure * airframe.flat_plate_area_m2 * drag_direction;
    const double alpha = core::angle_of_attack(velocity);
    if (airframe.fuselage_lift_vs_alpha.has_value()) {
      const double lift = dynamic_pressure * airframe.flat_plate_area_m2
                          * airframe.fuselage_lift_vs_alpha->at(alpha);
      out.fuselage.force_body_n += Eigen::Vector3d(0.0, 0.0, -lift);
    }
    if (airframe.fuselage_pitching_moment_vs_alpha.has_value()) {
      out.fuselage.moment_cg_body_n_m +=
          Eigen::Vector3d(0.0,
                          dynamic_pressure * airframe.flat_plate_area_m2 * main_rotor.radius_m
                              * airframe.fuselage_pitching_moment_vs_alpha->at(alpha),
                          0.0);
    }
    out.fuselage.moment_cg_body_n_m +=
        airframe.cg_to_fuselage_reference_body_m.cross(out.fuselage.force_body_n);
  }

  // ---- horizontal stabiliser -----------------------------------------
  //
  // Sees the free-stream plus a declared fraction of the main-rotor downwash.
  // The downwash reduces its local angle of attack, which is exactly why a
  // helicopter's pitch attitude changes through transition as the wake moves
  // off the tail. A constant factor cannot show that; the header says so.
  if (airframe.horizontal_tail_area_m2 > 0.0 && speed > 1.0e-6) {
    const Eigen::Vector3d tail_velocity =
        velocity + rate.cross(airframe.cg_to_horizontal_tail_body_m)
        + Eigen::Vector3d(
            0.0, 0.0, airframe.horizontal_tail_downwash_factor * out.main.induced_velocity_m_s);
    const double local_alpha =
        core::angle_of_attack(tail_velocity) + airframe.horizontal_tail_incidence_rad;
    const double local_q = 0.5 * environment.density_kg_m3 * tail_velocity.squaredNorm();
    const double lift =
        local_q * airframe.horizontal_tail_area_m2
        * surface_coefficient(
            airframe.horizontal_tail_lift_slope, local_alpha, airframe.surface_stall_angle_rad);
    out.horizontal_tail.force_body_n = Eigen::Vector3d(0.0, 0.0, -lift);
    out.horizontal_tail.moment_cg_body_n_m =
        airframe.cg_to_horizontal_tail_body_m.cross(out.horizontal_tail.force_body_n);
  }

  // ---- vertical stabiliser -------------------------------------------
  if (airframe.vertical_tail_area_m2 > 0.0 && speed > 1.0e-6) {
    const Eigen::Vector3d tail_velocity =
        velocity + rate.cross(airframe.cg_to_vertical_tail_body_m);
    const double local_beta =
        core::sideslip_angle(tail_velocity) + airframe.vertical_tail_incidence_rad;
    const double local_q = 0.5 * environment.density_kg_m3 * tail_velocity.squaredNorm();
    const double side =
        local_q * airframe.vertical_tail_area_m2
        * surface_coefficient(
            airframe.vertical_tail_side_slope, local_beta, airframe.surface_stall_angle_rad);
    out.vertical_tail.force_body_n = Eigen::Vector3d(0.0, -side, 0.0);
    out.vertical_tail.moment_cg_body_n_m =
        airframe.cg_to_vertical_tail_body_m.cross(out.vertical_tail.force_body_n);
  }

  // ---- totals ---------------------------------------------------------
  out.total = out.main.wrench;
  out.total += out.tail.wrench;
  out.total += out.fuselage;
  out.total += out.horizontal_tail;
  out.total += out.vertical_tail;

  // ---- drivetrain -----------------------------------------------------
  //
  // The governor holds rotor speed by supplying torque against what the rotors
  // demand. Proportional on the speed error plus a feed-forward of the demand
  // itself, which is what a real turboshaft's fuel control does and what makes
  // the steady droop finite rather than growing with load.
  const double demanded = out.main.torque_n_m + drivetrain.tail_gear_ratio * out.tail.torque_n_m
                          + drivetrain.accessory_torque_n_m;
  const double error = drivetrain.reference_rotor_speed_rad_s - omega;
  out.governor_requested_torque_n_m =
      failures.engine_available_fraction
      * saturate(demanded / std::max(drivetrain.transmission_efficiency, 1.0e-6)
                     + drivetrain.governor_proportional_n_m_s * error,
                 drivetrain.minimum_engine_torque_n_m,
                 drivetrain.maximum_engine_torque_n_m);
  // THE TORQUE THE ENGINE IS ACTUALLY DELIVERING is the state, not the request.
  // A failed engine delivers none of it whatever the state says, which is what
  // makes a flameout instantaneous while a governor correction is not.
  out.engine_torque_n_m = failures.engine_available_fraction
                          * saturate(auxiliary(kEngineTorque),
                                     drivetrain.minimum_engine_torque_n_m,
                                     drivetrain.maximum_engine_torque_n_m);
  out.governor_error_rad_s = error;
  out.total_power_w = out.main.power_w + out.tail.power_w;
  out.anti_torque_residual_n_m = out.total.moment_cg_body_n_m.z();
  return out;
}

sim::Wrench HelicopterModel::wrench(const core::State& state,
                                    const Eigen::VectorXd& auxiliary,
                                    const Eigen::VectorXd& controls,
                                    const Environment& environment) const {
  return breakdown(state, auxiliary, controls, environment).total;
}

Eigen::VectorXd HelicopterModel::auxiliary_derivative(const core::State& state,
                                                      const Eigen::VectorXd& auxiliary,
                                                      const Eigen::VectorXd& controls,
                                                      const Environment& environment) const {
  const Breakdown parts = breakdown(state, auxiliary, controls, environment);
  Eigen::VectorXd rate = Eigen::VectorXd::Zero(kHelicopterAuxCount);

  // ---- rotor speed ----------------------------------------------------
  //
  //   I_R dOmega/dt = eta Q_engine - Q_main - g Q_tail - Q_accessory
  //
  // The tail's torque is referred to the main shaft through the gear ratio.
  // This is the equation that makes collective-induced droop appear, and it is
  // why rotor speed is a STATE and not a constant.
  const double inertia =
      main_rotor.polar_inertia_kg_m2
      + drivetrain.tail_gear_ratio * drivetrain.tail_gear_ratio * tail_rotor.polar_inertia_kg_m2;
  if (inertia > 0.0) {
    const double net = drivetrain.transmission_efficiency * parts.engine_torque_n_m
                       - parts.main.torque_n_m - drivetrain.tail_gear_ratio * parts.tail.torque_n_m
                       - drivetrain.accessory_torque_n_m;
    rate(kMainRotorSpeed) = net / inertia;
  }

  // ---- inflow lags ----------------------------------------------------
  rate(kMainInflowRatio) = parts.main.inflow_rate_per_s;
  rate(kTailInflowRatio) = parts.tail.inflow_rate_per_s;

  // ---- engine torque --------------------------------------------------
  //
  // First order towards what the governor asks for. With no declared time
  // constant the engine is treated as instantaneous, which is a legitimate
  // declared choice and visibly so: the state then tracks the request exactly
  // and no droop appears.
  if (drivetrain.governor_time_constant_s > 0.0) {
    rate(kEngineTorque) = (parts.governor_requested_torque_n_m - auxiliary(kEngineTorque))
                          / drivetrain.governor_time_constant_s;
  }

  // ---- actuators ------------------------------------------------------
  //
  // Command -> position limit -> first-order lag -> rate limit. In that order,
  // because a rate limit applied before the lag limits the wrong quantity, and
  // a position limit applied after the lag lets the state sit outside travel.
  for (int i = 0; i < kHelicopterControlCount; ++i) {
    const auto& limits = actuators[static_cast<std::size_t>(i)];
    const double position = auxiliary(kCollectivePosition + i);
    if (failures.jammed_actuator_rad[static_cast<std::size_t>(i)].has_value()) {
      // A jammed actuator does not move. Not "moves slowly" and not "snaps to
      // the jam position": its rate is zero, and the initial condition is where
      // it jammed.
      rate(kCollectivePosition + i) = 0.0;
      continue;
    }
    const double commanded = saturate(controls(i), limits.minimum_rad, limits.maximum_rad);
    double demanded_rate =
        limits.time_constant_s > 0.0 ? (commanded - position) / limits.time_constant_s : 0.0;
    if (limits.rate_limit_rad_s > 0.0) {
      demanded_rate = saturate(demanded_rate, -limits.rate_limit_rad_s, limits.rate_limit_rad_s);
    }
    // At a position stop, a rate that would push further out is removed. This
    // makes the stop an absorbing boundary rather than a place the state
    // oscillates about.
    if ((position <= limits.minimum_rad && demanded_rate < 0.0)
        || (position >= limits.maximum_rad && demanded_rate > 0.0)) {
      demanded_rate = 0.0;
    }
    rate(kCollectivePosition + i) = demanded_rate;
  }
  return rate;
}

EnvelopeStatus HelicopterModel::envelope(const core::State& state,
                                         const Eigen::VectorXd& auxiliary,
                                         const Eigen::VectorXd& controls,
                                         const Environment& environment) const {
  const Breakdown parts = breakdown(state, auxiliary, controls, environment);
  EnvelopeStatus status;

  const auto note = [&status](bool condition, double departure, const std::string& reason) {
    if (condition && departure > status.worst_departure) {
      status.outside = true;
      status.worst_departure = departure;
      status.reason = reason;
    }
  };

  // Retreating-blade stall and reverse flow, both reported as a fraction over
  // the declared limit so the numbers are comparable.
  const double ct_sigma = parts.main.thrust_coefficient_solidity;
  note(ct_sigma > main_rotor.maximum_thrust_coefficient_solidity,
       ct_sigma / main_rotor.maximum_thrust_coefficient_solidity - 1.0,
       "main-rotor C_T/sigma is " + number(ct_sigma) + " against a declared limit of "
           + number(main_rotor.maximum_thrust_coefficient_solidity)
           + "; there is no retreating-blade stall in this model, so thrust and control power "
             "here are OPTIMISTIC");
  note(parts.main.advance_ratio > main_rotor.maximum_advance_ratio,
       parts.main.advance_ratio / main_rotor.maximum_advance_ratio - 1.0,
       "advance ratio is " + number(parts.main.advance_ratio) + " against a declared limit of "
           + number(main_rotor.maximum_advance_ratio)
           + "; the reverse-flow region is not modelled, so thrust here is OVER-predicted");

  // The vortex-ring state: descending at between roughly a half and one and a
  // half times the hover induced velocity, where momentum theory has no valid
  // solution and this model is qualitatively wrong rather than merely inaccurate.
  const double lambda_h = parts.main.quasi_static_inflow_ratio;
  if (lambda_h > 1.0e-9) {
    const double ratio = parts.main.axial_inflow_ratio / lambda_h;
    note(ratio < -0.5 && ratio > -1.5, 1.0,
         "the rotor is descending at " + number(-ratio)
             + " times its own induced velocity, which is inside the vortex-ring state. Momentum "
               "theory has no solution there and this model is QUALITATIVELY WRONG, not merely "
               "inaccurate");
  }

  const double omega = auxiliary(kMainRotorSpeed);
  if (drivetrain.minimum_rotor_speed_rad_s > 0.0) {
    note(omega < drivetrain.minimum_rotor_speed_rad_s,
         drivetrain.minimum_rotor_speed_rad_s / std::max(omega, 1.0e-6) - 1.0,
         "rotor speed is " + number(omega) + " rad/s, below the declared minimum of "
             + number(drivetrain.minimum_rotor_speed_rad_s));
  }
  if (drivetrain.maximum_rotor_speed_rad_s > 0.0) {
    note(omega > drivetrain.maximum_rotor_speed_rad_s,
         omega / drivetrain.maximum_rotor_speed_rad_s - 1.0,
         "rotor speed is " + number(omega) + " rad/s, above the declared maximum of "
             + number(drivetrain.maximum_rotor_speed_rad_s));
  }

  // Ground proximity. There is no ground effect in this model, so a hover near
  // the ground asks for MORE collective and MORE power than the aircraft needs.
  const double altitude = -state.position_ned_m.z();
  const double diameter = 2.0 * main_rotor.radius_m;
  note(altitude < diameter && altitude > -1.0e6, 1.0 - altitude / diameter,
       "the rotor is within one diameter of the ground (" + number(altitude)
           + " m). There is no ground effect in this model, so thrust here is UNDER-predicted and "
             "the collective and power it reports are too high");
  return status;
}

Eigen::VectorXd HelicopterModel::outputs(const core::State& state,
                                         const Eigen::VectorXd& auxiliary,
                                         const Eigen::VectorXd& controls,
                                         const Environment& environment) const {
  const Breakdown parts = breakdown(state, auxiliary, controls, environment);
  const core::EulerAngles euler = core::euler_from_quaternion(state.attitude_body_to_ned);
  Eigen::VectorXd out(18);
  out << state.position_ned_m.x(), state.position_ned_m.y(), -state.position_ned_m.z(),
      state.velocity_body_m_s.x(), state.velocity_body_m_s.y(), state.velocity_body_m_s.z(),
      euler.roll_rad, euler.pitch_rad, euler.yaw_rad, state.angular_rate_body_rad_s.x(),
      state.angular_rate_body_rad_s.y(), state.angular_rate_body_rad_s.z(),
      core::airspeed(state.velocity_body_m_s), auxiliary(kMainRotorSpeed), parts.main.thrust_n,
      parts.tail.thrust_n, parts.total_power_w, environment.atmospheric_altitude_m;
  return out;
}

numerics::StateBounds HelicopterModel::state_bounds() const {
  const auto names = state_names();
  Eigen::VectorXd limits(static_cast<Eigen::Index>(names.size()));
  // "This is not a helicopter any more" limits, not envelope limits. A run that
  // reaches any of these has diverged, and saying so beats returning a
  // plausible CSV of a divergence.
  limits << 1.0e6, 1.0e6, 1.0e6,  // position, m
      3.0e2, 3.0e2, 3.0e2,        // body velocity, m/s (600 kt is not a helicopter)
      2.0, 2.0, 2.0, 2.0,         // quaternion, renormalised every step
      2.0e1, 2.0e1, 2.0e1,        // body rates, rad/s (1100 deg/s)
      4.0 * std::max(drivetrain.reference_rotor_speed_rad_s, 1.0),  // rotor speed, rad/s
      5.0, 5.0,                                                     // inflow ratios, dimensionless
      3.14, 3.14, 3.14, 3.14,                                       // actuator positions, rad
      4.0 * std::max(drivetrain.maximum_engine_torque_n_m, 1.0);    // engine torque, N m
  return numerics::StateBounds(names, limits);
}

void HelicopterModel::validate() const {
  validate_vocabulary();
  mass.validate();
  main_rotor.validate();
  tail_rotor.validate();

  if (!(tail_rotor_blockage_factor > 0.0) || tail_rotor_blockage_factor > 1.0) {
    throw std::invalid_argument("helicopter: tail_rotor_blockage_factor is "
                                + number(tail_rotor_blockage_factor) + ", must be in (0, 1]");
  }
  if (!std::isfinite(pedal_to_tail_collective) || pedal_to_tail_collective == 0.0) {
    throw std::invalid_argument(
        "helicopter: pedal_to_tail_collective must be finite and non-zero; a zero gearing is a "
        "helicopter with no yaw control, which is a configuration error rather than a design");
  }
  if (!(drivetrain.reference_rotor_speed_rad_s > 0.0)) {
    throw std::invalid_argument("helicopter: reference_rotor_speed_rad_s must be positive");
  }
  if (!(drivetrain.tail_gear_ratio > 0.0)) {
    throw std::invalid_argument("helicopter: tail_gear_ratio must be positive");
  }
  if (!(drivetrain.transmission_efficiency > 0.0) || drivetrain.transmission_efficiency > 1.0) {
    throw std::invalid_argument("helicopter: transmission_efficiency must be in (0, 1]");
  }
  if (drivetrain.minimum_engine_torque_n_m < 0.0) {
    throw std::invalid_argument(
        "helicopter: minimum_engine_torque_n_m must be non-negative; a turboshaft through a "
        "freewheel unit cannot motor the rotor, so a negative floor would let the engine brake it");
  }
  if (!(drivetrain.maximum_engine_torque_n_m > drivetrain.minimum_engine_torque_n_m)) {
    throw std::invalid_argument("helicopter: maximum_engine_torque_n_m must exceed the minimum");
  }
  if (failures.tail_rotor_effectiveness < 0.0 || failures.tail_rotor_effectiveness > 1.0
      || failures.engine_available_fraction < 0.0 || failures.engine_available_fraction > 1.0) {
    throw std::invalid_argument("helicopter: failure fractions must be in [0, 1]");
  }
  for (int i = 0; i < kHelicopterControlCount; ++i) {
    const auto& limits = actuators[static_cast<std::size_t>(i)];
    const std::string what = control_names()[static_cast<std::size_t>(i)];
    if (!std::isfinite(limits.minimum_rad) || !std::isfinite(limits.maximum_rad)
        || !(limits.maximum_rad > limits.minimum_rad)) {
      throw std::invalid_argument("helicopter: actuator '" + what
                                  + "' needs maximum_rad above minimum_rad");
    }
    if (!(limits.rate_limit_rad_s > 0.0)) {
      throw std::invalid_argument(
          "helicopter: actuator '" + what
          + "' needs a positive rate_limit_rad_s. An unlimited rate lets a control law slew to "
            "its stop in one step and look stable doing it");
    }
    if (!(limits.time_constant_s > 0.0)) {
      throw std::invalid_argument("helicopter: actuator '" + what
                                  + "' needs a positive time_constant_s");
    }
  }

  // THE ANTI-TORQUE SENSE CHECK.
  //
  // The main rotor's reaction torque yaws the airframe one way; positive pedal
  // must yaw it the other. Getting this backwards produces an aircraft that
  // trims at the opposite pedal and departs when disturbed, and NOTHING else in
  // the model notices. So it is checked here, numerically, by asking both
  // questions of the model itself rather than by reasoning about signs.
  Environment environment = Environment::sea_level_still_air();
  core::State hover;
  hover.attitude_body_to_ned = core::identity_attitude();
  Eigen::VectorXd auxiliary = Eigen::VectorXd::Zero(kHelicopterAuxCount);
  auxiliary(kMainRotorSpeed) = drivetrain.reference_rotor_speed_rad_s;
  // A representative collective, mid-travel, so the main rotor is producing
  // torque at all.
  auxiliary(kCollectivePosition) =
      0.5 * (actuators[kCollectiveCommand].minimum_rad + actuators[kCollectiveCommand].maximum_rad);
  const Eigen::VectorXd controls = Eigen::VectorXd::Zero(kHelicopterControlCount);

  const Breakdown neutral = breakdown(hover, auxiliary, controls, environment);
  const double main_yaw = neutral.main.wrench.moment_cg_body_n_m.z();

  auxiliary(kPedalPosition) = 0.1;  // rad of pedal, a small positive input
  const Breakdown with_pedal = breakdown(hover, auxiliary, controls, environment);
  const double pedal_yaw =
      with_pedal.tail.wrench.moment_cg_body_n_m.z() - neutral.tail.wrench.moment_cg_body_n_m.z();

  if (std::fabs(main_yaw) > 1.0 && std::fabs(pedal_yaw) > 1.0 && main_yaw * pedal_yaw > 0.0) {
    throw std::invalid_argument(
        "helicopter: the anti-torque sense is wrong. The main rotor's reaction torque yaws the "
        "airframe by " + number(main_yaw) + " N m and positive pedal adds " + number(pedal_yaw)
        + " N m in the SAME direction, so the tail rotor reinforces the torque it exists to "
          "oppose. Check the main rotor's spin_about_shaft, the tail rotor's hub rotation and the "
          "sign of pedal_to_tail_collective. A helicopter with this backwards still trims, at the "
          "opposite pedal, and departs the moment it is disturbed");
  }
}

}  // namespace galata::model
