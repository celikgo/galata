// SPDX-License-Identifier: Apache-2.0
//
// Nonlinear aircraft model from a non-dimensional derivative buildup.
//
// Reference:
//   B. Etkin and L. D. Reid, "Dynamics of Flight: Stability and Control",
//   3rd ed., Wiley, 1996, chapters 3 and 4.
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 2.
//
// Validity envelope: the header's "WHAT THIS IS NOT" block. In short, this is a
// first-order expansion about one flight condition and it has no stall.

#include "galata/model/aircraft.hpp"

#include "galata/core/constants.hpp"
#include "galata/core/frames.hpp"
#include "galata/core/quaternion.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace galata::model {
namespace {

// Below this airspeed the non-dimensionalisation divides by something too close
// to zero to mean anything: qhat = q c / 2V grows without bound. A model asked
// about a stationary aircraft is being asked the wrong question.
constexpr double kMinimumAirspeed = 1.0;  // m/s

double read_number(const YAML::Node& node, const char* key, double fallback) {
  return node[key] ? node[key].as<double>() : fallback;
}

double require_number(const YAML::Node& node, const char* key, const std::string& path) {
  if (!node[key]) {
    throw std::invalid_argument(path + ": missing required key '" + std::string(key) + "'");
  }
  return node[key].as<double>();
}

}  // namespace

AeroDerivatives lateral_stability_to_body(const AeroDerivatives& stability, double alpha_rad) {
  const double c = std::cos(alpha_rad);
  const double s = std::sin(alpha_rad);

  AeroDerivatives body = stability;

  // The y-axis is shared, so the side force is untouched by the rotation.
  // Its rate and control derivatives likewise map straight across.

  // Moment coefficients rotate as a vector about y.
  body.rolling_moment_beta = stability.rolling_moment_beta * c - stability.yawing_moment_beta * s;
  body.yawing_moment_beta = stability.rolling_moment_beta * s + stability.yawing_moment_beta * c;

  body.rolling_moment_aileron =
      stability.rolling_moment_aileron * c - stability.yawing_moment_aileron * s;
  body.yawing_moment_aileron =
      stability.rolling_moment_aileron * s + stability.yawing_moment_aileron * c;
  body.rolling_moment_rudder =
      stability.rolling_moment_rudder * c - stability.yawing_moment_rudder * s;
  body.yawing_moment_rudder =
      stability.rolling_moment_rudder * s + stability.yawing_moment_rudder * c;

  // The rate derivatives rotate twice: once because the moment rotates, and
  // once because the rate the derivative is taken with respect to rotates.
  const double lp = stability.rolling_moment_roll_rate;
  const double lr = stability.rolling_moment_yaw_rate;
  const double np = stability.yawing_moment_roll_rate;
  const double nr = stability.yawing_moment_yaw_rate;

  body.rolling_moment_roll_rate = lp * c * c - np * s * c - lr * s * c + nr * s * s;
  body.rolling_moment_yaw_rate = lp * s * c - np * s * s + lr * c * c - nr * s * c;
  body.yawing_moment_roll_rate = lp * s * c + np * c * c - lr * s * s - nr * s * c;
  body.yawing_moment_yaw_rate = lp * s * s + np * s * c + lr * s * c + nr * c * c;

  return body;
}

Eigen::VectorXd Controls::to_vector() const {
  Eigen::VectorXd u(kSize);
  u << elevator_rad, aileron_rad, rudder_rad, thrust_n;
  return u;
}

Controls Controls::from_vector(const Eigen::VectorXd& u) {
  if (u.size() != kSize) {
    throw std::invalid_argument("Controls::from_vector: wrong length");
  }
  Controls controls;
  controls.elevator_rad = u(0);
  controls.aileron_rad = u(1);
  controls.rudder_rad = u(2);
  controls.thrust_n = u(3);
  return controls;
}

void Aircraft::validate() const {
  mass.validate();

  if (!(geometry.wing_area_m2 > 0.0)) {
    throw std::invalid_argument("Aircraft: wing area must be positive");
  }
  if (!(geometry.wing_span_m > 0.0)) {
    throw std::invalid_argument("Aircraft: wing span must be positive");
  }
  if (!(geometry.mean_aerodynamic_chord_m > 0.0)) {
    throw std::invalid_argument("Aircraft: mean aerodynamic chord must be positive");
  }
  // A chord longer than the span is a units mix-up, not an aircraft.
  if (geometry.mean_aerodynamic_chord_m > geometry.wing_span_m) {
    std::ostringstream message;
    message << "Aircraft: mean aerodynamic chord (" << geometry.mean_aerodynamic_chord_m
            << " m) exceeds the span (" << geometry.wing_span_m
            << " m). One of them is probably in the wrong units.";
    throw std::invalid_argument(message.str());
  }

  if (aero.lift_alpha_dot != 0.0 || aero.drag_alpha_dot != 0.0) {
    throw std::invalid_argument(
        "Aircraft: a non-zero alpha-dot FORCE derivative (C_L_alphadot or C_D_alphadot) makes "
        "the model implicit — the vertical acceleration would depend on alphadot, which "
        "depends on the vertical acceleration. Solving that is not implemented, and this is "
        "rejected rather than silently dropped. C_m_alphadot is supported.");
  }

  // A positive lift-curve slope is not a matter of taste. A sign error here
  // produces an aircraft that trims at a negative angle of attack and whose
  // short period diverges, which looks like a physics discovery.
  if (!(aero.lift_alpha > 0.0)) {
    throw std::invalid_argument(
        "Aircraft: the lift-curve slope C_L_alpha must be positive. A negative value is a sign "
        "error, not an exotic configuration.");
  }
  if (!(aero.pitching_moment_elevator != 0.0)) {
    throw std::invalid_argument(
        "Aircraft: C_m_elevator is zero, so the elevator has no pitching authority and no "
        "longitudinal trim exists.");
  }
}

EnvelopeWarning Aircraft::envelope(const core::State& state,
                                   const core::AtmosphereState& atmosphere) const {
  const core::WindAxisVelocity wind = core::to_wind_axes(state.velocity_body_m_s);
  EnvelopeWarning warning;
  warning.alpha_departure_rad = std::fabs(wind.alpha_rad - aero.reference_alpha_rad);
  const double mach = (atmosphere.speed_of_sound_m_s > 0.0)
                          ? wind.airspeed_m_s / atmosphere.speed_of_sound_m_s
                          : 0.0;
  warning.mach_departure = std::fabs(mach - aero.reference_mach);
  warning.outside_advisory_envelope =
      warning.alpha_departure_rad > EnvelopeWarning::kAdvisoryAlphaLimitRad
      || warning.mach_departure > EnvelopeWarning::kAdvisoryMachLimit;
  return warning;
}

sim::Wrench Aircraft::wrench(const core::State& state,
                             const Controls& controls,
                             const core::AtmosphereState& atmosphere,
                             double alpha_dot_rad_s) const {
  const core::WindAxisVelocity wind = core::to_wind_axes(state.velocity_body_m_s);
  const double speed = wind.airspeed_m_s;
  if (!(speed >= kMinimumAirspeed)) {
    std::ostringstream message;
    message << "Aircraft::wrench: airspeed is " << speed
            << " m/s. The rate non-dimensionalisation divides by it, so this model is not "
               "defined at rest.";
    throw std::runtime_error(message.str());
  }

  const double dynamic_pressure = 0.5 * atmosphere.density_kg_m3 * speed * speed;
  const double reference_force = dynamic_pressure * geometry.wing_area_m2;

  // Non-dimensional rates, ADR-0002.
  const double half_span_over_speed = geometry.wing_span_m / (2.0 * speed);
  const double half_chord_over_speed = geometry.mean_aerodynamic_chord_m / (2.0 * speed);
  const double roll_hat = state.angular_rate_body_rad_s.x() * half_span_over_speed;
  const double pitch_hat = state.angular_rate_body_rad_s.y() * half_chord_over_speed;
  const double yaw_hat = state.angular_rate_body_rad_s.z() * half_span_over_speed;
  const double alpha_dot_hat = alpha_dot_rad_s * half_chord_over_speed;

  const double delta_alpha = wind.alpha_rad - aero.reference_alpha_rad;
  const double beta = wind.beta_rad;

  const double lift = aero.lift_ref + aero.lift_alpha * delta_alpha
                      + aero.lift_pitch_rate * pitch_hat
                      + aero.lift_elevator * controls.elevator_rad;
  const double drag =
      aero.drag_ref + aero.drag_alpha * delta_alpha + aero.drag_elevator * controls.elevator_rad;
  const double side = aero.side_force_beta * beta + aero.side_force_aileron * controls.aileron_rad
                      + aero.side_force_rudder * controls.rudder_rad;

  const double roll = aero.rolling_moment_beta * beta + aero.rolling_moment_roll_rate * roll_hat
                      + aero.rolling_moment_yaw_rate * yaw_hat
                      + aero.rolling_moment_aileron * controls.aileron_rad
                      + aero.rolling_moment_rudder * controls.rudder_rad;
  const double pitch = aero.pitching_moment_ref + aero.pitching_moment_alpha * delta_alpha
                       + aero.pitching_moment_pitch_rate * pitch_hat
                       + aero.pitching_moment_alpha_dot * alpha_dot_hat
                       + aero.pitching_moment_elevator * controls.elevator_rad;
  const double yaw = aero.yawing_moment_beta * beta + aero.yawing_moment_roll_rate * roll_hat
                     + aero.yawing_moment_yaw_rate * yaw_hat
                     + aero.yawing_moment_aileron * controls.aileron_rad
                     + aero.yawing_moment_rudder * controls.rudder_rad;

  // Lift and drag are referred to the STABILITY axes — rotated from body by
  // alpha alone — and the side force is given directly in body axes. That is
  // the convention every published derivative set this project consumes uses,
  // and it is NOT the full wind-axis rotation.
  //
  // The difference is not cosmetic. Rotating the (-D, Y, -L) triple through
  // dcm_body_from_wind adds a -D sin(beta) term to the body y force, which is
  // a real force but one that a body-axis C_Y_beta already contains. Doing
  // both double-counts it: it inflated this model's side-force derivative from
  // -0.125 to -0.148, an 19% error, and with it the Dutch roll damping.
  const Eigen::Vector3d lift_drag_stability(-drag * reference_force, 0.0, -lift * reference_force);
  sim::Wrench aerodynamic;
  aerodynamic.force_body_n = core::dcm_body_from_stability(wind.alpha_rad) * lift_drag_stability;
  aerodynamic.force_body_n.y() += side * reference_force;
  aerodynamic.moment_cg_body_n_m =
      Eigen::Vector3d(roll * reference_force * geometry.wing_span_m,
                      pitch * reference_force * geometry.mean_aerodynamic_chord_m,
                      yaw * reference_force * geometry.wing_span_m);

  // Move the aerodynamic wrench from its reference point to the CG. This is
  // where a CG sweep does its work (ADR-0006).
  aerodynamic = sim::moved_to_cg(aerodynamic, cg_to_aero_reference_m);

  sim::Wrench propulsive;
  propulsive.force_body_n = Eigen::Vector3d(controls.thrust_n * std::cos(thrust_incidence_rad),
                                            0.0,
                                            -controls.thrust_n * std::sin(thrust_incidence_rad));

  return aerodynamic + propulsive;
}

core::StateVector Aircraft::derivative(const core::State& state,
                                       const Controls& controls,
                                       double delta_isa_k) const {
  const double altitude_m = -state.position_ned_m.z();
  const core::AtmosphereState atmosphere = core::isa(altitude_m, delta_isa_k);
  const Eigen::Vector3d gravity_ned(0.0, 0.0, core::kStandardGravity);

  // Pass one: no alpha-dot term. validate() has established that no FORCE
  // derivative depends on alphadot, so the translational accelerations this
  // produces are already final.
  const sim::Wrench without_lag = wrench(state, controls, atmosphere, 0.0);
  const core::StateVector first = sim::rigid_body_derivative(state, mass, without_lag, gravity_ned);

  if (aero.pitching_moment_alpha_dot == 0.0) {
    return first;
  }

  // alphadot = d/dt atan2(w, u) = (u wdot - w udot) / (u^2 + w^2).
  const double u = state.velocity_body_m_s.x();
  const double w = state.velocity_body_m_s.z();
  const double u_dot = first(core::kVelocityU);
  const double w_dot = first(core::kVelocityW);
  const double denominator = u * u + w * w;
  const double alpha_dot = (denominator > 0.0) ? (u * w_dot - w * u_dot) / denominator : 0.0;

  // Pass two: the same wrench with the downwash-lag term, which affects the
  // pitching moment only and therefore changes only the rotational equation.
  const sim::Wrench with_lag = wrench(state, controls, atmosphere, alpha_dot);
  return sim::rigid_body_derivative(state, mass, with_lag, gravity_ned);
}

Aircraft load_aircraft(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::invalid_argument("cannot open aircraft file: " + path);
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();

  YAML::Node root;
  try {
    root = YAML::Load(buffer.str());
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument(path + ": not valid YAML: " + error.what());
  }
  if (!root.IsMap()) {
    throw std::invalid_argument(path + ": the document must be a map");
  }

  Aircraft aircraft;
  aircraft.description = root["description"] ? root["description"].Scalar() : std::string{};
  aircraft.citation = root["citation"] ? root["citation"].Scalar() : std::string{};

  const YAML::Node geometry = root["geometry"];
  if (!geometry) {
    throw std::invalid_argument(path + ": missing 'geometry'");
  }
  aircraft.geometry.wing_area_m2 = require_number(geometry, "wing_area_m2", path);
  aircraft.geometry.wing_span_m = require_number(geometry, "wing_span_m", path);
  aircraft.geometry.mean_aerodynamic_chord_m =
      require_number(geometry, "mean_aerodynamic_chord_m", path);

  const YAML::Node mass = root["mass"];
  if (!mass) {
    throw std::invalid_argument(path + ": missing 'mass'");
  }
  aircraft.mass.mass_kg = require_number(mass, "mass_kg", path);
  const double ixx = require_number(mass, "inertia_xx_kg_m2", path);
  const double iyy = require_number(mass, "inertia_yy_kg_m2", path);
  const double izz = require_number(mass, "inertia_zz_kg_m2", path);
  // Aerospace convention: a positive quoted product of inertia I_xz appears as
  // a NEGATIVE off-diagonal entry. Handled here so that a model file can quote
  // the number its source prints.
  const double ixz = read_number(mass, "product_of_inertia_xz_kg_m2", 0.0);
  aircraft.mass.inertia_cg_body_kg_m2 << ixx, 0.0, -ixz,  //
      0.0, iyy, 0.0,                                      //
      -ixz, 0.0, izz;

  const YAML::Node a = root["aero"];
  if (!a) {
    throw std::invalid_argument(path + ": missing 'aero'");
  }
  AeroDerivatives& d = aircraft.aero;
  d.reference_alpha_rad = require_number(a, "reference_alpha_rad", path);
  d.reference_mach = read_number(a, "reference_mach", 0.0);
  d.lift_ref = require_number(a, "lift_ref", path);
  d.drag_ref = require_number(a, "drag_ref", path);
  d.pitching_moment_ref = read_number(a, "pitching_moment_ref", 0.0);
  d.lift_alpha = require_number(a, "lift_alpha", path);
  d.drag_alpha = read_number(a, "drag_alpha", 0.0);
  d.pitching_moment_alpha = require_number(a, "pitching_moment_alpha", path);
  d.lift_pitch_rate = read_number(a, "lift_pitch_rate", 0.0);
  d.pitching_moment_pitch_rate = read_number(a, "pitching_moment_pitch_rate", 0.0);
  d.lift_alpha_dot = read_number(a, "lift_alpha_dot", 0.0);
  d.drag_alpha_dot = read_number(a, "drag_alpha_dot", 0.0);
  d.pitching_moment_alpha_dot = read_number(a, "pitching_moment_alpha_dot", 0.0);
  d.lift_elevator = read_number(a, "lift_elevator", 0.0);
  d.drag_elevator = read_number(a, "drag_elevator", 0.0);
  d.pitching_moment_elevator = require_number(a, "pitching_moment_elevator", path);
  d.side_force_beta = read_number(a, "side_force_beta", 0.0);
  d.rolling_moment_beta = read_number(a, "rolling_moment_beta", 0.0);
  d.yawing_moment_beta = read_number(a, "yawing_moment_beta", 0.0);
  d.rolling_moment_roll_rate = read_number(a, "rolling_moment_roll_rate", 0.0);
  d.yawing_moment_roll_rate = read_number(a, "yawing_moment_roll_rate", 0.0);
  d.rolling_moment_yaw_rate = read_number(a, "rolling_moment_yaw_rate", 0.0);
  d.yawing_moment_yaw_rate = read_number(a, "yawing_moment_yaw_rate", 0.0);
  d.side_force_aileron = read_number(a, "side_force_aileron", 0.0);
  d.rolling_moment_aileron = read_number(a, "rolling_moment_aileron", 0.0);
  d.yawing_moment_aileron = read_number(a, "yawing_moment_aileron", 0.0);
  d.side_force_rudder = read_number(a, "side_force_rudder", 0.0);
  d.rolling_moment_rudder = read_number(a, "rolling_moment_rudder", 0.0);
  d.yawing_moment_rudder = read_number(a, "yawing_moment_rudder", 0.0);

  aircraft.thrust_incidence_rad = read_number(root, "thrust_incidence_rad", 0.0);

  // Which axes the lateral-directional derivatives are given in. Required
  // rather than defaulted: a set silently assumed to be body-axis when it is
  // stability-axis produces a Dutch roll damping tens of percent wrong, and
  // there is no way to tell from the numbers alone which was meant.
  if (!a["lateral_axes"]) {
    throw std::invalid_argument(
        path +
        ": 'aero.lateral_axes' is required and must be 'body' or 'stability'. Published "
        "derivative sets are very often stability-axis while the equations of motion are "
        "body-axis, the difference is tens of percent in the Dutch roll damping, and it "
        "cannot be inferred from the values. State it.");
  }
  const std::string lateral_axes = a["lateral_axes"].Scalar();
  if (lateral_axes == "stability") {
    aircraft.aero = lateral_stability_to_body(aircraft.aero, aircraft.aero.reference_alpha_rad);
  } else if (lateral_axes != "body") {
    throw std::invalid_argument(path + ": 'aero.lateral_axes' is '" + lateral_axes
                                + "'; it must be 'body' or 'stability'");
  }

  aircraft.validate();
  return aircraft;
}

}  // namespace galata::model
