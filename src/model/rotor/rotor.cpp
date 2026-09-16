// SPDX-License-Identifier: Apache-2.0
//
// Level-1 rotor: coupled blade-element/momentum thrust and inflow, quasi-static
// flapping, and a wrench resolved through the rotor's own frame.
//
// Reference:
//   G. D. Padfield, "Helicopter Flight Dynamics", 2nd ed., Blackwell, 2007,
//   chapter 3 — equations 3.135 onwards for the thrust coefficient and the
//   multi-blade flapping response used here.
//   W. Johnson, "Helicopter Theory", Princeton, 1980, sections 2.1 and 4.4.
//   J. G. Leishman, "Principles of Helicopter Aerodynamics", 2nd ed.,
//   Cambridge, 2006, sections 2.14 and 3.4.
//   H. Glauert, ARC R&M 1111, 1926.
//
// Validity envelope and known error direction are in the header's
// "WHAT THIS IS NOT" block, in full. In summary: valid to about mu = 0.35 and
// below the declared C_T/sigma; OVER-predicts thrust above either, because
// neither reverse flow nor retreating-blade stall is modelled; UNDER-predicts
// in ground effect; and is qualitatively wrong in the vortex-ring state.
//
// DETERMINISM. The inflow solve is a FIXED number of Newton iterations with no
// residual test, so the same inputs perform the same arithmetic in the same
// order on every run (ADR-0004). The count is a compile-time constant rather
// than a parameter because it is a property of this formulation, not a knob:
// Newton on an affine-in-lambda thrust relation converges quadratically, so six
// iterations from the hover-momentum starting guess reach machine precision and
// sixteen is pure margin. A test MEASURES the residual actually achieved against
// the momentum relation rather than assuming it.

#include "galata/model/rotor/rotor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>

namespace galata::model::rotor {
namespace {

// Fixed-point iterations for the coupled thrust/inflow solve. See the file
// header for why this is a constant and not a tolerance.
constexpr int kInflowIterations = 16;  // Newton; six suffice, the rest is margin

// Below this rotor speed the non-dimensional formulation is meaningless: mu and
// lambda both divide by Omega*R. A stopped rotor produces no thrust and no
// torque, which is the physically right answer and also the numerically safe
// one.
constexpr double kMinimumTipSpeedM_S = 1.0e-3;

std::string number(double value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(6) << value;
  return out.str();
}

void require_finite(double value, const std::string& name, const std::string& what) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument("rotor '" + name + "': " + what + " must be finite");
  }
}

}  // namespace

double RotorGeometry::solidity() const {
  if (!(radius_m > 0.0) || blade_count <= 0) {
    return 0.0;
  }
  return static_cast<double>(blade_count) * chord_m / (std::numbers::pi * radius_m);
}

double RotorGeometry::disc_area_m2() const {
  return std::numbers::pi * radius_m * radius_m;
}

void RotorGeometry::validate() const {
  if (name.empty()) {
    throw std::invalid_argument(
        "rotor: a rotor needs a name; it is what the state names and the report are built from");
  }
  require_finite(radius_m, name, "radius_m");
  require_finite(chord_m, name, "chord_m");
  require_finite(lift_curve_slope, name, "lift_curve_slope");
  require_finite(profile_drag_coefficient, name, "profile_drag_coefficient");
  require_finite(induced_power_factor, name, "induced_power_factor");
  require_finite(blade_twist_rad, name, "blade_twist_rad");
  require_finite(tip_loss_factor, name, "tip_loss_factor");
  require_finite(hinge_offset_m, name, "hinge_offset_m");
  require_finite(flap_stiffness_n_m_rad, name, "flap_stiffness_n_m_rad");
  require_finite(blade_flap_inertia_kg_m2, name, "blade_flap_inertia_kg_m2");
  require_finite(polar_inertia_kg_m2, name, "polar_inertia_kg_m2");
  require_finite(inflow_time_constant_s, name, "inflow_time_constant_s");
  require_finite(maximum_thrust_coefficient_solidity, name,
                 "maximum_thrust_coefficient_solidity");
  require_finite(maximum_advance_ratio, name, "maximum_advance_ratio");
  if (!position_cg_to_hub_body_m.allFinite() || !hub_to_body.allFinite()) {
    throw std::invalid_argument("rotor '" + name + "': hub position and rotation must be finite");
  }
  if (!(radius_m > 0.0)) {
    throw std::invalid_argument("rotor '" + name + "': radius_m must be positive");
  }
  if (!(chord_m > 0.0)) {
    throw std::invalid_argument("rotor '" + name + "': chord_m must be positive");
  }
  if (blade_count < 2) {
    throw std::invalid_argument(
        "rotor '" + name + "': blade_count is " + std::to_string(blade_count)
        + "; the multi-blade flapping coordinates this model uses need at least two blades");
  }
  if (!(lift_curve_slope > 0.0)) {
    throw std::invalid_argument("rotor '" + name + "': lift_curve_slope must be positive");
  }
  if (profile_drag_coefficient < 0.0) {
    throw std::invalid_argument("rotor '" + name
                                + "': profile_drag_coefficient must be non-negative");
  }
  if (!(induced_power_factor >= 1.0)) {
    throw std::invalid_argument(
        "rotor '" + name + "': induced_power_factor is " + number(induced_power_factor)
        + ", must be at least 1. Momentum theory's ideal induced power is a lower bound, so a "
          "factor below one claims a rotor better than the momentum limit");
  }
  if (!(tip_loss_factor > 0.0) || tip_loss_factor > 1.0) {
    throw std::invalid_argument("rotor '" + name + "': tip_loss_factor is "
                                + number(tip_loss_factor) + ", must be in (0, 1]");
  }
  if (spin_about_shaft != 1 && spin_about_shaft != -1) {
    throw std::invalid_argument("rotor '" + name + "': spin_about_shaft must be +1 or -1");
  }
  if (hinge_offset_m < 0.0 || hinge_offset_m >= radius_m) {
    throw std::invalid_argument("rotor '" + name + "': hinge_offset_m is " + number(hinge_offset_m)
                                + ", must be in [0, radius)");
  }
  if (flap_stiffness_n_m_rad < 0.0 || blade_flap_inertia_kg_m2 < 0.0
      || polar_inertia_kg_m2 < 0.0) {
    throw std::invalid_argument("rotor '" + name
                                + "': flap stiffness and the two inertias must be non-negative");
  }
  if (inflow_time_constant_s < 0.0) {
    throw std::invalid_argument("rotor '" + name + "': inflow_time_constant_s must be non-negative");
  }
  if (!(maximum_advance_ratio > 0.0) || !(maximum_thrust_coefficient_solidity > 0.0)) {
    throw std::invalid_argument("rotor '" + name + "': the declared envelope limits must be positive");
  }

  // A NEAR-ROTATION IS NOT A ROTATION, and the failure is silent. A hub matrix
  // that is 1% off orthonormal scales every force it resolves by 1% and tilts
  // it by half a degree, and the trim that results is a perfectly convergent
  // answer to the wrong problem.
  const Eigen::Matrix3d should_be_identity = hub_to_body.transpose() * hub_to_body;
  const double orthonormality =
      (should_be_identity - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
  if (orthonormality > 1.0e-9) {
    throw std::invalid_argument(
        "rotor '" + name + "': hub_to_body is not orthonormal (worst entry of R^T R - I is "
        + number(orthonormality)
        + "). A near-rotation silently scales and tilts every force it resolves");
  }
  if (hub_to_body.determinant() < 0.0) {
    throw std::invalid_argument(
        "rotor '" + name
        + "': hub_to_body has determinant -1; it is a reflection, not a rotation, and it would "
          "mirror the rotor's sense of rotation");
  }
}

double hover_induced_velocity_m_s(const RotorGeometry& geometry,
                                  double thrust_n,
                                  double density_kg_m3) {
  if (!(density_kg_m3 > 0.0)) {
    throw std::invalid_argument("rotor: density must be positive");
  }
  if (thrust_n <= 0.0) {
    return 0.0;
  }
  return std::sqrt(thrust_n / (2.0 * density_kg_m3 * geometry.disc_area_m2()));
}

Eigen::Vector3d rotor_angular_momentum_body(const RotorGeometry& geometry,
                                            const RotorState& state) {
  // Along +z_hub when spin is +1, magnitude I_R * Omega.
  const Eigen::Vector3d h_hub(0.0, 0.0,
                              static_cast<double>(geometry.spin_about_shaft)
                                  * geometry.polar_inertia_kg_m2 * state.speed_rad_s);
  return geometry.hub_to_body * h_hub;
}

Eigen::Vector3d gyroscopic_moment_body(const RotorGeometry& geometry,
                                       const RotorState& state,
                                       const Eigen::Vector3d& body_rate_rad_s) {
  // The airframe feels the reaction to the rate of change of the rotor's
  // angular momentum as seen from the rotating body: -(omega x h).
  return -body_rate_rad_s.cross(rotor_angular_momentum_body(geometry, state));
}

Eigen::Matrix3d main_rotor_hub(double forward_tilt_rad, double lateral_tilt_rad) {
  // Shaft tilted forward by `forward_tilt_rad` (rotation about body y) and then
  // laterally by `lateral_tilt_rad` (rotation about body x). With both zero the
  // result is the identity and thrust acts along body -z, which reproduces the
  // multirotor convention exactly.
  // A POSITIVE forward tilt must give a FORWARD force. R_y(theta) maps
  // (0, 0, -T) to (-T sin theta, 0, -T cos theta), so the rotation angle is the
  // NEGATIVE of the tilt. Likewise a positive lateral tilt (to starboard) needs
  // R_x(+lateral) to send the thrust vector towards +y.
  const Eigen::AngleAxisd pitch(-forward_tilt_rad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd roll(lateral_tilt_rad, Eigen::Vector3d::UnitX());
  return (roll * pitch).toRotationMatrix();
}

Eigen::Matrix3d tail_rotor_hub(bool thrust_towards_starboard) {
  // Thrust acts along -z_hub. To put it along +y_body (starboard) the hub frame
  // is rotated -90 degrees about body x: z_hub -> -y_body, so -z_hub -> +y_body.
  // R_x(+90) maps (0, 0, -T) to (0, +T, 0): thrust to starboard. The opposite
  // sign puts it to port. A test asserts both directions rather than trusting
  // the derivation, because a transposed rotation here yaws the aircraft the
  // wrong way and still trims.
  const double angle = thrust_towards_starboard ? std::numbers::pi / 2.0 : -std::numbers::pi / 2.0;
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

RotorSolution solve_rotor(const RotorGeometry& geometry,
                          const RotorState& state,
                          const RotorControls& controls,
                          const Eigen::Vector3d& velocity_hub_body_m_s,
                          const Eigen::Vector3d& body_rate_rad_s,
                          double density_kg_m3) {
  if (!(density_kg_m3 > 0.0) || !std::isfinite(density_kg_m3)) {
    throw std::invalid_argument("rotor '" + geometry.name + "': density must be positive and finite");
  }
  if (!velocity_hub_body_m_s.allFinite() || !body_rate_rad_s.allFinite()) {
    throw std::invalid_argument("rotor '" + geometry.name
                                + "': hub velocity and body rate must be finite");
  }
  if (!std::isfinite(state.speed_rad_s) || !std::isfinite(state.inflow_ratio)
      || !std::isfinite(controls.collective_rad) || !std::isfinite(controls.longitudinal_cyclic_rad)
      || !std::isfinite(controls.lateral_cyclic_rad)) {
    throw std::invalid_argument("rotor '" + geometry.name
                                + "': speed, inflow and controls must be finite");
  }

  RotorSolution solution;
  const double omega = std::fabs(state.speed_rad_s);
  const double radius = geometry.radius_m;
  const double tip_speed = omega * radius;

  // A STOPPED ROTOR PRODUCES NOTHING, and says so by returning zeros rather
  // than dividing by a tip speed of zero. This is the physically correct answer
  // and it is also what makes a rotor-speed-run-down simulation well posed.
  if (tip_speed < kMinimumTipSpeedM_S) {
    return solution;
  }

  const double area = geometry.disc_area_m2();
  const double sigma = geometry.solidity();
  const double a = geometry.lift_curve_slope;
  const double tip_loss = geometry.tip_loss_factor;

  // ---- 1. Hub velocity in HUB axes ------------------------------------
  const Eigen::Vector3d velocity_hub = geometry.hub_to_body.transpose() * velocity_hub_body_m_s;

  // mu is the in-plane component, lambda_c the through-disc component. The disc
  // normal is -z_hub, so a POSITIVE lambda_c (climb) corresponds to velocity
  // along -z_hub, hence the minus sign.
  const double in_plane = std::hypot(velocity_hub.x(), velocity_hub.y());
  const double mu = in_plane / tip_speed;
  const double lambda_c = -velocity_hub.z() / tip_speed;
  solution.advance_ratio = mu;
  solution.axial_inflow_ratio = lambda_c;

  // ---- 2. Coupled blade-element / momentum solve ----------------------
  //
  // Blade element, uniform inflow, linear twist, tip loss B (Padfield 3.135):
  //   C_T = (a sigma / 2) [ B^3 theta_0 / 3 + B mu^2 theta_0 / 2
  //                       + B^4 theta_tw / 4 + B^2 mu^2 theta_tw / 4
  //                       - B^2 lambda / 2 ]
  // with lambda = lambda_c + lambda_i the total inflow ratio.
  //
  // Momentum (Glauert, in forward flight):
  //   lambda_i = C_T / (2 sqrt(mu^2 + lambda^2))
  //
  // Solved by fixed-point iteration on lambda_i, with the blade-element
  // relation supplying C_T at each step. Under-relaxed at 0.5, which is what
  // makes the iteration contract in the axial-flight limit where the naive map
  // has a derivative near -1.
  const double b1 = tip_loss;
  const double b2 = b1 * b1;
  const double b3 = b2 * b1;
  const double b4 = b3 * b1;
  const double theta0 = controls.collective_rad;
  const double twist = geometry.blade_twist_rad;

  const double thrust_constant = 0.5 * a * sigma;
  const double collective_term = b3 * theta0 / 3.0 + 0.5 * b1 * mu * mu * theta0;
  const double twist_term = b4 * twist / 4.0 + 0.25 * b2 * mu * mu * twist;
  const double inflow_gain = 0.5 * b2;  // multiplies -lambda

  // The QUASI-STATIC inflow: the value the momentum balance settles at for this
  // collective and this flight condition. It is what a rotor with no inflow lag
  // uses directly, and what a rotor WITH a lag relaxes towards.
  // C_T is AFFINE in the total inflow ratio: C_T(lambda) = k (A - g lambda).
  // That is what makes the residual below differentiable in closed form, and it
  // is why this is solved by Newton rather than by the under-relaxed fixed
  // point a first draft used. The fixed point converges linearly at a rate that
  // depends on the flight condition — near hover it is slow enough that sixteen
  // iterations still leave a residual of 2e-3 relative, which a test measured
  // and which is far too coarse for a trim to sit on. Newton on the same
  // equation converges quadratically and reaches machine precision in six.
  //
  //   f(lambda_i) = lambda_i - C_T / (2 sqrt(mu^2 + lambda^2)),  lambda = lambda_c + lambda_i
  //   f'(lambda_i) = 1 + k g / D + 4 lambda C_T / D^3,   D = 2 sqrt(mu^2 + lambda^2)
  //
  // Every term of f' is positive for positive thrust and inflow, so the
  // iteration is well conditioned and cannot divide by a vanishing derivative.
  const auto thrust_at = [&](double lambda) {
    return thrust_constant * (collective_term + twist_term - inflow_gain * lambda);
  };

  // FIRST GUESS IS THE HOVER MOMENTUM SOLUTION, lambda_h = sqrt(C_T/2), which is
  // the right scale in every condition and — unlike zero — never puts the first
  // evaluation at the D = 0 singularity that hover has at lambda = mu = 0.
  const double thrust_at_zero_inflow = thrust_at(lambda_c);
  const double hover_scale =
      thrust_at_zero_inflow > 0.0 ? std::sqrt(0.5 * thrust_at_zero_inflow) : 0.0;
  double lambda_quasi_static =
      (std::isfinite(state.inflow_ratio) && state.inflow_ratio > 0.0) ? state.inflow_ratio
                                                                     : hover_scale;
  // A rotor at negative thrust has no momentum-theory inflow in this form.
  // Holding the inflow at zero is the declared treatment, not an approximation
  // to a solution that exists.
  if (!(thrust_at_zero_inflow > 0.0)) {
    lambda_quasi_static = 0.0;
  } else {
    for (int iteration = 0; iteration < kInflowIterations; ++iteration) {
      const double lambda = lambda_c + lambda_quasi_static;
      const double thrust = thrust_at(lambda);
      const double denominator = 2.0 * std::sqrt(mu * mu + lambda * lambda);
      if (!(denominator > 1.0e-12)) {
        // Only reachable if mu and lambda both vanish, which needs zero thrust.
        lambda_quasi_static = 0.0;
        break;
      }
      const double residual = lambda_quasi_static - thrust / denominator;
      const double slope = 1.0 + thrust_constant * inflow_gain / denominator
                           + 4.0 * lambda * thrust / (denominator * denominator * denominator);
      if (!std::isfinite(residual) || !std::isfinite(slope) || !(std::fabs(slope) > 1.0e-12)) {
        throw std::runtime_error(
            "rotor '" + geometry.name
            + "': the inflow solve produced a non-finite or degenerate step. This is the usual "
              "symptom of the vortex-ring state, where momentum theory has no valid solution");
      }
      lambda_quasi_static -= residual / slope;
    }
  }
  double thrust_coefficient = 0.0;

  // WHICH INFLOW THE THRUST IS BUILT ON, and why the distinction is not
  // cosmetic. With a declared lag the INFLOW STATE is the physical one: the
  // wake takes time to build, so a collective step produces a thrust overshoot
  // that decays as the inflow catches up. Using the quasi-static value here
  // would make the state an output that nothing reads, and the lag would be a
  // parameter with no effect — which is worse than not offering one.
  const bool has_inflow_lag = geometry.inflow_time_constant_s > 0.0;
  const double lambda_i = has_inflow_lag ? state.inflow_ratio : lambda_quasi_static;
  const double lambda_total = lambda_c + lambda_i;
  thrust_coefficient =
      thrust_constant * (collective_term + twist_term - inflow_gain * lambda_total);

  const double dynamic = density_kg_m3 * area * tip_speed * tip_speed;
  const double thrust_n = thrust_coefficient * dynamic;

  solution.thrust_coefficient = thrust_coefficient;
  solution.induced_inflow_ratio = lambda_i;
  solution.induced_velocity_m_s = lambda_i * tip_speed;
  solution.thrust_n = thrust_n;
  solution.thrust_coefficient_solidity = sigma > 0.0 ? thrust_coefficient / sigma : 0.0;

  // ---- 3. Torque ------------------------------------------------------
  //
  //   C_Q = C_Q_induced + C_Q_climb + C_Q_profile
  //       = kappa lambda_i C_T + lambda_c C_T + (sigma C_d0 / 8)(1 + k mu^2)
  //
  // with k = 4.65 (Bennett's forward-flight profile-power factor, as quoted by
  // Leishman §5.4.3) accounting for the rise of profile power with advance
  // ratio. The climb term lambda_c * C_T is included because a climbing rotor
  // demands the power that raises the aircraft, and it is NOT multiplied by
  // kappa: kappa covers the losses of the INDUCED flow, and climb power is not
  // one of them.
  constexpr double kProfilePowerAdvanceFactor = 4.65;
  const double induced_torque =
      (geometry.induced_power_factor * lambda_i + lambda_c) * thrust_coefficient;
  const double profile_torque = sigma * geometry.profile_drag_coefficient / 8.0
                                * (1.0 + kProfilePowerAdvanceFactor * mu * mu);
  const double torque_coefficient = induced_torque + profile_torque;
  const double torque_n_m = torque_coefficient * dynamic * radius;

  solution.torque_coefficient = torque_coefficient;
  solution.torque_n_m = torque_n_m;
  solution.power_w = torque_n_m * omega;

  // ---- 4. Quasi-static flapping --------------------------------------
  //
  // Lock number gamma = rho a c R^4 / I_beta, the ratio of aerodynamic to
  // inertial flapping moments. With no blade inertia declared the rotor is
  // treated as rigid in flap: the disc follows the swashplate exactly and the
  // rate-induced flapping vanishes. That is a legitimate degenerate case and
  // it is what makes an untilted, uncyclic rotor reproduce (0, 0, -T) exactly.
  double a1s = 0.0;
  double b1s = 0.0;
  const double flap_frequency_ratio_squared =
      1.0
      + (geometry.blade_flap_inertia_kg_m2 > 0.0
             ? geometry.flap_stiffness_n_m_rad
                   / (geometry.blade_flap_inertia_kg_m2 * omega * omega)
             : 0.0)
      + (geometry.hinge_offset_m > 0.0
             ? 1.5 * geometry.hinge_offset_m / (radius - geometry.hinge_offset_m)
             : 0.0);

  if (geometry.blade_flap_inertia_kg_m2 > 0.0) {
    const double lock =
        density_kg_m3 * a * geometry.chord_m * std::pow(radius, 4.0)
        / geometry.blade_flap_inertia_kg_m2;
    const double stiffening = flap_frequency_ratio_squared - 1.0;
    // Padfield 3.63-3.66, retaining the first-harmonic terms that matter at
    // Level 1: the disc tilts back with mu (flap-back), follows cyclic, and
    // lags the body rates by roughly 16/(gamma) per unit rate.
    const double denominator = 1.0 + 8.0 * stiffening / lock * (8.0 * stiffening / lock);
    const double flap_back = 2.0 * mu * (4.0 / 3.0 * theta0 - lambda_total);
    const double rate_scale = 16.0 / (lock * omega);
    a1s = (flap_back - controls.longitudinal_cyclic_rad - rate_scale * body_rate_rad_s.y())
          / denominator;
    b1s = (-controls.lateral_cyclic_rad - rate_scale * body_rate_rad_s.x()
           + 4.0 / 3.0 * mu * lambda_i)
          / denominator;
  } else {
    // Rigid in flap: the tip-path plane IS the swashplate.
    a1s = -controls.longitudinal_cyclic_rad;
    b1s = -controls.lateral_cyclic_rad;
  }
  solution.longitudinal_flap_rad = a1s;
  solution.lateral_flap_rad = b1s;

  // ---- 5. The wrench --------------------------------------------------
  //
  // THIS IS THE LINE THAT REPLACES Vector3d(0, 0, -thrust_n).
  //
  // Thrust acts along the tip-path-plane normal. In hub axes that normal is
  // -z tilted by the flapping angles: a longitudinal flap a1s (disc back) tips
  // the thrust vector AFT, giving a rearward force component; a lateral flap
  // b1s tips it sideways. To first order in the small flapping angles the
  // normal is (sin a1s, -sin b1s, -cos...) and the exact trigonometric form is
  // used rather than the linearisation, because the difference is free and the
  // small-angle assumption is one fewer thing to be wrong about.
  const double cos_a = std::cos(a1s);
  const double sin_a = std::sin(a1s);
  const double cos_b = std::cos(b1s);
  const double sin_b = std::sin(b1s);
  const Eigen::Vector3d thrust_hub(-thrust_n * sin_a * cos_b, thrust_n * sin_b,
                                   -thrust_n * cos_a * cos_b);

  // Shaft torque reacts onto the airframe in the sense opposite to the rotor's
  // own rotation: a rotor turning one way yaws the body the other.
  const Eigen::Vector3d shaft_moment_hub(
      0.0, 0.0, -static_cast<double>(geometry.spin_about_shaft) * torque_n_m);

  // Hub moment from flap stiffness: a hingeless rotor transmits a moment
  // proportional to the disc tilt, and this is where a helicopter's control
  // power mostly comes from at high disc loading. N_b/2 * K_beta is the
  // standard multi-blade result.
  const double hub_moment_gain =
      0.5 * static_cast<double>(geometry.blade_count) * geometry.flap_stiffness_n_m_rad;
  const Eigen::Vector3d hub_moment_hub(hub_moment_gain * b1s, hub_moment_gain * a1s, 0.0);

  const Eigen::Vector3d force_body = geometry.hub_to_body * thrust_hub;
  const Eigen::Vector3d moment_at_hub_body =
      geometry.hub_to_body * (shaft_moment_hub + hub_moment_hub);

  solution.wrench.force_body_n = force_body;
  // Moved to the CG: M_cg = M_hub + r_cg_to_hub x F.
  solution.wrench.moment_cg_body_n_m =
      moment_at_hub_body + geometry.position_cg_to_hub_body_m.cross(force_body);

  // Gyroscopic reaction of the spinning disc on the airframe.
  solution.wrench.moment_cg_body_n_m += gyroscopic_moment_body(geometry, state, body_rate_rad_s);

  // ---- 6. Dynamic inflow rate ----------------------------------------
  if (has_inflow_lag) {
    solution.inflow_rate_per_s =
        (lambda_quasi_static - state.inflow_ratio) / geometry.inflow_time_constant_s;
  }
  solution.quasi_static_inflow_ratio = lambda_quasi_static;

  if (!solution.wrench.force_body_n.allFinite() || !solution.wrench.moment_cg_body_n_m.allFinite()) {
    throw std::runtime_error("rotor '" + geometry.name + "': produced a non-finite wrench");
  }
  return solution;
}

}  // namespace galata::model::rotor
