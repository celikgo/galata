// SPDX-License-Identifier: Apache-2.0
//
// Central-difference linearisation of a VehicleModel in Euler coordinates.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 4.
//   G. D. Padfield, "Helicopter Flight Dynamics", 2nd ed., Blackwell, 2007, §4.3.
//   W. H. Press et al., "Numerical Recipes", 3rd ed., Cambridge, 2007, §5.7.
//
// Validity envelope and known error behaviour are in the header's
// "WHAT THIS IS NOT" block. Truncation error is O(h^2) with the constant set by
// the third derivative of the model; rounding error is O(eps/h); the step
// balances them at eps^(1/3). Across a kink neither bound holds and the
// Richardson estimate cannot detect it, which is why saturated controls are
// reported separately.
//
// DETERMINISM. Every loop bound is known before the loop starts, the
// perturbation steps are computed from the state rather than accumulated, and
// the difference divides by the step ACTUALLY taken rather than the one asked
// for — so a step that is not exactly representable does not bias the entry.

#include "galata/linearize/vehicle.hpp"

#include "galata/core/quaternion.hpp"
#include "galata/numerics/integration_method.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace galata::linearize {
namespace {

constexpr int kEulerStateSize = 12;

std::string number(double value, int digits = 4) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(digits) << std::scientific << value;
  return out.str();
}

// Euler coordinate names, in the order the linear state carries them.
std::vector<std::string> euler_names() {
  return {"position_north_m", "position_east_m", "position_down_m",
          "velocity_u_m_s",   "velocity_v_m_s",  "velocity_w_m_s",
          "roll_rad",         "pitch_rad",       "yaw_rad",
          "roll_rate_rad_s",  "pitch_rate_rad_s", "yaw_rate_rad_s"};
}

// Pack an extended (quaternion) state into Euler coordinates.
Eigen::VectorXd to_euler(const model::VehicleModel& model, const Eigen::VectorXd& extended) {
  const core::State state = model::VehicleModel::rigid_body_part(extended);
  const core::EulerAngles euler = core::euler_from_quaternion(state.attitude_body_to_ned);
  const int aux = model.auxiliary_state_count();
  Eigen::VectorXd out(kEulerStateSize + aux);
  out.segment<3>(0) = state.position_ned_m;
  out.segment<3>(3) = state.velocity_body_m_s;
  out(6) = euler.roll_rad;
  out(7) = euler.pitch_rad;
  out(8) = euler.yaw_rad;
  out.segment<3>(9) = state.angular_rate_body_rad_s;
  if (aux > 0) {
    out.segment(kEulerStateSize, aux) = extended.segment(core::kStateSize, aux);
  }
  return out;
}

// And back.
Eigen::VectorXd from_euler(const model::VehicleModel& model, const Eigen::VectorXd& euler_state) {
  core::State state;
  state.position_ned_m = euler_state.segment<3>(0);
  state.velocity_body_m_s = euler_state.segment<3>(3);
  state.attitude_body_to_ned = core::quaternion_from_euler(
      core::EulerAngles{euler_state(6), euler_state(7), euler_state(8)});
  state.angular_rate_body_rad_s = euler_state.segment<3>(9);
  const int aux = model.auxiliary_state_count();
  Eigen::VectorXd auxiliary = Eigen::VectorXd::Zero(aux);
  if (aux > 0) {
    auxiliary = euler_state.segment(kEulerStateSize, aux);
  }
  return model.join(state, auxiliary);
}

// The Euler-coordinate derivative. The rigid-body kernel returns a quaternion
// rate; the Euler rates are recovered from the body rates through the standard
// kinematic relation rather than by differentiating the quaternion, because the
// latter loses a digit and is singular in a different place.
Eigen::VectorXd euler_derivative(const model::VehicleModel& model,
                                 const Eigen::VectorXd& euler_state,
                                 const Eigen::VectorXd& controls,
                                 const model::Environment& environment) {
  const Eigen::VectorXd extended = from_euler(model, euler_state);
  const Eigen::VectorXd rate = model.derivative(extended, controls, environment);

  const double roll = euler_state(6);
  const double pitch = euler_state(7);
  const double p = euler_state(9);
  const double q = euler_state(10);
  const double r = euler_state(11);
  const double sin_roll = std::sin(roll);
  const double cos_roll = std::cos(roll);
  const double cos_pitch = std::cos(pitch);
  const double tan_pitch = std::tan(pitch);

  const int aux = model.auxiliary_state_count();
  Eigen::VectorXd out(kEulerStateSize + aux);
  out.segment<3>(0) = rate.segment<3>(core::kPositionNorth);
  out.segment<3>(3) = rate.segment<3>(core::kVelocityU);
  out(6) = p + (q * sin_roll + r * cos_roll) * tan_pitch;
  out(7) = q * cos_roll - r * sin_roll;
  // Near gimbal lock this divides by a vanishing cosine. The linearisation is
  // meaningless there anyway — the coordinates themselves are — and a caller
  // trimming at 90 degrees of pitch has a different problem.
  out(8) = std::fabs(cos_pitch) > 1.0e-9 ? (q * sin_roll + r * cos_roll) / cos_pitch : 0.0;
  out.segment<3>(9) = rate.segment<3>(core::kRateP);
  if (aux > 0) {
    out.segment(kEulerStateSize, aux) = rate.segment(core::kStateSize, aux);
  }
  return out;
}

}  // namespace

VehicleLinearisation linearize_vehicle(const model::VehicleModel& model,
                                       const Eigen::VectorXd& extended_state,
                                       const Eigen::VectorXd& controls,
                                       const model::Environment& environment,
                                       const VehicleLinearisationOptions& options) {
  model.validate_vocabulary();
  environment.validate();
  if (extended_state.size() != model.extended_state_size()) {
    throw std::invalid_argument("linearize_vehicle: the state has "
                                + std::to_string(extended_state.size()) + " entries, the model has "
                                + std::to_string(model.extended_state_size()));
  }
  if (controls.size() != model.control_count()) {
    throw std::invalid_argument("linearize_vehicle: the controls have "
                                + std::to_string(controls.size()) + " entries, the model has "
                                + std::to_string(model.control_count()));
  }
  if (!(options.relative_step > 0.0) || !(options.absolute_step > 0.0)) {
    throw std::invalid_argument("linearize_vehicle: the perturbation steps must be positive");
  }
  if (!(options.equilibrium_tolerance > 0.0)) {
    throw std::invalid_argument("linearize_vehicle: equilibrium_tolerance must be positive");
  }

  VehicleLinearisation result;
  result.trim_extended_state = extended_state;
  result.trim_controls = controls;
  result.control_names = model.control_names();

  const auto model_states = model.state_names();
  result.state_names = euler_names();
  for (int i = 0; i < model.auxiliary_state_count(); ++i) {
    result.state_names.push_back(
        model_states[static_cast<std::size_t>(core::kStateSize + i)]);
  }

  const Eigen::VectorXd point = to_euler(model, extended_state);
  const auto n = point.size();
  const auto m = static_cast<Eigen::Index>(result.control_names.size());

  // ---- is this actually an equilibrium? -------------------------------
  //
  // MEASURED, not assumed. The position and heading rates are excluded: a
  // helicopter in forward flight has a large position rate and is still in
  // equilibrium, and a trim at a heading is still a trim. What must vanish is
  // the DYNAMIC part — body velocity, attitude, body rate and the model's own
  // auxiliary states.
  const Eigen::VectorXd rate_at_point = euler_derivative(model, point, controls, environment);
  double worst_residual = 0.0;
  Eigen::Index worst_index = 3;
  for (Eigen::Index i = 3; i < n; ++i) {
    if (i == 8) {
      continue;  // heading rate: free in still air, not part of the equilibrium
    }
    if (std::fabs(rate_at_point(i)) > worst_residual) {
      worst_residual = std::fabs(rate_at_point(i));
      worst_index = i;
    }
  }
  result.equilibrium_residual = worst_residual;
  if (worst_residual > options.equilibrium_tolerance) {
    throw std::runtime_error(
        "linearize_vehicle: this point is not an equilibrium. The worst dynamic residual is "
        + number(worst_residual) + " in state '"
        + result.state_names[static_cast<std::size_t>(worst_index)] + "', against a budget of "
        + number(options.equilibrium_tolerance)
        + ". A linearisation about a non-equilibrium carries a constant term the A matrix cannot "
          "represent, so its trajectories drift from the plant's for a reason no eigenvalue "
          "shows. Trim first, or raise equilibrium_tolerance deliberately and say why");
  }

  // ---- the Jacobians --------------------------------------------------
  const auto jacobian_at = [&](double scale) {
    Eigen::MatrixXd a(n, n);
    Eigen::MatrixXd b(n, m);
    for (Eigen::Index j = 0; j < n; ++j) {
      const double step =
          scale * std::max(options.relative_step * std::fabs(point(j)), options.absolute_step);
      Eigen::VectorXd probe = point;
      probe(j) = point(j) + step;
      const double upper = probe(j);
      const Eigen::VectorXd forward = euler_derivative(model, probe, controls, environment);
      probe(j) = point(j) - step;
      const double lower = probe(j);
      const Eigen::VectorXd backward = euler_derivative(model, probe, controls, environment);
      a.col(j) = (forward - backward) / (upper - lower);
    }
    for (Eigen::Index j = 0; j < m; ++j) {
      const double step =
          scale * std::max(options.relative_step * std::fabs(controls(j)), options.absolute_step);
      Eigen::VectorXd probe = controls;
      probe(j) = controls(j) + step;
      const double upper = probe(j);
      const Eigen::VectorXd forward = euler_derivative(model, point, probe, environment);
      probe(j) = controls(j) - step;
      const double lower = probe(j);
      const Eigen::VectorXd backward = euler_derivative(model, point, probe, environment);
      b.col(j) = (forward - backward) / (upper - lower);
    }
    return std::make_pair(a, b);
  };

  auto [a, b] = jacobian_at(1.0);
  result.a = a;
  result.b = b;

  if (options.estimate_truncation_error) {
    // Richardson: for a second-order method, the error at h is
    // |J(h) - J(2h)| / 3. Computed per entry and reported relative to the
    // entry's own magnitude, because an absolute error means nothing when the
    // entries span ten orders of magnitude.
    const auto [a2, b2] = jacobian_at(2.0);
    // SCALED BY THE COLUMN, NOT BY THE ENTRY. An entry that is physically zero
    // has a truncation estimate at the level of rounding noise, and dividing
    // one by the other produces a relative error near 1 that says nothing
    // about the matrix. What matters is the estimate against the size of the
    // derivatives that column actually carries, which is its largest entry.
    for (Eigen::Index j = 0; j < n; ++j) {
      const double column_scale = std::max(a.col(j).cwiseAbs().maxCoeff(), 1.0e-12);
      for (Eigen::Index i = 0; i < n; ++i) {
        const double estimate = std::fabs(a(i, j) - a2(i, j)) / 3.0;
        const double relative = estimate / column_scale;
        if (relative > result.worst_relative_truncation) {
          result.worst_relative_truncation = relative;
          result.worst_truncation_row = static_cast<int>(i);
          result.worst_truncation_column = static_cast<int>(j);
        }
      }
    }
  }

  // ---- which controls were at a stop? ---------------------------------
  //
  // A control column differenced about a saturated actuator is a one-sided
  // difference dressed as a central one: the model's response to the
  // perturbation in one direction is zero, so the derivative comes out at HALF
  // its true value with nothing to show that anything happened. The test is
  // cheap — does the model's auxiliary rate respond to the perturbation at
  // all? — and the answer goes in the report.
  for (Eigen::Index j = 0; j < m; ++j) {
    const double step = std::max(options.relative_step * std::fabs(controls(j)),
                                 options.absolute_step);
    Eigen::VectorXd up = controls;
    Eigen::VectorXd down = controls;
    up(j) += step;
    down(j) -= step;
    const Eigen::VectorXd rate_up = euler_derivative(model, point, up, environment);
    const Eigen::VectorXd rate_down = euler_derivative(model, point, down, environment);
    const Eigen::VectorXd base = rate_at_point;
    const double response_up = (rate_up - base).norm();
    const double response_down = (rate_down - base).norm();
    const double larger = std::max(response_up, response_down);
    if (larger > 0.0 && std::min(response_up, response_down) < 1.0e-6 * larger) {
      result.saturated_controls.push_back(result.control_names[static_cast<std::size_t>(j)]);
    }
  }
  return result;
}

model::LinearSystem VehicleLinearisation::to_linear_system(const std::string& description) const {
  model::LinearSystem system;
  system.a = a;
  system.b = b;
  system.state_names = state_names;
  system.input_names = control_names;
  system.description = description;
  system.units = "velocities m/s, angles rad, rates rad/s, rotor speed rad/s, inflow dimensionless";
  system.validate();
  return system;
}

VehicleLinearisation VehicleLinearisation::reduced() const {
  // Dropped BY NAME, so the choice is visible in the result rather than
  // implied by an index.
  static const std::vector<std::string> dropped = {"position_north_m", "position_east_m",
                                                   "position_down_m", "yaw_rad"};
  std::vector<Eigen::Index> keep;
  std::vector<std::string> names;
  for (std::size_t i = 0; i < state_names.size(); ++i) {
    if (std::find(dropped.begin(), dropped.end(), state_names[i]) == dropped.end()) {
      keep.push_back(static_cast<Eigen::Index>(i));
      names.push_back(state_names[i]);
    }
  }
  VehicleLinearisation out = *this;
  out.state_names = names;
  out.a.resize(static_cast<Eigen::Index>(keep.size()), static_cast<Eigen::Index>(keep.size()));
  out.b.resize(static_cast<Eigen::Index>(keep.size()), b.cols());
  for (std::size_t i = 0; i < keep.size(); ++i) {
    for (std::size_t j = 0; j < keep.size(); ++j) {
      out.a(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = a(keep[i], keep[j]);
    }
    out.b.row(static_cast<Eigen::Index>(i)) = b.row(keep[i]);
  }
  return out;
}

NonlinearAgreement nonlinear_agreement(const model::VehicleModel& model,
                                       const VehicleLinearisation& linearisation,
                                       const model::Environment& environment,
                                       const Eigen::VectorXd& perturbation_direction,
                                       const std::vector<double>& epsilons,
                                       double horizon_s,
                                       double step_s) {
  if (epsilons.size() < 2) {
    throw std::invalid_argument(
        "nonlinear_agreement: at least two perturbation sizes are needed. The measurement is an "
        "ORDER, and an order needs two points");
  }
  if (!(step_s > 0.0) || !(horizon_s > 0.0)) {
    throw std::invalid_argument("nonlinear_agreement: horizon and step must be positive");
  }
  const Eigen::VectorXd base = to_euler(model, linearisation.trim_extended_state);
  if (perturbation_direction.size() != base.size()) {
    throw std::invalid_argument("nonlinear_agreement: the perturbation direction has "
                                + std::to_string(perturbation_direction.size())
                                + " entries, the linear state has " + std::to_string(base.size()));
  }
  const int steps = static_cast<int>(std::llround(horizon_s / step_s));
  if (steps < 1) {
    throw std::invalid_argument("nonlinear_agreement: the horizon is shorter than one step");
  }

  NonlinearAgreement out;
  out.epsilons = epsilons;
  const Eigen::VectorXd direction = perturbation_direction.normalized();

  const auto nonlinear_rate = [&](double, const Eigen::VectorXd& x) {
    return euler_derivative(model, x, linearisation.trim_controls, environment);
  };
  numerics::IntegrationOptions integration;
  integration.step_s = step_s;
  integration.step_count = steps;
  integration.sample_stride = steps;

  // THE COMPARISON IS AGAINST THE BASE TRAJECTORY, NOT AGAINST THE BASE POINT.
  //
  // A first draft compared the perturbed nonlinear trajectory against
  // `base + dx_linear(T)`, which is only right when the base does not move. In
  // forward flight it moves a great deal: the aircraft's own position advances
  // by V*T, and that advance appeared in every discrepancy as a constant. The
  // measured "discrepancies" at 5, 20 and 40 m/s over half a second came out at
  // exactly 2.5, 10.0 and 20.0 — V*T to four figures, independent of the
  // perturbation entirely, which is what made the error visible.
  //
  // What the linearisation actually predicts is the DIFFERENCE between two
  // nonlinear trajectories, so that is what it is measured against.
  const auto base_trajectory = numerics::integrate(nonlinear_rate, base, 0.0, integration);
  if (!base_trajectory.completed()) {
    throw std::runtime_error(
        "nonlinear_agreement: the unperturbed trajectory did not complete: "
        + base_trajectory.detail);
  }
  const Eigen::VectorXd base_final = base_trajectory.trajectory.states.back();

  for (const double epsilon : epsilons) {
    const Eigen::VectorXd offset = epsilon * direction;

    const auto nonlinear =
        numerics::integrate(nonlinear_rate, Eigen::VectorXd(base + offset), 0.0, integration);
    if (!nonlinear.completed()) {
      throw std::runtime_error("nonlinear_agreement: the perturbed trajectory at epsilon = "
                               + number(epsilon) + " did not complete: " + nonlinear.detail);
    }

    // Linear: integrate d(dx)/dt = A dx from the same offset, with the SAME
    // integrator and the same step, so the comparison measures the
    // LINEARISATION's error and not the difference between two schemes.
    const auto linear_rate = [&](double, const Eigen::VectorXd& dx) {
      return Eigen::VectorXd(linearisation.a * dx);
    };
    const auto linear = numerics::integrate(linear_rate, offset, 0.0, integration);

    const Eigen::VectorXd nonlinear_difference = nonlinear.trajectory.states.back() - base_final;
    // SCALED BEFORE THE NORM IS TAKEN. The state vector mixes m/s, rad, rad/s
    // and — since the engine torque became a state — N m of order 1e4. A
    // Euclidean norm over those is dominated by whichever carries the largest
    // units, and the measured "discrepancy" then reports the torque's relative
    // error in absolute newton-metres. Dividing each component by the base
    // point's own magnitude makes the norm dimensionless and comparable between
    // models. The scaling is independent of epsilon, so the ORDER the ratios
    // measure is unchanged by it.
    const Eigen::VectorXd error = nonlinear_difference - linear.trajectory.states.back();
    double sum = 0.0;
    for (Eigen::Index i = 0; i < error.size(); ++i) {
      const double scale = std::max(std::fabs(base(i)), 1.0);
      const double term = error(i) / scale;
      sum += term * term;
    }
    out.discrepancies.push_back(std::sqrt(sum));
  }

  for (std::size_t i = 1; i < out.discrepancies.size(); ++i) {
    const double coarse = out.discrepancies[i - 1];
    const double fine = out.discrepancies[i];
    const double ratio = epsilons[i - 1] / epsilons[i];
    const double order = (fine > 0.0 && ratio > 1.0)
                             ? std::log(coarse / fine) / std::log(ratio)
                             : 0.0;
    out.observed_orders.push_back(order);
  }
  if (!out.observed_orders.empty()) {
    out.worst_order = *std::min_element(out.observed_orders.begin(), out.observed_orders.end());
    out.best_order = *std::max_element(out.observed_orders.begin(), out.observed_orders.end());
  }
  return out;
}

}  // namespace galata::linearize
