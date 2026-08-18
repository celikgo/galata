// SPDX-License-Identifier: Apache-2.0
//
// Numerical linearisation about a trim point.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 3.
//   B. Etkin and L. D. Reid, "Dynamics of Flight", 3rd ed., Wiley, 1996.
//
// Coordinates, validity envelope and what the truncation estimate cannot see:
// the header's block comments.

#include "galata/linearize/finite_difference.hpp"

#include "galata/core/quaternion.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace galata::linearize {
namespace {

core::State state_from_euler(const Eigen::VectorXd& x) {
  core::State state;
  state.position_ned_m = Eigen::Vector3d(x(kPositionNorth), x(kPositionEast), x(kPositionDown));
  state.velocity_body_m_s = Eigen::Vector3d(x(kVelocityU), x(kVelocityV), x(kVelocityW));
  state.attitude_body_to_ned = core::quaternion_from_euler({x(kRoll), x(kPitch), x(kYaw)});
  state.angular_rate_body_rad_s = Eigen::Vector3d(x(kRateP), x(kRateQ), x(kRateR));
  return state;
}

Eigen::VectorXd euler_from_state(const core::State& state) {
  const core::EulerAngles euler = core::euler_from_quaternion(state.attitude_body_to_ned);
  Eigen::VectorXd x(kEulerStateSize);
  x(kPositionNorth) = state.position_ned_m.x();
  x(kPositionEast) = state.position_ned_m.y();
  x(kPositionDown) = state.position_ned_m.z();
  x(kVelocityU) = state.velocity_body_m_s.x();
  x(kVelocityV) = state.velocity_body_m_s.y();
  x(kVelocityW) = state.velocity_body_m_s.z();
  x(kRoll) = euler.roll_rad;
  x(kPitch) = euler.pitch_rad;
  x(kYaw) = euler.yaw_rad;
  x(kRateP) = state.angular_rate_body_rad_s.x();
  x(kRateQ) = state.angular_rate_body_rad_s.y();
  x(kRateR) = state.angular_rate_body_rad_s.z();
  return x;
}

// The Euler kinematic relation. Singular at |theta| = 90 degrees, which is why
// the header refuses to pretend this chart works everywhere.
Eigen::Vector3d euler_rates(double roll, double pitch, const Eigen::Vector3d& body_rates) {
  const double sin_roll = std::sin(roll);
  const double cos_roll = std::cos(roll);
  const double cos_pitch = std::cos(pitch);
  const double tan_pitch = std::tan(pitch);
  const double p = body_rates.x();
  const double q = body_rates.y();
  const double r = body_rates.z();

  Eigen::Vector3d rates;
  rates.x() = p + (q * sin_roll + r * cos_roll) * tan_pitch;
  rates.y() = q * cos_roll - r * sin_roll;
  rates.z() = (q * sin_roll + r * cos_roll) / cos_pitch;
  return rates;
}

}  // namespace

std::vector<std::string> euler_state_names() {
  return {"p_n", "p_e", "p_d", "u", "v", "w", "phi", "theta", "psi", "p", "q", "r"};
}

std::vector<std::string> control_names() {
  return {"elevator", "aileron", "rudder", "thrust"};
}

std::vector<int> longitudinal_states() {
  return {kVelocityU, kVelocityW, kRateQ, kPitch};
}

std::vector<int> lateral_states() {
  return {kVelocityV, kRateP, kRateR, kRoll};
}

model::LinearSystem Linearisation::to_linear_system(const std::string& description,
                                                    const std::string& citation) const {
  model::LinearSystem system;
  system.a = a;
  system.b = b;
  system.state_names = state_names;
  system.input_names = input_names;
  system.description = description;
  system.citation = citation;
  std::ostringstream units;
  units << "linearised about " << trim_airspeed_m_s << " m/s at " << trim_altitude_m
        << " m; velocities m/s, angles rad, rates rad/s";
  system.units = units.str();
  system.validate();
  return system;
}

Linearisation linearize_finite_difference(const model::Aircraft& aircraft,
                                          const trim::TrimPoint& trim,
                                          const LinearisationOptions& options) {
  aircraft.validate();

  const Eigen::VectorXd x0 = euler_from_state(trim.state);
  const Eigen::VectorXd u0 = trim.controls.to_vector();

  // dx/dt in the Euler coordinates.
  const numerics::VectorFunction state_dynamics = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    const core::State state = state_from_euler(x);
    const core::StateVector rate = aircraft.derivative(state, model::Controls::from_vector(u0));
    const Eigen::Vector3d attitude_rates =
        euler_rates(x(kRoll), x(kPitch), Eigen::Vector3d(x(kRateP), x(kRateQ), x(kRateR)));

    Eigen::VectorXd out(kEulerStateSize);
    out(kPositionNorth) = rate(core::kPositionNorth);
    out(kPositionEast) = rate(core::kPositionEast);
    out(kPositionDown) = rate(core::kPositionDown);
    out(kVelocityU) = rate(core::kVelocityU);
    out(kVelocityV) = rate(core::kVelocityV);
    out(kVelocityW) = rate(core::kVelocityW);
    out(kRoll) = attitude_rates.x();
    out(kPitch) = attitude_rates.y();
    out(kYaw) = attitude_rates.z();
    out(kRateP) = rate(core::kRateP);
    out(kRateQ) = rate(core::kRateQ);
    out(kRateR) = rate(core::kRateR);
    return out;
  };

  const numerics::VectorFunction control_dynamics =
      [&](const Eigen::VectorXd& u) -> Eigen::VectorXd {
    const core::State state = state_from_euler(x0);
    const core::StateVector rate = aircraft.derivative(state, model::Controls::from_vector(u));
    const Eigen::Vector3d attitude_rates =
        euler_rates(x0(kRoll), x0(kPitch), Eigen::Vector3d(x0(kRateP), x0(kRateQ), x0(kRateR)));

    Eigen::VectorXd out(kEulerStateSize);
    out(kPositionNorth) = rate(core::kPositionNorth);
    out(kPositionEast) = rate(core::kPositionEast);
    out(kPositionDown) = rate(core::kPositionDown);
    out(kVelocityU) = rate(core::kVelocityU);
    out(kVelocityV) = rate(core::kVelocityV);
    out(kVelocityW) = rate(core::kVelocityW);
    out(kRoll) = attitude_rates.x();
    out(kPitch) = attitude_rates.y();
    out(kYaw) = attitude_rates.z();
    out(kRateP) = rate(core::kRateP);
    out(kRateQ) = rate(core::kRateQ);
    out(kRateR) = rate(core::kRateR);
    return out;
  };

  numerics::JacobianOptions state_options = options.state_jacobian;
  if (state_options.absolute_step_per_component.size() == 0) {
    // Per-component floors. Positions are metres and are large, velocities are
    // tens of metres per second, angles and rates are small. One floor for all
    // of them would either swamp the angles or fail to move the positions.
    // Sized so that eps * |f| / h stays well below the accuracy wanted, where
    // |f| is the derivative component's own magnitude. The position rows carry
    // the largest values — a ground speed of tens of metres per second — so
    // their floor is the largest.
    Eigen::VectorXd floors(kEulerStateSize);
    floors << 1e-2, 1e-2, 1e-2,  // position, m
        1e-4, 1e-4, 1e-4,        // velocity, m/s
        1e-6, 1e-6, 1e-6,        // attitude, rad
        1e-6, 1e-6, 1e-6;        // body rates, rad/s
    state_options.absolute_step_per_component = floors;
  }
  state_options.estimate_truncation_error = options.report_truncation_error;

  numerics::JacobianOptions control_options = options.control_jacobian;
  if (control_options.absolute_step_per_component.size() == 0) {
    Eigen::VectorXd floors(model::Controls::kSize);
    floors << 1e-6, 1e-6, 1e-6,  // surface deflections, rad
        1e-1;                    // thrust, N
    control_options.absolute_step_per_component = floors;
  }
  control_options.estimate_truncation_error = options.report_truncation_error;

  const numerics::Jacobian full_a =
      numerics::central_difference_jacobian(state_dynamics, x0, state_options);
  const numerics::Jacobian full_b =
      numerics::central_difference_jacobian(control_dynamics, u0, control_options);

  Linearisation result;
  result.input_names = control_names();
  result.control_steps = full_b.steps;
  result.chart_conditioning = std::fabs(std::cos(x0(kPitch)));
  result.trim_altitude_m = -trim.state.position_ned_m.z();
  result.trim_airspeed_m_s = trim.airspeed_m_s;
  result.trim_alpha_rad = trim.alpha_rad;
  result.trim_residual_norm = trim.residual_norm;

  const std::vector<std::string> all_names = euler_state_names();

  if (options.state_subset.empty()) {
    result.a = full_a.value;
    result.b = full_b.value;
    result.state_names = all_names;
    result.state_steps = full_a.steps;
    result.a_truncation = full_a.truncation_estimate;
    result.b_truncation = full_b.truncation_estimate;
    result.neglected_coupling = 0.0;
  } else {
    const std::vector<int>& keep = options.state_subset;
    for (const int index : keep) {
      if (index < 0 || index >= kEulerStateSize) {
        throw std::invalid_argument("linearize_finite_difference: state_subset index out of range");
      }
    }
    const auto n = static_cast<Eigen::Index>(keep.size());
    result.a.resize(n, n);
    result.b.resize(n, full_b.value.cols());
    result.state_steps.resize(n);
    if (options.report_truncation_error) {
      result.a_truncation.resize(n, n);
      result.b_truncation.resize(n, full_b.value.cols());
    }
    for (Eigen::Index i = 0; i < n; ++i) {
      const auto row = static_cast<Eigen::Index>(keep[static_cast<std::size_t>(i)]);
      result.state_names.push_back(
          all_names[static_cast<std::size_t>(keep[static_cast<std::size_t>(i)])]);
      result.state_steps(i) = full_a.steps(row);
      result.b.row(i) = full_b.value.row(row);
      if (options.report_truncation_error) {
        result.b_truncation.row(i) = full_b.truncation_estimate.row(row);
      }
      for (Eigen::Index j = 0; j < n; ++j) {
        const auto col = static_cast<Eigen::Index>(keep[static_cast<std::size_t>(j)]);
        result.a(i, j) = full_a.value(row, col);
        if (options.report_truncation_error) {
          result.a_truncation(i, j) = full_a.truncation_estimate(row, col);
        }
      }
    }

    // How much of the full dynamics the reduction threw away. Measured, not
    // assumed: the decoupling of the longitudinal and lateral axes is a
    // property of a symmetric wings-level trim and not of the aircraft.
    std::vector<bool> retained(kEulerStateSize, false);
    for (const int index : keep) {
      retained[static_cast<std::size_t>(index)] = true;
    }
    double worst_coupling = 0.0;
    for (const int row_index : keep) {
      for (Eigen::Index j = 0; j < kEulerStateSize; ++j) {
        if (retained[static_cast<std::size_t>(j)]) {
          continue;
        }
        worst_coupling = std::fmax(
            worst_coupling, std::fabs(full_a.value(static_cast<Eigen::Index>(row_index), j)));
      }
    }
    const double scale = result.a.cwiseAbs().maxCoeff();
    result.neglected_coupling = (scale > 0.0) ? worst_coupling / scale : worst_coupling;
  }

  result.c = Eigen::MatrixXd::Identity(result.a.rows(), result.a.rows());
  result.d = Eigen::MatrixXd::Zero(result.a.rows(), result.b.cols());

  if (options.report_truncation_error) {
    const double scale = result.a.cwiseAbs().maxCoeff();
    result.worst_relative_truncation = (scale > 0.0) ? result.a_truncation.maxCoeff() / scale : 0.0;
  }
  return result;
}

}  // namespace galata::linearize
