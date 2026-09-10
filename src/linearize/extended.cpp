// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the model-generic linearisation declared in
// include/galata/linearize/extended.hpp. The chart, its attitude derivative and
// the validity envelope are documented there.

#include "galata/linearize/extended.hpp"

#include "galata/core/frames.hpp"
#include "galata/core/quaternion.hpp"
#include "galata/core/state.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace galata::linearize {
namespace {

int output_width(OutputKind kind) {
  return kind == OutputKind::Altitude ? 1 : 3;
}

std::vector<std::string> names_for(OutputKind kind) {
  switch (kind) {
    case OutputKind::BodySpecificForce:
      return {"specific_force_x_m_s2", "specific_force_y_m_s2", "specific_force_z_m_s2"};
    case OutputKind::BodyRates:
      return {"body_rate_p_rad_s", "body_rate_q_rad_s", "body_rate_r_rad_s"};
    case OutputKind::PositionNed:
      return {"position_north_m", "position_east_m", "position_down_m"};
    case OutputKind::Altitude:
      return {"altitude_m"};
    case OutputKind::GroundVelocityNed:
      return {"ground_velocity_north_m_s", "ground_velocity_east_m_s", "ground_velocity_down_m_s"};
  }
  throw std::invalid_argument("linearize_extended: unknown output kind");
}

}  // namespace

Eigen::VectorXd extended_from_chart(const Eigen::VectorXd& chart,
                                    const Eigen::VectorXd& reference_state) {
  if (reference_state.size() < core::kStateSize) {
    throw std::invalid_argument(
        "extended_from_chart: the reference state is shorter than the thirteen ADR-0002 "
        "components");
  }
  const auto appended = static_cast<int>(reference_state.size()) - core::kStateSize;
  if (chart.size() != kRigidChartSize + appended) {
    std::ostringstream message;
    message << "extended_from_chart: the chart carries " << chart.size()
            << " coordinate(s); this reference has " << appended << " appended state(s) and so "
            << "needs " << (kRigidChartSize + appended);
    throw std::invalid_argument(message.str());
  }
  const core::Quaternion nominal(reference_state(core::kQuaternionW),
                                 reference_state(core::kQuaternionX),
                                 reference_state(core::kQuaternionY),
                                 reference_state(core::kQuaternionZ));
  Eigen::VectorXd full = reference_state;
  full.segment<3>(core::kPositionNorth) += chart.segment<3>(kChartPositionNorth);
  full.segment<3>(core::kVelocityU) += chart.segment<3>(kChartVelocityU);
  full.segment<3>(core::kRateP) += chart.segment<3>(kChartRateP);
  const core::Quaternion attitude = core::normalised(
      nominal * core::quaternion_from_rotation_vector(chart.segment<3>(kChartAttitudeErrorX)));
  full(core::kQuaternionW) = attitude.w();
  full(core::kQuaternionX) = attitude.x();
  full(core::kQuaternionY) = attitude.y();
  full(core::kQuaternionZ) = attitude.z();
  for (int i = 0; i < appended; ++i) {
    full(core::kStateSize + i) += chart(kRigidChartSize + i);
  }
  return full;
}

Eigen::VectorXd chart_from_extended(const Eigen::VectorXd& extended_state,
                                    const Eigen::VectorXd& reference_state) {
  if (reference_state.size() < core::kStateSize) {
    throw std::invalid_argument(
        "chart_from_extended: the reference state is shorter than the thirteen ADR-0002 "
        "components");
  }
  if (extended_state.size() != reference_state.size()) {
    std::ostringstream message;
    message << "chart_from_extended: the state carries " << extended_state.size()
            << " component(s) and the reference " << reference_state.size()
            << "; a displacement between states of different width is not defined";
    throw std::invalid_argument(message.str());
  }
  const auto appended = static_cast<int>(reference_state.size()) - core::kStateSize;
  const core::Quaternion nominal(reference_state(core::kQuaternionW),
                                 reference_state(core::kQuaternionX),
                                 reference_state(core::kQuaternionY),
                                 reference_state(core::kQuaternionZ));
  const core::Quaternion actual(extended_state(core::kQuaternionW),
                                extended_state(core::kQuaternionX),
                                extended_state(core::kQuaternionY),
                                extended_state(core::kQuaternionZ));

  Eigen::VectorXd chart = Eigen::VectorXd::Zero(kRigidChartSize + appended);
  chart.segment<3>(kChartPositionNorth) = extended_state.segment<3>(core::kPositionNorth)
                                          - reference_state.segment<3>(core::kPositionNorth);
  chart.segment<3>(kChartVelocityU) =
      extended_state.segment<3>(core::kVelocityU) - reference_state.segment<3>(core::kVelocityU);
  chart.segment<3>(kChartRateP) =
      extended_state.segment<3>(core::kRateP) - reference_state.segment<3>(core::kRateP);
  // The multiplicative one: q = q0 * exp(e/2), so exp(e/2) = q0^-1 * q.
  chart.segment<3>(kChartAttitudeErrorX) = core::rotation_vector_from_quaternion(
      core::normalised(nominal).conjugate() * core::normalised(actual));
  for (int i = 0; i < appended; ++i) {
    chart(kRigidChartSize + i) =
        extended_state(core::kStateSize + i) - reference_state(core::kStateSize + i);
  }
  return chart;
}

model::LinearSystem ExtendedLinearisation::to_linear_system(const std::string& description,
                                                            const std::string& citation) const {
  model::LinearSystem system;
  system.a = a;
  system.b = b;
  system.c = c;
  system.d = d;
  system.state_names = state_names;
  system.input_names = input_names;
  system.output_names = output_names;
  system.description = description;
  system.citation = citation;
  system.validate();
  return system;
}

ExtendedLinearisation linearize_extended(const ExtendedDynamics& dynamics,
                                         const Eigen::VectorXd& extended_state,
                                         const Eigen::VectorXd& input,
                                         const ExtendedLinearisationOptions& options) {
  if (!dynamics) {
    throw std::invalid_argument("linearize_extended: no dynamics function supplied");
  }
  const int appended = static_cast<int>(options.appended_state_names.size());
  const int chart_size = kRigidChartSize + appended;
  if (extended_state.size() != core::kStateSize + appended) {
    std::ostringstream message;
    message << "linearize_extended: the state has " << extended_state.size() << " components but "
            << appended << " appended state names were declared, which "
            << "describes a state of " << (core::kStateSize + appended) << ".";
    throw std::invalid_argument(message.str());
  }
  if (input.size() != static_cast<Eigen::Index>(options.input_names.size())) {
    throw std::invalid_argument("linearize_extended: one input name is required per input column");
  }
  if (options.outputs.empty()) {
    throw std::invalid_argument(
        "linearize_extended: declare at least one output; an empty observation model is not "
        "the identity, it is nothing");
  }
  if (options.wind_input_offset >= 0
      && options.wind_input_offset + 3 > static_cast<int>(input.size())) {
    throw std::invalid_argument(
        "linearize_extended: wind_input_offset does not admit three consecutive columns");
  }
  if (!extended_state.allFinite() || !input.allFinite()) {
    throw std::invalid_argument("linearize_extended: the operating point is not finite");
  }
  std::vector<bool> frozen(static_cast<std::size_t>(appended), false);
  for (const int index : options.frozen_appended_states) {
    if (index < 0 || index >= appended) {
      std::ostringstream message;
      message << "linearize_extended: frozen appended state " << index << " is outside the "
              << appended << " appended states declared. The index is "
              << "into the appended block, where 0 is the first state after the twelfth chart "
                 "coordinate.";
      throw std::invalid_argument(message.str());
    }
    if (frozen[static_cast<std::size_t>(index)]) {
      std::ostringstream message;
      message << "linearize_extended: appended state " << index
              << " is frozen twice. A repeated index means the caller has miscounted, and "
                 "silently ignoring it would freeze a state nobody named.";
      throw std::invalid_argument(message.str());
    }
    frozen[static_cast<std::size_t>(index)] = true;
  }

  const core::Quaternion nominal_attitude(extended_state(core::kQuaternionW),
                                          extended_state(core::kQuaternionX),
                                          extended_state(core::kQuaternionY),
                                          extended_state(core::kQuaternionZ));

  // Chart -> full state.
  // One implementation, called from here rather than repeated. The map used by
  // a controller and the map the matrices were built on must be the same map;
  // two copies that must agree about a half-angle convention and a sign are a
  // drift waiting to happen, and ADR-0017 rejects that explicitly.
  const auto unpack = [&](const Eigen::VectorXd& delta) {
    return extended_from_chart(delta, extended_state);
  };

  // Full-state derivative -> chart derivative. The attitude rows are the chart's
  // own kinematics rather than anything f returns, because f returns a
  // quaternion rate and the chart's coordinate is a rotation vector.
  const auto to_chart_rate = [&](const Eigen::VectorXd& full,
                                 const Eigen::VectorXd& rate,
                                 const Eigen::Vector3d& attitude_error) {
    Eigen::VectorXd chart = Eigen::VectorXd::Zero(chart_size);
    chart.segment<3>(kChartPositionNorth) = rate.segment<3>(core::kPositionNorth);
    chart.segment<3>(kChartVelocityU) = rate.segment<3>(core::kVelocityU);
    const Eigen::Vector3d omega = full.segment<3>(core::kRateP);
    // edot = J_r^-1(e) omega, kept to first order. See the header.
    chart.segment<3>(kChartAttitudeErrorX) = omega + 0.5 * attitude_error.cross(omega);
    chart.segment<3>(kChartRateP) = rate.segment<3>(core::kRateP);
    for (int i = 0; i < appended; ++i) {
      // A frozen coordinate's derivative is DECLARED zero, not measured. Doing
      // it here rather than by clearing a row afterwards means B, and any
      // future rate consumer, gets the same declaration for free — a zeroed A
      // row beside a live B row would be a model in which the state moves only
      // when someone pushes it, which is not what freezing means.
      chart(kRigidChartSize + i) =
          frozen[static_cast<std::size_t>(i)] ? 0.0 : rate(core::kStateSize + i);
    }
    return chart;
  };

  // The point must be an equilibrium in every DYNAMIC coordinate. Position is
  // excluded: a relative equilibrium translates.
  const Eigen::VectorXd nominal_rate = dynamics(extended_state, input);
  if (nominal_rate.size() != extended_state.size()) {
    throw std::invalid_argument(
        "linearize_extended: the dynamics returned a derivative of the wrong length");
  }
  double residual_squared = nominal_rate.segment<3>(core::kVelocityU).squaredNorm()
                            + nominal_rate.segment<3>(core::kRateP).squaredNorm();
  for (int i = 0; i < appended; ++i) {
    // A frozen coordinate is not required to be at rest — see the header. Its
    // rate is recorded below rather than summed into a residual it would fail.
    if (frozen[static_cast<std::size_t>(i)]) {
      continue;
    }
    const double value = nominal_rate(core::kStateSize + i);
    residual_squared += value * value;
  }
  const double residual_norm = std::sqrt(residual_squared);
  if (!(residual_norm <= options.equilibrium_tolerance)) {
    std::ostringstream message;
    message << "linearize_extended: the supplied point is not an equilibrium. Dynamic residual "
            << residual_norm << " exceeds the budget " << options.equilibrium_tolerance
            << ". Linearising here would produce a model with a constant term that A cannot "
               "represent, so the answer would be wrong rather than approximate.";
    throw std::runtime_error(message.str());
  }

  // The wind perturbation holds the GROUND velocity fixed, so a change of wind
  // moves the air-relative velocity state by minus the same vector resolved
  // into body axes. The header derives why this is the physical direction and
  // why taking it leaves A untouched and the model self-consistent. Applied to
  // B and to D identically, because a column of one and a column of the other
  // must describe the same perturbation or the pair does not form a system.
  const Eigen::Matrix3d nominal_ned_from_body = core::dcm_ned_from_body(nominal_attitude);
  const Eigen::Vector3d nominal_wind_ned_m_s =
      options.wind_input_offset >= 0 ? Eigen::Vector3d(input.segment<3>(options.wind_input_offset))
                                     : Eigen::Vector3d::Zero();
  const auto rebase_for_wind = [&](const Eigen::VectorXd& u) {
    Eigen::VectorXd full = extended_state;
    if (options.wind_input_offset >= 0) {
      const Eigen::Vector3d wind_change =
          u.segment<3>(options.wind_input_offset) - nominal_wind_ned_m_s;
      full.segment<3>(core::kVelocityU) -= nominal_ned_from_body.transpose() * wind_change;
    }
    return full;
  };

  const auto observe = [&](const Eigen::VectorXd& full, const Eigen::VectorXd& u) {
    const Eigen::VectorXd rate = dynamics(full, u);
    const core::Quaternion attitude(full(core::kQuaternionW),
                                    full(core::kQuaternionX),
                                    full(core::kQuaternionY),
                                    full(core::kQuaternionZ));
    const Eigen::Matrix3d ned_from_body = core::dcm_ned_from_body(attitude);
    const Eigen::Vector3d velocity = full.segment<3>(core::kVelocityU);
    const Eigen::Vector3d omega = full.segment<3>(core::kRateP);
    const Eigen::Vector3d wind = options.wind_input_offset >= 0
                                     ? Eigen::Vector3d(u.segment<3>(options.wind_input_offset))
                                     : Eigen::Vector3d::Zero();

    int width = 0;
    for (OutputKind kind : options.outputs) {
      width += output_width(kind);
    }
    Eigen::VectorXd y(width);
    int at = 0;
    for (OutputKind kind : options.outputs) {
      switch (kind) {
        case OutputKind::BodySpecificForce: {
          // Recovered from the dynamics rather than asked of the model, so this
          // works for any f: vdot = f_spec + g_body - omega x v, therefore
          // f_spec = vdot + omega x v - R^T g_ned. That inversion is what lets
          // the observation model stay model-generic.
          y.segment<3>(at) = rate.segment<3>(core::kVelocityU) + omega.cross(velocity)
                             - ned_from_body.transpose() * options.gravity_ned_m_s2;
          break;
        }
        case OutputKind::BodyRates:
          y.segment<3>(at) = omega;
          break;
        case OutputKind::PositionNed:
          y.segment<3>(at) = full.segment<3>(core::kPositionNorth);
          break;
        case OutputKind::Altitude:
          y(at) = -full(core::kPositionDown);
          break;
        case OutputKind::GroundVelocityNed:
          y.segment<3>(at) = ned_from_body * velocity + wind;
          break;
      }
      at += output_width(kind);
    }
    return y;
  };

  numerics::JacobianOptions state_options = options.state_jacobian;
  state_options.estimate_truncation_error = options.report_truncation_error;

  // THE CHART DESTROYS THE RELATIVE-STEP RULE, AND THIS PUTS IT BACK.
  //
  // `central_difference_jacobian` sizes each step as
  // max(relative_step * |x_i|, absolute_step), which is right when it is
  // differentiating at the state itself. Here it differentiates at the CHART
  // ORIGIN, where every coordinate is zero by construction, so the relative term
  // is zero for every column and every step collapses to the one absolute floor.
  //
  // That floor is eps^(1/3), chosen for a component of order one. The chart's
  // components are not of order one and not of one kind: a rotor speed sits at
  // several hundred rad/s while a body rate sits at zero. Perturbing a state of
  // 626 by 6e-6 and subtracting leaves about eight of sixteen digits, and the
  // rotor-lag entries of A came out wrong in the eighth figure for exactly that
  // reason — found by a pole gate, not by the Richardson estimate, which cannot
  // see cancellation.
  //
  // So the floors are derived from the magnitude of the state each coordinate
  // perturbs, which is the scale the shared routine would have used had it been
  // handed the state instead of the chart. The attitude coordinates take a scale
  // of one radian: the quaternion they parameterise is a unit, so there is no
  // magnitude to read off it, and one radian is the natural size of a rotation.
  //
  // A caller who supplies floors keeps them. This only fills in a default that
  // the chart would otherwise make meaningless.
  if (state_options.absolute_step_per_component.size() == 0) {
    Eigen::VectorXd floors = Eigen::VectorXd::Constant(chart_size, state_options.absolute_step);
    const auto floor_from = [&](int chart_index, double scale) {
      floors(chart_index) =
          std::fmax(state_options.relative_step * std::abs(scale), state_options.absolute_step);
    };
    for (int axis = 0; axis < 3; ++axis) {
      floor_from(kChartPositionNorth + axis, extended_state(core::kPositionNorth + axis));
      floor_from(kChartVelocityU + axis, extended_state(core::kVelocityU + axis));
      floor_from(kChartAttitudeErrorX + axis, 1.0);
      floor_from(kChartRateP + axis, extended_state(core::kRateP + axis));
    }
    for (int i = 0; i < appended; ++i) {
      floor_from(kRigidChartSize + i, extended_state(core::kStateSize + i));
    }
    state_options.absolute_step_per_component = floors;
  }

  numerics::JacobianOptions input_options = options.input_jacobian;
  input_options.estimate_truncation_error = options.report_truncation_error;

  const Eigen::VectorXd chart_origin = Eigen::VectorXd::Zero(chart_size);

  const numerics::Jacobian a_jacobian = numerics::central_difference_jacobian(
      [&](const Eigen::VectorXd& delta) {
        const Eigen::VectorXd full = unpack(delta);
        return to_chart_rate(full, dynamics(full, input), delta.segment<3>(kChartAttitudeErrorX));
      },
      chart_origin,
      state_options);

  const numerics::Jacobian b_jacobian = numerics::central_difference_jacobian(
      [&](const Eigen::VectorXd& u) {
        const Eigen::VectorXd full = rebase_for_wind(u);
        return to_chart_rate(full, dynamics(full, u), Eigen::Vector3d::Zero());
      },
      input,
      input_options);

  const numerics::Jacobian c_jacobian = numerics::central_difference_jacobian(
      [&](const Eigen::VectorXd& delta) { return observe(unpack(delta), input); },
      chart_origin,
      state_options);

  const numerics::Jacobian d_jacobian = numerics::central_difference_jacobian(
      [&](const Eigen::VectorXd& u) { return observe(rebase_for_wind(u), u); },
      input,
      input_options);

  ExtendedLinearisation result;
  result.a = a_jacobian.value;
  result.b = b_jacobian.value;
  result.c = c_jacobian.value;
  result.d = d_jacobian.value;
  result.state_steps = a_jacobian.steps;
  result.input_steps = b_jacobian.steps;
  result.a_truncation = a_jacobian.truncation_estimate;
  result.b_truncation = b_jacobian.truncation_estimate;
  result.c_truncation = c_jacobian.truncation_estimate;
  result.d_truncation = d_jacobian.truncation_estimate;
  result.worst_relative_truncation_a = a_jacobian.worst_relative_truncation;
  result.worst_relative_truncation_b = b_jacobian.worst_relative_truncation;
  result.worst_relative_truncation_c = c_jacobian.worst_relative_truncation;
  result.worst_relative_truncation_d = d_jacobian.worst_relative_truncation;
  result.worst_relative_truncation = std::fmax(
      std::fmax(a_jacobian.worst_relative_truncation, b_jacobian.worst_relative_truncation),
      std::fmax(c_jacobian.worst_relative_truncation, d_jacobian.worst_relative_truncation));
  result.equilibrium_residual_norm = residual_norm;
  result.equilibrium_tolerance = options.equilibrium_tolerance;

  result.state_names = {"position_north_m",
                        "position_east_m",
                        "position_down_m",
                        "velocity_u_m_s",
                        "velocity_v_m_s",
                        "velocity_w_m_s",
                        "attitude_error_x_rad",
                        "attitude_error_y_rad",
                        "attitude_error_z_rad",
                        "body_rate_p_rad_s",
                        "body_rate_q_rad_s",
                        "body_rate_r_rad_s"};
  for (const std::string& name : options.appended_state_names) {
    result.state_names.push_back(name);
  }
  result.input_names = options.input_names;

  // Recorded after state_names is complete, so a frozen coordinate is reported
  // by the name it carries in the exported matrix rather than by a bare index.
  result.frozen_appended_states = options.frozen_appended_states;
  for (const int index : options.frozen_appended_states) {
    result.frozen_state_names.push_back(
        result.state_names[static_cast<std::size_t>(kRigidChartSize + index)]);
    result.frozen_state_rates.push_back(nominal_rate(core::kStateSize + index));
  }
  for (OutputKind kind : options.outputs) {
    for (const std::string& name : names_for(kind)) {
      result.output_names.push_back(name);
    }
  }

  return result;
}

}  // namespace galata::linearize
