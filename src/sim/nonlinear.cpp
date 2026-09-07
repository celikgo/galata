// SPDX-License-Identifier: Apache-2.0
//
// References: Stevens, Lewis & Johnson, Aircraft Control and Simulation,
// 3rd ed., Wiley, 2016, chapters 2-3; Hairer, Norsett & Wanner, Solving Ordinary
// Differential Equations I, 2nd revised ed., Springer, 1993. Validity limits:
// nonlinear.hpp. The actuator equations are
// u_dot = clamp((clamp(u_command, min, max)-u)/tau, -rate, rate).
// Commands and feedback are evaluated at every classical RK4 stage; quaternion
// normalisation and actuator position projection follow each completed step.

#include "galata/sim/nonlinear.hpp"

#include "galata/core/quaternion.hpp"
#include "galata/linearize/finite_difference.hpp"
#include "galata/numerics/integrator.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace galata::sim {
namespace {

using Flags = std::array<bool, model::Controls::kSize>;
constexpr int kAugmentedSize = core::kStateSize + model::Controls::kSize;

std::vector<int> named_indices(const std::vector<std::string>& names,
                               const std::vector<std::string>& allowed) {
  std::vector<int> indices;
  for (const auto& name : names) {
    const auto found = std::find(allowed.begin(), allowed.end(), name);
    if (found == allowed.end()) {
      throw std::invalid_argument("simulate_nonlinear: unsupported state or input name '" + name
                                  + "'");
    }
    const int index = static_cast<int>(found - allowed.begin());
    if (std::find(indices.begin(), indices.end(), index) != indices.end()) {
      throw std::invalid_argument("simulate_nonlinear: duplicate state or input name '" + name
                                  + "'");
    }
    indices.push_back(index);
  }
  return indices;
}

Eigen::VectorXd euler_coordinates(const core::State& state) {
  const auto angles = core::euler_from_quaternion(core::normalised(state.attitude_body_to_ned));
  Eigen::VectorXd coordinates(linearize::kEulerStateSize);
  coordinates << state.position_ned_m, state.velocity_body_m_s, angles.roll_rad, angles.pitch_rad,
      angles.yaw_rad, state.angular_rate_body_rad_s;
  return coordinates;
}

core::State state_from_euler(const Eigen::VectorXd& coordinates) {
  core::State state;
  state.position_ned_m = coordinates.segment<3>(linearize::kPositionNorth);
  state.velocity_body_m_s = coordinates.segment<3>(linearize::kVelocityU);
  state.attitude_body_to_ned = core::quaternion_from_euler({coordinates(linearize::kRoll),
                                                            coordinates(linearize::kPitch),
                                                            coordinates(linearize::kYaw)});
  state.angular_rate_body_rad_s = coordinates.segment<3>(linearize::kRateP);
  return state;
}

void require_state(const core::State& state) {
  const double norm = state.attitude_body_to_ned.norm();
  if (!state.to_vector().allFinite() || !std::isfinite(norm) || !(norm > 0.0)) {
    throw std::runtime_error("simulate_nonlinear: non-finite state or invalid quaternion");
  }
}

struct Actuation {
  Eigen::VectorXd requested;
  Eigen::VectorXd actual;
  Eigen::VectorXd derivative;
  Flags position_limited{};
  Flags rate_limited{};
};

}  // namespace

NonlinearResult simulate_nonlinear(const model::Aircraft& aircraft,
                                   const trim::TrimPoint& trim,
                                   const NonlinearRequest& request) {
  aircraft.validate();
  if (!std::isfinite(request.step_s) || !(request.step_s > 0.0) || request.step_count < 0
      || request.sample_stride < 1
      || !std::isfinite(request.step_s * static_cast<double>(request.step_count))) {
    throw std::invalid_argument("simulate_nonlinear: invalid integration step, count or stride");
  }
  if (!std::isfinite(trim.flight_path_angle_rad) || trim.flight_path_angle_rad != 0.0) {
    throw std::invalid_argument(
        "simulate_nonlinear: the constant local reference requires a level-flight trim");
  }
  require_state(trim.state);
  if (std::abs(trim.state.attitude_body_to_ned.norm() - 1.0) > 1e-10) {
    throw std::invalid_argument("simulate_nonlinear: trim quaternion must be unit length");
  }
  const auto trim_rate =
      aircraft.derivative(trim.state, trim.controls, trim.atmosphere.delta_isa_k);
  Eigen::Matrix<double, 6, 1> trim_accelerations;
  trim_accelerations << trim_rate.segment<3>(core::kVelocityU), trim_rate.segment<3>(core::kRateP);
  if (!std::isfinite(trim.residual_tolerance) || !(trim.residual_tolerance > 0.0)
      || !trim_rate.allFinite() || !(trim_accelerations.norm() <= trim.residual_tolerance)) {
    throw std::invalid_argument("simulate_nonlinear: reference does not satisfy trim equilibrium");
  }
  const Eigen::VectorXd trim_controls = trim.controls.to_vector();
  const Eigen::VectorXd increment = request.command_increment.to_vector();
  if (!trim_controls.allFinite() || !increment.allFinite()
      || !request.initial_perturbation.allFinite()) {
    throw std::invalid_argument("simulate_nonlinear: controls and perturbations must be finite");
  }
  for (int channel = 0; channel < model::Controls::kSize; ++channel) {
    const auto& limits = request.actuators[static_cast<std::size_t>(channel)];
    if (!std::isfinite(limits.minimum) || !std::isfinite(limits.maximum)
        || !(limits.maximum > limits.minimum) || !std::isfinite(limits.rate_limit_per_s)
        || !(limits.rate_limit_per_s > 0.0) || !std::isfinite(limits.time_constant_s)
        || !(limits.time_constant_s > 0.0)) {
      throw std::invalid_argument(
          "simulate_nonlinear: every actuator needs finite position bounds, a positive rate "
          "limit and a positive time constant");
    }
    if (trim_controls(channel) < limits.minimum || trim_controls(channel) > limits.maximum) {
      throw std::invalid_argument("simulate_nonlinear: trim control lies outside actuator limits");
    }
    // h <= tau keeps a resolved actuator lag; it is stricter than RK4's
    // absolute stability boundary, and does not replace a step-halving check.
    if (request.step_s > limits.time_constant_s) {
      throw std::invalid_argument(
          "simulate_nonlinear: step_s must not exceed any actuator time_constant_s");
    }
  }

  const auto state_names = linearize::euler_state_names();
  const auto perturbations = named_indices(request.perturbation_state_names, state_names);
  if (static_cast<Eigen::Index>(perturbations.size()) != request.initial_perturbation.size()) {
    throw std::invalid_argument("simulate_nonlinear: perturbation values and names differ in size");
  }
  const auto feedback_states = named_indices(request.feedback.state_names, state_names);
  const auto feedback_inputs =
      named_indices(request.feedback.input_names, linearize::control_names());
  const bool has_feedback = request.feedback.k.size() != 0;
  if (!request.feedback.k.allFinite()
      || request.feedback.k.rows() != static_cast<Eigen::Index>(feedback_inputs.size())
      || request.feedback.k.cols() != static_cast<Eigen::Index>(feedback_states.size())
      || (!has_feedback && (!feedback_states.empty() || !feedback_inputs.empty()))) {
    throw std::invalid_argument(
        "simulate_nonlinear: feedback gain dimensions or values are invalid");
  }

  const Eigen::VectorXd reference = euler_coordinates(trim.state);
  const Eigen::Vector3d reference_velocity = core::velocity_ned(trim.state);
  Eigen::VectorXd initial_coordinates = reference;
  for (std::size_t index = 0; index < perturbations.size(); ++index) {
    initial_coordinates(perturbations[index]) +=
        request.initial_perturbation(static_cast<Eigen::Index>(index));
  }
  Eigen::VectorXd augmented(kAugmentedSize);
  augmented << state_from_euler(initial_coordinates).to_vector(), trim_controls;

  NonlinearResult result;
  result.step_s = request.step_s;
  result.requested_steps = request.step_count;
  Flags step_position_limited{};
  Flags step_rate_limited{};
  double evaluation_time = 0.0;

  const auto actuation = [&](double time, const core::State& state, const Eigen::VectorXd& actual) {
    Actuation output;
    output.requested = trim_controls + increment;
    if (has_feedback) {
      if (core::gimbal_lock_proximity(core::normalised(state.attitude_body_to_ned)) < 0.1) {
        throw std::runtime_error("simulate_nonlinear: Euler feedback chart is ill-conditioned");
      }
      Eigen::VectorXd deviation = euler_coordinates(state) - reference;
      deviation.head<3>() -= time * reference_velocity;
      for (int index = linearize::kRoll; index <= linearize::kYaw; ++index) {
        deviation(index) = std::remainder(deviation(index), 2.0 * std::numbers::pi_v<double>);
      }
      Eigen::VectorXd selected(static_cast<Eigen::Index>(feedback_states.size()));
      for (std::size_t index = 0; index < feedback_states.size(); ++index) {
        selected(static_cast<Eigen::Index>(index)) = deviation(feedback_states[index]);
      }
      const Eigen::VectorXd correction = request.feedback.k * selected;
      for (std::size_t index = 0; index < feedback_inputs.size(); ++index) {
        output.requested(feedback_inputs[index]) -= correction(static_cast<Eigen::Index>(index));
      }
    }
    if (!output.requested.allFinite() || !actual.allFinite()) {
      throw std::runtime_error("simulate_nonlinear: non-finite actuator command or state");
    }
    output.actual.resize(model::Controls::kSize);
    output.derivative.resize(model::Controls::kSize);
    for (int channel = 0; channel < model::Controls::kSize; ++channel) {
      const auto index = static_cast<std::size_t>(channel);
      const auto& limits = request.actuators[index];
      const double command = std::clamp(output.requested(channel), limits.minimum, limits.maximum);
      output.actual(channel) = std::clamp(actual(channel), limits.minimum, limits.maximum);
      const double rate = (command - actual(channel)) / limits.time_constant_s;
      output.derivative(channel) =
          std::clamp(rate, -limits.rate_limit_per_s, limits.rate_limit_per_s);
      output.position_limited[index] =
          command != output.requested(channel) || output.actual(channel) != actual(channel);
      output.rate_limited[index] = output.derivative(channel) != rate;
    }
    return output;
  };

  const auto envelope_at = [&](const core::State& state) {
    require_state(state);
    const auto atmosphere = core::isa(-state.position_ned_m.z(), trim.atmosphere.delta_isa_k);
    const auto envelope = aircraft.envelope(state, atmosphere);
    result.max_alpha_departure_rad =
        std::max(result.max_alpha_departure_rad, envelope.alpha_departure_rad);
    result.max_mach_departure = std::max(result.max_mach_departure, envelope.mach_departure);
    result.outside_envelope_encountered |= envelope.outside_advisory_envelope;
    return envelope;
  };

  const auto sample = [&](double time, const Eigen::VectorXd& vector) {
    NonlinearSample point;
    point.time_s = time;
    point.state = core::State::from_vector(vector.head<core::kStateSize>());
    require_state(point.state);
    const auto output = actuation(time, point.state, vector.tail<model::Controls::kSize>());
    point.controls = model::Controls::from_vector(output.actual);
    point.command = model::Controls::from_vector(output.requested);
    point.position_limited = output.position_limited;
    point.rate_limited = output.rate_limited;
    for (std::size_t index = 0; index < step_position_limited.size(); ++index) {
      point.position_limited[index] = point.position_limited[index] || step_position_limited[index];
      point.rate_limited[index] = point.rate_limited[index] || step_rate_limited[index];
    }
    point.envelope = envelope_at(point.state);
    return point;
  };

  const numerics::DerivativeFunction derivative = [&](double time, const Eigen::VectorXd& vector) {
    evaluation_time = time;
    const auto state = core::State::from_vector(vector.head<core::kStateSize>());
    const auto envelope = envelope_at(state);
    if (envelope.outside_advisory_envelope && request.stop_outside_envelope) {
      throw std::runtime_error(
          "simulate_nonlinear: aircraft left the advisory alpha/Mach envelope");
    }
    const auto output = actuation(time, state, vector.tail<model::Controls::kSize>());
    for (std::size_t index = 0; index < step_position_limited.size(); ++index) {
      step_position_limited[index] = step_position_limited[index] || output.position_limited[index];
      step_rate_limited[index] = step_rate_limited[index] || output.rate_limited[index];
    }
    Eigen::VectorXd rate(kAugmentedSize);
    rate << aircraft.derivative(
        state, model::Controls::from_vector(output.actual), trim.atmosphere.delta_isa_k),
        output.derivative;
    if (!rate.allFinite()) {
      throw std::runtime_error("simulate_nonlinear: non-finite state derivative");
    }
    return rate;
  };

  try {
    result.samples.push_back(sample(0.0, augmented));
    // Also evaluate a zero-duration request, so an invalid initial state is
    // never reported as a successful trajectory simply because no step ran.
    (void)derivative(0.0, augmented);
    for (int step = 0; step < request.step_count; ++step) {
      step_position_limited.fill(false);
      step_rate_limited.fill(false);
      const double time = static_cast<double>(step) * request.step_s;
      Eigen::VectorXd next = numerics::rk4_step(derivative, time, augmented, request.step_s);
      auto state = core::State::from_vector(next.head<core::kStateSize>());
      require_state(state);
      state.renormalise_attitude();
      next.head<core::kStateSize>() = state.to_vector();
      for (int channel = 0; channel < model::Controls::kSize; ++channel) {
        const auto index = static_cast<std::size_t>(channel);
        const auto& limits = request.actuators[index];
        const double position = next(core::kStateSize + channel);
        next(core::kStateSize + channel) = std::clamp(position, limits.minimum, limits.maximum);
        step_position_limited[index] =
            step_position_limited[index] || position != next(core::kStateSize + channel);
      }
      evaluation_time = static_cast<double>(step + 1) * request.step_s;
      const auto next_sample = sample(evaluation_time, next);
      if (next_sample.envelope.outside_advisory_envelope && request.stop_outside_envelope) {
        throw std::runtime_error(
            "simulate_nonlinear: aircraft left the advisory alpha/Mach envelope");
      }
      augmented = std::move(next);
      result.completed_steps = step + 1;
      for (std::size_t index = 0; index < step_position_limited.size(); ++index) {
        result.position_limited_steps[index] += step_position_limited[index] ? 1 : 0;
        result.rate_limited_steps[index] += step_rate_limited[index] ? 1 : 0;
      }
      if (result.completed_steps % request.sample_stride == 0
          || result.completed_steps == request.step_count) {
        result.samples.push_back(next_sample);
      }
    }
    result.completed = true;
    result.termination_reason = "completed";
  } catch (const std::exception& error) {
    result.termination_reason = error.what();
  }
  result.termination_time_s = evaluation_time;
  const double last_time = static_cast<double>(result.completed_steps) * request.step_s;
  if (result.completed_steps > 0
      && (result.samples.empty() || result.samples.back().time_s != last_time)) {
    // A failed intermediate stage does not commit a step. Preserve the last
    // accepted state even when it falls between requested output samples.
    step_position_limited.fill(false);
    step_rate_limited.fill(false);
    result.samples.push_back(sample(last_time, augmented));
  }
  return result;
}

}  // namespace galata::sim
