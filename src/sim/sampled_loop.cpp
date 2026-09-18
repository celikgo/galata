// SPDX-License-Identifier: Apache-2.0

#include "galata/sim/sampled_loop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::sim {

SampledLoopResult run_sampled_loop(const SampledLoopOptions& options) {
  if (!(options.step_s > 0.0) || !std::isfinite(options.step_s)
      || !(options.controller_period_s > 0.0) || !std::isfinite(options.controller_period_s)
      || options.steps < 1 || options.sample_stride < 1 || options.delay_periods < 0
      || options.initial_state.size() < 1 || !options.initial_state.allFinite()
      || options.trim_controls.size() < 1 || !options.trim_controls.allFinite()
      || !options.derivative || !options.on_tick || !options.saturate) {
    throw std::invalid_argument("run_sampled_loop: incomplete or non-finite options");
  }
  const double ratio = options.controller_period_s / options.step_s;
  const double nearest = std::round(ratio);
  if (nearest < 1.0 || nearest > static_cast<double>(std::numeric_limits<int>::max())
      || std::fabs(ratio - nearest) > 1.0e-12 * std::fmax(1.0, std::fabs(ratio))) {
    std::ostringstream message;
    message << "run_sampled_loop: controller_period_s must be a whole number of integration "
               "steps; period "
            << options.controller_period_s << " s and step " << options.step_s << " s are not";
    throw std::invalid_argument(message.str());
  }
  const int steps_per_tick = static_cast<int>(nearest);
  if (options.steps % steps_per_tick != 0) {
    throw std::invalid_argument(
        "run_sampled_loop: the integration horizon must end on a controller tick");
  }
  const int tick_count = options.steps / steps_per_tick;
  const Eigen::Index controls = options.trim_controls.size();
  std::vector<Eigen::VectorXd> delay_queue(static_cast<std::size_t>(options.delay_periods) + 1,
                                           options.trim_controls);
  std::size_t delay_head = 0;
  Eigen::VectorXd applied_controls = options.trim_controls;
  Eigen::VectorXd state = options.initial_state;

  SampledLoopResult result;
  result.steps_per_controller_tick = steps_per_tick;
  result.tick_count = tick_count;
  result.integration.method = numerics::IntegrationMethod::Rk4Fixed;
  result.integration.trajectory.step_s = options.step_s;
  result.integration.trajectory.sample_stride = options.sample_stride;
  result.integration.trajectory.times_s.push_back(0.0);
  result.integration.trajectory.states.push_back(state);
  result.held_controls.push_back(applied_controls);

  for (int step = 0; step <= options.steps; ++step) {
    const double time_s = static_cast<double>(step) * options.step_s;
    if (options.on_boundary) {
      options.on_boundary(step, time_s, state);
      if (step == 0) {
        result.integration.trajectory.states.front() = state;
      }
    }
    if (step % steps_per_tick == 0 && step < options.steps) {
      const int tick = step / steps_per_tick;
      SampledTick tick_record = options.on_tick(tick, time_s, state);
      if (tick_record.requested_controls.size() != controls
          || !tick_record.requested_controls.allFinite()) {
        throw std::invalid_argument(
            "run_sampled_loop: controller returned the wrong number of finite controls");
      }
      tick_record.time_s = time_s;
      tick_record.saturated_controls = options.saturate(tick_record.requested_controls);
      if (tick_record.saturated_controls.size() != controls
          || !tick_record.saturated_controls.allFinite()) {
        throw std::invalid_argument(
            "run_sampled_loop: saturation returned the wrong number of finite controls");
      }
      delay_queue[delay_head] = tick_record.saturated_controls;
      delay_head = (delay_head + 1) % delay_queue.size();
      tick_record.applied_controls = delay_queue[delay_head];
      if (!tick_record.applied_controls.allFinite()) {
        throw std::runtime_error("run_sampled_loop: the delay line produced a non-finite command");
      }
      applied_controls = tick_record.applied_controls;
      if (options.on_command_applied) {
        options.on_command_applied(applied_controls);
      }
      if (!result.held_controls.empty() && result.integration.trajectory.times_s.back() == time_s) {
        result.held_controls.back() = applied_controls;
      }
      result.ticks.push_back(std::move(tick_record));
    }
    if (step == options.steps) {
      break;
    }
    const auto derivative = [&](double time, const Eigen::VectorXd& current) {
      return options.derivative(time, current, applied_controls);
    };
    numerics::IntegrationOptions integration;
    integration.method = numerics::IntegrationMethod::Rk4Fixed;
    integration.step_s = options.step_s;
    integration.step_count = 1;
    integration.sample_stride = 1;
    const auto one_step = numerics::integrate(
        derivative, state, time_s, integration, options.projection, options.state_bounds);
    result.integration.newton_iterations_performed += one_step.newton_iterations_performed;
    if (!one_step.completed()) {
      result.integration.reason = one_step.reason;
      result.integration.detail = one_step.detail;
      result.integration.steps_taken = step + one_step.steps_taken;
      result.integration.termination_time_s = one_step.termination_time_s;
      result.integration.trajectory.step_count = result.integration.steps_taken;
      return result;
    }
    state = one_step.trajectory.states.back();
    const int completed_step = step + 1;
    if (completed_step % options.sample_stride == 0 || completed_step == options.steps) {
      result.integration.trajectory.times_s.push_back(static_cast<double>(completed_step)
                                                      * options.step_s);
      result.integration.trajectory.states.push_back(state);
      result.held_controls.push_back(applied_controls);
    }
  }
  result.integration.steps_taken = options.steps;
  result.integration.termination_time_s = static_cast<double>(options.steps) * options.step_s;
  result.integration.trajectory.step_count = options.steps;
  return result;
}

}  // namespace galata::sim
