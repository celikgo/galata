// SPDX-License-Identifier: Apache-2.0
//
// Generic deterministic sampled control execution for nonlinear VehicleModel
// plants. The controller and measurement policy are callbacks; the timing,
// sample-and-hold, delay and saturation ordering lives here once.
#ifndef GALATA_SIM_SAMPLED_LOOP_HPP
#define GALATA_SIM_SAMPLED_LOOP_HPP

#include "galata/model/vehicle.hpp"
#include "galata/numerics/integration_method.hpp"

#include <Eigen/Core>

#include <functional>
#include <string>
#include <vector>

namespace galata::sim {

struct SampledTick {
  double time_s = 0.0;  // s
  Eigen::VectorXd measurement;
  Eigen::VectorXd references;
  Eigen::VectorXd errors;
  Eigen::VectorXd requested_controls;
  Eigen::VectorXd saturated_controls;
  Eigen::VectorXd applied_controls;
  Eigen::VectorXd controller_state;
  bool measurement_available = false;
  bool measurement_stale = false;
};

struct SampledLoopResult {
  numerics::IntegrationResult integration;
  std::vector<SampledTick> ticks;
  // Command held from each stored trajectory sample to the next. The first
  // entry is corrected after the t=0 tick, so delayed runs record the command
  // actually applied over their first interval.
  std::vector<Eigen::VectorXd> held_controls;
  int steps_per_controller_tick = 0;
  int tick_count = 0;
};

struct SampledLoopOptions {
  double step_s = 0.0;               // s
  double controller_period_s = 0.0;  // s
  int steps = 0;
  int sample_stride = 1;
  int delay_periods = 0;
  Eigen::VectorXd initial_state;
  Eigen::VectorXd trim_controls;

  // The derivative sees only the command released from the delay line. A
  // mutable model, environment or failure schedule can be captured by this
  // callback without giving numerical stages access to sensor randomness.
  std::function<Eigen::VectorXd(double time_s,
                                const Eigen::VectorXd& state,
                                const Eigen::VectorXd& applied_controls)>
      derivative;
  numerics::ProjectionFunction projection;
  numerics::StateBounds state_bounds;

  // Called at every fixed-step boundary, before the sensor sample and before
  // the controller update. This is where scheduled failures are applied.
  std::function<void(int step, double time_s, Eigen::VectorXd& state)> on_boundary;

  // Called only at controller ticks. It must return a requested command and
  // may carry measurement/controller-state evidence for the tick.
  std::function<SampledTick(int tick, double time_s, const Eigen::VectorXd& state)> on_tick;

  // Called after saturation and delay release, when the command that will be
  // held for the next integration step is known.
  std::function<void(const Eigen::VectorXd& applied_controls)> on_command_applied;

  // Clamps a requested command to the model's declared actuator semantics.
  // The returned vector must retain the command width.
  std::function<Eigen::VectorXd(const Eigen::VectorXd& requested)> saturate;
};

// Ordering at a coincident boundary is fixed and part of the contract:
// failure/event callback -> sensor work inside on_boundary -> controller tick
// callback -> saturation -> delay release -> one zero-order-held integration
// step. No callback is made from RK4 derivative stages for measurements.
[[nodiscard]] SampledLoopResult run_sampled_loop(const SampledLoopOptions& options);

}  // namespace galata::sim

#endif  // GALATA_SIM_SAMPLED_LOOP_HPP
