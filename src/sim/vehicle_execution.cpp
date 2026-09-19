// SPDX-License-Identifier: Apache-2.0

#include "galata/sim/vehicle_execution.hpp"

#include <cmath>
#include <stdexcept>

namespace galata::sim {

VehicleExecutionResult execute_vehicle(const model::VehicleModel& model,
                                       const VehicleExecutionOptions& options) {
  model.validate_vocabulary();
  options.environment.validate();
  if (!(options.step_s > 0.0) || !std::isfinite(options.step_s) || options.steps < 0
      || options.sample_stride < 1 || options.initial_state.size() != model.extended_state_size()
      || !options.initial_state.allFinite() || options.trim_controls.size() != model.control_count()
      || !options.trim_controls.allFinite()) {
    throw std::invalid_argument(
        "execute_vehicle: invalid step, state, control, stride or model dimensions");
  }

  const auto control_at = [&](double time_s, const Eigen::VectorXd& state) {
    const Eigen::VectorXd controls = options.controls ? options.controls(time_s, state)
                                                      : options.trim_controls;
    if (controls.size() != model.control_count() || !controls.allFinite()) {
      throw std::runtime_error("execute_vehicle: control callback returned invalid controls");
    }
    return controls;
  };

  const numerics::DerivativeFunction derivative = [&](double time_s,
                                                       const Eigen::VectorXd& state) {
    return model.derivative(state, control_at(time_s, state), options.environment);
  };
  const numerics::ProjectionFunction projection = options.projection
                                            ? options.projection
                                            : [](Eigen::VectorXd& state) { model::VehicleModel::project(state); };

  numerics::IntegrationOptions integration;
  integration.method = numerics::IntegrationMethod::Rk4Fixed;
  integration.step_s = options.step_s;
  integration.step_count = options.steps;
  integration.sample_stride = options.sample_stride;
  VehicleExecutionResult result;
  result.integration = numerics::integrate(derivative,
                                           options.initial_state,
                                           0.0,
                                           integration,
                                           projection,
                                           options.state_bounds);

  result.controls.reserve(result.integration.trajectory.states.size());
  result.outputs.reserve(result.integration.trajectory.states.size());
  result.envelope.reserve(result.integration.trajectory.states.size());
  for (std::size_t index = 0; index < result.integration.trajectory.states.size(); ++index) {
    const double time_s = result.integration.trajectory.times_s[index];
    const Eigen::VectorXd& state = result.integration.trajectory.states[index];
    const Eigen::VectorXd controls = control_at(time_s, state);
    const core::State rigid = model::VehicleModel::rigid_body_part(state);
    const Eigen::VectorXd auxiliary = model.auxiliary_part(state);
    result.controls.push_back(controls);
    result.outputs.push_back(model.outputs(rigid, auxiliary, controls, options.environment));
    result.envelope.push_back(model.envelope(rigid, auxiliary, controls, options.environment));
  }
  return result;
}

}  // namespace galata::sim
