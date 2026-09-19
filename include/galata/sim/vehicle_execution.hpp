// SPDX-License-Identifier: Apache-2.0
//
// The vehicle-neutral execution service.  Model adapters provide only
// equations and vocabulary; this service owns fixed-step integration,
// quaternion projection, bounds, output sampling and provenance-ready
// trajectories.
#ifndef GALATA_SIM_VEHICLE_EXECUTION_HPP
#define GALATA_SIM_VEHICLE_EXECUTION_HPP

#include "galata/model/vehicle.hpp"
#include "galata/numerics/integration_method.hpp"

#include <functional>
#include <vector>

namespace galata::sim {

struct VehicleExecutionOptions {
  double step_s = 0.0;
  int steps = 0;
  int sample_stride = 1;
  Eigen::VectorXd initial_state;
  Eigen::VectorXd trim_controls;
  model::Environment environment = model::Environment::sea_level_still_air();
  std::function<Eigen::VectorXd(double time_s, const Eigen::VectorXd& state)> controls;
  numerics::ProjectionFunction projection;
  numerics::StateBounds state_bounds;
};

struct VehicleExecutionResult {
  numerics::IntegrationResult integration;
  std::vector<Eigen::VectorXd> controls;
  std::vector<Eigen::VectorXd> outputs;
  std::vector<model::EnvelopeStatus> envelope;
};

// Executes any VehicleModel, including all built-in fixed-wing, multirotor and
// helicopter adapters.  A null control callback means the trim command is
// held.  Controller callbacks are intentionally evaluated at every RK stage;
// sampled control belongs to run_sampled_loop, whose timing contract is
// separate and explicit.
[[nodiscard]] VehicleExecutionResult execute_vehicle(
    const model::VehicleModel& model,
    const VehicleExecutionOptions& options);

}  // namespace galata::sim

#endif  // GALATA_SIM_VEHICLE_EXECUTION_HPP
