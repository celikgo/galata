// SPDX-License-Identifier: Apache-2.0
//
// Reference: Hairer, Norsett & Wanner, Solving Ordinary Differential Equations I,
// 2nd revised ed., Springer, 1993. R(z)=1+z+z^2/2+z^3/6+z^4/24 is the classical
// RK4 stability polynomial; stable physical poles must satisfy |R(h lambda)|<=1.
// Validity limits are documented in linear.hpp.
#include "galata/sim/linear.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <stdexcept>

namespace galata::sim {

numerics::Trajectory simulate_linear(const model::LinearSystem& system,
                                     const Eigen::VectorXd& initial_state,
                                     const Eigen::VectorXd& constant_input,
                                     double step_s,
                                     int step_count,
                                     int sample_stride) {
  system.validate();
  if (initial_state.size() != system.state_count() || constant_input.size() != system.input_count()
      || !initial_state.allFinite() || !constant_input.allFinite()) {
    throw std::invalid_argument("simulate_linear: state/input dimensions or values are invalid");
  }
  if (!std::isfinite(step_s) || !(step_s > 0.0) || step_count < 0 || sample_stride < 1
      || !std::isfinite(step_s * static_cast<double>(step_count))) {
    throw std::invalid_argument("simulate_linear: step, count and stride must define a finite run");
  }
  Eigen::EigenSolver<Eigen::MatrixXd> solver(system.a, /*computeEigenvectors=*/false);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    throw std::runtime_error("simulate_linear: eigenvalue computation failed");
  }
  for (Eigen::Index index = 0; index < solver.eigenvalues().size(); ++index) {
    const auto pole = solver.eigenvalues()(index);
    const auto z = step_s * pole;
    const auto amplification = 1.0 + z * (1.0 + z * (0.5 + z * (1.0 / 6.0 + z / 24.0)));
    if (!std::isfinite(std::abs(amplification))
        || (pole.real() <= 0.0 && std::abs(amplification) > 1.0)) {
      throw std::invalid_argument("simulate_linear: step lies outside the RK4 stability region");
    }
  }
  Eigen::VectorXd forcing = Eigen::VectorXd::Zero(system.state_count());
  if (system.input_count() > 0) {
    forcing = system.b * constant_input;
  }
  const numerics::DerivativeFunction derivative = [&](double, const Eigen::VectorXd& state) {
    const Eigen::VectorXd rate = system.a * state + forcing;
    if (!rate.allFinite()) {
      throw std::runtime_error("simulate_linear: state derivative overflowed");
    }
    return rate;
  };
  const numerics::ProjectionFunction finite = [](Eigen::VectorXd& state) {
    if (!state.allFinite()) {
      throw std::runtime_error("simulate_linear: integration produced a non-finite state");
    }
  };
  numerics::Trajectory trajectory;
  trajectory.step_s = step_s;
  trajectory.step_count = step_count;
  trajectory.sample_stride = sample_stride;
  Eigen::VectorXd state = initial_state;
  trajectory.times_s.push_back(0.0);
  trajectory.states.push_back(state);
  for (int step = 0; step < step_count; ++step) {
    const double time = static_cast<double>(step) * step_s;
    state = numerics::rk4_step(derivative, time, state, step_s);
    finite(state);
    const int completed = step + 1;
    if (completed % sample_stride == 0 || completed == step_count) {
      trajectory.times_s.push_back(static_cast<double>(completed) * step_s);
      trajectory.states.push_back(state);
    }
  }
  return trajectory;
}

}  // namespace galata::sim
