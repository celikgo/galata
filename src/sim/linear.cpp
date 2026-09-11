// SPDX-License-Identifier: Apache-2.0
//
// Reference: Hairer, Norsett & Wanner, Solving Ordinary Differential Equations I,
// 2nd revised ed., Springer, 1993. R(z)=1+z+z^2/2+z^3/6+z^4/24 is the classical
// RK4 stability polynomial; stable physical poles must satisfy |R(h lambda)|<=1.
// Validity limits are documented in linear.hpp.
#include "galata/sim/linear.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>

namespace galata::sim {
namespace {

void require_finite_run(double step_s, int step_count, int sample_stride) {
  if (!std::isfinite(step_s) || !(step_s > 0.0) || step_count < 0 || sample_stride < 1
      || !std::isfinite(step_s * static_cast<double>(step_count))) {
    throw std::invalid_argument("simulate_linear: step, count and stride must define a finite run");
  }
}

void require_rk4_stable(const model::LinearSystem& system, double step_s) {
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
}

void require_finite_state(Eigen::VectorXd& state) {
  if (!state.allFinite()) {
    throw std::runtime_error("simulate_linear: integration produced a non-finite state");
  }
}

}  // namespace

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
  require_finite_run(step_s, step_count, sample_stride);
  require_rk4_stable(system, step_s);
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
    require_finite_state(state);
    const int completed = step + 1;
    if (completed % sample_stride == 0 || completed == step_count) {
      trajectory.times_s.push_back(static_cast<double>(completed) * step_s);
      trajectory.states.push_back(state);
    }
  }
  return trajectory;
}

ScheduledLinearRun simulate_linear(const model::LinearSystem& system,
                                   const Eigen::VectorXd& initial_state,
                                   const InputSchedule& input,
                                   double step_s,
                                   int step_count,
                                   int sample_stride) {
  system.validate();
  if (initial_state.size() != system.state_count() || !initial_state.allFinite()) {
    throw std::invalid_argument("simulate_linear: state dimensions or values are invalid");
  }
  if (input.empty()) {
    throw std::invalid_argument(
        "simulate_linear: the input history is empty. A run with one input throughout is the "
        "constant-input form; a history needs at least one sample");
  }
  if (system.input_count() == 0) {
    throw std::invalid_argument(
        "simulate_linear: the system has no inputs, so there is nothing for a history to drive");
  }
  if (input.width() != system.input_count()) {
    std::ostringstream message;
    message << "simulate_linear: the input history carries " << input.width()
            << " channel(s) and the system has " << system.input_count()
            << " input(s). Each sample must give every input, in the system's input order";
    throw std::invalid_argument(message.str());
  }
  require_finite_run(step_s, step_count, sample_stride);
  require_rk4_stable(system, step_s);

  // OUTSIDE THE SPAN. Under `refuse` this throws for a horizon the history does
  // not cover, before a single step is taken; under `hold` it clamps.
  const double end_s = static_cast<double>(step_count) * step_s;
  (void)input.at(0.0);
  (void)input.at(end_s);

  // EVENTS. Every zero-order change the run can see — from the start to the end
  // inclusive — must fall on a step boundary. The map keeps the event's OWN
  // time, so the value is read there rather than at step * step_s, which can
  // land one rounding error before the event and return the old value.
  std::map<int, double> events;
  for (const double time_s : input.discontinuities()) {
    if (time_s < 0.0 || time_s > end_s) {
      continue;
    }
    const double exact = time_s / step_s;
    const double nearest = std::round(exact);
    if (std::fabs(exact - nearest) > 1e-9 * std::fmax(1.0, std::fabs(exact))) {
      std::ostringstream message;
      message << "simulate_linear: the input history changes at t = " << time_s
              << " s, which is not a whole number of " << step_s
              << " s steps from the start. A zero-order jump inside a step is not "
                 "representable: RK4's stages would straddle it and the method would be first "
                 "order there. Align the history to the step, or choose a step that divides it";
      throw std::invalid_argument(message.str());
    }
    events.emplace(static_cast<int>(nearest), time_s);
  }

  const bool per_stage = input.hold() == HoldPolicy::Linear;
  const auto value_at_step = [&](int step) -> Eigen::VectorXd {
    if (!per_stage) {
      const auto found = events.find(step);
      if (found != events.end()) {
        return input.at(found->second);
      }
    }
    return input.at(std::clamp(static_cast<double>(step) * step_s, 0.0, end_s));
  };

  // Under a zero-order hold the forcing is formed exactly as the constant-input
  // overload forms it — B times the held value, once per segment — so a history
  // that never changes reproduces that overload bit for bit.
  Eigen::VectorXd forcing = system.b * value_at_step(0);
  const numerics::DerivativeFunction derivative = [&](double time_s, const Eigen::VectorXd& state) {
    Eigen::VectorXd rate;
    if (per_stage) {
      // Stage times can pass the end by a rounding error; the system is time
      // invariant, so clamping the lookup changes nothing but the lookup.
      rate = system.a * state + system.b * input.at(std::clamp(time_s, 0.0, end_s));
    } else {
      rate = system.a * state + forcing;
    }
    if (!rate.allFinite()) {
      throw std::runtime_error("simulate_linear: state derivative overflowed");
    }
    return rate;
  };

  ScheduledLinearRun run;
  numerics::Trajectory& trajectory = run.trajectory;
  trajectory.step_s = step_s;
  trajectory.step_count = step_count;
  trajectory.sample_stride = sample_stride;
  Eigen::VectorXd state = initial_state;
  trajectory.times_s.push_back(0.0);
  trajectory.states.push_back(state);
  run.input_samples.push_back(value_at_step(0));
  for (int step = 0; step < step_count; ++step) {
    if (!per_stage && step > 0 && events.count(step) != 0) {
      forcing = system.b * value_at_step(step);
      run.event_steps.push_back(step);
    }
    const double time = static_cast<double>(step) * step_s;
    state = numerics::rk4_step(derivative, time, state, step_s);
    require_finite_state(state);
    const int completed = step + 1;
    if (completed % sample_stride == 0 || completed == step_count) {
      trajectory.times_s.push_back(static_cast<double>(completed) * step_s);
      trajectory.states.push_back(state);
      run.input_samples.push_back(value_at_step(completed));
    }
  }
  return run;
}

}  // namespace galata::sim
