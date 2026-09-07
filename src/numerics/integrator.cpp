// SPDX-License-Identifier: Apache-2.0
//
// Classical fourth-order Runge-Kutta, fixed step.
//
// Reference:
//   E. Hairer, S. P. Norsett and G. Wanner, "Solving Ordinary Differential
//   Equations I: Nonstiff Problems", 2nd revised ed., Springer, 1993.
//   The method is Kutta's (1901); the tableau is in every reference above.
//
// Validity envelope and known error behaviour are in the header's
// "WHAT THIS IS NOT" block. Local truncation error is O(h^5) per step and
// global error O(h^4); the constant depends on the fifth derivative of the
// solution, which for a rigid body is set by the fastest rotational mode.

#include "galata/numerics/integrator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace galata::numerics {

Eigen::VectorXd rk4_step(const DerivativeFunction& derivative,
                         double time_s,
                         const Eigen::VectorXd& state,
                         double step_s) {
  if (!derivative || state.size() == 0 || !state.allFinite() || !std::isfinite(time_s)
      || !std::isfinite(step_s) || !(step_s > 0.0)) {
    throw std::invalid_argument(
        "rk4_step: derivative/state must be non-empty, state/time finite and step positive/finite");
  }
  const double half = 0.5 * step_s;
  if (!(step_s / 6.0 > 0.0) || !(time_s + half > time_s) || !(time_s + step_s > time_s + half)
      || !std::isfinite(time_s + step_s)) {
    throw std::invalid_argument("rk4_step: stage times or integration weight are unrepresentable");
  }
  const auto evaluate = [&](double time, const Eigen::VectorXd& point) -> Eigen::VectorXd {
    if (!point.allFinite()) {
      throw std::runtime_error("rk4_step: intermediate state overflowed");
    }
    const Eigen::VectorXd rate = derivative(time, point);
    if (rate.size() != state.size() || !rate.allFinite()) {
      throw std::runtime_error("rk4_step: derivative dimension changed or value is non-finite");
    }
    return rate;
  };

  const Eigen::VectorXd k1 = evaluate(time_s, state);
  const Eigen::VectorXd k2 = evaluate(time_s + half, state + half * k1);
  const Eigen::VectorXd k3 = evaluate(time_s + half, state + half * k2);
  const Eigen::VectorXd k4 = evaluate(time_s + step_s, state + step_s * k3);

  // Summed as (k1 + 2(k2 + k3) + k4) * h/6 rather than as four separately
  // scaled terms. The grouping is fixed and written out because floating-point
  // addition is not associative: a different grouping gives a different last
  // bit, and ADR-0004's bit-identity tier is a claim about this expression.
  const Eigen::VectorXd next = state + (step_s / 6.0) * (k1 + 2.0 * (k2 + k3) + k4);
  if (!next.allFinite()) {
    throw std::runtime_error("rk4_step: completed state overflowed");
  }
  return next;
}

Trajectory integrate_fixed_step(const DerivativeFunction& derivative,
                                const Eigen::VectorXd& initial_state,
                                double initial_time_s,
                                double step_s,
                                int step_count,
                                int sample_stride,
                                const ProjectionFunction& projection) {
  if (step_count < 0) {
    throw std::invalid_argument("integrate_fixed_step: step_count is " + std::to_string(step_count)
                                + ", must be non-negative");
  }
  if (sample_stride < 1) {
    throw std::invalid_argument("integrate_fixed_step: sample_stride is "
                                + std::to_string(sample_stride) + ", must be at least 1");
  }
  if (!std::isfinite(step_s) || !(step_s > 0.0)) {
    throw std::invalid_argument("integrate_fixed_step: step is " + std::to_string(step_s)
                                + ", must be positive and finite");
  }
  if (!derivative || initial_state.size() == 0 || !initial_state.allFinite()
      || !std::isfinite(initial_time_s) || !std::isfinite(step_s * static_cast<double>(step_count))
      || !std::isfinite(initial_time_s + step_s * static_cast<double>(step_count))) {
    throw std::invalid_argument(
        "integrate_fixed_step: derivative/state must be non-empty and state/time span finite");
  }

  const auto require_projection = [&](const Eigen::VectorXd& projected) {
    if (projected.size() != initial_state.size() || !projected.allFinite()) {
      throw std::runtime_error(
          "integrate_fixed_step: projection changed state dimension or produced non-finite values");
    }
  };

  Trajectory trajectory;
  trajectory.step_s = step_s;
  trajectory.step_count = step_count;
  trajectory.sample_stride = sample_stride;

  const std::size_t samples = static_cast<std::size_t>(step_count / sample_stride) + 1U;
  trajectory.times_s.reserve(samples);
  trajectory.states.reserve(samples);

  Eigen::VectorXd state = initial_state;
  if (projection) {
    // Project the initial condition too. A caller who hands over a quaternion
    // that is unit only to six digits should not have that error integrated.
    projection(state);
    require_projection(state);
  }

  trajectory.times_s.push_back(initial_time_s);
  trajectory.states.push_back(state);

  for (int step = 0; step < step_count; ++step) {
    // t0 + k*h, not an accumulated sum: accumulation drifts by O(k*eps) over a
    // long run, and the drift is in the argument the derivative is evaluated at.
    const double time_s = initial_time_s + static_cast<double>(step) * step_s;
    state = rk4_step(derivative, time_s, state, step_s);
    if (projection) {
      projection(state);
      require_projection(state);
    }
    const int completed = step + 1;
    if (completed % sample_stride == 0) {
      trajectory.times_s.push_back(initial_time_s + static_cast<double>(completed) * step_s);
      trajectory.states.push_back(state);
    }
  }
  return trajectory;
}

StepSizeStudy step_size_study(const DerivativeFunction& derivative,
                              const Eigen::VectorXd& initial_state,
                              double initial_time_s,
                              double coarse_step_s,
                              int coarse_step_count,
                              const ProjectionFunction& projection) {
  if (coarse_step_count < 1 || coarse_step_count > std::numeric_limits<int>::max() / 4
      || !std::isfinite(coarse_step_s) || !(0.25 * coarse_step_s > 0.0)) {
    throw std::invalid_argument(
        "step_size_study: coarse count and step must permit positive representable refinements");
  }
  const auto final_state = [&](double step_s, int count) {
    return integrate_fixed_step(
               derivative, initial_state, initial_time_s, step_s, count, count, projection)
        .states.back();
  };

  const Eigen::VectorXd coarse = final_state(coarse_step_s, coarse_step_count);
  const Eigen::VectorXd medium = final_state(0.5 * coarse_step_s, 2 * coarse_step_count);
  const Eigen::VectorXd fine = final_state(0.25 * coarse_step_s, 4 * coarse_step_count);

  StepSizeStudy study;
  study.step_s = coarse_step_s;
  study.final_at_h = coarse;
  study.final_at_h_2 = medium;
  study.final_at_h_4 = fine;
  study.difference_h_to_h2 = (coarse - medium).norm();
  study.difference_h2_to_h4 = (medium - fine).norm();
  if (!std::isfinite(study.difference_h_to_h2) || !std::isfinite(study.difference_h2_to_h4)) {
    throw std::runtime_error("step_size_study: state-difference norm overflowed");
  }

  constexpr double kOrder = 4.0;
  const double two_to_p = std::pow(2.0, kOrder);
  study.estimated_error_at_h_2 = study.difference_h_to_h2 / (two_to_p - 1.0);
  study.estimated_error_at_h = study.estimated_error_at_h_2 * two_to_p;

  const double fine_difference = study.difference_h2_to_h4;
  if (fine_difference > 0.0 && study.difference_h_to_h2 > 0.0) {
    study.observed_order = std::log2(study.difference_h_to_h2 / fine_difference);
  } else {
    // Both differences vanished: either the problem is exactly integrated by
    // the method, or the steps are so small that rounding dominates and the
    // order estimate is meaningless. Report zero rather than a NaN or an
    // infinity that a caller might plot.
    study.observed_order = 0.0;
  }
  if (!std::isfinite(study.estimated_error_at_h) || !std::isfinite(study.observed_order)) {
    throw std::runtime_error("step_size_study: error or order estimate is unrepresentable");
  }
  return study;
}

}  // namespace galata::numerics
