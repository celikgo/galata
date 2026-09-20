// SPDX-License-Identifier: Apache-2.0
//
// Deterministic fixed-step integration: explicit RK4 by delegation, and two
// A-stable implicit methods solved by a fixed-count simplified Newton.
//
// Reference:
//   E. Hairer and G. Wanner, "Solving Ordinary Differential Equations II:
//   Stiff and Differential-Algebraic Problems", 2nd revised ed., Springer,
//   1996 — IV.3 for A- and L-stability, IV.8 for simplified Newton on the
//   stage equation.
//   E. Hairer, S. P. Norsett and G. Wanner, "Solving Ordinary Differential
//   Equations I", 2nd revised ed., Springer, 1993.
//
// Validity envelope and known error behaviour are in the header's
// "WHAT THIS IS NOT" block. Implicit Euler is first order with global error
// O(h) and is L-stable; the trapezoidal rule is second order with global error
// O(h^2) and is A-stable but not L-stable, so a mode with |h*lambda| >> 1 is
// reflected rather than damped and rings at the step frequency.
//
// DETERMINISM. Every loop bound here is an integer known before the loop
// starts. There is no residual test, no tolerance and no early exit anywhere,
// so the same inputs perform the same arithmetic in the same order on every
// run. The linear solve is a partial-pivot LU, whose pivot sequence is a
// function of the matrix and not of any accumulated state.

#include "galata/numerics/integration_method.hpp"

#include <Eigen/LU>

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace galata::numerics {
namespace {

std::string number(double value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(4) << std::scientific << value;
  return out.str();
}

// Numerical Jacobian of f at (t, x), by central differences.
//
// Written here rather than reusing central_difference_jacobian() because that
// routine takes a VectorFunction of one argument and also computes a Richardson
// truncation estimate, which costs a further two evaluations per column and is
// not wanted inside a stage solve. The step rule is the same one
// jacobian.hpp derives and documents: eps^(1/3) scaled by the entry magnitude.
Eigen::MatrixXd stage_jacobian(const DerivativeFunction& derivative,
                               double time_s,
                               const Eigen::VectorXd& state) {
  constexpr double kRelativeStep = 6.055454452393343e-06;  // eps^(1/3) for binary64
  const Eigen::Index n = state.size();
  Eigen::MatrixXd jacobian(n, n);
  Eigen::VectorXd probe = state;
  for (Eigen::Index j = 0; j < n; ++j) {
    const double scale = std::max(std::fabs(state(j)), 1.0);
    const double step = kRelativeStep * scale;
    // The perturbed values are computed and then differenced, so the effective
    // step is exactly representable and the divisor is the step actually taken.
    probe(j) = state(j) + step;
    const double upper = probe(j);
    const Eigen::VectorXd forward = derivative(time_s, probe);
    probe(j) = state(j) - step;
    const double lower = probe(j);
    const Eigen::VectorXd backward = derivative(time_s, probe);
    probe(j) = state(j);
    if (forward.size() != n || backward.size() != n) {
      throw std::runtime_error("integrate: the derivative changed dimension during a Jacobian");
    }
    jacobian.col(j) = (forward - backward) / (upper - lower);
  }
  return jacobian;
}

// One implicit step. `theta` is 1 for implicit Euler and 1/2 for trapezoidal:
//
//   x_{n+1} = x_n + h [ (1 - theta) f(t_n, x_n) + theta f(t_{n+1}, x_{n+1}) ]
//
// Solved for x_{n+1} by exactly `iterations` Newton steps on
//   G(y) = y - x_n - h[(1-theta) f_n + theta f(t+h, y)] = 0
// whose Jacobian is I - h*theta*df/dy.
Eigen::VectorXd theta_step(const DerivativeFunction& derivative,
                           double time_s,
                           const Eigen::VectorXd& state,
                           double step_s,
                           double theta,
                           int iterations,
                           long long* iterations_performed) {
  const Eigen::Index n = state.size();
  const double next_time = time_s + step_s;

  const Eigen::VectorXd explicit_part =
      theta < 1.0 ? derivative(time_s, state) : Eigen::VectorXd::Zero(n).eval();
  if (theta < 1.0 && (explicit_part.size() != n || !explicit_part.allFinite())) {
    throw std::runtime_error("integrate: the derivative is non-finite at the step's start");
  }
  const Eigen::VectorXd anchor =
      theta < 1.0 ? Eigen::VectorXd(state + step_s * (1.0 - theta) * explicit_part)
                  : Eigen::VectorXd(state);

  // FIRST GUESS IS ONE EXPLICIT EULER STEP, always, for every method and every
  // step. A guess taken from the previous step's answer would make the result
  // depend on the trajectory's history in a way that is harder to reason about
  // and no cheaper.
  const Eigen::VectorXd slope = theta < 1.0 ? explicit_part : derivative(time_s, state);
  if (slope.size() != n || !slope.allFinite()) {
    throw std::runtime_error("integrate: the derivative is non-finite at the step's start");
  }
  Eigen::VectorXd guess = state + step_s * slope;

  // Jacobian is formed once per step and REUSED across the iterations. This is
  // the simplified Newton of Hairer and Wanner II.IV.8: it converges linearly
  // rather than quadratically, which is why the iteration count is a parameter,
  // and it costs one Jacobian per step rather than one per iteration.
  const Eigen::MatrixXd jacobian = stage_jacobian(derivative, next_time, guess);
  Eigen::MatrixXd iteration_matrix = Eigen::MatrixXd::Identity(n, n) - (step_s * theta) * jacobian;
  const Eigen::PartialPivLU<Eigen::MatrixXd> factorisation(iteration_matrix);

  for (int k = 0; k < iterations; ++k) {
    const Eigen::VectorXd rate = derivative(next_time, guess);
    if (rate.size() != n || !rate.allFinite()) {
      throw std::runtime_error("integrate: the derivative is non-finite inside an implicit stage");
    }
    const Eigen::VectorXd residual = guess - anchor - (step_s * theta) * rate;
    const Eigen::VectorXd correction = factorisation.solve(residual);
    if (!correction.allFinite()) {
      throw std::runtime_error(
          "integrate: the implicit stage's linear solve produced a non-finite correction; the "
          "iteration matrix I - h*theta*J is singular or nearly so at this step");
    }
    guess -= correction;
    if (iterations_performed != nullptr) {
      ++(*iterations_performed);
    }
  }
  if (!guess.allFinite()) {
    throw std::runtime_error("integrate: the implicit stage did not produce a finite state");
  }
  return guess;
}

}  // namespace

std::string to_string(IntegrationMethod method) {
  switch (method) {
    case IntegrationMethod::Rk4Fixed:
      return "rk4_fixed";
    case IntegrationMethod::ImplicitEulerFixed:
      return "implicit_euler_fixed";
    case IntegrationMethod::TrapezoidalFixed:
      return "trapezoidal_fixed";
  }
  return "unknown";
}

int method_order(IntegrationMethod method) {
  switch (method) {
    case IntegrationMethod::Rk4Fixed:
      return 4;
    case IntegrationMethod::ImplicitEulerFixed:
      return 1;
    case IntegrationMethod::TrapezoidalFixed:
      return 2;
  }
  return 0;
}

bool method_is_implicit(IntegrationMethod method) {
  return method != IntegrationMethod::Rk4Fixed;
}

std::string to_string(TerminationReason reason) {
  switch (reason) {
    case TerminationReason::Completed:
      return "completed";
    case TerminationReason::BoundExceeded:
      return "bound_exceeded";
    case TerminationReason::NonFinite:
      return "non_finite";
    case TerminationReason::DerivativeFailed:
      return "derivative_failed";
  }
  return "unknown";
}

StateBounds::StateBounds(std::vector<std::string> names, Eigen::VectorXd magnitudes)
    : names_(std::move(names)), magnitudes_(std::move(magnitudes)) {
  if (static_cast<Eigen::Index>(names_.size()) != magnitudes_.size()) {
    throw std::invalid_argument("StateBounds: " + std::to_string(names_.size()) + " names against "
                                + std::to_string(magnitudes_.size())
                                + " magnitudes; a bound the report cannot name is a bound nobody "
                                  "can act on");
  }
}

std::string StateBounds::violation(const Eigen::VectorXd& state) const {
  // The finiteness check applies whether or not magnitudes were declared, and
  // it runs first: a NaN compares false against every bound, so checking
  // magnitudes first would let one through.
  for (Eigen::Index i = 0; i < state.size(); ++i) {
    if (!std::isfinite(state(i))) {
      const std::string name = i < static_cast<Eigen::Index>(names_.size())
                                   ? names_[static_cast<std::size_t>(i)]
                                   : ("state[" + std::to_string(i) + "]");
      return "state '" + name + "' is not finite";
    }
  }
  if (magnitudes_.size() == 0) {
    return {};
  }
  if (magnitudes_.size() != state.size()) {
    throw std::invalid_argument("StateBounds: declared for " + std::to_string(magnitudes_.size())
                                + " states but checked against " + std::to_string(state.size()));
  }
  for (Eigen::Index i = 0; i < state.size(); ++i) {
    const double limit = magnitudes_(i);
    if (!std::isfinite(limit) || !(limit > 0.0)) {
      continue;  // declared unbounded
    }
    if (std::fabs(state(i)) > limit) {
      return "state '" + names_[static_cast<std::size_t>(i)] + "' reached " + number(state(i))
             + " against a declared magnitude bound of " + number(limit);
    }
  }
  return {};
}

Eigen::VectorXd integration_step(IntegrationMethod method,
                                 const DerivativeFunction& derivative,
                                 double time_s,
                                 const Eigen::VectorXd& state,
                                 double step_s,
                                 int newton_iterations,
                                 long long* iterations_performed) {
  if (!derivative) {
    throw std::invalid_argument("integration_step: no derivative supplied");
  }
  switch (method) {
    case IntegrationMethod::Rk4Fixed:
      // Delegated unchanged, so a study that does not choose another method
      // gets exactly the bits integrator.hpp produced before this file existed.
      return rk4_step(derivative, time_s, state, step_s);
    case IntegrationMethod::ImplicitEulerFixed:
      return theta_step(
          derivative, time_s, state, step_s, 1.0, newton_iterations, iterations_performed);
    case IntegrationMethod::TrapezoidalFixed:
      return theta_step(
          derivative, time_s, state, step_s, 0.5, newton_iterations, iterations_performed);
  }
  throw std::invalid_argument("integration_step: unknown method");
}

IntegrationResult integrate(const DerivativeFunction& derivative,
                            const Eigen::VectorXd& initial_state,
                            double initial_time_s,
                            const IntegrationOptions& options,
                            const ProjectionFunction& projection,
                            const StateBounds& bounds) {
  if (!derivative) {
    throw std::invalid_argument("integrate: no derivative supplied");
  }
  if (initial_state.size() == 0 || !initial_state.allFinite()) {
    throw std::invalid_argument("integrate: the initial state must be non-empty and finite");
  }
  if (!std::isfinite(initial_time_s)) {
    throw std::invalid_argument("integrate: the initial time must be finite");
  }
  if (!std::isfinite(options.step_s) || !(options.step_s > 0.0)) {
    throw std::invalid_argument("integrate: the step must be positive and finite");
  }
  if (options.step_count < 0) {
    throw std::invalid_argument("integrate: step_count must be non-negative");
  }
  if (options.sample_stride < 1) {
    throw std::invalid_argument("integrate: sample_stride must be at least 1");
  }
  if (method_is_implicit(options.method) && options.newton_iterations < 1) {
    throw std::invalid_argument(
        "integrate: an implicit method needs at least one Newton iteration. The count is fixed "
        "rather than a tolerance, so zero is not 'until converged' — it is 'do not solve'");
  }
  if (options.jacobian_refresh_steps < 1) {
    throw std::invalid_argument("integrate: jacobian_refresh_steps must be at least 1");
  }
  if (!bounds.empty() && bounds.size() != initial_state.size()) {
    throw std::invalid_argument("integrate: bounds are declared for "
                                + std::to_string(bounds.size()) + " states but the state has "
                                + std::to_string(initial_state.size()));
  }

  IntegrationResult result;
  result.method = options.method;
  result.trajectory.step_s = options.step_s;
  result.trajectory.sample_stride = options.sample_stride;

  Eigen::VectorXd state = initial_state;
  if (projection) {
    projection(state);
  }
  if (const std::string bad = bounds.violation(state); !bad.empty()) {
    result.reason = TerminationReason::BoundExceeded;
    result.detail = "at the initial condition, " + bad;
    result.termination_time_s = initial_time_s;
    result.trajectory.times_s.push_back(initial_time_s);
    result.trajectory.states.push_back(state);
    return result;
  }

  result.trajectory.times_s.push_back(initial_time_s);
  result.trajectory.states.push_back(state);

  for (int step = 0; step < options.step_count; ++step) {
    // Sample times are computed as t0 + k*step rather than accumulated, so the
    // reported time does not drift over a long run (integrator.hpp, same rule).
    const double time_s = initial_time_s + static_cast<double>(step) * options.step_s;
    Eigen::VectorXd next;
    try {
      next = integration_step(options.method,
                              derivative,
                              time_s,
                              state,
                              options.step_s,
                              options.newton_iterations,
                              &result.newton_iterations_performed);
    } catch (const std::exception& error) {
      result.reason = TerminationReason::DerivativeFailed;
      result.detail = std::string("step ") + std::to_string(step + 1) + " failed: " + error.what();
      result.steps_taken = step;
      result.termination_time_s = time_s;
      return result;
    }
    if (projection) {
      projection(next);
    }

    const double next_time = initial_time_s + static_cast<double>(step + 1) * options.step_s;

    // THE BOUND IS CHECKED AFTER THE PROJECTION AND BEFORE THE SAMPLE IS KEPT.
    // A state that has already left the bound is not recorded as though it were
    // a result: the trajectory ends at the last good sample and the reason says
    // what happened at the next one.
    if (const std::string bad = bounds.violation(next); !bad.empty()) {
      result.reason =
          !next.allFinite() ? TerminationReason::NonFinite : TerminationReason::BoundExceeded;
      result.detail = "at t = " + number(next_time) + " s, after " + std::to_string(step + 1)
                      + " steps, " + bad
                      + ". The trajectory returned ends at the last state that satisfied it";
      result.steps_taken = step + 1;
      result.termination_time_s = next_time;
      return result;
    }

    state = std::move(next);
    if (((step + 1) % options.sample_stride) == 0 || step + 1 == options.step_count) {
      result.trajectory.times_s.push_back(next_time);
      result.trajectory.states.push_back(state);
    }
  }

  result.trajectory.step_count = options.step_count;
  result.steps_taken = options.step_count;
  result.termination_time_s =
      initial_time_s + static_cast<double>(options.step_count) * options.step_s;
  result.reason = TerminationReason::Completed;
  return result;
}

}  // namespace galata::numerics
