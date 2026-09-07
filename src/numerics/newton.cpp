// SPDX-License-Identifier: Apache-2.0
//
// Newton's method with a backtracking line search.
//
// Reference:
//   J. Nocedal and S. J. Wright, "Numerical Optimization", 2nd ed., Springer,
//   2006, chapter 11.
//   C. T. Kelley, "Iterative Methods for Linear and Nonlinear Equations",
//   SIAM, 1995, chapter 8.

#include "galata/numerics/newton.hpp"

#include <Eigen/QR>
#include <Eigen/SVD>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace galata::numerics {

NewtonResult solve_newton(const VectorFunction& residual,
                          const Eigen::VectorXd& initial_guess,
                          const NewtonOptions& options) {
  if (!residual || initial_guess.size() == 0 || !initial_guess.allFinite()) {
    throw std::invalid_argument(
        "solve_newton: residual is empty or initial guess is empty/non-finite");
  }
  if (!std::isfinite(options.residual_tolerance) || !(options.residual_tolerance > 0.0)) {
    throw std::invalid_argument("solve_newton: residual_tolerance must be positive and finite");
  }
  if (options.iterations < 1) {
    throw std::invalid_argument("solve_newton: iterations must be at least 1");
  }
  if (options.line_search_trials < 1) {
    throw std::invalid_argument("solve_newton: line_search_trials must be at least 1");
  }

  Eigen::VectorXd x = initial_guess;
  Eigen::VectorXd r = residual(x);
  if (r.size() != x.size()) {
    throw std::invalid_argument(
        "solve_newton: the residual is not the same length as x, so the "
        "system is not square and Newton does not apply");
  }
  if (!r.allFinite() || !std::isfinite(r.norm())) {
    throw std::runtime_error("solve_newton: the initial residual or its norm is non-finite");
  }

  NewtonResult result;
  result.residual_history.reserve(static_cast<std::size_t>(options.iterations) + 1U);
  result.residual_history.push_back(r.norm());

  Jacobian jacobian;
  for (int iteration = 0; iteration < options.iterations; ++iteration) {
    JacobianOptions jacobian_options = options.jacobian;
    // The truncation estimate costs a second Jacobian and is not used here;
    // the linearisation capability wants it, a root-find does not.
    jacobian_options.estimate_truncation_error = false;
    jacobian = central_difference_jacobian(residual, x, jacobian_options);
    if (jacobian.value.rows() != x.size()) {
      throw std::runtime_error("solve_newton: residual dimension changed during differentiation");
    }

    // Solved with column-pivoting QR rather than an inverse or an LU. It is
    // the cheapest decomposition that copes gracefully with a rank-deficient
    // Jacobian, which is exactly the case that arises when a control has no
    // authority — and there it returns a least-squares step instead of
    // infinities.
    const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> factorization(jacobian.value);
    if (!factorization.matrixQR().allFinite()) {
      throw std::runtime_error("solve_newton: the Jacobian factorization is non-finite");
    }
    // With rank zero every step has the same linearized residual. The
    // minimum-norm least-squares step is zero, but Eigen's zero-matrix QR
    // solve divides by a zero pivot. Stagnation is ordinary nonconvergence,
    // not an invalid numerical input: retain the fixed iteration history and
    // let the actual residual decide whether this point is already a root.
    Eigen::VectorXd step = Eigen::VectorXd::Zero(x.size());
    if (factorization.rank() != 0) {
      step = factorization.solve(-r);
    }
    if (!step.allFinite()) {
      throw std::runtime_error("solve_newton: the Newton step is non-finite");
    }

    // Backtracking. Every fraction is evaluated and the best kept.
    double best_norm = std::numeric_limits<double>::infinity();
    Eigen::VectorXd best_x = x;
    Eigen::VectorXd best_r = r;
    double fraction = 1.0;
    for (int trial = 0; trial < options.line_search_trials; ++trial) {
      const Eigen::VectorXd candidate = x + fraction * step;
      if (!candidate.allFinite()) {
        fraction *= 0.5;
        continue;
      }
      Eigen::VectorXd candidate_residual;
      try {
        candidate_residual = residual(candidate);
      } catch (const std::exception&) {
        // A candidate outside the model's domain — below its minimum airspeed,
        // outside the atmosphere envelope — is not a failure of the solve. It
        // is a step that was too long, which is what backtracking is for.
        fraction *= 0.5;
        continue;
      }
      if (candidate_residual.size() != x.size()) {
        throw std::runtime_error("solve_newton: residual dimension changed during line search");
      }
      const double norm = candidate_residual.norm();
      if (std::isfinite(norm) && norm < best_norm) {
        best_norm = norm;
        best_x = candidate;
        best_r = candidate_residual;
      }
      fraction *= 0.5;
    }

    // If no trial improved on where we already are, stay put: the iteration is
    // spent but the answer does not get worse.
    if (best_norm < r.norm()) {
      x = best_x;
      r = best_r;
    }
    result.residual_history.push_back(r.norm());
  }

  result.solution = x;
  result.residual = r;
  result.residual_norm = r.norm();
  result.converged = result.residual_norm <= options.residual_tolerance;

  // Recomputed at the solution rather than reused from the last iteration:
  // the reported conditioning should describe the answer, not the point
  // before it.
  JacobianOptions final_options = options.jacobian;
  final_options.estimate_truncation_error = false;
  result.final_jacobian = central_difference_jacobian(residual, x, final_options).value;
  if (result.final_jacobian.rows() != x.size()) {
    throw std::runtime_error("solve_newton: residual dimension changed at the solution");
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(result.final_jacobian);
  const Eigen::VectorXd singular = svd.singularValues();
  if (svd.info() != Eigen::Success || !singular.allFinite()) {
    throw std::runtime_error("solve_newton: final Jacobian singular-value decomposition failed");
  }
  if (singular.size() > 0 && singular(singular.size() - 1) > 0.0) {
    result.jacobian_condition_number = singular(0) / singular(singular.size() - 1);
  } else {
    result.jacobian_condition_number = std::numeric_limits<double>::infinity();
  }
  return result;
}

}  // namespace galata::numerics
