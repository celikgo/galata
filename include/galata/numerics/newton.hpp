// SPDX-License-Identifier: Apache-2.0
//
// Newton's method for a square nonlinear system, with a numerically computed
// Jacobian and a backtracking line search.
//
// Reference:
//   J. Nocedal and S. J. Wright, "Numerical Optimization", 2nd ed., Springer,
//   2006, chapter 11 — Newton's method for nonlinear equations, the merit
//   function ||r||^2 / 2, and backtracking line search.
//   C. T. Kelley, "Iterative Methods for Linear and Nonlinear Equations",
//   SIAM, 1995, chapter 8.
//
// WHY A ROOT-FIND AND NOT A MINIMISATION. Trim is a set of equations that are
// satisfied or are not: the accelerations are zero or they are not. Posing it
// as "minimise the sum of squared accelerations" replaces that with a question
// that always has an answer, and an optimiser will happily return the least-bad
// point in a region where no trim exists — a trim that is not a trim, reported
// with a small number next to it. A root-find either converges or says it did
// not.
//
// WHY A FIXED ITERATION COUNT. ADR-0004: exiting when a tolerance is met makes
// the iteration count a function of floating-point noise, and with it the
// answer. This runs exactly `iterations` steps and THEN checks the residual.
// The cost is a few wasted iterations on an easy problem, which for a 3-by-3
// trim is nothing.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not globally convergent. Newton converges quadratically near a root and
//   can wander anywhere far from one. The line search makes it much harder to
//   diverge but does not make it impossible; a bad initial guess still fails,
//   and it fails by reporting a residual, not by returning a wrong answer
//   quietly.
//
// * Not a solver for a singular or rank-deficient system. If the Jacobian is
//   singular the problem is under-determined — for a trim, that usually means
//   a control with no authority over any residual — and the reported condition
//   number is the diagnostic. It is reported for exactly this reason and is
//   worth reading every time.
//
// * Not aware of constraints. Control limits, envelope limits and positivity
//   are the caller's problem. A trim that needs 40 degrees of elevator will be
//   found and returned, and only the caller knows the elevator stops at 25.

#ifndef GALATA_NUMERICS_NEWTON_HPP
#define GALATA_NUMERICS_NEWTON_HPP

#include "galata/numerics/jacobian.hpp"

#include <Eigen/Core>

#include <vector>

namespace galata::numerics {

struct NewtonOptions {
  // Exactly this many iterations run. Not a maximum — see the header.
  int iterations = 40;

  // Backtracking step fractions tried per iteration: 1, 1/2, 1/4, ...
  //
  // All of them are evaluated and the best is taken, rather than accepting the
  // first that improves. Both are deterministic; evaluating all of them makes
  // the cost per iteration constant, which keeps the total work independent of
  // the data.
  //
  // Sixteen halvings shortens a step by a factor of 65,000. That is enough for
  // any trim, and it is NOT enough for every problem: from deep inside a flat
  // region — the tail of an exponential, say — no shortening of a Newton step
  // improves the residual at all, and the iteration stalls where it stands.
  // The residual history shows that plainly, as a flat line.
  int line_search_trials = 16;

  // The residual norm at or below which the answer is a solution. Checked
  // ONCE, after the fixed iteration budget is spent.
  double residual_tolerance = 1e-10;

  JacobianOptions jacobian;
};

struct NewtonResult {
  Eigen::VectorXd solution;
  Eigen::VectorXd residual;
  double residual_norm = 0.0;

  // 2-norm condition number of the Jacobian at the solution.
  //
  // Large means the problem is nearly singular there: some combination of the
  // unknowns barely moves the residual, so the solution is poorly determined
  // even when the residual is small. For a trim this is the number that says
  // "this aircraft is nearly uncontrollable in that axis at this condition".
  double jacobian_condition_number = 0.0;

  // residual_norm <= options.residual_tolerance.
  bool converged = false;

  // Residual norm before each iteration, then after the last. Length is
  // iterations + 1. Quadratic convergence is visible in it, and so is a stall.
  std::vector<double> residual_history;

  Eigen::MatrixXd final_jacobian;
};

// Solves residual(x) = 0.
//
// Returns a result whose `converged` flag must be checked. This deliberately
// does not throw: whether a non-converged trim is fatal is the caller's
// decision, and the residual and condition number are more useful than an
// exception message.
[[nodiscard]] NewtonResult solve_newton(const VectorFunction& residual,
                                        const Eigen::VectorXd& initial_guess,
                                        const NewtonOptions& options = {});

}  // namespace galata::numerics

#endif  // GALATA_NUMERICS_NEWTON_HPP
