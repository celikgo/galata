// SPDX-License-Identifier: Apache-2.0
//
// Numerical Jacobians by central differences, with a truncation-error estimate.
//
// Reference:
//   J. Nocedal and S. J. Wright, "Numerical Optimization", 2nd ed., Springer,
//   2006, chapter 8 — finite-difference derivative approximation and the
//   choice of perturbation size.
//   W. H. Press, S. A. Teukolsky, W. T. Vetterling and B. P. Flannery,
//   "Numerical Recipes: The Art of Scientific Computing", 3rd ed., Cambridge
//   University Press, 2007, section 5.7 — the same, and Richardson
//   extrapolation.
//
// WHY CENTRAL AND NOT FORWARD. A forward difference has truncation error
// O(h f'') and needs n+1 evaluations; a central difference has O(h^2 f''') and
// needs 2n. For a linearisation that feeds a control design, the extra factor
// of two in cost buys roughly seven digits of accuracy at the optimal step, and
// the linearisation is computed once per operating point rather than in an
// inner loop. Central wins on every axis that matters here.
//
// THE STEP SIZE IS THE WHOLE PROBLEM. Too large and truncation error dominates;
// too small and cancellation in (f(x+h) - f(x-h)) destroys the answer, because
// the two values agree in their leading digits and subtracting them throws
// those digits away. The optimal central-difference step scales as
// eps^(1/3) |x| — about 6e-6 relative for double precision — and this is the
// rule used, with an absolute floor so that a state passing through zero does
// not get a perturbation of zero.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not exact. A finite-difference Jacobian carries truncation error that no
//   step size removes, only trades against cancellation. `truncation_estimate`
//   reports it per entry so a caller can see which entries are trustworthy,
//   and the answer is "most of them, usually" rather than "all of them".
//
// * Not valid across a discontinuity. If the function has a kink inside the
//   perturbation window — a rate limit hitting its stop, a table lookup
//   crossing a breakpoint, a saturation — the difference quotient averages
//   across it and returns a slope the function never has. This is the single
//   most common way a linearisation of an aircraft model goes quietly wrong,
//   and nothing here can detect it: the Richardson estimate will look
//   perfectly healthy.
//
// * Not a substitute for an analytic derivative where one exists. Where the
//   model is analytic, complex-step differentiation gives the derivative to
//   machine precision with no cancellation at all. That is a separate
//   capability and is not implemented.

#ifndef GALATA_NUMERICS_JACOBIAN_HPP
#define GALATA_NUMERICS_JACOBIAN_HPP

#include <Eigen/Core>

#include <functional>

namespace galata::numerics {

using VectorFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

struct JacobianOptions {
  // h_i = max(relative_step * |x_i|, absolute_step).
  //
  // The default relative step is cbrt(machine epsilon), which is where the
  // central-difference truncation error and the cancellation error are of the
  // same order. Nocedal and Wright section 8.1 derives it.
  double relative_step = 6.055454452393343e-06;  // eps^(1/3) for binary64

  // Floor, so a component that happens to be zero still gets perturbed.
  //
  // The default is the SAME eps^(1/3), not some conveniently small number, and
  // the difference matters more than it looks. Cancellation error in a central
  // difference is about eps |f| / h. With f of order 10 — which is ordinary,
  // an acceleration in m/s^2 — a floor of 1e-8 gives 2e-16 * 10 / 1e-8 = 2e-7
  // of error in the Jacobian entry. That is not a rounding detail: it put
  // 4e-7 of error into the first Newton step of a LINEAR system, where the
  // step should have been exact.
  //
  // eps^(1/3) is where truncation and cancellation balance for a function of
  // order one, which is the right default for a floor precisely because the
  // floor only applies where the component itself carries no scale.
  //
  // Units are those of the corresponding component, which is why a caller
  // whose states span metres, metres per second and radians should set
  // absolute_step_per_component instead.
  double absolute_step = 6.055454452393343e-06;

  // Per-component floors, overriding absolute_step where non-empty. An
  // aircraft state mixes metres, metres per second and radians; one absolute
  // floor cannot be right for all three.
  Eigen::VectorXd absolute_step_per_component;

  // Compute the Richardson estimate. Costs a second Jacobian at half the step.
  bool estimate_truncation_error = true;
};

struct Jacobian {
  Eigen::MatrixXd value;  // d f_i / d x_j

  // The perturbation actually used for each column. Reported because the step
  // is a choice, and a reader checking a suspicious entry needs to know what
  // window it was measured over.
  Eigen::VectorXd steps;

  // Per-entry Richardson estimate of the truncation error in `value`.
  //
  // For a method of order p = 2, J_h - J_{h/2} = E_h (1 - 2^-p), so the error
  // in the FULL-STEP Jacobian is (J_h - J_{h/2}) * 4/3. `value` is the
  // half-step Jacobian, whose error is a quarter of that. Empty when
  // estimate_truncation_error is false.
  Eigen::MatrixXd truncation_estimate;

  // Largest entry of truncation_estimate relative to the corresponding entry of
  // value, ignoring entries whose value is negligible. A single number to look
  // at before trusting the whole matrix.
  double worst_relative_truncation = 0.0;
};

// Central-difference Jacobian of `f` at `x`.
//
// Uses 2n evaluations, or 4n when estimating truncation error. `f` must be
// defined at x +/- h in every component; a caller whose function has a domain
// boundary near x must move x, not shrink h.
[[nodiscard]] Jacobian central_difference_jacobian(const VectorFunction& f,
                                                   const Eigen::VectorXd& x,
                                                   const JacobianOptions& options = {});

}  // namespace galata::numerics

#endif  // GALATA_NUMERICS_JACOBIAN_HPP
