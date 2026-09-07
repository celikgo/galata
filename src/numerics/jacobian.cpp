// SPDX-License-Identifier: Apache-2.0
//
// Central-difference Jacobians with Richardson truncation-error estimation.
//
// Reference:
//   J. Nocedal and S. J. Wright, "Numerical Optimization", 2nd ed., Springer,
//   2006, section 8.1.
//   W. H. Press et al., "Numerical Recipes", 3rd ed., CUP, 2007, section 5.7.
//
// Validity envelope and the failure modes finite differences cannot detect are
// in the header's "WHAT THIS IS NOT" block.

#include "galata/numerics/jacobian.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace galata::numerics {
namespace {

Eigen::MatrixXd difference_jacobian(const VectorFunction& f,
                                    const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& steps) {
  Eigen::MatrixXd jacobian;
  for (Eigen::Index j = 0; j < x.size(); ++j) {
    Eigen::VectorXd forward = x;
    Eigen::VectorXd backward = x;
    // The perturbed points are formed first and the ACTUAL step read back
    // from them. x + h is not exactly x plus h in floating point, and using
    // the nominal h while the function saw a different one is a systematic
    // error in every entry of the column. Numerical Recipes calls this out
    // specifically; it costs nothing to get right.
    forward(j) = x(j) + steps(j);
    backward(j) = x(j) - steps(j);
    const double actual = forward(j) - backward(j);
    if (!forward.allFinite() || !backward.allFinite() || !std::isfinite(actual)) {
      throw std::runtime_error("central_difference_jacobian: the perturbation overflowed");
    }
    if (actual == 0.0) {
      std::ostringstream message;
      message << "central_difference_jacobian: the perturbation for component " << j
              << " vanished at x = " << x(j)
              << ". The step underflowed relative to the value; raise absolute_step.";
      throw std::runtime_error(message.str());
    }

    const Eigen::VectorXd high = f(forward);
    const Eigen::VectorXd low = f(backward);
    if (high.size() == 0 || high.size() != low.size()
        || (j != 0 && high.size() != jacobian.rows())) {
      throw std::runtime_error(
          "central_difference_jacobian: f must return a fixed non-empty dimension");
    }
    if (!high.allFinite() || !low.allFinite()) {
      throw std::runtime_error("central_difference_jacobian: f returned a non-finite value");
    }
    if (j == 0) {
      jacobian.resize(high.size(), x.size());
    }
    jacobian.col(j) = (high - low) / actual;
    if (!jacobian.col(j).allFinite()) {
      throw std::runtime_error("central_difference_jacobian: the difference quotient overflowed");
    }
  }
  return jacobian;
}

}  // namespace

Jacobian central_difference_jacobian(const VectorFunction& f,
                                     const Eigen::VectorXd& x,
                                     const JacobianOptions& options) {
  if (!f || x.size() == 0 || !x.allFinite()) {
    throw std::invalid_argument("central_difference_jacobian: f is empty or x is empty/non-finite");
  }
  if (!std::isfinite(options.relative_step) || options.relative_step < 0.0
      || !std::isfinite(options.absolute_step) || options.absolute_step < 0.0
      || !options.absolute_step_per_component.allFinite()
      || (options.absolute_step_per_component.array() < 0.0).any()) {
    throw std::invalid_argument(
        "central_difference_jacobian: step options must be finite and non-negative");
  }
  if (options.absolute_step_per_component.size() != 0
      && options.absolute_step_per_component.size() != x.size()) {
    throw std::invalid_argument(
        "central_difference_jacobian: absolute_step_per_component has the wrong length");
  }

  Eigen::VectorXd steps(x.size());
  for (Eigen::Index j = 0; j < x.size(); ++j) {
    const double floor = (options.absolute_step_per_component.size() != 0)
                             ? options.absolute_step_per_component(j)
                             : options.absolute_step;
    steps(j) = std::fmax(options.relative_step * std::fabs(x(j)), floor);
  }
  if (!steps.allFinite()) {
    throw std::runtime_error("central_difference_jacobian: a perturbation size overflowed");
  }

  Jacobian result;
  result.steps = steps;

  if (!options.estimate_truncation_error) {
    result.value = difference_jacobian(f, x, steps);
    return result;
  }

  const Eigen::MatrixXd coarse = difference_jacobian(f, x, steps);
  const Eigen::MatrixXd fine = difference_jacobian(f, x, 0.5 * steps);
  if (coarse.rows() != fine.rows()) {
    throw std::runtime_error(
        "central_difference_jacobian: f changed dimension between perturbation sizes");
  }

  // Richardson, order p = 2. The half-step Jacobian is returned as the answer
  // and its error is (coarse - fine) / 3 — a quarter of the full-step error,
  // which is (coarse - fine) * 4/3. Reporting the full-step error alongside
  // the half-step answer would overstate it by four.
  result.value = fine;
  result.truncation_estimate = (coarse - fine).cwiseAbs() / 3.0;
  if (!result.truncation_estimate.allFinite()) {
    throw std::runtime_error("central_difference_jacobian: truncation estimate overflowed");
  }
  result.steps = 0.5 * steps;

  double worst = 0.0;
  const double scale = result.value.cwiseAbs().maxCoeff();
  for (Eigen::Index i = 0; i < result.value.rows(); ++i) {
    for (Eigen::Index j = 0; j < result.value.cols(); ++j) {
      // Relative to the largest entry of the matrix rather than to the entry
      // itself: an entry that is legitimately zero would otherwise report an
      // infinite relative error and dominate the summary.
      if (scale > 0.0) {
        worst = std::fmax(worst, result.truncation_estimate(i, j) / scale);
      }
    }
  }
  if (!std::isfinite(worst)) {
    throw std::runtime_error(
        "central_difference_jacobian: relative truncation estimate overflowed");
  }
  result.worst_relative_truncation = worst;
  return result;
}

}  // namespace galata::numerics
