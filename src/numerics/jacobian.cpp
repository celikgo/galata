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

    const Eigen::VectorXd high = f(forward);
    const Eigen::VectorXd low = f(backward);
    if (high.size() != low.size()) {
      throw std::runtime_error("central_difference_jacobian: f returned different sizes");
    }
    if (jacobian.size() == 0) {
      jacobian.resize(high.size(), x.size());
    }
    if (actual == 0.0) {
      std::ostringstream message;
      message << "central_difference_jacobian: the perturbation for component " << j
              << " vanished at x = " << x(j)
              << ". The step underflowed relative to the value; raise absolute_step.";
      throw std::runtime_error(message.str());
    }
    jacobian.col(j) = (high - low) / actual;
  }
  return jacobian;
}

}  // namespace

Jacobian central_difference_jacobian(const VectorFunction& f,
                                     const Eigen::VectorXd& x,
                                     const JacobianOptions& options) {
  if (x.size() == 0) {
    throw std::invalid_argument("central_difference_jacobian: x is empty");
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

  Jacobian result;
  result.steps = steps;

  if (!options.estimate_truncation_error) {
    result.value = difference_jacobian(f, x, steps);
    return result;
  }

  const Eigen::MatrixXd coarse = difference_jacobian(f, x, steps);
  const Eigen::MatrixXd fine = difference_jacobian(f, x, 0.5 * steps);

  // Richardson, order p = 2. The half-step Jacobian is returned as the answer
  // and its error is (coarse - fine) / 3 — a quarter of the full-step error,
  // which is (coarse - fine) * 4/3. Reporting the full-step error alongside
  // the half-step answer would overstate it by four.
  result.value = fine;
  result.truncation_estimate = (coarse - fine).cwiseAbs() / 3.0;
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
  result.worst_relative_truncation = worst;
  return result;
}

}  // namespace galata::numerics
