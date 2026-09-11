// SPDX-License-Identifier: Apache-2.0
// Golub and Van Loan, Matrix Computations, 4th ed. (2013), ch. 7:
// eigensystem backward error and Bauer-Fike eigenvalue sensitivity.
// WHAT THIS IS NOT: an interval certificate or a general stability-radius
// computation. Eigenvector conditioning bounds the effect of backward error
// only for a resolved diagonalizable realization. Ill-conditioned/defective
// stable systems can therefore be refused; this error is conservative.
#include "analysis_checks.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::analyze::detail {
HurwitzAssessment assess_hurwitz(const Eigen::MatrixXd& a) {
  if (a.rows() < 1 || a.rows() != a.cols() || !a.allFinite()) {
    throw std::invalid_argument("internal stability: A must be finite, square and nonempty");
  }
  const double scale = std::max(a.stableNorm(), std::numeric_limits<double>::min());
  if (!std::isfinite(scale)) {
    return {HurwitzStatus::Unresolved, "internal stability unresolved: matrix scale overflow"};
  }
  const Eigen::EigenSolver<Eigen::MatrixXd> eigen(a, true);
  if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite()
      || !eigen.eigenvectors().allFinite()) {
    return {HurwitzStatus::Unresolved, "internal stability unresolved: eigensystem failed"};
  }
  const Eigen::MatrixXcd vectors = eigen.eigenvectors();
  const Eigen::JacobiSVD<Eigen::MatrixXcd> svd(vectors);
  if (svd.info() != Eigen::Success || !svd.singularValues().allFinite()) {
    return {HurwitzStatus::Unresolved, "internal stability unresolved: conditioning check failed"};
  }
  const double condition = svd.singularValues()(0) / svd.singularValues()(a.rows() - 1);
  // Conservative policy limit, shared with the Hamiltonian norm checks: this
  // is not a theorem-derived universal threshold. Beyond it, an eigenvalue's
  // sign is too sensitive to support this implementation's stability claim.
  constexpr double kMaxCondition = 1e8;
  if (!std::isfinite(condition) || condition > kMaxCondition) {
    // The verdict is unchanged and stays a refusal: with the eigenvectors this
    // ill-conditioned, Bauer-Fike supports no stability claim and this function
    // will not make one.
    //
    // The DIAGNOSTIC changes, because the old one named the wrong cause. It
    // said "rescale the model", which is advice for a scaling problem, and the
    // common case is not a scaling problem: it is a loop that leaves modes
    // unstabilised, whose eigenvector matrix is near-singular precisely BECAUSE
    // the unstabilised modes form defective chains at the origin. A caller told
    // to rescale rescales, gets the same refusal, and learns nothing.
    //
    // So the computed spectrum is reported alongside the conditioning, as an
    // OBSERVATION rather than a certified result — it is exactly the quantity
    // the conditioning makes uncertain, and saying otherwise would be claiming
    // the precision this branch exists to deny. It still distinguishes the two
    // situations a caller confuses: poles sitting on the axis, and a genuinely
    // well-separated spectrum that only the conditioning refuses.
    const double axis_floor =
        1024.0 * static_cast<double>(a.rows()) * std::numeric_limits<double>::epsilon() * scale;
    int on_axis = 0;
    int right_half = 0;
    for (Eigen::Index i = 0; i < eigen.eigenvalues().size(); ++i) {
      const double real_part = eigen.eigenvalues()(i).real();
      if (real_part > axis_floor) {
        ++right_half;
      } else if (real_part > -axis_floor) {
        ++on_axis;
      }
    }
    std::ostringstream diagnostic;
    diagnostic << "internal stability unresolved: eigenvectors are ill-conditioned (condition "
               << condition << "), so no stability claim is supported. The computed spectrum — "
               << "uncertain by exactly that conditioning, and reported as an observation — has "
               << on_axis << " eigenvalue(s) within " << axis_floor << " of the imaginary axis and "
               << right_half << " in the right half-plane";
    if (on_axis > 0 || right_half > 0) {
      diagnostic << ". That is what an unstabilised mode looks like, not what a badly scaled "
                    "model looks like: rescaling will not resolve it. Check that the loop "
                    "actually closes around every mode you need stabilised";
    } else {
      diagnostic << ". The spectrum is well separated from the axis, so the conditioning alone "
                    "is what refuses this; rescaling the model may resolve it";
    }
    return {HurwitzStatus::Unresolved, diagnostic.str()};
  }
  // Normalize before multiplication to avoid manufacturing an overflow while
  // measuring the residual of a finite, well-scaled eigensystem.
  const double residual = ((a / scale).cast<std::complex<double>>() * vectors
                           - vectors * (eigen.eigenvalues() / scale).asDiagonal())
                              .stableNorm()
                          / vectors.stableNorm();
  const double budget =
      256.0 * static_cast<double>(a.rows()) * std::numeric_limits<double>::epsilon();
  if (!std::isfinite(residual) || residual > budget) {
    return {HurwitzStatus::Unresolved, "internal stability unresolved: eigensystem residual"};
  }
  const double axis_uncertainty = std::max(budget, 8.0 * residual) * condition;
  const double rightmost = eigen.eigenvalues().real().maxCoeff() / scale;
  if (rightmost < -axis_uncertainty) {
    return {HurwitzStatus::Stable, "internally Hurwitz within the numerical separation budget"};
  }
  if (rightmost > axis_uncertainty) {
    return {HurwitzStatus::NonStable, "nominal closed loop is unstable"};
  }
  return {HurwitzStatus::Boundary,
          "internal stability unresolved at the imaginary-axis boundary; no positive delay "
          "tolerance is established"};
}

void require_hurwitz(const Eigen::MatrixXd& a, const char* context) {
  const auto assessment = assess_hurwitz(a);
  if (assessment.status == HurwitzStatus::Unresolved) {
    throw std::runtime_error(std::string(context) + ": " + assessment.diagnostic);
  }
  if (assessment.status != HurwitzStatus::Stable) {
    throw std::invalid_argument(std::string(context) + ": " + assessment.diagnostic);
  }
}

std::complex<double> checked_loop_value(const LoopEvaluator& loop,
                                        double frequency,
                                        const char* context) {
  const auto value = loop(frequency);
  if (!std::isfinite(value.real()) || !std::isfinite(value.imag())
      || !std::isfinite(std::abs(value))) {
    throw std::domain_error(std::string(context) + ": nonfinite loop evaluation");
  }
  return value;
}

void require_frequency_grid(const std::vector<double>& grid, const char* context) {
  if (grid.size() < 2) {
    throw std::invalid_argument(std::string(context) + ": need at least two frequencies");
  }
  for (std::size_t index = 0; index < grid.size(); ++index) {
    if (!std::isfinite(grid[index]) || grid[index] <= 0.0
        || (index > 0 && grid[index] <= grid[index - 1])) {
      throw std::invalid_argument(
          std::string(context) + ": frequencies must be finite, positive and strictly increasing");
    }
  }
}
}  // namespace galata::analyze::detail
