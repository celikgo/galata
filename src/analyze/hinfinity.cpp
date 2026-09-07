// SPDX-License-Identifier: Apache-2.0
// Hamiltonian level-set bisection: Boyd, Balakrishnan and Kabamba (1989),
// MCSS 2:207-219. General feedthrough formula checked against Benner and Mitchell,
// "Faster and more accurate computation of the H-infinity norm via optimization",
// arXiv:1707.02497, Theorem 2.1 (2018): https://arxiv.org/abs/1707.02497.
// E=I; their R=D'D-gamma^2 I is negative definite above ||D||.
// WHAT THIS IS NOT: this implements the bisection characterization, not the
// accelerated BB/BS peak iteration, and uses floating point, not interval proof.
// Eigensystem residual, eigenvector conditioning and imaginary-axis separation
// are checked; unresolvable cases fail. No frequency-grid maximum is an upper bound.
#include "galata/analyze/hinfinity.hpp"

#include "analysis_checks.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace galata::analyze {
namespace {
constexpr double kEps = std::numeric_limits<double>::epsilon();
constexpr double kMaxCondition = 1e8;

double norm2(const Eigen::MatrixXcd& matrix) {
  if (!matrix.allFinite()) {
    throw std::runtime_error("hinfinity_norm: nonfinite transfer matrix");
  }
  const Eigen::JacobiSVD<Eigen::MatrixXcd> svd(matrix);
  if (svd.info() != Eigen::Success || !svd.singularValues().allFinite()) {
    throw std::runtime_error("hinfinity_norm: singular-value decomposition failed");
  }
  return svd.singularValues()(0);
}

struct Gain {
  double value;
  double lower;
};

enum class Level { Below, Above, Unresolved };

class Bracket {
 public:
  explicit Bracket(const model::LinearSystem& plant)
      : system(plant), c(plant.output_matrix()), d(plant.feedthrough_matrix()),
        count(plant.state_count()) {}

  Gain sample(double frequency) const {
    const Eigen::MatrixXcd denominator =
        std::complex<double>(0, frequency) * Eigen::MatrixXcd::Identity(count, count)
        - system.a.cast<std::complex<double>>();
    const Eigen::FullPivLU<Eigen::MatrixXcd> solve(denominator);
    const double condition = 1.0 / solve.rcond();
    if (!solve.isInvertible() || !std::isfinite(condition) || condition > kMaxCondition) {
      throw std::runtime_error(
          "hinfinity_norm: frequency solve ill-conditioned; rescale the model");
    }
    const Eigen::MatrixXcd state = solve.solve(system.b.cast<std::complex<double>>());
    const Eigen::MatrixXcd transfer =
        c.cast<std::complex<double>>() * state + d.cast<std::complex<double>>();
    const double gain = norm2(transfer);
    // Conservative roundoff allowance, not a directed-rounding enclosure.
    const double allowance = 256.0 * static_cast<double>(count) * kEps * condition
                             * (c.stableNorm() * state.stableNorm() + d.stableNorm());
    if (!std::isfinite(allowance)) {
      throw std::runtime_error("hinfinity_norm: frequency evaluation overflow");
    }
    return {gain, std::max(0.0, gain - allowance)};
  }

  Level classify(double gamma, HinfinityNorm& result) const {
    ++result.hamiltonian_evaluations;
    if (!std::isfinite(gamma) || gamma <= result.feedthrough_gain || gamma <= 0) {
      return Level::Unresolved;
    }
    const Eigen::MatrixXd ds = d / gamma;
    const Eigen::MatrixXd r = Eigen::MatrixXd::Identity(d.cols(), d.cols()) - ds.transpose() * ds;
    const Eigen::LLT<Eigen::MatrixXd> factor(r);
    const Eigen::FullPivLU<Eigen::MatrixXd> r_condition(r);
    if (factor.info() != Eigen::Success || r_condition.rcond() < 1.0 / kMaxCondition) {
      return Level::Unresolved;
    }
    const Eigen::MatrixXd rb = factor.solve(system.b.transpose());
    const Eigen::MatrixXd rd = factor.solve(ds.transpose());
    const Eigen::MatrixXd ahat = system.a + (system.b / gamma) * rd * c;
    const Eigen::MatrixXd upper = (system.b / gamma) * rb;
    const Eigen::MatrixXd lower =
        -(c.transpose() / gamma) * (Eigen::MatrixXd::Identity(d.rows(), d.rows()) + ds * rd) * c;
    Eigen::MatrixXd h(2 * count, 2 * count);
    h << ahat, upper, lower, -ahat.transpose();
    if (!h.allFinite() || !std::isfinite(h.stableNorm())) {
      throw std::runtime_error("hinfinity_norm: Hamiltonian overflow; rescale the model");
    }
    Eigen::ComplexEigenSolver<Eigen::MatrixXd> eigen(h);
    if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite()
        || !eigen.eigenvectors().allFinite()) {
      return Level::Unresolved;
    }
    const Eigen::JacobiSVD<Eigen::MatrixXcd> vectors(eigen.eigenvectors());
    const auto& singular = vectors.singularValues();
    const double condition = singular(0) / singular(singular.size() - 1);
    if (!std::isfinite(condition) || condition > kMaxCondition) {
      return Level::Unresolved;
    }
    const double hscale = std::max(h.stableNorm(), std::numeric_limits<double>::min());
    const double error = (h.cast<std::complex<double>>() * eigen.eigenvectors()
                          - eigen.eigenvectors() * eigen.eigenvalues().asDiagonal())
                             .stableNorm()
                         / (hscale * eigen.eigenvectors().stableNorm());
    const double budget = 256.0 * static_cast<double>(count) * kEps;
    if (!std::isfinite(error) || error > budget) {
      return Level::Unresolved;
    }
    const double axis_budget = std::max(budget, 8 * error) * condition * hscale;
    double separation = std::numeric_limits<double>::infinity();
    bool imaginary = false;
    double witnessed_lower = result.lower_bound;
    for (Eigen::Index i = 0; i < 2 * count; ++i) {
      const auto pole = eigen.eigenvalues()(i);
      separation = std::min(separation, std::abs(pole.real()));
      if (std::abs(pole.real()) <= axis_budget) {
        imaginary = true;
        witnessed_lower = std::max(witnessed_lower, sample(std::abs(pole.imag())).lower);
      }
    }
    if (imaginary) {
      // An ambiguous eigenvalue never raises the lower bound by assertion:
      // use an independently evaluated singular value at its frequency.
      result.lower_bound = witnessed_lower;
      return witnessed_lower >= gamma * (1.0 - 16 * budget) ? Level::Below : Level::Unresolved;
    }
    result.worst_eigenvector_condition = std::max(result.worst_eigenvector_condition, condition);
    result.smallest_axis_separation = std::min(result.smallest_axis_separation, separation);
    return Level::Above;
  }

  const model::LinearSystem& system;
  Eigen::MatrixXd c, d;
  Eigen::Index count;
};

std::pair<model::LinearSystem, model::LinearSystem> sensitivity_systems(
    const model::LinearSystem& loop) {
  loop.validate();
  if (loop.input_count() < 1 || loop.output_count() != loop.input_count()) {
    throw std::invalid_argument("sensitivity_norm_bounds: a nonempty square loop is required");
  }
  const Eigen::Index count = loop.input_count();
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(count, count);
  const Eigen::MatrixXd denominator = identity + loop.feedthrough_matrix();
  const Eigen::FullPivLU<Eigen::MatrixXd> solve(denominator);
  if (!denominator.allFinite() || !solve.isInvertible() || solve.rcond() < 1.0 / kMaxCondition) {
    throw std::invalid_argument("sensitivity_norm_bounds: I+D is singular or ill-conditioned");
  }
  const Eigen::MatrixXd dc = solve.solve(loop.output_matrix());
  const Eigen::MatrixXd direct = solve.solve(identity);
  model::LinearSystem sensitivity = loop;
  sensitivity.a = loop.a - loop.b * dc;
  sensitivity.b = loop.b * direct;
  sensitivity.c = -dc;
  sensitivity.d = direct;
  sensitivity.output_names = loop.input_names;
  sensitivity.description = "Sensitivity S=(I+L)^-1: " + loop.description;
  model::LinearSystem complementary = sensitivity;
  complementary.c = dc;
  complementary.d = identity - direct;
  complementary.description = "Complementary sensitivity T=I-S: " + loop.description;
  sensitivity.validate();
  complementary.validate();
  return {sensitivity, complementary};
}
}  // namespace

HinfinityNorm hinfinity_norm(const model::LinearSystem& system, const HinfinityOptions& options) {
  system.validate();
  if (system.input_count() < 1 || system.state_count() > 128
      || !std::isfinite(options.relative_tolerance) || options.relative_tolerance < 1e-10
      || options.relative_tolerance > 0.1 || !std::isfinite(options.absolute_tolerance)
      || options.absolute_tolerance < 0 || options.bracket_expansions < 1
      || options.bracket_expansions > 256 || options.bisection_iterations < 1
      || options.bisection_iterations > 256) {
    throw std::invalid_argument(
        "hinfinity_norm: invalid dimensions or bounded iteration/tolerance options");
  }
  detail::require_hurwitz(system.a, "hinfinity_norm");
  Bracket bracket(system);
  HinfinityNorm result;
  result.smallest_axis_separation = std::numeric_limits<double>::infinity();
  result.feedthrough_gain = norm2(bracket.d.cast<std::complex<double>>());
  const Gain dc = bracket.sample(0.0);
  result.dc_gain = dc.value;
  result.lower_bound = std::max(dc.lower, result.feedthrough_gain * (1 - 256 * kEps));
  if (system.b.isZero(0.0) || bracket.c.isZero(0.0)) {
    // A structurally constant transfer needs no Hamiltonian search. Retaining
    // a redundant B/gamma block when C=0 would manufacture ill-conditioning
    // as gamma tends to zero, despite the transfer being identically zero.
    const double allowance =
        256 * kEps * static_cast<double>(std::max(system.input_count(), system.output_count()));
    result.lower_bound = result.feedthrough_gain * (1 - allowance);
    result.upper_bound = result.feedthrough_gain * (1 + allowance);
    if (!std::isfinite(result.upper_bound)) {
      throw std::runtime_error("hinfinity_norm: constant-transfer norm overflow");
    }
    result.smallest_axis_separation = 0.0;  // no Hamiltonian level test needed
    result.relative_gap = result.upper_bound == 0
                              ? 0
                              : (result.upper_bound - result.lower_bound) / result.upper_bound;
    result.numerically_reliable = true;
    result.tolerance_met =
        result.upper_bound - result.lower_bound
        <= options.absolute_tolerance + options.relative_tolerance * result.upper_bound;
    result.diagnostic =
        "Structurally constant transfer D; singular value with roundoff allowance. "
        "Internal A stability checked. Not an interval-arithmetic certificate.";
    return result;
  }
  double candidate = std::max(1.0, 2 * std::max(result.dc_gain, result.feedthrough_gain));
  bool found_upper = false;
  for (int i = 0; i < options.bracket_expansions; ++i) {
    const auto classification = bracket.classify(candidate, result);
    if (classification == Level::Above) {
      found_upper = true;
      result.upper_bound = candidate;
    } else if (!found_upper) {
      candidate *= 2;
      if (!std::isfinite(candidate)) {
        throw std::runtime_error("hinfinity_norm: upper-bound search overflow");
      }
    } else {
      throw std::runtime_error("hinfinity_norm: upper-bound test was not repeatable");
    }
  }
  if (!found_upper) {
    throw std::runtime_error(
        "hinfinity_norm: no numerically resolved upper bound within the expansion budget");
  }
  const auto satisfied = [&]() {
    return result.upper_bound - result.lower_bound
           <= options.absolute_tolerance + options.relative_tolerance * result.upper_bound;
  };
  for (int i = 0; i < options.bisection_iterations; ++i) {
    const bool hold = satisfied();
    double trial = hold ? result.upper_bound
                        : result.lower_bound + 0.5 * (result.upper_bound - result.lower_bound);
    auto classification = bracket.classify(trial, result);
    if (!hold && classification == Level::Unresolved) {
      // A trial at the exact peak has a defective Hamiltonian eigenvalue.
      // Retry a fixed small offset; never assign an unresolved trial to a bound.
      trial =
          std::min(result.upper_bound,
                   trial + (options.absolute_tolerance + options.relative_tolerance * trial) / 8);
      classification = bracket.classify(trial, result);
    }
    if (classification == Level::Above && !hold) {
      result.upper_bound = trial;
    }
    if (result.lower_bound > result.upper_bound) {
      throw std::runtime_error(
          "hinfinity_norm: numerical level tests produced inconsistent bounds");
    }
  }
  if (bracket.classify(result.upper_bound, result) != Level::Above) {
    throw std::runtime_error("hinfinity_norm: final upper bound is not numerically resolved");
  }
  result.tolerance_met = satisfied();
  result.relative_gap =
      result.upper_bound == 0 ? 0 : (result.upper_bound - result.lower_bound) / result.upper_bound;
  result.numerically_reliable = true;
  result.diagnostic = result.tolerance_met
                          ? "Hamiltonian numerical bracket met tolerance; DC and feedthrough "
                            "included. Not an interval-arithmetic certificate."
                          : "Numerical upper bound resolved, but requested bracket tolerance was "
                            "not reached; increase the iteration budget or rescale.";
  return result;
}

SensitivityNormBounds sensitivity_norm_bounds(const model::LinearSystem& loop,
                                              const HinfinityOptions& options) {
  const auto [sensitivity, complementary] = sensitivity_systems(loop);
  SensitivityNormBounds result;
  result.sensitivity = hinfinity_norm(sensitivity, options);
  result.complementary = hinfinity_norm(complementary, options);
  result.internally_stable = true;
  result.diagnostic =
      "Negative identity feedback; S and T include algebraic feedthrough. "
      "Upper bounds are numerical Hamiltonian bounds, not interval certificates.";
  return result;
}

DiskMarginBounds disk_margin_bounds(const model::LinearSystem& loop,
                                    double skew,
                                    const HinfinityOptions& options) {
  loop.validate();
  if (loop.input_count() != 1 || loop.output_count() != 1 || !std::isfinite(skew)) {
    throw std::invalid_argument("disk_margin_bounds: a SISO loop and finite skew are required");
  }
  auto [sensitivity, complementary] = sensitivity_systems(loop);
  (void)complementary;
  sensitivity.d(0, 0) += (skew - 1.0) / 2.0;
  sensitivity.validate();
  DiskMarginBounds result;
  result.skew = skew;
  result.shifted_sensitivity = hinfinity_norm(sensitivity, options);
  if (result.shifted_sensitivity.upper_bound == 0.0) {
    result.alpha_lower = std::numeric_limits<double>::infinity();
    result.alpha_upper = std::numeric_limits<double>::infinity();
    result.internally_stable = true;
    result.diagnostic =
        "The shifted sensitivity is identically zero; the SISO disk theorem gives an "
        "unbounded alpha margin.";
    return result;
  }
  result.alpha_lower = 1.0 / result.shifted_sensitivity.upper_bound;
  result.alpha_upper = result.shifted_sensitivity.lower_bound > 0
                           ? 1.0 / result.shifted_sensitivity.lower_bound
                           : std::numeric_limits<double>::infinity();
  if (!std::isfinite(result.alpha_lower)) {
    throw std::runtime_error(
        "disk_margin_bounds: reciprocal lower bound overflow; rescale the problem");
  }
  result.internally_stable = true;
  result.diagnostic =
      "SISO disk theorem: alpha=1/||S+(skew-1)/2||inf. "
      "The conservative alpha endpoint uses the numerical norm upper bound.";
  return result;
}
}  // namespace galata::analyze
