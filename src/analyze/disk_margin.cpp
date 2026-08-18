// SPDX-License-Identifier: Apache-2.0

#include "galata/analyze/disk_margin.hpp"

#include "galata/analyze/frequency_response.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::analyze {
namespace {

constexpr double kDegreesPerRadian = 57.295779513082320876798154814105;
// (3 - sqrt 5) / 2, the golden-section fraction.
constexpr double kGoldenFraction = 0.3819660112501051517954131656344;

// | S(jw) + (sigma - 1)/2 |, the quantity whose peak IS the reciprocal of the
// disk margin (eq:alphadm).
double shifted_sensitivity_gain(const LoopEvaluator& loop, double frequency, double skew) {
  const std::complex<double> l = loop(frequency);
  const std::complex<double> sensitivity = 1.0 / (1.0 + l);
  return std::abs(sensitivity + (skew - 1.0) / 2.0);
}

}  // namespace

DiskMargin disk_margin(const LoopEvaluator& loop, double skew, const MarginOptions& options) {
  if (!loop) {
    throw std::invalid_argument("disk_margin: no loop evaluator");
  }
  if (!std::isfinite(skew)) {
    throw std::invalid_argument("disk_margin: the skew must be finite");
  }
  if (options.peak_refinement_iterations < 1) {
    throw std::invalid_argument("disk_margin: peak_refinement_iterations must be positive");
  }

  std::vector<double> grid = options.frequencies;
  if (grid.empty()) {
    grid = logarithmic_grid(options.start_rad_s, options.stop_rad_s, options.grid_points);
  }
  if (grid.size() < 2) {
    throw std::invalid_argument("disk_margin: need at least two frequencies");
  }

  const auto gain = [&loop, skew](double frequency) {
    return shifted_sensitivity_gain(loop, frequency, skew);
  };

  // Locate the peak on the grid, then refine it by golden section on the
  // bracket its neighbours provide. A fixed step count, not a tolerance
  // (ADR-0004).
  std::size_t peak_index = 0;
  double peak_value = gain(grid[0]);
  for (std::size_t index = 1; index < grid.size(); ++index) {
    const double value = gain(grid[index]);
    if (value > peak_value) {
      peak_value = value;
      peak_index = index;
    }
  }

  double low = grid[peak_index == 0 ? 0 : peak_index - 1];
  double high = grid[peak_index + 1 < grid.size() ? peak_index + 1 : grid.size() - 1];
  if (high > low) {
    for (int step = 0; step < options.peak_refinement_iterations; ++step) {
      const double first = low + (high - low) * kGoldenFraction;
      const double second = high - (high - low) * kGoldenFraction;
      if (first >= second) {
        break;
      }
      if (gain(first) > gain(second)) {
        high = second;
      } else {
        low = first;
      }
    }
    const double refined = 0.5 * (low + high);
    const double refined_value = gain(refined);
    // The grid point wins if refinement somehow did worse: the peak is a
    // maximum and this function must never report a smaller one than it
    // actually saw.
    if (refined_value > peak_value) {
      peak_value = refined_value;
      grid[peak_index] = refined;
    }
  }
  const double critical_frequency = grid[peak_index];

  DiskMargin margin{};
  margin.skew = skew;
  margin.peak_gain = peak_value;
  margin.critical_frequency_rad_s = critical_frequency;
  margin.searched_from_rad_s = grid.front();
  margin.searched_to_rad_s = grid.back();
  margin.grid_points = static_cast<int>(grid.size());

  const double infinity = std::numeric_limits<double>::infinity();
  if (!(peak_value > 0.0)) {
    // |S + (sigma-1)/2| vanishes identically: no perturbation in any disk can
    // destabilise, and the margin is unbounded.
    margin.alpha = infinity;
    margin.gain_variation_min = 0.0;
    margin.gain_variation_max = infinity;
    margin.gain_variation_is_bounded = false;
    margin.gain_variation_min_db = -infinity;
    margin.gain_variation_max_db = infinity;
    margin.phase_variation_deg = infinity;
    margin.phase_variation_is_bounded = false;
    margin.destabilising_delta = {infinity, infinity};
    margin.destabilising_perturbation = {infinity, infinity};
    return margin;
  }

  const double alpha = 1.0 / peak_value;
  margin.alpha = alpha;

  // eq:galphamax.
  const double lower_denominator = 2.0 + alpha * (1.0 + skew);
  const double upper_denominator = 2.0 - alpha * (1.0 + skew);
  margin.gain_variation_min = (2.0 - alpha * (1.0 - skew)) / lower_denominator;
  margin.gain_variation_is_bounded = upper_denominator > 0.0;
  margin.gain_variation_max = margin.gain_variation_is_bounded
                                  ? (2.0 + alpha * (1.0 - skew)) / upper_denominator
                                  : infinity;
  margin.gain_variation_min_db =
      margin.gain_variation_min > 0.0 ? 20.0 * std::log10(margin.gain_variation_min) : -infinity;
  margin.gain_variation_max_db =
      margin.gain_variation_is_bounded ? 20.0 * std::log10(margin.gain_variation_max) : infinity;

  // eq:cosphim. Unbounded when the disk swallows the unit circle, which shows
  // up as the cosine leaving [-1, 1].
  if (!margin.gain_variation_is_bounded) {
    margin.phase_variation_deg = infinity;
    margin.phase_variation_is_bounded = false;
  } else {
    const double cosine = (1.0 + margin.gain_variation_min * margin.gain_variation_max)
                          / (margin.gain_variation_min + margin.gain_variation_max);
    if (!(std::abs(cosine) <= 1.0)) {
      margin.phase_variation_deg = infinity;
      margin.phase_variation_is_bounded = false;
    } else {
      margin.phase_variation_deg = std::acos(cosine) * kDegreesPerRadian;
      margin.phase_variation_is_bounded = true;
    }
  }

  // The destabilising perturbation from the theorem's proof.
  const std::complex<double> sensitivity = 1.0 / (1.0 + loop(critical_frequency));
  const std::complex<double> shifted = sensitivity + (skew - 1.0) / 2.0;
  margin.destabilising_delta = 1.0 / shifted;
  const std::complex<double> numerator = 2.0 + (1.0 - skew) * margin.destabilising_delta;
  const std::complex<double> denominator = 2.0 - (1.0 + skew) * margin.destabilising_delta;
  margin.destabilising_perturbation =
      denominator == std::complex<double>(0.0, 0.0)
          ? std::complex<double>(infinity, infinity)
          : numerator / denominator;

  return margin;
}

DiskMargin disk_margin(const model::LinearSystem& loop, int input_index, int output_index,
                       double skew, const MarginOptions& options) {
  loop.validate();

  // The theorem assumes the NOMINAL closed loop is stable. Without that,
  // alpha_max is a number with no meaning attached, so refuse rather than
  // return it.
  const Eigen::MatrixXd c = loop.output_matrix().row(output_index);
  const Eigen::MatrixXd b = loop.b.col(input_index);
  const double feedthrough = loop.d.size() == 0 ? 0.0 : loop.d(output_index, input_index);
  if (1.0 + feedthrough == 0.0) {
    throw std::invalid_argument(
        "disk_margin: the loop is ill-posed — its direct feedthrough is -1, so the closed loop "
        "has no solution at high frequency");
  }
  const Eigen::MatrixXd closed = loop.a - b * c / (1.0 + feedthrough);

  Eigen::EigenSolver<Eigen::MatrixXd> solver(closed, /*computeEigenvectors=*/false);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("disk_margin: closed-loop eigenvalue computation failed");
  }
  double worst_real = -std::numeric_limits<double>::infinity();
  for (Eigen::Index index = 0; index < solver.eigenvalues().size(); ++index) {
    worst_real = std::max(worst_real, solver.eigenvalues()(index).real());
  }
  if (worst_real >= 0.0) {
    std::ostringstream message;
    message << "disk_margin: the nominal closed loop is already unstable (rightmost pole at real "
               "part "
            << worst_real
            << "). The disk margin theorem assumes nominal stability; applied here it would "
               "return a number that means nothing.";
    throw std::invalid_argument(message.str());
  }

  MarginOptions resolved = options;
  if (resolved.frequencies.empty()) {
    // Refined around the CLOSED-loop modes: the peak of S sits at a lightly
    // damped closed-loop pole, not at an open-loop one.
    resolved.frequencies = grid_refined_for_modes(closed, resolved.start_rad_s,
                                                  resolved.stop_rad_s, resolved.grid_points);
  }

  const LoopEvaluator evaluator = [&loop, input_index, output_index](double frequency) {
    return single_loop_response(loop, input_index, output_index, {frequency}).response.front()(0, 0);
  };
  return disk_margin(evaluator, skew, resolved);
}

}  // namespace galata::analyze
