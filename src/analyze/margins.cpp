// SPDX-License-Identifier: Apache-2.0

#include "galata/analyze/margins.hpp"

#include "galata/analyze/frequency_response.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace galata::analyze {
namespace {

constexpr double kDegreesPerRadian = 57.295779513082320876798154814105;

// Bisect a sign-changing function on [low, high] for a FIXED number of steps.
// The count is fixed rather than tolerance-driven so that the answer does not
// depend on the order in which rounding happens (ADR-0004).
template <typename Function>
double bisect(const Function& f, double low, double high, double value_at_low, int iterations) {
  const bool low_is_negative = value_at_low < 0.0;
  for (int step = 0; step < iterations; ++step) {
    const double middle = 0.5 * (low + high);
    if (middle <= low || middle >= high) {
      break;  // The bracket is down to adjacent representable numbers.
    }
    const double value = f(middle);
    if ((value < 0.0) == low_is_negative) {
      low = middle;
    } else {
      high = middle;
    }
  }
  return 0.5 * (low + high);
}

// arg L + 180 degrees, reduced into (-180, 180].
double phase_margin_at(std::complex<double> value) {
  double margin = 180.0 + std::arg(value) * kDegreesPerRadian;
  while (margin > 180.0) {
    margin -= 360.0;
  }
  while (margin <= -180.0) {
    margin += 360.0;
  }
  return margin;
}

}  // namespace

StabilityMargins stability_margins(const LoopEvaluator& loop, const MarginOptions& options) {
  if (!loop) {
    throw std::invalid_argument("stability_margins: no loop evaluator");
  }
  if (options.bisection_iterations < 1) {
    throw std::invalid_argument("stability_margins: bisection_iterations must be positive");
  }

  std::vector<double> grid = options.frequencies;
  if (grid.empty()) {
    grid = logarithmic_grid(options.start_rad_s, options.stop_rad_s, options.grid_points);
  }
  if (grid.size() < 2) {
    throw std::invalid_argument("stability_margins: need at least two frequencies to bracket a "
                                "crossing");
  }

  std::vector<std::complex<double>> sampled;
  sampled.reserve(grid.size());
  for (const double frequency : grid) {
    sampled.push_back(loop(frequency));
  }

  StabilityMargins margins{};
  margins.searched_from_rad_s = grid.front();
  margins.searched_to_rad_s = grid.back();
  margins.grid_points = static_cast<int>(grid.size());

  // --- Gain crossings: |L| = 1. -------------------------------------------
  const auto magnitude_excess = [&loop](double frequency) {
    return std::abs(loop(frequency)) - 1.0;
  };
  for (std::size_t index = 1; index < grid.size(); ++index) {
    const double before = std::abs(sampled[index - 1]) - 1.0;
    const double after = std::abs(sampled[index]) - 1.0;
    if (before == 0.0) {
      // The grid landed exactly on it; the bracket below would not see a sign
      // change, so take the point itself.
    } else if ((before < 0.0) == (after < 0.0)) {
      continue;
    }
    const double frequency =
        before == 0.0 ? grid[index - 1]
                      : bisect(magnitude_excess, grid[index - 1], grid[index], before,
                               options.bisection_iterations);
    const double phase_margin = phase_margin_at(loop(frequency));
    GainCrossing crossing{};
    crossing.frequency_rad_s = frequency;
    crossing.phase_margin_deg = phase_margin;
    crossing.delay_margin_s = phase_margin / kDegreesPerRadian / frequency;
    margins.gain_crossings.push_back(crossing);
  }

  // --- Phase crossings: L real and negative. ------------------------------
  //
  // Detected on the sign of the IMAGINARY PART rather than on an unwrapped
  // phase. The two are equivalent, but the imaginary part needs no branch
  // bookkeeping: a bisection driven by unwrapped phase has to decide which
  // 360-degree branch an intermediate sample belongs to, and get it right at
  // exactly the frequencies where the phase is moving fastest.
  const auto imaginary_part = [&loop](double frequency) { return loop(frequency).imag(); };
  for (std::size_t index = 1; index < grid.size(); ++index) {
    const double before = sampled[index - 1].imag();
    const double after = sampled[index].imag();
    if (before != 0.0 && (before < 0.0) == (after < 0.0)) {
      continue;
    }
    // Both ends on the negative-real side. A bracket that straddles the
    // positive real axis is a crossing of 0 degrees, not of 180, and is not a
    // phase crossover.
    if (sampled[index - 1].real() >= 0.0 || sampled[index].real() >= 0.0) {
      continue;
    }
    const double frequency =
        before == 0.0 ? grid[index - 1]
                      : bisect(imaginary_part, grid[index - 1], grid[index], before,
                               options.bisection_iterations);
    const std::complex<double> value = loop(frequency);
    if (value.real() >= 0.0) {
      continue;
    }
    const double gain = std::abs(value);
    if (!(gain > 0.0)) {
      continue;  // |L| = 0 is an infinite gain margin, not a crossing.
    }
    PhaseCrossing crossing{};
    crossing.frequency_rad_s = frequency;
    crossing.gain_margin = 1.0 / gain;
    crossing.gain_margin_db = -20.0 * std::log10(gain);
    margins.phase_crossings.push_back(crossing);
  }

  // --- Governing values. ---------------------------------------------------
  const double infinity = std::numeric_limits<double>::infinity();
  const double not_a_number = std::numeric_limits<double>::quiet_NaN();

  margins.has_gain_margin = !margins.phase_crossings.empty();
  if (margins.has_gain_margin) {
    // Nearest 0 dB, not smallest ratio: a gain margin of 0.5 (the gain must be
    // halved) is exactly as tight as one of 2.0, and taking the smaller number
    // would call the first one worse.
    const auto worst = std::min_element(
        margins.phase_crossings.begin(), margins.phase_crossings.end(),
        [](const PhaseCrossing& left, const PhaseCrossing& right) {
          return std::abs(left.gain_margin_db) < std::abs(right.gain_margin_db);
        });
    margins.gain_margin = worst->gain_margin;
    margins.gain_margin_db = worst->gain_margin_db;
    margins.gain_margin_frequency_rad_s = worst->frequency_rad_s;
  } else {
    margins.gain_margin = infinity;
    margins.gain_margin_db = infinity;
    margins.gain_margin_frequency_rad_s = not_a_number;
  }

  margins.has_phase_margin = !margins.gain_crossings.empty();
  if (margins.has_phase_margin) {
    const auto worst = std::min_element(
        margins.gain_crossings.begin(), margins.gain_crossings.end(),
        [](const GainCrossing& left, const GainCrossing& right) {
          return std::abs(left.phase_margin_deg) < std::abs(right.phase_margin_deg);
        });
    margins.phase_margin_deg = worst->phase_margin_deg;
    margins.phase_margin_frequency_rad_s = worst->frequency_rad_s;
  } else {
    margins.phase_margin_deg = infinity;
    margins.phase_margin_frequency_rad_s = not_a_number;
  }

  margins.has_delay_margin = false;
  margins.delay_margin_s = infinity;
  margins.delay_margin_frequency_rad_s = not_a_number;
  for (const GainCrossing& crossing : margins.gain_crossings) {
    if (crossing.phase_margin_deg > 0.0 && crossing.delay_margin_s < margins.delay_margin_s) {
      margins.has_delay_margin = true;
      margins.delay_margin_s = crossing.delay_margin_s;
      margins.delay_margin_frequency_rad_s = crossing.frequency_rad_s;
    }
  }
  if (!margins.has_delay_margin) {
    margins.delay_margin_s = infinity;
    margins.delay_margin_frequency_rad_s = not_a_number;
  }

  return margins;
}

StabilityMargins stability_margins(const model::LinearSystem& loop, int input_index,
                                   int output_index, const MarginOptions& options) {
  loop.validate();

  MarginOptions resolved = options;
  if (resolved.frequencies.empty()) {
    resolved.frequencies = grid_refined_for_modes(loop.a, resolved.start_rad_s,
                                                  resolved.stop_rad_s, resolved.grid_points);
  }

  // One evaluation per call keeps the evaluator general, at the cost of
  // repeating the Hessenberg reduction. The bisections are the only repeated
  // evaluations and there are a fixed few of them per crossing.
  const LoopEvaluator evaluator = [&loop, input_index, output_index](double frequency) {
    return single_loop_response(loop, input_index, output_index, {frequency}).response.front()(0, 0);
  };
  return stability_margins(evaluator, resolved);
}

}  // namespace galata::analyze
