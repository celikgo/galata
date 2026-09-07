// SPDX-License-Identifier: Apache-2.0

#include "galata/analyze/margins.hpp"

#include "galata/analyze/frequency_response.hpp"

#include "analysis_checks.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace galata::analyze {
namespace {

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

// Where, if anywhere, a sign-changing function has a root in the grid interval
// ending at `index`.
//
// The exact-zero case is what this exists for. A root that lands ON a grid
// point is visible from BOTH adjacent intervals, and recording it from each
// gives the same crossing twice. Worse, a function that is identically zero —
// a constant, negative, real loop has Im L = 0 everywhere — would be recorded
// at every single grid point, turning one crossing into two thousand.
//
// The rule: a zero belongs to the interval that ENDS at it, never the one that
// starts there. A zero at the very first grid point has no interval ending at
// it, so it is claimed by the first interval instead.
enum class Bracket { None, AtLowEnd, AtHighEnd, Straddles };

Bracket classify_bracket(double before, double after, std::size_t index) {
  if (before == 0.0) {
    return index == 1 ? Bracket::AtLowEnd : Bracket::None;
  }
  if (after == 0.0) {
    return Bracket::AtHighEnd;
  }
  return (before < 0.0) == (after < 0.0) ? Bracket::None : Bracket::Straddles;
}

// arg L + pi, reduced into (-pi, pi]. Radians: ADR-0003.
double phase_margin_at(std::complex<double> value) {
  constexpr double kPi = std::numbers::pi_v<double>;
  double margin = kPi + std::arg(value);
  while (margin > kPi) {
    margin -= 2.0 * kPi;
  }
  while (margin <= -kPi) {
    margin += 2.0 * kPi;
  }
  return margin;
}

// Delay only adds phase LAG. A negative signed phase margin describes the
// shorter rotation in the opposite direction; it does not establish nominal
// instability. Continue around the circle to obtain the first non-negative
// lag which reaches -1, retaining zero when the loop is already there.
double delay_margin_at(double phase_margin, double frequency) {
  const double lag =
      phase_margin < 0.0 ? phase_margin + 2.0 * std::numbers::pi_v<double> : phase_margin;
  return lag / frequency;
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
  detail::require_frequency_grid(grid, "stability_margins");
  const auto evaluate = [&loop](double frequency) {
    return detail::checked_loop_value(loop, frequency, "stability_margins");
  };

  std::vector<std::complex<double>> sampled;
  sampled.reserve(grid.size());
  for (const double frequency : grid) {
    sampled.push_back(evaluate(frequency));
  }

  StabilityMargins margins{};
  margins.searched_from_rad_s = grid.front();
  margins.searched_to_rad_s = grid.back();
  margins.grid_points = static_cast<int>(grid.size());

  // --- Gain crossings: |L| = 1. -------------------------------------------
  const auto magnitude_excess = [&evaluate](double frequency) {
    return std::abs(evaluate(frequency)) - 1.0;
  };
  for (std::size_t index = 1; index < grid.size(); ++index) {
    const double before = std::abs(sampled[index - 1]) - 1.0;
    const double after = std::abs(sampled[index]) - 1.0;
    const Bracket bracket = classify_bracket(before, after, index);
    if (bracket == Bracket::None) {
      continue;
    }
    const double frequency = bracket == Bracket::AtLowEnd    ? grid[index - 1]
                             : bracket == Bracket::AtHighEnd ? grid[index]
                                                             : bisect(magnitude_excess,
                                                                      grid[index - 1],
                                                                      grid[index],
                                                                      before,
                                                                      options.bisection_iterations);
    const double phase_margin = phase_margin_at(evaluate(frequency));
    GainCrossing crossing{};
    crossing.frequency_rad_s = frequency;
    crossing.phase_margin_rad = phase_margin;
    crossing.delay_margin_s = delay_margin_at(phase_margin, frequency);
    margins.gain_crossings.push_back(crossing);
  }

  // --- Phase crossings: L real and negative. ------------------------------
  //
  // Detected on the sign of the IMAGINARY PART rather than on an unwrapped
  // phase. The two are equivalent, but the imaginary part needs no branch
  // bookkeeping: a bisection driven by unwrapped phase has to decide which
  // 360-degree branch an intermediate sample belongs to, and get it right at
  // exactly the frequencies where the phase is moving fastest.
  const auto imaginary_part = [&evaluate](double frequency) { return evaluate(frequency).imag(); };
  for (std::size_t index = 1; index < grid.size(); ++index) {
    const double before = sampled[index - 1].imag();
    const double after = sampled[index].imag();
    const Bracket bracket = classify_bracket(before, after, index);
    if (bracket == Bracket::None) {
      continue;
    }
    // Both ends on the negative-real side. A bracket that straddles the
    // positive real axis is a crossing of 0 degrees, not of 180, and is not a
    // phase crossover.
    if (sampled[index - 1].real() >= 0.0 || sampled[index].real() >= 0.0) {
      continue;
    }
    const double frequency = bracket == Bracket::AtLowEnd    ? grid[index - 1]
                             : bracket == Bracket::AtHighEnd ? grid[index]
                                                             : bisect(imaginary_part,
                                                                      grid[index - 1],
                                                                      grid[index],
                                                                      before,
                                                                      options.bisection_iterations);
    const std::complex<double> value = evaluate(frequency);
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
    const auto worst =
        std::min_element(margins.phase_crossings.begin(),
                         margins.phase_crossings.end(),
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
    const auto worst = std::min_element(margins.gain_crossings.begin(),
                                        margins.gain_crossings.end(),
                                        [](const GainCrossing& left, const GainCrossing& right) {
                                          return std::abs(left.phase_margin_rad)
                                                 < std::abs(right.phase_margin_rad);
                                        });
    margins.phase_margin_rad = worst->phase_margin_rad;
    margins.phase_margin_frequency_rad_s = worst->frequency_rad_s;
  } else {
    margins.phase_margin_rad = infinity;
    margins.phase_margin_frequency_rad_s = not_a_number;
  }

  margins.has_delay_margin = false;
  margins.delay_margin_s = infinity;
  margins.delay_margin_frequency_rad_s = not_a_number;
  for (const GainCrossing& crossing : margins.gain_crossings) {
    if (crossing.delay_margin_s < margins.delay_margin_s) {
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

StabilityMargins stability_margins(const model::LinearSystem& loop,
                                   int input_index,
                                   int output_index,
                                   const MarginOptions& options) {
  loop.validate();

  if (input_index < 0 || input_index >= loop.input_count() || output_index < 0
      || output_index >= loop.output_count()) {
    throw std::out_of_range("stability_margins: input or output index is out of range");
  }
  const double feedthrough = loop.feedthrough_matrix()(output_index, input_index);
  if (1.0 + feedthrough == 0.0) {
    throw std::invalid_argument(
        "stability_margins: direct feedthrough -1 makes negative unit feedback ill-posed");
  }
  const Eigen::MatrixXd closed =
      loop.a
      - loop.b.col(input_index) * loop.output_matrix().row(output_index) / (1.0 + feedthrough);
  if (!closed.allFinite()) {
    throw std::invalid_argument("stability_margins: closing the feedback loop overflowed");
  }
  const auto stability = detail::assess_hurwitz(closed);
  if (stability.status == detail::HurwitzStatus::Unresolved) {
    throw std::runtime_error("stability_margins: " + stability.diagnostic);
  }
  const bool nominal_stable = stability.status == detail::HurwitzStatus::Stable;

  MarginOptions resolved = options;
  if (resolved.frequencies.empty()) {
    resolved.frequencies = grid_refined_for_modes(
        loop.a, resolved.start_rad_s, resolved.stop_rad_s, resolved.grid_points);
  }

  // One evaluation per call keeps the evaluator general, at the cost of
  // repeating the Hessenberg reduction. The bisections are the only repeated
  // evaluations and there are a fixed few of them per crossing.
  const LoopEvaluator evaluator = [&loop, input_index, output_index](double frequency) {
    return single_loop_response(loop, input_index, output_index, {frequency})
        .response.front()(0, 0);
  };
  StabilityMargins margins = stability_margins(evaluator, resolved);
  margins.nominal_stability_checked = true;
  margins.nominal_closed_loop_stable = nominal_stable;
  margins.nominal_stability_diagnostic = stability.diagnostic;
  if (!nominal_stable) {
    // There is no positive stability tolerance around an unstable or marginal
    // baseline. Keep the crossover geometry, but do not label it a delay that
    // can safely be added to the nominal system.
    margins.has_delay_margin = false;
    margins.delay_margin_s = 0.0;
    margins.delay_margin_frequency_rad_s = std::numeric_limits<double>::quiet_NaN();
  }
  return margins;
}

}  // namespace galata::analyze
