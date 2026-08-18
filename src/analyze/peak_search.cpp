// SPDX-License-Identifier: Apache-2.0

#include "peak_search.hpp"

#include <stdexcept>

namespace galata::analyze::detail {
namespace {

// (3 - sqrt 5) / 2.
constexpr double kGoldenFraction = 0.3819660112501051517954131656344;

}  // namespace

Peak find_peak(const std::function<double(double)>& f,
               const std::vector<double>& grid,
               int refinement_iterations) {
  if (!f) {
    throw std::invalid_argument("find_peak: no function");
  }
  if (grid.size() < 2) {
    throw std::invalid_argument("find_peak: need at least two grid points");
  }
  if (refinement_iterations < 1) {
    throw std::invalid_argument("find_peak: refinement_iterations must be positive");
  }

  std::size_t peak_index = 0;
  double peak_value = f(grid[0]);
  for (std::size_t index = 1; index < grid.size(); ++index) {
    const double value = f(grid[index]);
    if (value > peak_value) {
      peak_value = value;
      peak_index = index;
    }
  }

  double low = grid[peak_index == 0 ? 0 : peak_index - 1];
  double high = grid[peak_index + 1 < grid.size() ? peak_index + 1 : grid.size() - 1];
  double frequency = grid[peak_index];

  if (high > low) {
    for (int step = 0; step < refinement_iterations; ++step) {
      const double first = low + (high - low) * kGoldenFraction;
      const double second = high - (high - low) * kGoldenFraction;
      if (first >= second) {
        break;
      }
      if (f(first) > f(second)) {
        high = second;
      } else {
        low = first;
      }
    }
    const double refined = 0.5 * (low + high);
    const double refined_value = f(refined);
    if (refined_value > peak_value) {
      peak_value = refined_value;
      frequency = refined;
    }
  }

  return {frequency, peak_value};
}

}  // namespace galata::analyze::detail
