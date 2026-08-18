// SPDX-License-Identifier: Apache-2.0

#include "galata/analyze/sensitivity.hpp"

#include "galata/analyze/frequency_response.hpp"

#include "peak_search.hpp"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::analyze {
namespace {

struct SensitivityPair {
  double sensitivity;
  double complementary;
};

// sigma_max(S) and sigma_max(T) at one frequency, from L(jw).
//
// The sensitivity comes from the SMALLEST singular value of I + L, never from
// an inverse:  sigma_max((I+L)^-1) = 1 / sigma_min(I+L).  Near the peak I + L
// is nearly singular by definition, which is exactly where inverting it would
// lose the digits that matter.
SensitivityPair pair_at(const Eigen::MatrixXcd& loop_response) {
  const Eigen::Index n = loop_response.rows();
  const Eigen::MatrixXcd shifted = Eigen::MatrixXcd::Identity(n, n) + loop_response;

  Eigen::JacobiSVD<Eigen::MatrixXcd> svd(shifted);
  const double smallest = svd.singularValues()(n - 1);

  SensitivityPair pair{};
  pair.sensitivity = smallest > 0.0 ? 1.0 / smallest : std::numeric_limits<double>::infinity();

  // T from a SOLVE of (I + L) T = L, not from an inversion.
  const Eigen::MatrixXcd complementary = shifted.partialPivLu().solve(loop_response);
  Eigen::JacobiSVD<Eigen::MatrixXcd> complementary_svd(complementary);
  pair.complementary = complementary_svd.singularValues()(0);
  return pair;
}

}  // namespace

SensitivityPeaks sensitivity_peaks(const model::LinearSystem& loop, const MarginOptions& options) {
  loop.validate();

  const Eigen::Index inputs = loop.input_count();
  const Eigen::Index outputs = loop.output_count();
  if (inputs == 0) {
    throw std::invalid_argument("sensitivity_peaks: the system has no inputs, so it is not a loop");
  }
  if (inputs != outputs) {
    std::ostringstream message;
    message << "sensitivity_peaks: the loop is " << outputs << " outputs by " << inputs
            << " inputs. S = (I + L)^-1 is defined only for a SQUARE loop; galata will not "
               "guess which channels to drop.";
    throw std::invalid_argument(message.str());
  }

  const Eigen::MatrixXd c = loop.output_matrix();
  const Eigen::MatrixXd d = loop.feedthrough_matrix();
  const Eigen::MatrixXd shifted_feedthrough = Eigen::MatrixXd::Identity(outputs, inputs) + d;
  if (std::abs(shifted_feedthrough.determinant()) == 0.0) {
    throw std::invalid_argument(
        "sensitivity_peaks: I + D is singular, so the loop is ill-posed — the closed loop has "
        "no solution at infinite frequency");
  }

  // S has the closed-loop poles. A peak of an unstable S over a finite grid is
  // not a robustness measure, so refuse rather than report one.
  const Eigen::MatrixXd closed = loop.a - loop.b * shifted_feedthrough.inverse() * c;
  Eigen::EigenSolver<Eigen::MatrixXd> solver(closed, /*computeEigenvectors=*/false);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("sensitivity_peaks: closed-loop eigenvalue computation failed");
  }
  double worst_real = -std::numeric_limits<double>::infinity();
  for (Eigen::Index index = 0; index < solver.eigenvalues().size(); ++index) {
    worst_real = std::max(worst_real, solver.eigenvalues()(index).real());
  }
  if (worst_real >= 0.0) {
    std::ostringstream message;
    message << "sensitivity_peaks: the nominal closed loop is unstable (rightmost pole at real "
               "part "
            << worst_real
            << "). S has that pole too, and the peak of an unstable S over a finite grid is a "
               "number that means nothing.";
    throw std::invalid_argument(message.str());
  }

  std::vector<double> grid = options.frequencies;
  if (grid.empty()) {
    // Refined around the CLOSED-loop modes: S and T peak at lightly damped
    // closed-loop poles, not at open-loop ones.
    grid = grid_refined_for_modes(
        closed, options.start_rad_s, options.stop_rad_s, options.grid_points);
  }
  if (grid.size() < 2) {
    throw std::invalid_argument("sensitivity_peaks: need at least two frequencies");
  }

  const FrequencyResponse response = frequency_response(loop, grid);

  SensitivityPeaks peaks{};
  peaks.frequencies_rad_s = grid;
  peaks.sensitivity.reserve(grid.size());
  peaks.complementary_sensitivity.reserve(grid.size());
  peaks.searched_from_rad_s = grid.front();
  peaks.searched_to_rad_s = grid.back();
  peaks.grid_points = static_cast<int>(grid.size());

  for (const Eigen::MatrixXcd& value : response.response) {
    const SensitivityPair pair = pair_at(value);
    peaks.sensitivity.push_back(pair.sensitivity);
    peaks.complementary_sensitivity.push_back(pair.complementary);
  }

  // Refined by the same search every other peak in this library uses, so two
  // of galata's own numbers cannot disagree about the same maximum.
  const auto evaluate = [&loop](double frequency) {
    return frequency_response(loop, {frequency}).response.front();
  };
  const detail::Peak sensitivity_peak = detail::find_peak(
      [&evaluate](double frequency) { return pair_at(evaluate(frequency)).sensitivity; },
      grid,
      options.peak_refinement_iterations);
  const detail::Peak complementary_peak = detail::find_peak(
      [&evaluate](double frequency) { return pair_at(evaluate(frequency)).complementary; },
      grid,
      options.peak_refinement_iterations);

  peaks.sensitivity_peak = sensitivity_peak.value;
  peaks.sensitivity_peak_frequency_rad_s = sensitivity_peak.frequency_rad_s;
  peaks.is_single_loop = inputs == 1;
  peaks.complementary_peak = complementary_peak.value;
  peaks.complementary_peak_frequency_rad_s = complementary_peak.frequency_rad_s;
  return peaks;
}

GuaranteedMargins guaranteed_margins(const SensitivityPeaks& peaks) {
  GuaranteedMargins result{};
  result.applies = peaks.is_single_loop;
  if (!result.applies) {
    return result;
  }

  const double sensitivity = peaks.sensitivity_peak;
  const double complementary = peaks.complementary_peak;

  // Domain guards. The book's own Remark on p. 37 notes that M_S must exceed 1
  // whenever a -180 degree crossing exists at all, so M_S <= 1 here means the
  // grid did not reach the peak rather than that the loop is extraordinary.
  const bool gain_bounds_defined = sensitivity > 1.0 && complementary > 0.0;
  const bool phase_bounds_defined = sensitivity >= 0.5 && complementary >= 0.5;
  result.valid = gain_bounds_defined && phase_bounds_defined;
  if (!result.valid) {
    return result;
  }

  result.gain_margin_from_sensitivity = sensitivity / (sensitivity - 1.0);
  result.gain_margin_from_complementary = 1.0 + 1.0 / complementary;
  result.phase_margin_from_sensitivity_rad = 2.0 * std::asin(1.0 / (2.0 * sensitivity));
  result.phase_margin_from_complementary_rad = 2.0 * std::asin(1.0 / (2.0 * complementary));
  return result;
}

}  // namespace galata::analyze
