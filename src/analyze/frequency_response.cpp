// SPDX-License-Identifier: Apache-2.0

#include "galata/analyze/frequency_response.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace galata::analyze {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kDegreesPerRadian = 57.295779513082320876798154814105;

struct HessenbergSolution {
  Eigen::MatrixXcd solution;
  // min|U_ii| / max|U_ii| from the factorisation. See the note on
  // FrequencyResponse::pivot_ratio for what this does and does not tell you.
  double pivot_ratio;
};

// Solve M X = R for an UPPER HESSENBERG M, in O(n^2) per right-hand side.
//
// Gaussian elimination with partial pivoting, exploiting the fact that each
// column has exactly one subdiagonal entry to remove. Swapping rows k and k+1
// preserves the Hessenberg pattern of the trailing submatrix — row k+2 still
// begins at column k+1 — so no fill-in appears and the O(n^3) of a general LU
// never materialises.
HessenbergSolution solve_upper_hessenberg(Eigen::MatrixXcd m, Eigen::MatrixXcd r,
                                          double frequency_rad_s) {
  const Eigen::Index n = m.rows();

  for (Eigen::Index k = 0; k + 1 < n; ++k) {
    if (std::abs(m(k + 1, k)) > std::abs(m(k, k))) {
      m.row(k).tail(n - k).swap(m.row(k + 1).tail(n - k));
      r.row(k).swap(r.row(k + 1));
    }
    if (m(k, k) == std::complex<double>(0.0, 0.0)) {
      std::ostringstream message;
      message << "frequency response: (jw I - A) is exactly singular at w = " << frequency_rad_s
              << " rad/s. That frequency is a pole of the system, where the response is "
                 "infinite. Move the grid off it.";
      throw std::domain_error(message.str());
    }
    const std::complex<double> multiplier = m(k + 1, k) / m(k, k);
    m(k + 1, k) = std::complex<double>(0.0, 0.0);
    m.row(k + 1).tail(n - k - 1) -= multiplier * m.row(k).tail(n - k - 1);
    r.row(k + 1) -= multiplier * r.row(k);
  }
  if (m(n - 1, n - 1) == std::complex<double>(0.0, 0.0)) {
    std::ostringstream message;
    message << "frequency response: (jw I - A) is exactly singular at w = " << frequency_rad_s
            << " rad/s. That frequency is a pole of the system, where the response is "
               "infinite. Move the grid off it.";
    throw std::domain_error(message.str());
  }

  double smallest = std::abs(m(0, 0));
  double largest = smallest;
  for (Eigen::Index k = 1; k < n; ++k) {
    const double pivot = std::abs(m(k, k));
    smallest = std::min(smallest, pivot);
    largest = std::max(largest, pivot);
  }

  // Back-substitution.
  for (Eigen::Index k = n - 1; k >= 0; --k) {
    if (k + 1 < n) {
      r.row(k) -= m.row(k).tail(n - k - 1) * r.bottomRows(n - k - 1);
    }
    r.row(k) /= m(k, k);
  }

  return {std::move(r), largest > 0.0 ? smallest / largest : 0.0};
}

}  // namespace

std::vector<double> logarithmic_grid(double start_rad_s, double stop_rad_s, int count) {
  if (!(start_rad_s > 0.0) || !(stop_rad_s > 0.0)) {
    throw std::invalid_argument(
        "logarithmic_grid: both endpoints must be strictly positive — a logarithmic sweep "
        "cannot contain zero");
  }
  if (!(stop_rad_s > start_rad_s)) {
    throw std::invalid_argument("logarithmic_grid: stop must be greater than start");
  }
  if (count < 2) {
    throw std::invalid_argument("logarithmic_grid: need at least two points");
  }

  const double log_start = std::log10(start_rad_s);
  const double log_stop = std::log10(stop_rad_s);
  const double step = (log_stop - log_start) / static_cast<double>(count - 1);

  std::vector<double> frequencies;
  frequencies.reserve(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    // The endpoints are assigned, not computed, so that a caller asking for a
    // decade gets exactly the decade it asked for.
    if (index == 0) {
      frequencies.push_back(start_rad_s);
    } else if (index == count - 1) {
      frequencies.push_back(stop_rad_s);
    } else {
      frequencies.push_back(std::pow(10.0, log_start + step * static_cast<double>(index)));
    }
  }
  return frequencies;
}

std::vector<double> grid_refined_for_modes(const Eigen::MatrixXd& a, double start_rad_s,
                                           double stop_rad_s, int count,
                                           double damping_threshold) {
  std::vector<double> frequencies = logarithmic_grid(start_rad_s, stop_rad_s, count);
  if (a.rows() == 0 || a.rows() != a.cols()) {
    throw std::invalid_argument("grid_refined_for_modes: A must be square and non-empty");
  }

  // Fixed offsets in units of the mode's own damping ratio. The half-power
  // points of a lightly damped second-order peak sit near w_n (1 +/- zeta), so
  // a band of +/- 4 zeta brackets the peak and both of its shoulders whatever
  // the damping is.
  static constexpr double kOffsets[] = {-4.0, -3.0, -2.0, -1.5, -1.0, -0.5, 0.0,
                                        0.5,  1.0,  1.5,  2.0,  3.0,  4.0};
  // A mode with zeta below this is treated as if it had this much, so that a
  // near-undamped mode gets a narrow cluster rather than a degenerate one.
  constexpr double kMinimumEffectiveDamping = 1.0e-3;

  Eigen::EigenSolver<Eigen::MatrixXd> solver(a, /*computeEigenvectors=*/false);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("grid_refined_for_modes: eigenvalue computation failed");
  }

  for (Eigen::Index index = 0; index < solver.eigenvalues().size(); ++index) {
    const std::complex<double> eigenvalue = solver.eigenvalues()(index);
    if (eigenvalue.imag() <= 0.0) {
      continue;  // Take one of each conjugate pair.
    }
    const double natural_frequency = std::abs(eigenvalue);
    if (!(natural_frequency > 0.0)) {
      continue;
    }
    const double damping = -eigenvalue.real() / natural_frequency;
    if (damping > damping_threshold) {
      continue;  // No peak worth resolving.
    }
    const double effective = std::max(std::abs(damping), kMinimumEffectiveDamping);
    for (const double offset : kOffsets) {
      const double frequency = natural_frequency * (1.0 + offset * effective);
      if (frequency > start_rad_s && frequency < stop_rad_s) {
        frequencies.push_back(frequency);
      }
    }
  }

  std::sort(frequencies.begin(), frequencies.end());
  // Relative dedup: two points a part in 10^9 apart carry no extra information
  // and would only make the response table longer.
  const auto last = std::unique(frequencies.begin(), frequencies.end(),
                                [](double left, double right) {
                                  return std::abs(right - left) <= 1.0e-9 * std::abs(right);
                                });
  frequencies.erase(last, frequencies.end());
  return frequencies;
}

FrequencyResponse frequency_response(const model::LinearSystem& system,
                                     const std::vector<double>& frequencies_rad_s) {
  system.validate();
  if (system.input_count() == 0) {
    throw std::invalid_argument(
        "frequency response: the system has no inputs, so there is no transfer function to "
        "evaluate");
  }
  if (frequencies_rad_s.empty()) {
    throw std::invalid_argument("frequency response: the frequency grid is empty");
  }
  for (const double frequency : frequencies_rad_s) {
    if (!std::isfinite(frequency)) {
      throw std::invalid_argument("frequency response: the grid contains a non-finite frequency");
    }
  }

  const Eigen::MatrixXd c = system.output_matrix();
  const Eigen::MatrixXd d = system.feedthrough_matrix();
  const Eigen::Index n = system.state_count();

  // The reduction happens once, outside the sweep. This is the whole point of
  // the method: the O(n^3) is paid a single time and each frequency then costs
  // O(n^2).
  const Eigen::HessenbergDecomposition<Eigen::MatrixXd> hessenberg(system.a);
  const Eigen::MatrixXd h = hessenberg.matrixH();
  const Eigen::MatrixXd q = hessenberg.matrixQ();

  const Eigen::MatrixXcd reduced_b = (q.transpose() * system.b).cast<std::complex<double>>();
  const Eigen::MatrixXcd reduced_c = (c * q).cast<std::complex<double>>();
  const Eigen::MatrixXcd complex_h = h.cast<std::complex<double>>();
  const Eigen::MatrixXcd complex_d = d.cast<std::complex<double>>();

  FrequencyResponse result;
  result.frequencies_rad_s = frequencies_rad_s;
  result.input_names = system.input_names;
  result.output_names = system.output_labels();
  result.response.reserve(frequencies_rad_s.size());
  result.pivot_ratio.reserve(frequencies_rad_s.size());

  for (const double frequency : frequencies_rad_s) {
    Eigen::MatrixXcd shifted = -complex_h;
    for (Eigen::Index k = 0; k < n; ++k) {
      shifted(k, k) += std::complex<double>(0.0, frequency);
    }
    HessenbergSolution solved = solve_upper_hessenberg(std::move(shifted), reduced_b, frequency);
    result.response.push_back(reduced_c * solved.solution + complex_d);
    result.pivot_ratio.push_back(solved.pivot_ratio);
  }
  return result;
}

FrequencyResponse single_loop_response(const model::LinearSystem& system, int input_index,
                                       int output_index,
                                       const std::vector<double>& frequencies_rad_s) {
  system.validate();
  const Eigen::Index inputs = system.input_count();
  const Eigen::Index outputs = system.output_count();
  if (input_index < 0 || static_cast<Eigen::Index>(input_index) >= inputs) {
    std::ostringstream message;
    message << "single_loop_response: input index " << input_index << " is out of range for "
            << inputs << " inputs";
    throw std::out_of_range(message.str());
  }
  if (output_index < 0 || static_cast<Eigen::Index>(output_index) >= outputs) {
    std::ostringstream message;
    message << "single_loop_response: output index " << output_index << " is out of range for "
            << outputs << " outputs";
    throw std::out_of_range(message.str());
  }

  model::LinearSystem loop = system;
  loop.b = system.b.col(input_index);
  loop.input_names = {system.input_names.at(static_cast<std::size_t>(input_index))};
  loop.c = system.output_matrix().row(output_index);
  loop.output_names = {system.output_labels().at(static_cast<std::size_t>(output_index))};
  if (system.d.size() != 0) {
    loop.d = system.d.block(output_index, input_index, 1, 1);
  } else {
    loop.d = Eigen::MatrixXd::Zero(1, 1);
  }

  return frequency_response(loop, frequencies_rad_s);
}

bool FrequencyResponse::is_single_loop() const {
  return !response.empty() && response.front().rows() == 1 && response.front().cols() == 1;
}

namespace {

void require_single_loop(const FrequencyResponse& response, const char* what) {
  if (!response.is_single_loop()) {
    std::ostringstream message;
    message << what << " is defined for a single loop only; this response is "
            << (response.response.empty() ? 0 : response.response.front().rows()) << "x"
            << (response.response.empty() ? 0 : response.response.front().cols())
            << ". Use single_loop_response to pick one input and one output.";
    throw std::invalid_argument(message.str());
  }
}

}  // namespace

std::vector<double> FrequencyResponse::magnitude() const {
  require_single_loop(*this, "magnitude");
  std::vector<double> values;
  values.reserve(response.size());
  for (const Eigen::MatrixXcd& entry : response) {
    values.push_back(std::abs(entry(0, 0)));
  }
  return values;
}

std::vector<double> FrequencyResponse::magnitude_db() const {
  std::vector<double> values = magnitude();
  for (double& value : values) {
    value = 20.0 * std::log10(value);
  }
  return values;
}

std::vector<double> FrequencyResponse::phase_deg() const {
  require_single_loop(*this, "phase");
  std::vector<double> values;
  values.reserve(response.size());

  double turns = 0.0;
  double previous = 0.0;
  for (std::size_t index = 0; index < response.size(); ++index) {
    const double principal = std::arg(response[index](0, 0));
    if (index > 0) {
      // Unwrap: choose the branch that keeps the step below half a turn. A
      // wrapped phase makes a margin search find -180 degree crossings that the
      // loop does not have.
      const double step = principal - previous;
      if (step > kTwoPi / 2.0) {
        turns -= kTwoPi;
      } else if (step < -kTwoPi / 2.0) {
        turns += kTwoPi;
      }
    }
    previous = principal;
    values.push_back((principal + turns) * kDegreesPerRadian);
  }
  return values;
}

}  // namespace galata::analyze
