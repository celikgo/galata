// SPDX-License-Identifier: Apache-2.0
//
// Sensitivity and complementary sensitivity peaks.
//
// Three independent kinds of evidence, in increasing order of how hard they
// are to pass by accident:
//
//   * CLOSED FORM. For a SISO loop, sigma_max(S) is just |1/(1+L)|, which can
//     be written down.
//
//   * THE IDENTITY S + T = I, checked as a matrix identity rather than through
//     the magnitudes, since |S| + |T| is not 1 and a test that assumed it were
//     would be asserting something false.
//
//   * AGREEMENT WITH THE DISK MARGIN, which reaches the same two numbers by a
//     completely different route. The disk margin evaluates the scalar
//     |S + (sigma-1)/2| and takes its peak; this file computes singular values
//     of I + L and inverts the smallest. Seiler, Packard & Gahinet (2020) prove
//     the two must coincide at skew +1 and -1. Nothing about the two
//     implementations is shared below the frequency response, so agreement is
//     evidence rather than tautology.

#include "galata/analyze/disk_margin.hpp"
#include "galata/analyze/frequency_response.hpp"
#include "galata/analyze/sensitivity.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>
#include <complex>

namespace {

using galata::analyze::disk_margin;
using galata::analyze::logarithmic_grid;
using galata::analyze::MarginOptions;
using galata::analyze::sensitivity_peaks;
using galata::model::LinearSystem;

// 25 / (s^3 + 10 s^2 + 10 s + 10), the tutorial loop used throughout.
LinearSystem tutorial_loop() {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(3, 3);
  system.a << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -10.0, -10.0, -10.0;
  system.b = Eigen::MatrixXd::Zero(3, 1);
  system.b(2, 0) = 1.0;
  system.c = Eigen::MatrixXd::Zero(1, 3);
  system.c(0, 0) = 25.0;
  system.state_names = {"x0", "x1", "x2"};
  system.input_names = {"u"};
  system.output_names = {"y"};
  return system;
}

// diag(2/(s+1), 5/(s+3)) — a decoupled MIMO loop, for which S is diagonal and
// its singular values can be written down.
LinearSystem diagonal_loop() {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(2, 2);
  system.a << -1.0, 0.0, 0.0, -3.0;
  system.b = Eigen::MatrixXd::Zero(2, 2);
  system.b << 2.0, 0.0, 0.0, 5.0;
  system.state_names = {"x0", "x1"};
  system.input_names = {"u0", "u1"};
  return system;
}

MarginOptions wide_sweep() {
  MarginOptions options;
  options.start_rad_s = 1.0e-2;
  options.stop_rad_s = 1.0e3;
  options.grid_points = 4000;
  return options;
}

TEST(Sensitivity, SisoTracesMatchTheirClosedForms) {
  const LinearSystem loop = tutorial_loop();
  const auto peaks = sensitivity_peaks(loop, wide_sweep());

  for (std::size_t index = 0; index < peaks.frequencies_rad_s.size(); ++index) {
    const std::complex<double> s{0.0, peaks.frequencies_rad_s[index]};
    const std::complex<double> l = 25.0 / (s * s * s + 10.0 * s * s + 10.0 * s + 10.0);
    const std::complex<double> sensitivity = 1.0 / (1.0 + l);
    const std::complex<double> complementary = l / (1.0 + l);

    EXPECT_NEAR(peaks.sensitivity[index], std::abs(sensitivity), 1.0e-13 * std::abs(sensitivity));
    EXPECT_NEAR(peaks.complementary_sensitivity[index],
                std::abs(complementary),
                1.0e-13 * std::abs(complementary));
  }
}

TEST(Sensitivity, SplusTIsTheIdentityAsMatrices) {
  // Checked on the MATRICES, not on the magnitudes: |S| + |T| is not 1, and a
  // test that assumed it were would pass for the wrong reason on a SISO loop
  // and fail for the right one on a MIMO loop.
  const LinearSystem loop = diagonal_loop();
  const std::vector<double> grid = logarithmic_grid(0.01, 100.0, 40);
  const auto response = galata::analyze::frequency_response(loop, grid);

  for (const Eigen::MatrixXcd& l : response.response) {
    const Eigen::MatrixXcd shifted = Eigen::MatrixXcd::Identity(2, 2) + l;
    const Eigen::MatrixXcd sensitivity =
        shifted.partialPivLu().solve(Eigen::MatrixXcd::Identity(2, 2));
    const Eigen::MatrixXcd complementary = shifted.partialPivLu().solve(l);
    const Eigen::MatrixXcd sum = sensitivity + complementary;
    EXPECT_LT((sum - Eigen::MatrixXcd::Identity(2, 2)).cwiseAbs().maxCoeff(), 1.0e-13);
  }
}

TEST(Sensitivity, DiagonalMimoLoopMatchesItsClosedForm) {
  // For a DIAGONAL loop, S is diagonal with entries 1/(1+L_ii), so its largest
  // singular value is the largest of those magnitudes.
  const LinearSystem loop = diagonal_loop();
  const auto peaks = sensitivity_peaks(loop, wide_sweep());

  for (std::size_t index = 0; index < peaks.frequencies_rad_s.size(); ++index) {
    const std::complex<double> s{0.0, peaks.frequencies_rad_s[index]};
    const std::complex<double> first = 2.0 / (s + 1.0);
    const std::complex<double> second = 5.0 / (s + 3.0);
    const double expected_sensitivity =
        std::max(std::abs(1.0 / (1.0 + first)), std::abs(1.0 / (1.0 + second)));
    const double expected_complementary =
        std::max(std::abs(first / (1.0 + first)), std::abs(second / (1.0 + second)));

    EXPECT_NEAR(peaks.sensitivity[index], expected_sensitivity, 1.0e-13 * expected_sensitivity);
    EXPECT_NEAR(peaks.complementary_sensitivity[index],
                expected_complementary,
                1.0e-13 * expected_complementary);
  }
}

TEST(Sensitivity, PeaksAgreeWithTheDiskMarginAtTheNamedSkews) {
  // The theorem: alpha_max at skew +1 is 1/||S||_inf, and at skew -1 is
  // 1/||T||_inf (Seiler, Packard & Gahinet 2020). Two implementations that
  // share nothing below the frequency response must land on the same numbers.
  for (const LinearSystem& loop : {tutorial_loop(), diagonal_loop()}) {
    MarginOptions options = wide_sweep();
    // The same explicit grid on both sides, so any disagreement is the
    // computation and not a different set of sample points.
    options.frequencies = logarithmic_grid(options.start_rad_s, options.stop_rad_s, 2000);

    const auto peaks = sensitivity_peaks(loop, options);

    // The disk margin is a single-loop calculation, so only the SISO case can
    // be compared channel for channel.
    if (loop.input_count() != 1) {
      continue;
    }
    const auto s_based = disk_margin(loop, 0, 0, 1.0, options);
    const auto t_based = disk_margin(loop, 0, 0, -1.0, options);

    EXPECT_NEAR(peaks.sensitivity_peak, 1.0 / s_based.alpha, 1.0e-12 * peaks.sensitivity_peak);
    EXPECT_NEAR(peaks.sensitivity_peak_frequency_rad_s,
                s_based.critical_frequency_rad_s,
                1.0e-9 * s_based.critical_frequency_rad_s);
    EXPECT_NEAR(peaks.complementary_peak, 1.0 / t_based.alpha, 1.0e-12 * peaks.complementary_peak);
    EXPECT_NEAR(peaks.complementary_peak_frequency_rad_s,
                t_based.critical_frequency_rad_s,
                1.0e-9 * t_based.critical_frequency_rad_s);
  }
}

TEST(Sensitivity, TheReportedPeakIsALowerBoundOnTheTrueOne) {
  // For any loop whose L rolls off, S -> I at high frequency, so the TRUE M_S
  // is at least 1. The REPORTED M_S is a grid maximum and can therefore be
  // below 1 — and for one of these two loops it is.
  //
  // That is not a defect, and asserting M_S >= 1 would be asserting the true
  // norm about a number that is explicitly a lower bound on it. The two loops
  // below show both sides:
  //
  //   * The tutorial loop peaks at about 2.49 in the middle of the band, well
  //     inside the sweep, so its grid maximum is the real one.
  //
  //   * diag(2/(s+1), 5/(s+3)) has S = diag((s+1)/(s+3), (s+3)/(s+8)), whose
  //     magnitudes rise MONOTONICALLY to 1 and reach it only at infinity. Its
  //     grid maximum approaches 1 from below and never attains it, however far
  //     the sweep is extended.
  const auto tutorial = sensitivity_peaks(tutorial_loop(), wide_sweep());
  EXPECT_GT(tutorial.sensitivity_peak, 1.0);
  EXPECT_LT(tutorial.sensitivity_peak_frequency_rad_s, tutorial.searched_to_rad_s)
      << "this peak should be interior to the band, not pinned to its edge";

  MarginOptions narrow = wide_sweep();
  narrow.stop_rad_s = 1.0e3;
  MarginOptions wider = wide_sweep();
  wider.stop_rad_s = 1.0e6;

  const auto near = sensitivity_peaks(diagonal_loop(), narrow);
  const auto far = sensitivity_peaks(diagonal_loop(), wider);

  EXPECT_LT(near.sensitivity_peak, 1.0);
  EXPECT_LT(far.sensitivity_peak, 1.0);
  EXPECT_GT(far.sensitivity_peak, near.sensitivity_peak)
      << "extending the sweep must climb closer to the true supremum of 1";
  EXPECT_NEAR(far.sensitivity_peak, 1.0, 1.0e-5);

  // The complementary peak is strictly positive for both, and for the
  // monotone loop T -> 0 at high frequency so its peak is interior.
  EXPECT_GT(near.complementary_peak, 0.0);
  EXPECT_GT(tutorial.complementary_peak, 0.0);
}

TEST(Sensitivity, RefusesALoopItCannotClose) {
  // Non-square: S = (I+L)^-1 is not defined.
  LinearSystem wide = tutorial_loop();
  wide.b = Eigen::MatrixXd::Zero(3, 2);
  wide.b(2, 0) = 1.0;
  wide.input_names = {"u0", "u1"};
  EXPECT_THROW((void)sensitivity_peaks(wide), std::invalid_argument);

  // Nominally unstable closed loop: S has that pole, and a peak of an unstable
  // S over a finite grid is not a robustness measure.
  LinearSystem hot = tutorial_loop();
  hot.c *= 10.0;
  EXPECT_THROW((void)sensitivity_peaks(hot), std::invalid_argument);
  try {
    (void)sensitivity_peaks(hot);
    FAIL() << "expected a refusal";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("unstable"), std::string::npos) << error.what();
  }

  // Ill-posed: I + D singular.
  LinearSystem ill = tutorial_loop();
  ill.d = Eigen::MatrixXd::Constant(1, 1, -1.0);
  EXPECT_THROW((void)sensitivity_peaks(ill), std::invalid_argument);

  // No inputs at all.
  LinearSystem bare;
  bare.a = Eigen::MatrixXd::Constant(1, 1, -1.0);
  bare.state_names = {"x"};
  EXPECT_THROW((void)sensitivity_peaks(bare), std::invalid_argument);
}

TEST(Sensitivity, ReportsTheSearchedBand) {
  MarginOptions options;
  options.start_rad_s = 0.05;
  options.stop_rad_s = 50.0;
  options.grid_points = 300;
  const auto peaks = sensitivity_peaks(tutorial_loop(), options);

  EXPECT_GE(peaks.searched_from_rad_s, 0.05);
  EXPECT_LE(peaks.searched_to_rad_s, 50.0);
  EXPECT_GE(peaks.grid_points, 300);
  EXPECT_EQ(peaks.sensitivity.size(), peaks.frequencies_rad_s.size());
  EXPECT_EQ(peaks.complementary_sensitivity.size(), peaks.frequencies_rad_s.size());
}

}  // namespace
