// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: the classical margins guaranteed by M_S and M_T, against the
// textbook that states them, and against the loops they are supposed to bound.
//
// Reference:
//   S. Skogestad and I. Postlethwaite, "Multivariable Feedback Control:
//   Analysis and Design", 2nd ed., Wiley, 2005, Section 2.4.3, equations
//   (2.47), (2.48) and (2.50), printed pages 35-37. Author-hosted PDF at
//   https://folk.ntnu.no/skoge/book/ps/bookall.pdf
//
// Reference values and the rights position are in
// reference/skogestad2005_sensitivity_bounds.csv; ADR-0007 covers why scalar
// results from a copyrighted textbook may be committed here.
//
// THREE KINDS OF CHECK:
//
//   * Against the book's PRINTED worked values — M_S = 2 guarantees GM >= 2 and
//     PM >= 29.0 degrees.
//
//   * Against the book's IDENTITIES, which are exact and involve no printed
//     precision at all: equation (2.50) says the magnitudes of S and T are
//     equal at the gain crossover and both equal 1/(2 sin(PM/2)), and the text
//     says the shortest distance from the Nyquist curve to -1 is 1/M_S.
//
//   * Against the INEQUALITIES DOING THEIR JOB. A bound that is merely computed
//     correctly is worth little; what matters is that a real loop's actual
//     margins are at least as good as the bound predicts. That is checked on
//     several loops, and it is the check that would catch a bound applied with
//     M_S and M_T the wrong way round.

#include "galata/analyze/frequency_response.hpp"
#include "galata/analyze/margins.hpp"
#include "galata/analyze/sensitivity.hpp"
#include "galata/units.hpp"

#include "reference_table.hpp"
#include "validation_config.hpp"
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <map>
#include <string>

namespace {

using galata::analyze::guaranteed_margins;
using galata::analyze::MarginOptions;
using galata::analyze::sensitivity_peaks;
using galata::analyze::stability_margins;
using galata::model::LinearSystem;
using galata::testing::load_reference;
using galata::testing::printed_precision_tolerance;

double degrees(double radians) {
  return galata::units::radians_to_degrees(radians);
}

struct Published {
  double value;
  int significant_figures;
  double units_in_last_place;
  std::string location;

  [[nodiscard]] double tolerance() const {
    return printed_precision_tolerance(value, significant_figures, units_in_last_place);
  }
};

std::map<std::string, Published> published_values() {
  const auto table =
      load_reference(GALATA_VALIDATION_REFERENCE_DIR, "skogestad2005_sensitivity_bounds.csv");
  std::map<std::string, Published> values;
  for (std::size_t row = 0; row < table.rows.size(); ++row) {
    Published entry{};
    entry.value = table.at(row, "value");
    entry.significant_figures = static_cast<int>(table.at(row, "significant_figures"));
    entry.units_in_last_place = table.at(row, "units_in_last_place");
    entry.location = table.text(row, "location");
    values.emplace(table.text(row, "quantity"), entry);
  }
  return values;
}

// A single-loop system with a deliberately chosen sensitivity peak is not
// something one can construct directly, so the bounds are exercised through a
// SensitivityPeaks value assembled by hand for the formula checks, and through
// real loops for the inequality checks.
galata::analyze::SensitivityPeaks peaks_with(double sensitivity, double complementary) {
  galata::analyze::SensitivityPeaks peaks{};
  peaks.sensitivity_peak = sensitivity;
  peaks.complementary_peak = complementary;
  peaks.is_single_loop = true;
  return peaks;
}

// k / (s (s+1) (s+2)).
LinearSystem chain(double gain) {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(3, 3);
  system.a << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -2.0, -3.0;
  system.b = Eigen::MatrixXd::Zero(3, 1);
  system.b(2, 0) = 1.0;
  system.c = Eigen::MatrixXd::Zero(1, 3);
  system.c(0, 0) = gain;
  system.state_names = {"x0", "x1", "x2"};
  system.input_names = {"u"};
  system.output_names = {"y"};
  return system;
}

MarginOptions wide_sweep() {
  MarginOptions options;
  options.start_rad_s = 1.0e-3;
  options.stop_rad_s = 1.0e4;
  options.grid_points = 6000;
  return options;
}

TEST(SkogestadSensitivityBounds, WorkedValuesMatchTheBook) {
  const auto published = published_values();

  // "For example, with M_S = 2 we are guaranteed GM >= 2 and PM >= 29.0 degrees."
  const auto from_sensitivity = guaranteed_margins(peaks_with(2.0, 1.0));
  ASSERT_TRUE(from_sensitivity.applies);
  ASSERT_TRUE(from_sensitivity.valid);
  EXPECT_NEAR(from_sensitivity.gain_margin_from_sensitivity,
              published.at("gain_margin_at_ms_2").value,
              published.at("gain_margin_at_ms_2").tolerance())
      << published.at("gain_margin_at_ms_2").location;
  EXPECT_NEAR(degrees(from_sensitivity.phase_margin_from_sensitivity_rad),
              published.at("phase_margin_at_ms_2").value,
              published.at("phase_margin_at_ms_2").tolerance())
      << published.at("phase_margin_at_ms_2").location;

  // "and specifically with M_T = 2 we have GM >= 1.5 and PM >= 29.0 degrees."
  const auto from_complementary = guaranteed_margins(peaks_with(2.0, 2.0));
  EXPECT_NEAR(from_complementary.gain_margin_from_complementary,
              published.at("gain_margin_at_mt_2").value,
              published.at("gain_margin_at_mt_2").tolerance())
      << published.at("gain_margin_at_mt_2").location;
  EXPECT_NEAR(degrees(from_complementary.phase_margin_from_complementary_rad),
              published.at("phase_margin_at_mt_2").value,
              published.at("phase_margin_at_mt_2").tolerance())
      << published.at("phase_margin_at_mt_2").location;

  // The two gain-margin bounds have DIFFERENT functional forms — M_S/(M_S-1)
  // against 1 + 1/M_T — and at M_S = M_T = 2 they give different answers, 2
  // and 1.5. An implementation that used one formula for both would pass every
  // other check in this file.
  EXPECT_NE(from_complementary.gain_margin_from_sensitivity,
            from_complementary.gain_margin_from_complementary);
}

TEST(SkogestadSensitivityBounds, TheBoundsActuallyBoundRealLoops) {
  // The check that matters. For each loop, the ACTUAL gain and phase margins
  // must be at least as good as M_S and M_T promise.
  for (const double gain : {0.5, 1.0, 2.0, 4.0}) {
    const LinearSystem loop = chain(gain);
    const auto peaks = sensitivity_peaks(loop, wide_sweep());
    const auto bounds = guaranteed_margins(peaks);
    ASSERT_TRUE(bounds.applies) << "gain " << gain;
    ASSERT_TRUE(bounds.valid) << "gain " << gain << ", M_S = " << peaks.sensitivity_peak;

    const auto actual = stability_margins(loop, 0, 0, wide_sweep());
    ASSERT_TRUE(actual.has_gain_margin) << "gain " << gain;
    ASSERT_TRUE(actual.has_phase_margin) << "gain " << gain;

    EXPECT_GE(actual.gain_margin, bounds.gain_margin_from_sensitivity * (1.0 - 1.0e-9))
        << "gain " << gain << ": actual GM " << actual.gain_margin << " below the (2.47) bound "
        << bounds.gain_margin_from_sensitivity << " from M_S = " << peaks.sensitivity_peak;
    EXPECT_GE(actual.gain_margin, bounds.gain_margin_from_complementary * (1.0 - 1.0e-9))
        << "gain " << gain
        << ": actual GM below the (2.48) bound from M_T = " << peaks.complementary_peak;
    EXPECT_GE(actual.phase_margin_rad, bounds.phase_margin_from_sensitivity_rad * (1.0 - 1.0e-9))
        << "gain " << gain << ": actual PM " << degrees(actual.phase_margin_rad)
        << " deg below the (2.47) bound " << degrees(bounds.phase_margin_from_sensitivity_rad)
        << " deg";
    EXPECT_GE(actual.phase_margin_rad, bounds.phase_margin_from_complementary_rad * (1.0 - 1.0e-9))
        << "gain " << gain << ": actual PM below the (2.48) bound";
  }
}

TEST(SkogestadSensitivityBounds, MsIsTheReciprocalOfTheDistanceToTheCriticalPoint) {
  // Printed page 36, verbatim: "The smallest distance between L(j omega) and -1
  // is M_S^(-1)". |1 + L| IS that distance, and 1/|1+L| is |S|, so this is an
  // identity rather than an approximation — and it is the reason M_S is a
  // robustness measure at all.
  const LinearSystem loop = chain(1.0);
  const auto peaks = sensitivity_peaks(loop, wide_sweep());
  const auto response = galata::analyze::frequency_response(loop, peaks.frequencies_rad_s);

  double shortest = std::numeric_limits<double>::infinity();
  for (const Eigen::MatrixXcd& value : response.response) {
    shortest = std::min(shortest, std::abs(1.0 + value(0, 0)));
  }
  // Compared on the same grid, so the two are the same maximum seen from two
  // sides rather than two different searches.
  double peak_on_grid = 0.0;
  for (const double value : peaks.sensitivity) {
    peak_on_grid = std::max(peak_on_grid, value);
  }
  EXPECT_NEAR(shortest, 1.0 / peak_on_grid, 1.0e-12 * shortest);
}

TEST(SkogestadSensitivityBounds, SensitivityAndComplementaryAgreeAtTheGainCrossover) {
  // Equation (2.50), printed page 36, verbatim:
  //   |S(j w_c)| = |T(j w_c)| = 1 / (2 sin(PM/2))
  // at the gain crossover, with PM in radians. Three quantities that must all
  // coincide, computed by three different parts of galata.
  const LinearSystem loop = chain(1.0);
  const auto margins = stability_margins(loop, 0, 0, wide_sweep());
  ASSERT_TRUE(margins.has_phase_margin);

  const double crossover = margins.phase_margin_frequency_rad_s;
  const std::complex<double> l =
      galata::analyze::frequency_response(loop, {crossover}).response.front()(0, 0);

  // |L| = 1 there, by definition of a gain crossover.
  EXPECT_NEAR(std::abs(l), 1.0, 1.0e-11);

  const double sensitivity = std::abs(1.0 / (1.0 + l));
  const double complementary = std::abs(l / (1.0 + l));
  const double from_phase_margin = 1.0 / (2.0 * std::sin(margins.phase_margin_rad / 2.0));

  EXPECT_NEAR(sensitivity, complementary, 1.0e-11);
  EXPECT_NEAR(sensitivity, from_phase_margin, 1.0e-10);
}

TEST(SkogestadSensitivityBounds, ThePeaksDifferByAtMostOne) {
  // Printed pages 35-36, verbatim: "Since S + T = 1 ... it follows that at any
  // frequency | |S| - |T| | <= |S + T| = 1 so M_S and M_T differ at most by 1."
  // The MIMO analogue is equation (6.3), p. 222.
  for (const double gain : {0.5, 1.0, 2.0, 4.0}) {
    const auto peaks = sensitivity_peaks(chain(gain), wide_sweep());
    EXPECT_LE(std::abs(peaks.sensitivity_peak - peaks.complementary_peak), 1.0 + 1.0e-9)
        << "gain " << gain << ": M_S = " << peaks.sensitivity_peak
        << ", M_T = " << peaks.complementary_peak;
  }
}

TEST(SkogestadSensitivityBounds, TheBoundsAreRefusedForAMimoLoop) {
  // The source's own scope: (2.47) and (2.48) are stated for SISO and the book
  // never restates them for MIMO. Its spinning-satellite example shows exactly
  // why — excellent margins one loop at a time, destabilised by small
  // simultaneous input gain errors.
  LinearSystem mimo;
  mimo.a = Eigen::MatrixXd::Zero(2, 2);
  mimo.a << -1.0, 0.0, 0.0, -3.0;
  mimo.b = Eigen::MatrixXd::Zero(2, 2);
  mimo.b << 2.0, 0.0, 0.0, 5.0;
  mimo.state_names = {"x0", "x1"};
  mimo.input_names = {"u0", "u1"};

  const auto peaks = sensitivity_peaks(mimo, wide_sweep());
  EXPECT_FALSE(peaks.is_single_loop);
  const auto bounds = guaranteed_margins(peaks);
  EXPECT_FALSE(bounds.applies)
      << "these bounds must not be offered for a multi-loop system on this source's authority";
  EXPECT_FALSE(bounds.valid);
}

TEST(SkogestadSensitivityBounds, OutOfDomainPeaksAreRefusedRatherThanComputed) {
  // The book's Remark on p. 37 notes that M_S must exceed 1 whenever a -180
  // degree crossing exists. M_S <= 1 therefore means the grid missed the peak,
  // and M_S/(M_S - 1) would be divergent or negative — a number that looks like
  // a margin and is not one.
  EXPECT_FALSE(guaranteed_margins(peaks_with(1.0, 1.0)).valid);
  EXPECT_FALSE(guaranteed_margins(peaks_with(0.9, 1.0)).valid);
  // A peak below 0.5 puts 1/(2M) outside the domain of arcsin.
  EXPECT_FALSE(guaranteed_margins(peaks_with(2.0, 0.4)).valid);
  EXPECT_TRUE(guaranteed_margins(peaks_with(1.5, 1.5)).valid);
}

}  // namespace
