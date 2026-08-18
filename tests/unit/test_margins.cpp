// SPDX-License-Identifier: Apache-2.0
//
// Stability margins against loops whose margins can be written down.
//
// Every reference value in this file is derived here from the transfer
// function, either in closed form or as the root of a stated polynomial solved
// by an independent bisection. None is taken from the implementation, and none
// is a number that happened to make a test pass.
//
// The two workhorses:
//
//   L1(s) = 1 / (s (s+1) (s+2))
//     Phase crossover: -180 deg requires atan(w) + atan(w/2) = 90 deg. Using
//     tan(A+B) = (tanA + tanB) / (1 - tanA tanB), the sum reaches 90 deg when
//     the denominator 1 - w^2/2 vanishes, so w_pc = sqrt(2) EXACTLY.
//     |L1(j sqrt2)| = 1 / (sqrt2 * |1 + j sqrt2| * |2 + j sqrt2|)
//                   = 1 / (sqrt2 * sqrt3 * sqrt6) = 1 / sqrt(36) = 1/6.
//     Gain margin = 6 EXACTLY (15.563 dB).
//
//   L2(s) = 1 / (s (s+1)^2)
//     -90 - 2 atan(w) = -180 gives atan(w) = 45 deg, so w_pc = 1 EXACTLY, and
//     |L2(j)| = 1 / |1 + j|^2 = 1/2. Gain margin = 2 EXACTLY.
//
// Their gain crossovers are algebraic rather than rational: |L| = 1 gives
// u^3 + 5u^2 + 4u - 1 = 0 with u = w^2 for L1, and w^3 + w - 1 = 0 for L2.
// Both are solved below by bisection on the polynomial itself.

#include "galata/analyze/margins.hpp"

#include "galata/analyze/frequency_response.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <functional>
#include <limits>
#include <vector>

namespace {

using galata::analyze::LoopEvaluator;
using galata::analyze::MarginOptions;
using galata::analyze::stability_margins;
using galata::model::LinearSystem;

constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr double kDegreesPerRadian = 57.295779513082320876798154814105;

// Independent root finder for the reference values: 200 bisections on a
// polynomial, nothing to do with the code under test.
double root_of(const std::function<double(double)>& f, double low, double high) {
  const bool low_is_negative = f(low) < 0.0;
  for (int step = 0; step < 200; ++step) {
    const double middle = 0.5 * (low + high);
    if ((f(middle) < 0.0) == low_is_negative) {
      low = middle;
    } else {
      high = middle;
    }
  }
  return 0.5 * (low + high);
}

LinearSystem integrator_chain(const std::vector<double>& poles) {
  // 1 / (s * prod(s + p_i)), as a companion realisation. Built here rather
  // than borrowed from production code so the test's model is its own.
  std::vector<double> denominator{1.0};  // coefficients, lowest order last
  denominator.insert(denominator.begin(), 0.0);
  // Expand prod(s + p) * s by repeated multiplication, lowest order FIRST.
  std::vector<double> coefficients{0.0, 1.0};  // s
  for (const double pole : poles) {
    std::vector<double> next(coefficients.size() + 1, 0.0);
    for (std::size_t index = 0; index < coefficients.size(); ++index) {
      next[index] += coefficients[index] * pole;  // * p
      next[index + 1] += coefficients[index];     // * s
    }
    coefficients = next;
  }
  // coefficients is now monic of degree n; drop the leading 1.
  const Eigen::Index n = static_cast<Eigen::Index>(coefficients.size()) - 1;
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(n, n);
  for (Eigen::Index row = 0; row + 1 < n; ++row) {
    system.a(row, row + 1) = 1.0;
  }
  for (Eigen::Index column = 0; column < n; ++column) {
    system.a(n - 1, column) = -coefficients[static_cast<std::size_t>(column)];
  }
  system.b = Eigen::MatrixXd::Zero(n, 1);
  system.b(n - 1, 0) = 1.0;
  system.c = Eigen::MatrixXd::Zero(1, n);
  system.c(0, 0) = 1.0;
  for (Eigen::Index index = 0; index < n; ++index) {
    system.state_names.push_back("x" + std::to_string(index));
  }
  system.input_names = {"u"};
  system.output_names = {"y"};
  return system;
}

TEST(Margins, ThirdOrderIntegratorChainMatchesItsClosedFormMargins) {
  // L1(s) = 1 / (s (s+1) (s+2)). See the header comment for the derivation.
  const LinearSystem loop = integrator_chain({1.0, 2.0});
  const auto margins = stability_margins(loop, 0, 0);

  ASSERT_TRUE(margins.has_gain_margin);
  EXPECT_NEAR(margins.gain_margin_frequency_rad_s, std::sqrt(2.0), 1.0e-12);
  EXPECT_NEAR(margins.gain_margin, 6.0, 1.0e-11);
  EXPECT_NEAR(margins.gain_margin_db, 20.0 * std::log10(6.0), 1.0e-10);
  EXPECT_EQ(margins.phase_crossings.size(), 1U);

  // Gain crossover: w^2 (1 + w^2) (4 + w^2) = 1, i.e. u^3 + 5u^2 + 4u - 1 = 0.
  const double u = root_of([](double x) { return x * x * x + 5.0 * x * x + 4.0 * x - 1.0; },
                           0.0, 1.0);
  const double expected_crossover = std::sqrt(u);
  const double expected_phase_margin =
      90.0 - std::atan(expected_crossover) * kDegreesPerRadian
      - std::atan(expected_crossover / 2.0) * kDegreesPerRadian;

  ASSERT_TRUE(margins.has_phase_margin);
  EXPECT_NEAR(margins.phase_margin_frequency_rad_s, expected_crossover, 1.0e-12);
  EXPECT_NEAR(margins.phase_margin_deg, expected_phase_margin, 1.0e-10);
  EXPECT_EQ(margins.gain_crossings.size(), 1U);

  ASSERT_TRUE(margins.has_delay_margin);
  EXPECT_NEAR(margins.delay_margin_s,
              expected_phase_margin / kDegreesPerRadian / expected_crossover, 1.0e-11);
}

TEST(Margins, RepeatedPoleChainMatchesItsClosedFormMargins) {
  // L2(s) = 1 / (s (s+1)^2): w_pc = 1 and gain margin = 2, both exact.
  const LinearSystem loop = integrator_chain({1.0, 1.0});
  const auto margins = stability_margins(loop, 0, 0);

  ASSERT_TRUE(margins.has_gain_margin);
  EXPECT_NEAR(margins.gain_margin_frequency_rad_s, 1.0, 1.0e-12);
  EXPECT_NEAR(margins.gain_margin, 2.0, 1.0e-11);

  // Gain crossover: w (1 + w^2) = 1, i.e. w^3 + w - 1 = 0.
  const double crossover = root_of([](double w) { return w * w * w + w - 1.0; }, 0.0, 1.0);
  const double expected_phase_margin = 90.0 - 2.0 * std::atan(crossover) * kDegreesPerRadian;

  ASSERT_TRUE(margins.has_phase_margin);
  EXPECT_NEAR(margins.phase_margin_frequency_rad_s, crossover, 1.0e-12);
  EXPECT_NEAR(margins.phase_margin_deg, expected_phase_margin, 1.0e-10);
}

TEST(Margins, DelayMarginIsTheDelayThatReachesTheCriticalPoint) {
  // The formula check above would pass for a delay margin computed as
  // PM/w_gc even if that expression were the wrong one. This checks the
  // PROPERTY instead: applying exactly the reported delay must put the loop
  // exactly on -1 at the reported frequency.
  const LinearSystem loop = integrator_chain({1.0, 2.0});
  const auto margins = stability_margins(loop, 0, 0);
  ASSERT_TRUE(margins.has_delay_margin);

  const double w = margins.delay_margin_frequency_rad_s;
  const double tau = margins.delay_margin_s;
  const std::complex<double> undelayed{-3.0 * w * w, 2.0 * w - w * w * w};
  const std::complex<double> value =
      (1.0 / undelayed) * std::exp(std::complex<double>(0.0, -w * tau));

  EXPECT_NEAR(std::abs(value + 1.0), 0.0, 1.0e-12)
      << "with the reported delay the loop should sit on the critical point, but L(jw) = "
      << value;
}

TEST(Margins, PureDelayLoopIsMeasuredThroughTheEvaluator) {
  // L(s) = exp(-tau s) / s, which has no state-space realisation at all: a
  // transport delay is not a rational function. This is what the evaluator
  // overload is for.
  //
  //   |L| = 1/w              => w_gc = 1 exactly
  //   arg L = -90 deg - w tau
  //   phase margin = 90 deg - tau * (180/pi)   at w = 1
  //   delay margin = (pi/2 - tau) / 1 seconds
  //   -180 deg requires w tau = pi/2, so w_pc = pi/(2 tau) and GM = w_pc.
  const double tau = 0.5;
  const LoopEvaluator loop = [tau](double w) {
    return std::exp(std::complex<double>(0.0, -w * tau)) / std::complex<double>(0.0, w);
  };

  MarginOptions options;
  options.start_rad_s = 1.0e-2;
  options.stop_rad_s = 20.0;
  options.grid_points = 4000;
  const auto margins = stability_margins(loop, options);

  ASSERT_TRUE(margins.has_phase_margin);
  EXPECT_NEAR(margins.phase_margin_frequency_rad_s, 1.0, 1.0e-12);
  EXPECT_NEAR(margins.phase_margin_deg, 90.0 - tau * kDegreesPerRadian, 1.0e-11);

  ASSERT_TRUE(margins.has_delay_margin);
  EXPECT_NEAR(margins.delay_margin_s, kPi / 2.0 - tau, 1.0e-12);

  ASSERT_TRUE(margins.has_gain_margin);
  EXPECT_NEAR(margins.gain_margin_frequency_rad_s, kPi / (2.0 * tau), 1.0e-11);
  EXPECT_NEAR(margins.gain_margin, kPi / (2.0 * tau), 1.0e-10);

  // The next crossover is at w tau = 5 pi / 2, and the governing margin must be
  // the FIRST one, not whichever the sweep happened to reach last.
  ASSERT_EQ(margins.phase_crossings.size(), 2U);
  EXPECT_NEAR(margins.phase_crossings[1].frequency_rad_s, 5.0 * kPi / (2.0 * tau), 1.0e-10);
  EXPECT_LT(margins.gain_margin, margins.phase_crossings[1].gain_margin);
}

TEST(Margins, GainMarginBelowUnityMeansTheGainMustComeDown) {
  // L1 with a loop gain of 12: at w = sqrt(2), |L| = 12/6 = 2, so the gain
  // must be HALVED. Reported as a ratio of 0.5 and -6.02 dB.
  //
  // An implementation reporting only decibels, or taking absolute values,
  // would call this a healthy 6 dB.
  LinearSystem loop = integrator_chain({1.0, 2.0});
  loop.c *= 12.0;
  const auto margins = stability_margins(loop, 0, 0);

  ASSERT_TRUE(margins.has_gain_margin);
  EXPECT_NEAR(margins.gain_margin_frequency_rad_s, std::sqrt(2.0), 1.0e-12);
  EXPECT_NEAR(margins.gain_margin, 0.5, 1.0e-12);
  EXPECT_NEAR(margins.gain_margin_db, -20.0 * std::log10(2.0), 1.0e-11);
  EXPECT_LT(margins.gain_margin, 1.0);
}

TEST(Margins, UnstableClosedLoopReportsANegativePhaseMargin) {
  // Same loop, gain 12, closed loop unstable. The phase margin must come out
  // negative rather than as a positive number measured from the wrong side.
  LinearSystem loop = integrator_chain({1.0, 2.0});
  loop.c *= 12.0;
  const auto margins = stability_margins(loop, 0, 0);

  ASSERT_TRUE(margins.has_phase_margin);
  EXPECT_LT(margins.phase_margin_deg, 0.0);
  // And a loop that is already unstable has no delay margin: no amount of
  // delay is what is wrong with it.
  EXPECT_FALSE(margins.has_delay_margin);

  // Cross-check against the closed-loop poles: 12/(s(s+1)(s+2)) closed with
  // unit feedback gives s^3 + 3s^2 + 2s + 12, which has right-half-plane roots.
  Eigen::MatrixXd closed = loop.a - loop.b * loop.c;
  Eigen::EigenSolver<Eigen::MatrixXd> solver(closed, false);
  double worst = -1.0e300;
  for (Eigen::Index index = 0; index < solver.eigenvalues().size(); ++index) {
    worst = std::max(worst, solver.eigenvalues()(index).real());
  }
  EXPECT_GT(worst, 0.0) << "the cross-check itself is wrong if this loop is stable";
}

TEST(Margins, AbsentCrossoversAreReportedAsAbsentNotAsZero) {
  // 1/(s+1): the phase never reaches -180, so the gain margin is infinite.
  LinearSystem lag;
  lag.a = Eigen::MatrixXd::Constant(1, 1, -1.0);
  lag.b = Eigen::MatrixXd::Constant(1, 1, 1.0);
  lag.state_names = {"x"};
  lag.input_names = {"u"};

  const auto margins = stability_margins(lag, 0, 0);
  EXPECT_FALSE(margins.has_gain_margin);
  EXPECT_TRUE(std::isinf(margins.gain_margin));
  EXPECT_TRUE(std::isnan(margins.gain_margin_frequency_rad_s));
  EXPECT_TRUE(margins.phase_crossings.empty());
  // |L| does cross unity, at w -> 0, but only below the searched band.
  EXPECT_EQ(margins.searched_from_rad_s, 1.0e-3);

  // 0.1/(s+1): |L| never reaches unity either.
  LinearSystem quiet = lag;
  quiet.c = Eigen::MatrixXd::Constant(1, 1, 0.1);
  const auto quiet_margins = stability_margins(quiet, 0, 0);
  EXPECT_FALSE(quiet_margins.has_phase_margin);
  EXPECT_TRUE(std::isinf(quiet_margins.phase_margin_deg));
  EXPECT_FALSE(quiet_margins.has_delay_margin);
  EXPECT_TRUE(quiet_margins.gain_crossings.empty());
}

TEST(Margins, EveryGainCrossoverIsReportedAndTheWorstOneGoverns) {
  // A lightly damped lag with DC gain 0.5: the magnitude starts below unity,
  // is pushed above it by the resonant peak, then falls away. Two gain
  // crossovers, two phase margins.
  //
  // An implementation returning the first crossing it found would report the
  // margin at the lower one and miss that the loop is fragile at the peak.
  const double wn = 4.0;
  const double zeta = 0.05;
  LinearSystem loop;
  loop.a = Eigen::MatrixXd::Zero(2, 2);
  loop.a << 0.0, 1.0, -wn * wn, -2.0 * zeta * wn;
  loop.b = Eigen::MatrixXd::Zero(2, 1);
  loop.b(1, 0) = 1.0;
  loop.c = Eigen::MatrixXd::Zero(1, 2);
  loop.c(0, 0) = 0.5 * wn * wn;
  loop.state_names = {"x0", "x1"};
  loop.input_names = {"u"};

  const auto margins = stability_margins(loop, 0, 0);
  ASSERT_EQ(margins.gain_crossings.size(), 2U);
  EXPECT_LT(margins.gain_crossings[0].frequency_rad_s, wn);
  EXPECT_GT(margins.gain_crossings[1].frequency_rad_s, wn);

  ASSERT_TRUE(margins.has_phase_margin);
  const double smaller = std::min(std::abs(margins.gain_crossings[0].phase_margin_deg),
                                  std::abs(margins.gain_crossings[1].phase_margin_deg));
  EXPECT_NEAR(std::abs(margins.phase_margin_deg), smaller, 1.0e-12);
  EXPECT_NEAR(margins.phase_margin_frequency_rad_s, margins.gain_crossings[1].frequency_rad_s,
              1.0e-12);

  // The delay margin is the smallest over the crossings, which need not be the
  // one with the smallest phase margin: a larger phase margin at a much higher
  // frequency can buy less time.
  double expected_delay = std::numeric_limits<double>::infinity();
  for (const auto& crossing : margins.gain_crossings) {
    if (crossing.phase_margin_deg > 0.0) {
      expected_delay = std::min(expected_delay, crossing.delay_margin_s);
    }
  }
  ASSERT_TRUE(margins.has_delay_margin);
  EXPECT_NEAR(margins.delay_margin_s, expected_delay, 1.0e-15);

  // Phase approaches -180 asymptotically without reaching it: no gain margin.
  EXPECT_FALSE(margins.has_gain_margin);
}

TEST(Margins, ARefinedGridFindsAResonantCrossingAPlainOneMisses) {
  // The searched band and grid are reported precisely because a crossing can be
  // missed.
  //
  // The band where |L| > 1 has to be made narrow deliberately: light damping
  // sets the HEIGHT of the peak, not the width of the region above unity. With
  // a DC gain of 0.5 the loop is already within a factor of two of unity and
  // crosses over a span of more than a decade. Pushing the DC gain down to 0.01
  // leaves only the tip of the resonance above unity.
  //
  // Solving |(wn^2 - w^2) + j 2 zeta wn w| = 0.01 wn^2 about w = wn gives
  // crossings at wn (1 +/- 0.0046): a band 0.9% wide, which a 40-point grid
  // spanning four decades (points a factor 1.27 apart) steps straight over.
  const double wn = 4.0;
  const double zeta = 0.002;
  LinearSystem loop;
  loop.a = Eigen::MatrixXd::Zero(2, 2);
  loop.a << 0.0, 1.0, -wn * wn, -2.0 * zeta * wn;
  loop.b = Eigen::MatrixXd::Zero(2, 1);
  loop.b(1, 0) = 1.0;
  loop.c = Eigen::MatrixXd::Zero(1, 2);
  loop.c(0, 0) = 0.01 * wn * wn;
  loop.state_names = {"x0", "x1"};
  loop.input_names = {"u"};

  MarginOptions coarse;
  coarse.frequencies = galata::analyze::logarithmic_grid(1.0e-2, 1.0e2, 40);
  EXPECT_TRUE(stability_margins(loop, 0, 0, coarse).gain_crossings.empty());

  // The default path refines around the system's own modes and finds both.
  const auto refined = stability_margins(loop, 0, 0);
  ASSERT_EQ(refined.gain_crossings.size(), 2U);
  EXPECT_NEAR(refined.gain_crossings[0].frequency_rad_s, wn * (1.0 - 0.00458), 1.0e-3);
  EXPECT_NEAR(refined.gain_crossings[1].frequency_rad_s, wn * (1.0 + 0.00458), 1.0e-3);
}

TEST(Margins, RefusesWhatItCannotSearch) {
  MarginOptions bad_iterations;
  bad_iterations.bisection_iterations = 0;
  EXPECT_THROW((void)stability_margins([](double) { return std::complex<double>(1.0, 0.0); },
                                       bad_iterations),
               std::invalid_argument);

  MarginOptions single_point;
  single_point.frequencies = {1.0};
  EXPECT_THROW((void)stability_margins([](double) { return std::complex<double>(1.0, 0.0); },
                                       single_point),
               std::invalid_argument);

  EXPECT_THROW((void)stability_margins(galata::analyze::LoopEvaluator{}), std::invalid_argument);
}

}  // namespace
