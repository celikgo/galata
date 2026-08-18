// SPDX-License-Identifier: Apache-2.0
//
// Frequency response against transfer functions whose value at jw can be
// written down in closed form, and against a deliberately naive computation of
// the same thing.
//
// Two independent kinds of check are needed here and neither substitutes for
// the other:
//
//   * The CLOSED-FORM checks verify that the state-space evaluation computes
//     the transfer function it claims to. They would still pass if the
//     Hessenberg machinery were replaced by anything correct.
//
//   * The AGREES-WITH-NAIVE check verifies the hand-written Hessenberg solver
//     specifically, on matrices too big to write a transfer function for. It is
//     the test that earns the right to ship a hand-rolled linear solver.

#include "galata/analyze/frequency_response.hpp"
#include "galata/units.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <limits>
#include <vector>

namespace {

// The core reports angles in radians (ADR-0003). Tests state their
// expectations in degrees, as the literature does, and convert here — the
// same boundary conversion the report writers make.
double degrees(double radians) {
  return galata::units::radians_to_degrees(radians);
}

using galata::analyze::frequency_response;
using galata::analyze::grid_refined_for_modes;
using galata::analyze::logarithmic_grid;
using galata::analyze::single_loop_response;
using galata::model::LinearSystem;

constexpr double kPi = 3.14159265358979323846;

// A controllable-canonical realisation of  num / den, where den is
// s^n + a_{n-1} s^{n-1} + ... + a_0 given LOWEST ORDER FIRST, and num likewise
// with at most n coefficients. Writing the realisation out here rather than
// reusing production code keeps the test's model independent of the code under
// test.
LinearSystem canonical(const std::vector<double>& numerator_low_first,
                       const std::vector<double>& denominator_low_first) {
  const Eigen::Index n = static_cast<Eigen::Index>(denominator_low_first.size());
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(n, n);
  for (Eigen::Index row = 0; row + 1 < n; ++row) {
    system.a(row, row + 1) = 1.0;
  }
  for (Eigen::Index column = 0; column < n; ++column) {
    system.a(n - 1, column) = -denominator_low_first[static_cast<std::size_t>(column)];
  }
  system.b = Eigen::MatrixXd::Zero(n, 1);
  system.b(n - 1, 0) = 1.0;
  system.c = Eigen::MatrixXd::Zero(1, n);
  for (std::size_t index = 0; index < numerator_low_first.size(); ++index) {
    system.c(0, static_cast<Eigen::Index>(index)) = numerator_low_first[index];
  }
  system.d = Eigen::MatrixXd::Zero(1, 1);
  for (Eigen::Index index = 0; index < n; ++index) {
    system.state_names.push_back("x" + std::to_string(index));
  }
  system.input_names = {"u"};
  system.output_names = {"y"};
  return system;
}

std::complex<double> polynomial(const std::vector<double>& low_first, std::complex<double> s) {
  std::complex<double> value{0.0, 0.0};
  std::complex<double> power{1.0, 0.0};
  for (const double coefficient : low_first) {
    value += coefficient * power;
    power *= s;
  }
  return value;
}

TEST(LogarithmicGrid, EndpointsAreExactAndSpacingIsGeometric) {
  const std::vector<double> grid = logarithmic_grid(0.01, 100.0, 41);
  ASSERT_EQ(grid.size(), 41U);
  // Assigned, not computed: a caller asking for four decades gets exactly the
  // decades it asked for, with no pow(10, log10(x)) round trip on the ends.
  EXPECT_EQ(grid.front(), 0.01);
  EXPECT_EQ(grid.back(), 100.0);

  const double expected_ratio = std::pow(10.0, 4.0 / 40.0);
  for (std::size_t index = 1; index < grid.size(); ++index) {
    EXPECT_NEAR(grid[index] / grid[index - 1], expected_ratio, 1.0e-12) << "at index " << index;
  }
}

TEST(LogarithmicGrid, RejectsAGridThatCannotBeLogarithmic) {
  EXPECT_THROW((void)logarithmic_grid(0.0, 10.0, 10), std::invalid_argument);
  EXPECT_THROW((void)logarithmic_grid(-1.0, 10.0, 10), std::invalid_argument);
  EXPECT_THROW((void)logarithmic_grid(10.0, 1.0, 10), std::invalid_argument);
  EXPECT_THROW((void)logarithmic_grid(1.0, 10.0, 1), std::invalid_argument);
}

TEST(FrequencyResponse, FirstOrderLagMatchesItsClosedForm) {
  // G(s) = 1 / (s + 1),  G(jw) = 1 / (1 + jw).
  const LinearSystem system = canonical({1.0}, {1.0});
  const std::vector<double> grid = logarithmic_grid(0.01, 100.0, 61);
  const auto response = frequency_response(system, grid);

  const std::vector<double> magnitude = response.magnitude();
  const std::vector<double> phase = response.phase_rad();
  for (std::size_t index = 0; index < grid.size(); ++index) {
    const double w = grid[index];
    EXPECT_NEAR(magnitude[index], 1.0 / std::sqrt(1.0 + w * w), 1.0e-14) << "at w = " << w;
    EXPECT_NEAR(degrees(phase[index]), -std::atan(w) * 180.0 / kPi, 1.0e-12) << "at w = " << w;
  }

  // The corner: -3.01 dB and -45 degrees at w = 1, the two numbers every
  // textbook prints.
  const auto corner = frequency_response(system, {1.0});
  EXPECT_NEAR(corner.magnitude_db().front(), -20.0 * std::log10(std::sqrt(2.0)), 1.0e-13);
  EXPECT_NEAR(degrees(corner.phase_rad().front()), -45.0, 1.0e-12);
}

TEST(FrequencyResponse, SecondOrderResonantPeakMatchesItsClosedForm) {
  // G(s) = wn^2 / (s^2 + 2 zeta wn s + wn^2).
  //
  // For zeta < 1/sqrt(2) the magnitude peaks at wr = wn sqrt(1 - 2 zeta^2)
  // with value 1 / (2 zeta sqrt(1 - zeta^2)). Both are standard results
  // obtained by differentiating |G|^2; see Franklin, Powell & Emami-Naeini,
  // "Feedback Control of Dynamic Systems", the second-order frequency response
  // section.
  const double wn = 3.0;
  const double zeta = 0.1;
  const LinearSystem system = canonical({wn * wn}, {wn * wn, 2.0 * zeta * wn});

  const double resonant = wn * std::sqrt(1.0 - 2.0 * zeta * zeta);
  const double peak = 1.0 / (2.0 * zeta * std::sqrt(1.0 - zeta * zeta));

  const auto at_peak = frequency_response(system, {resonant});
  EXPECT_NEAR(at_peak.magnitude().front(), peak, 1.0e-12);

  // Phase is exactly -90 degrees at w = wn, whatever the damping is.
  const auto at_natural = frequency_response(system, {wn});
  EXPECT_NEAR(degrees(at_natural.phase_rad().front()), -90.0, 1.0e-12);

  // And the peak really is the maximum, not merely a point that matches a
  // formula: no grid point exceeds it.
  const auto swept = frequency_response(system, logarithmic_grid(0.1, 30.0, 400));
  for (const double value : swept.magnitude()) {
    EXPECT_LE(value, peak * (1.0 + 1.0e-12));
  }
}

TEST(FrequencyResponse, RationalTransferFunctionWithZerosMatchesItsRatio) {
  // G(s) = (s^2 + 0.5 s + 4) / (s^3 + 2.4 s^2 + 5.1 s + 3), evaluated as a
  // ratio of polynomials at each jw and compared against the state-space
  // evaluation. Zeros matter here: a numerator that is not a constant is where
  // a wrong C row would show up, and the first two tests would not catch it.
  const std::vector<double> numerator{4.0, 0.5, 1.0};
  const std::vector<double> denominator{3.0, 5.1, 2.4};
  std::vector<double> monic_denominator = denominator;
  monic_denominator.push_back(1.0);

  const LinearSystem system = canonical(numerator, denominator);
  const std::vector<double> grid = logarithmic_grid(0.01, 100.0, 121);
  const auto response = frequency_response(system, grid);

  for (std::size_t index = 0; index < grid.size(); ++index) {
    const std::complex<double> s{0.0, grid[index]};
    const std::complex<double> expected =
        polynomial(numerator, s) / polynomial(monic_denominator, s);
    const std::complex<double> actual = response.response[index](0, 0);
    EXPECT_NEAR(std::abs(actual - expected), 0.0, 1.0e-13 * std::abs(expected))
        << "at w = " << grid[index];
  }
}

TEST(FrequencyResponse, DirectFeedthroughAppearsInTheResponse) {
  // G(s) = 1/(s+1) + 2, whose high-frequency limit is D = 2 rather than zero.
  LinearSystem system = canonical({1.0}, {1.0});
  system.d = Eigen::MatrixXd::Constant(1, 1, 2.0);

  const auto response = frequency_response(system, {1.0e-6, 1.0e6});
  EXPECT_NEAR(response.magnitude().front(), 3.0, 1.0e-9);
  EXPECT_NEAR(response.magnitude().back(), 2.0, 1.0e-9);
}

TEST(FrequencyResponse, PhaseIsUnwrappedAcrossTheHalfTurnBoundary) {
  // G(s) = 1 / (s (s+1) (s+2)) runs from -90 to -270 degrees. Wrapped, that
  // would jump to +90 somewhere in the middle and a margin search would find a
  // -180 crossing that is not there.
  const LinearSystem system = canonical({1.0}, {0.0, 2.0, 3.0});
  const std::vector<double> grid = logarithmic_grid(0.01, 100.0, 400);
  const std::vector<double> phase = frequency_response(system, grid).phase_rad();

  // Compared against the closed form, not against the -90 and -270 asymptotes:
  // those are reached only at zero and infinity, and a test that allowed a
  // degree of slack at the ends would be asserting a tolerance rather than the
  // phase.
  for (std::size_t index = 0; index < grid.size(); ++index) {
    const double w = grid[index];
    const double expected = -kPi / 2.0 - std::atan(w) - std::atan(w / 2.0);
    EXPECT_NEAR(phase[index], expected, 1.0e-13) << "at w = " << w;
  }
  for (std::size_t index = 1; index < phase.size(); ++index) {
    EXPECT_LT(phase[index], phase[index - 1]) << "phase must fall monotonically here";
    EXPECT_LT(phase[index - 1] - phase[index], kPi) << "unwrapping missed a branch";
  }
  // The half-turn boundary really is crossed inside this sweep, so the test
  // would fail if unwrapping were removed rather than merely being unexercised.
  EXPECT_LT(phase.back(), -kPi);
  EXPECT_GT(phase.front(), -kPi);
}

TEST(FrequencyResponse, HessenbergSolveAgreesWithADirectlyFormedSolve) {
  // The check that earns the hand-written Hessenberg solver its place.
  //
  // A deliberately awkward 8-state system: unsymmetric, mixed scales, and a
  // pair of lightly damped modes, so the shifted matrix is genuinely
  // ill-conditioned somewhere in the sweep. Reference values come from Eigen's
  // own PartialPivLU on the UNREDUCED matrix — a different algorithm on a
  // different matrix, so agreement is evidence and not a tautology.
  constexpr Eigen::Index n = 8;
  Eigen::MatrixXd a(n, n);
  a << -0.30, 12.00, 0.00, -9.81, 0.00, 0.00, 0.00, 0.00, -0.05, -1.20, 25.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.01, -3.40, -2.10, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, -0.02, 1.90, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      -1.90, -0.02, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, -40.0, 0.00, 0.70, 0.00, 0.00,
      0.00, 0.30, 0.00, 1.00, -0.001;

  Eigen::MatrixXd b(n, 2);
  b << 0.10, 0.00, -0.02, 0.50, -8.00, 1.20, 0.00, 0.00, 0.00, 2.00, 0.00, -1.00, 60.0, 0.00, 0.05,
      0.05;

  Eigen::MatrixXd c(3, n);
  c << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5,
      0.0, 0.0, 2.0, 0.0, 0.0, -1.0;

  LinearSystem system;
  system.a = a;
  system.b = b;
  system.c = c;
  system.d = Eigen::MatrixXd::Zero(3, 2);
  for (Eigen::Index index = 0; index < n; ++index) {
    system.state_names.push_back("x" + std::to_string(index));
  }
  system.input_names = {"u0", "u1"};
  system.output_names = {"y0", "y1", "y2"};

  const std::vector<double> grid = grid_refined_for_modes(a, 1.0e-3, 1.0e3, 200);
  const auto response = frequency_response(system, grid);
  ASSERT_EQ(response.response.size(), grid.size());

  double worst_relative = 0.0;
  double worst_condition = 0.0;
  for (std::size_t index = 0; index < grid.size(); ++index) {
    const std::complex<double> s{0.0, grid[index]};
    Eigen::MatrixXcd shifted =
        s * Eigen::MatrixXcd::Identity(n, n) - a.cast<std::complex<double>>();

    Eigen::JacobiSVD<Eigen::MatrixXcd> svd(shifted);
    worst_condition =
        std::max(worst_condition, svd.singularValues()(0) / svd.singularValues()(n - 1));

    const Eigen::MatrixXcd expected =
        c.cast<std::complex<double>>()
        * shifted.partialPivLu().solve(b.cast<std::complex<double>>());
    const double scale = std::max(expected.cwiseAbs().maxCoeff(), 1.0e-300);
    worst_relative = std::max(worst_relative,
                              (response.response[index] - expected).cwiseAbs().maxCoeff() / scale);
  }

  // The gate is the CONDITIONING of the problem, not a number chosen because
  // the test passed with it. Two backward-stable eliminations of the same
  // system cannot be expected to agree more closely than kappa * eps, and there
  // is no honest reason to demand that they agree less closely either.
  //
  // This grid reaches kappa ~ 1e7 near the pole at -0.001 rad/s, so the bound
  // is around 2e-9; the observed disagreement sits well inside it. A genuine
  // defect in the Hessenberg elimination would not.
  const double bound = worst_condition * std::numeric_limits<double>::epsilon();
  EXPECT_LT(worst_relative, bound)
      << "worst relative disagreement " << worst_relative << " against a conditioning bound of "
      << bound << " (kappa = " << worst_condition << ")";
  // The grid must actually visit an ill-conditioned frequency, or the check
  // above is vacuous.
  EXPECT_GT(worst_condition, 1.0e6);
}

TEST(FrequencyResponse, PivotRatioCollapsesNearAPole) {
  // A mode at 2 rad/s with damping 1e-4. Sweeping onto it should make the
  // shifted matrix nearly singular, and the pivot ratio is what says so.
  const double wn = 2.0;
  const double zeta = 1.0e-4;
  const LinearSystem system = canonical({wn * wn}, {wn * wn, 2.0 * zeta * wn});

  const auto away = frequency_response(system, {0.1});
  const auto onto = frequency_response(system, {wn});
  EXPECT_GT(away.pivot_ratio.front(), 1.0e-2);
  EXPECT_LT(onto.pivot_ratio.front(), 1.0e-3);
  // And the response there is large, which is the thing the ratio is warning
  // about the trailing digits of.
  EXPECT_GT(onto.magnitude().front(), 1.0e3);
}

TEST(FrequencyResponse, RefinementPlacesPointsAcrossALightlyDampedPeak) {
  const double wn = 5.0;
  const double zeta = 0.02;
  const LinearSystem system = canonical({wn * wn}, {wn * wn, 2.0 * zeta * wn});

  const std::vector<double> plain = logarithmic_grid(0.1, 100.0, 30);
  const std::vector<double> refined = grid_refined_for_modes(system.a, 0.1, 100.0, 30);
  EXPECT_GT(refined.size(), plain.size());
  EXPECT_TRUE(std::is_sorted(refined.begin(), refined.end()));

  // The half-power band is roughly wn (1 +/- zeta). A plain 30-point grid over
  // three decades has points a factor 1.26 apart and cannot resolve a band 4%
  // wide; the refined one must put several inside it.
  const auto in_band = [wn, zeta](const std::vector<double>& grid) {
    int count = 0;
    for (const double w : grid) {
      if (w > wn * (1.0 - zeta) && w < wn * (1.0 + zeta)) {
        ++count;
      }
    }
    return count;
  };
  EXPECT_LE(in_band(plain), 1);
  EXPECT_GE(in_band(refined), 3);

  // Refinement must not lose the base grid.
  for (const double w : plain) {
    EXPECT_NE(std::find(refined.begin(), refined.end(), w), refined.end())
        << "base grid point " << w << " was dropped";
  }

  // A well damped system gets no refinement at all: there is no peak to
  // resolve, and inventing points would only make the table longer.
  const LinearSystem damped = canonical({1.0}, {1.0, 2.0});
  EXPECT_EQ(grid_refined_for_modes(damped.a, 0.1, 100.0, 30).size(), plain.size());
}

TEST(FrequencyResponse, SingleLoopSelectsTheNamedInputAndOutput) {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(2, 2);
  system.a << -1.0, 0.0, 0.0, -4.0;
  system.b = Eigen::MatrixXd::Zero(2, 2);
  system.b << 1.0, 0.0, 0.0, 1.0;
  system.state_names = {"x0", "x1"};
  system.input_names = {"aileron", "rudder"};

  const auto loop = single_loop_response(system, 1, 1, {2.0});
  EXPECT_TRUE(loop.is_single_loop());
  ASSERT_EQ(loop.input_names.size(), 1U);
  EXPECT_EQ(loop.input_names.front(), "rudder");
  EXPECT_EQ(loop.output_names.front(), "x1");
  // 1/(s+4) at w = 2.
  EXPECT_NEAR(loop.magnitude().front(), 1.0 / std::sqrt(20.0), 1.0e-14);

  EXPECT_THROW((void)single_loop_response(system, 2, 0, {1.0}), std::out_of_range);
  EXPECT_THROW((void)single_loop_response(system, 0, 5, {1.0}), std::out_of_range);
}

TEST(FrequencyResponse, MimoResponseRefusesTheSingleLoopViews) {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Constant(2, 2, -1.0);
  system.b = Eigen::MatrixXd::Identity(2, 2);
  system.state_names = {"x0", "x1"};
  system.input_names = {"u0", "u1"};

  const auto response = frequency_response(system, {1.0});
  EXPECT_FALSE(response.is_single_loop());
  EXPECT_THROW((void)response.magnitude(), std::invalid_argument);
  EXPECT_THROW((void)response.phase_rad(), std::invalid_argument);
}

TEST(FrequencyResponse, RefusesWhatItCannotEvaluate) {
  LinearSystem no_inputs;
  no_inputs.a = Eigen::MatrixXd::Identity(2, 2) * -1.0;
  no_inputs.state_names = {"x0", "x1"};
  EXPECT_THROW((void)frequency_response(no_inputs, {1.0}), std::invalid_argument);

  const LinearSystem lag = canonical({1.0}, {1.0});
  EXPECT_THROW((void)frequency_response(lag, {}), std::invalid_argument);
  EXPECT_THROW((void)frequency_response(lag, {std::nan("")}), std::invalid_argument);

  // A pure integrator evaluated exactly at DC: the shifted matrix is exactly
  // singular and the honest answer is to say so, not to return a large number.
  const LinearSystem integrator = canonical({1.0}, {0.0});
  EXPECT_THROW((void)frequency_response(integrator, {0.0}), std::domain_error);
}

}  // namespace
