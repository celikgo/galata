// SPDX-License-Identifier: Apache-2.0
// Closed-form rational norms and diagonal singular values, independent of the
// Hamiltonian algorithm (Boyd/Balakrishnan/Kabamba 1989, Theorem 1).
#include "galata/analyze/hinfinity.hpp"
#include "galata/analyze/singular_values.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>

namespace {
galata::model::LinearSystem first_order(double pole, double gain, double direct = 0) {
  galata::model::LinearSystem plant;
  plant.a = Eigen::MatrixXd::Constant(1, 1, -pole);
  plant.b = Eigen::MatrixXd::Ones(1, 1);
  plant.c = Eigen::MatrixXd::Constant(1, 1, gain);
  plant.d = Eigen::MatrixXd::Constant(1, 1, direct);
  plant.state_names = {"x"};
  plant.input_names = {"u"};
  plant.output_names = {"y"};
  return plant;
}

void encloses(const galata::analyze::HinfinityNorm& norm, double exact) {
  EXPECT_TRUE(norm.numerically_reliable) << norm.diagnostic;
  EXPECT_TRUE(norm.tolerance_met) << norm.diagnostic;
  EXPECT_LE(norm.lower_bound, exact);
  EXPECT_GE(norm.upper_bound, exact);
  EXPECT_LE(norm.upper_bound - norm.lower_bound, 1e-12 + 1e-6 * norm.upper_bound);
}

TEST(Hinfinity, FirstOrderIncludesTheDcMaximumAndRepeatsExactly) {
  const auto plant = first_order(2, 3);
  const auto norm = galata::analyze::hinfinity_norm(plant);
  encloses(norm, 1.5);
  EXPECT_NEAR(norm.dc_gain, 1.5, 1e-14);
  EXPECT_EQ(norm.feedthrough_gain, 0);
  const auto repeated = galata::analyze::hinfinity_norm(plant);
  EXPECT_EQ(norm.lower_bound, repeated.lower_bound);
  EXPECT_EQ(norm.upper_bound, repeated.upper_bound);
  EXPECT_EQ(norm.hamiltonian_evaluations, repeated.hamiltonian_evaluations);
}

TEST(Hinfinity, NarrowResonanceMissedByAFrequencyGridIsBracketed) {
  constexpr double wn = 2.3;
  constexpr double zeta = 0.001;
  auto plant = first_order(1, 1);
  plant.a.resize(2, 2);
  plant.a << 0, 1, -wn * wn, -2 * zeta * wn;
  plant.b = Eigen::Vector2d(0, wn * wn);
  plant.c = Eigen::RowVector2d(1, 0);
  plant.state_names = {"position", "velocity"};
  const double exact = 1.0 / (2 * zeta * std::sqrt(1 - zeta * zeta));
  const auto grid =
      galata::analyze::singular_values(plant, galata::analyze::logarithmic_grid(0.01, 100, 12));
  EXPECT_LT(grid.peak_gain, exact / 10);
  encloses(galata::analyze::hinfinity_norm(plant), exact);
}

TEST(Hinfinity, IncludesNonzeroFeedthroughAndTheInfiniteFrequencyLimit) {
  encloses(galata::analyze::hinfinity_norm(first_order(1, 1, 1)), 2);
  // s/(s+1): norm 1 is approached only as frequency tends to infinity.
  const auto highpass = galata::analyze::hinfinity_norm(first_order(1, -1, 1));
  encloses(highpass, 1);
  EXPECT_EQ(highpass.dc_gain, 0);
  EXPECT_EQ(highpass.feedthrough_gain, 1);
  encloses(galata::analyze::hinfinity_norm(first_order(1, 0, 2)), 2);
}

TEST(Hinfinity, DiagonalAndRectangularMimoMatchAnalyticSingularValues) {
  galata::model::LinearSystem plant;
  plant.a = Eigen::Matrix2d::Zero();
  plant.a.diagonal() << -1, -2;
  plant.b = Eigen::Matrix2d::Identity();
  plant.c = Eigen::MatrixXd::Zero(3, 2);
  plant.c(0, 0) = 2;
  plant.c(1, 1) = 6;
  plant.d = Eigen::MatrixXd::Zero(3, 2);
  plant.d(0, 0) = 0.5;
  plant.state_names = {"x1", "x2"};
  plant.input_names = {"u1", "u2"};
  plant.output_names = {"y1", "y2", "zero"};
  // max(|.5+2/(s+1)|, |6/(s+2)|) is exactly 3 at DC.
  encloses(galata::analyze::hinfinity_norm(plant), 3);
}

TEST(Hinfinity, ZeroTransferReturnsAnAbsoluteToleranceBound) {
  const auto norm = galata::analyze::hinfinity_norm(first_order(1, 0));
  encloses(norm, 0);
  EXPECT_EQ(norm.lower_bound, 0);
}

TEST(Hinfinity, RejectsUnstableHiddenModesNonfiniteInputAndUnresolvedBudgets) {
  EXPECT_THROW((void)galata::analyze::hinfinity_norm(first_order(-1, 1)), std::invalid_argument);
  EXPECT_THROW((void)galata::analyze::hinfinity_norm(first_order(-1, 0)), std::invalid_argument);
  EXPECT_THROW((void)galata::analyze::hinfinity_norm(first_order(0, 1)), std::invalid_argument);
  EXPECT_THROW((void)galata::analyze::hinfinity_norm(
                   first_order(1, std::numeric_limits<double>::infinity())),
               std::invalid_argument);
  galata::analyze::HinfinityOptions bad;
  bad.relative_tolerance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)galata::analyze::hinfinity_norm(first_order(1, 1), bad),
               std::invalid_argument);
  bad = {};
  bad.bracket_expansions = 0;
  EXPECT_THROW((void)galata::analyze::hinfinity_norm(first_order(1, 1), bad),
               std::invalid_argument);
  bad = {};
  bad.bisection_iterations = 1;
  const auto incomplete = galata::analyze::hinfinity_norm(first_order(1, 1), bad);
  EXPECT_FALSE(incomplete.tolerance_met);
  EXPECT_GE(incomplete.upper_bound, 1);
}

TEST(RobustBounds, ScalarSensitivityAndComplementaryPeaksIncludeFeedthroughLimits) {
  const auto bounds = galata::analyze::sensitivity_norm_bounds(first_order(1, 2, 0.5));
  // S=(s+1)/(1.5s+3.5), T=(.5s+2.5)/(1.5s+3.5).
  // S peaks at infinity (2/3), T at DC (5/7).
  EXPECT_TRUE(bounds.internally_stable);
  encloses(bounds.sensitivity, 2.0 / 3.0);
  encloses(bounds.complementary, 5.0 / 7.0);
}

TEST(RobustBounds, NonDiagonalMimoChannelsPreserveTheNormUnderOrthogonalCoordinates) {
  galata::model::LinearSystem loop;
  Eigen::Matrix2d rotation;
  const double root = 1.0 / std::sqrt(2.0);
  rotation << root, -root, root, root;
  loop.a = Eigen::Matrix2d::Zero();
  loop.a.diagonal() << -1, -2;
  loop.b = rotation.transpose();
  loop.c = rotation * Eigen::Vector2d(2, 6).asDiagonal();
  loop.d = rotation * Eigen::Vector2d(0, 0.5).asDiagonal() * rotation.transpose();
  loop.state_names = {"x1", "x2"};
  loop.input_names = {"u1", "u2"};
  loop.output_names = {"y1", "y2"};
  const auto bounds = galata::analyze::sensitivity_norm_bounds(loop);
  encloses(bounds.sensitivity, 1);
  encloses(bounds.complementary, 7.0 / 9.0);
  EXPECT_THROW((void)galata::analyze::disk_margin_bounds(loop), std::invalid_argument);
}

TEST(RobustBounds, DiskEndpointsInvertTheNormInTheConservativeDirection) {
  const auto loop = first_order(1, 2);
  for (const auto [skew, exact] :
       {std::pair{0.0, 2.0}, std::pair{1.0, 1.0}, std::pair{-1.0, 1.5}}) {
    const auto disk = galata::analyze::disk_margin_bounds(loop, skew);
    EXPECT_TRUE(disk.internally_stable);
    EXPECT_TRUE(disk.shifted_sensitivity.tolerance_met) << disk.shifted_sensitivity.diagnostic;
    EXPECT_LE(disk.alpha_lower, exact);
    EXPECT_GE(disk.alpha_upper, exact);
    EXPECT_DOUBLE_EQ(disk.alpha_lower, 1.0 / disk.shifted_sensitivity.upper_bound);
    EXPECT_DOUBLE_EQ(disk.alpha_upper, 1.0 / disk.shifted_sensitivity.lower_bound);
  }
}

TEST(RobustBounds, IdenticallyZeroShiftedSensitivityHasAnUnboundedDiskMargin) {
  // L=1 implies S=1/2, so the balanced shifted sensitivity S-1/2 is zero.
  const auto disk = galata::analyze::disk_margin_bounds(first_order(1, 0, 1));
  EXPECT_TRUE(disk.internally_stable);
  EXPECT_TRUE(disk.shifted_sensitivity.numerically_reliable);
  EXPECT_TRUE(disk.shifted_sensitivity.tolerance_met);
  EXPECT_EQ(disk.shifted_sensitivity.upper_bound, 0);
  EXPECT_EQ(disk.alpha_lower, std::numeric_limits<double>::infinity());
  EXPECT_EQ(disk.alpha_upper, std::numeric_limits<double>::infinity());
}

TEST(RobustBounds, RefusesUnstableNominalLoopsAndSingularAlgebraicFeedback) {
  EXPECT_THROW((void)galata::analyze::sensitivity_norm_bounds(first_order(1, -2)),
               std::invalid_argument);
  EXPECT_THROW((void)galata::analyze::disk_margin_bounds(first_order(1, -2)),
               std::invalid_argument);
  EXPECT_THROW((void)galata::analyze::sensitivity_norm_bounds(first_order(1, 1, -1)),
               std::invalid_argument);
  EXPECT_THROW((void)galata::analyze::disk_margin_bounds(first_order(1, 1),
                                                         std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}
}  // namespace
