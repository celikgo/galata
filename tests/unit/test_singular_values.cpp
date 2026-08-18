// SPDX-License-Identifier: Apache-2.0
//
// Singular values against matrices whose SVD can be written down.
//
// The reference here is algebra, not a document. For a 2x2 matrix the singular
// values are the square roots of the eigenvalues of A^T A, which for the cases
// below come out in closed form, so the comparison is exact to rounding rather
// than bounded by anything's printed precision.

#include "galata/analyze/singular_values.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using galata::analyze::frequency_response;
using galata::analyze::logarithmic_grid;
using galata::analyze::singular_values;
using galata::model::LinearSystem;

// A system whose transfer matrix is the CONSTANT `gain` at every frequency:
// one state that nothing reaches and nothing reads, with the whole response in
// the direct feedthrough.
LinearSystem constant_gain(const Eigen::MatrixXd& gain) {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Constant(1, 1, -1.0);
  system.b = Eigen::MatrixXd::Zero(1, gain.cols());
  system.c = Eigen::MatrixXd::Zero(gain.rows(), 1);
  system.d = gain;
  system.state_names = {"x"};
  for (Eigen::Index index = 0; index < gain.cols(); ++index) {
    system.input_names.push_back("u" + std::to_string(index));
  }
  for (Eigen::Index index = 0; index < gain.rows(); ++index) {
    system.output_names.push_back("y" + std::to_string(index));
  }
  return system;
}

// diag(1/(s+1), 2/(s+3)).
LinearSystem diagonal_pair() {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(2, 2);
  system.a << -1.0, 0.0, 0.0, -3.0;
  system.b = Eigen::MatrixXd::Zero(2, 2);
  system.b << 1.0, 0.0, 0.0, 2.0;
  system.state_names = {"x0", "x1"};
  system.input_names = {"u0", "u1"};
  return system;
}

TEST(SingularValues, SisoSingularValueIsTheMagnitude) {
  // A 1x1 matrix has exactly one singular value, equal to its magnitude. This
  // is the check that ties the MIMO path back to the already-validated SISO
  // frequency response.
  LinearSystem lag;
  lag.a = Eigen::MatrixXd::Constant(1, 1, -2.0);
  lag.b = Eigen::MatrixXd::Constant(1, 1, 1.0);
  lag.state_names = {"x"};
  lag.input_names = {"u"};

  const auto response = frequency_response(lag, logarithmic_grid(0.01, 100.0, 101));
  const auto sigma = singular_values(response);
  const std::vector<double> magnitude = response.magnitude();

  ASSERT_EQ(sigma.channel_count(), 1U);
  for (std::size_t index = 0; index < magnitude.size(); ++index) {
    EXPECT_NEAR(sigma.singular_values[index][0], magnitude[index], 1.0e-15 * magnitude[index]);
  }
  // And the condition number of a 1x1 matrix is exactly one, everywhere.
  for (const double value : sigma.condition_number()) {
    EXPECT_NEAR(value, 1.0, 1.0e-14);
  }
}

TEST(SingularValues, TriangularGainHasTheGoldenRatioSingularValues) {
  // G = [[1, 1], [0, 1]]. G^T G = [[1, 1], [1, 2]], whose eigenvalues solve
  // lambda^2 - 3 lambda + 1 = 0, giving lambda = (3 +/- sqrt5)/2. So
  //
  //   sigma_max = sqrt((3 + sqrt5)/2) = (1 + sqrt5)/2, the golden ratio
  //   sigma_min = sqrt((3 - sqrt5)/2) = (sqrt5 - 1)/2, its reciprocal
  //
  // and their product is 1, which is |det G|.
  //
  // This is the case that makes the point of the whole file: EVERY ELEMENT of
  // G has magnitude at most 1, and yet the system's largest gain is 1.618.
  // Reading the elements one at a time would have said the gain was 1.
  Eigen::MatrixXd gain(2, 2);
  gain << 1.0, 1.0, 0.0, 1.0;

  const auto sigma = singular_values(constant_gain(gain), {1.0, 10.0});
  ASSERT_EQ(sigma.channel_count(), 2U);

  const double golden = (1.0 + std::sqrt(5.0)) / 2.0;
  for (const std::vector<double>& row : sigma.singular_values) {
    EXPECT_NEAR(row[0], golden, 1.0e-14);
    EXPECT_NEAR(row[1], 1.0 / golden, 1.0e-14);
    EXPECT_NEAR(row[0] * row[1], std::abs(gain.determinant()), 1.0e-14);
    EXPECT_GT(row[0], gain.cwiseAbs().maxCoeff())
        << "the largest gain must exceed the largest element — that is the point";
  }
  EXPECT_NEAR(sigma.peak_gain, golden, 1.0e-14);
}

TEST(SingularValues, SingularValuesAreOrderedAndCountedByTheSmallerDimension) {
  // A non-square system has min(outputs, inputs) singular values, not one per
  // channel of the larger side.
  Eigen::MatrixXd wide(2, 3);
  wide << 3.0, 0.0, 0.0, 0.0, 4.0, 0.0;
  const auto sigma = singular_values(constant_gain(wide), {1.0});
  ASSERT_EQ(sigma.channel_count(), 2U);
  EXPECT_NEAR(sigma.singular_values[0][0], 4.0, 1.0e-14);
  EXPECT_NEAR(sigma.singular_values[0][1], 3.0, 1.0e-14);

  Eigen::MatrixXd tall(3, 2);
  tall << 3.0, 0.0, 0.0, 4.0, 0.0, 0.0;
  EXPECT_EQ(singular_values(constant_gain(tall), {1.0}).channel_count(), 2U);
}

TEST(SingularValues, DiagonalSystemHasTheElementMagnitudesAsItsSingularValues) {
  // For a DIAGONAL transfer matrix — and only then — the singular values are
  // the element magnitudes, sorted. diag(1/(s+1), 2/(s+3)).
  const std::vector<double> grid = logarithmic_grid(0.01, 100.0, 61);
  const auto sigma = singular_values(diagonal_pair(), grid);

  for (std::size_t index = 0; index < grid.size(); ++index) {
    const double w = grid[index];
    const double first = 1.0 / std::sqrt(1.0 + w * w);
    const double second = 2.0 / std::sqrt(9.0 + w * w);
    const double larger = std::max(first, second);
    const double smaller = std::min(first, second);
    EXPECT_NEAR(sigma.singular_values[index][0], larger, 1.0e-14) << "at w = " << w;
    EXPECT_NEAR(sigma.singular_values[index][1], smaller, 1.0e-14) << "at w = " << w;
  }
}

TEST(SingularValues, RankDeficiencyIsReportedAsAnInfiniteConditionNumber) {
  // A gain matrix of rank 1: there is an input direction the system does not
  // respond to at all. That is a property of the system, not an error, and the
  // condition number says so rather than throwing or returning a large finite
  // number that looks like a measurement.
  Eigen::MatrixXd singular(2, 2);
  singular << 1.0, 2.0, 2.0, 4.0;
  const auto sigma = singular_values(constant_gain(singular), {1.0});

  EXPECT_NEAR(sigma.singular_values[0][0], std::sqrt(25.0), 1.0e-13);
  EXPECT_NEAR(sigma.singular_values[0][1], 0.0, 1.0e-14);
  EXPECT_TRUE(std::isinf(sigma.condition_number().front()));
}

TEST(SingularValues, ThePeakIsALowerBoundOnTheTrueNorm) {
  // 1/(s+1) has an H-infinity norm of exactly 1, attained at w = 0 — which no
  // logarithmic grid contains. The reported peak must therefore be BELOW 1,
  // and approach it as the grid reaches lower.
  //
  // This is the caveat in the header made into a test: a grid maximum
  // understates the true supremum, and the direction of that error is knowable.
  LinearSystem lag;
  lag.a = Eigen::MatrixXd::Constant(1, 1, -1.0);
  lag.b = Eigen::MatrixXd::Constant(1, 1, 1.0);
  lag.state_names = {"x"};
  lag.input_names = {"u"};

  const auto narrow = singular_values(lag, logarithmic_grid(0.1, 100.0, 200));
  const auto wide = singular_values(lag, logarithmic_grid(1.0e-4, 100.0, 200));

  EXPECT_LT(narrow.peak_gain, 1.0);
  EXPECT_LT(wide.peak_gain, 1.0);
  EXPECT_GT(wide.peak_gain, narrow.peak_gain) << "a wider sweep must get closer to the norm";
  EXPECT_NEAR(wide.peak_gain, 1.0, 1.0e-6);

  // And the searched band travels with the answer, so the shortfall is
  // attributable rather than mysterious.
  EXPECT_EQ(narrow.searched_from_rad_s, 0.1);
  EXPECT_EQ(narrow.grid_points, 200);
  EXPECT_EQ(narrow.peak_frequency_rad_s, 0.1);
}

TEST(SingularValues, RefusesAnEmptyResponse) {
  galata::analyze::FrequencyResponse empty;
  EXPECT_THROW((void)singular_values(empty), std::invalid_argument);
}

}  // namespace
