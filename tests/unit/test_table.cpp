// SPDX-License-Identifier: Apache-2.0
//
// Lookup tables against closed-form answers, and against the two refusals that
// are the whole reason the type exists: a non-monotonic axis, and a query that
// left the data.

#include "galata/numerics/table.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using galata::numerics::Table1D;
using galata::numerics::Table2D;
using galata::numerics::TableExtrapolation;
using galata::numerics::TableInterpolation;

namespace {

Table1D linear_table(TableExtrapolation outside = TableExtrapolation::Refuse) {
  // y = 2x + 1 sampled at four breakpoints. Linear interpolation of a linear
  // function is EXACT everywhere inside, which is what makes this a closed-form
  // check rather than a tolerance check.
  return Table1D("cd0_vs_alpha", {-2.0, 0.0, 1.0, 4.0}, {-3.0, 1.0, 3.0, 9.0},
                 TableInterpolation::Linear, outside);
}

}  // namespace

TEST(Table1D, IsExactAtEveryBreakpoint) {
  const auto table = linear_table();
  EXPECT_EQ(table.at(-2.0), -3.0);
  EXPECT_EQ(table.at(0.0), 1.0);
  EXPECT_EQ(table.at(1.0), 3.0);
  EXPECT_EQ(table.at(4.0), 9.0);
  EXPECT_EQ(table.out_of_range_count(), 0u);
}

TEST(Table1D, ReproducesALinearFunctionExactlyBetweenBreakpoints) {
  const auto table = linear_table();
  for (double x = -2.0; x <= 4.0; x += 0.125) {
    EXPECT_NEAR(table.at(x), 2.0 * x + 1.0, 1e-14) << "at x = " << x;
  }
}

TEST(Table1D, RefusesAnAxisThatDoesNotStrictlyIncrease) {
  // A repeated breakpoint is how a step gets written by somebody who has not
  // read the header, and reading it as either value picks a different function.
  EXPECT_THROW(Table1D("t", {0.0, 1.0, 1.0}, {0.0, 1.0, 2.0}, TableInterpolation::Linear,
                       TableExtrapolation::Hold),
               std::invalid_argument);
  EXPECT_THROW(Table1D("t", {0.0, 2.0, 1.0}, {0.0, 1.0, 2.0}, TableInterpolation::Linear,
                       TableExtrapolation::Hold),
               std::invalid_argument);
}

TEST(Table1D, RefusesASizeMismatchAndATooShortAxis) {
  EXPECT_THROW(Table1D("t", {0.0, 1.0}, {0.0}, TableInterpolation::Linear,
                       TableExtrapolation::Hold),
               std::invalid_argument);
  EXPECT_THROW(
      Table1D("t", {0.0}, {0.0}, TableInterpolation::Linear, TableExtrapolation::Hold),
      std::invalid_argument);
}

TEST(Table1D, RefusesOutOfRangeWhenTheStudyDeclaredRefuse) {
  const auto table = linear_table(TableExtrapolation::Refuse);
  EXPECT_THROW((void)table.at(4.001), std::out_of_range);
  EXPECT_THROW((void)table.at(-2.001), std::out_of_range);
  // The refusal still counts the query: a run that was refused four times tried
  // to leave the data four times, and the report says so.
  EXPECT_EQ(table.out_of_range_count(), 2u);
  EXPECT_NEAR(table.worst_excursion(), 0.001, 1e-12);
}

TEST(Table1D, HoldClampsAndCountsRatherThanRefusing) {
  const auto table = linear_table(TableExtrapolation::Hold);
  EXPECT_EQ(table.at(10.0), 9.0);
  EXPECT_EQ(table.at(-10.0), -3.0);
  EXPECT_EQ(table.out_of_range_count(), 2u);
  EXPECT_NEAR(table.worst_excursion(), 8.0, 1e-12);
  table.reset_counts();
  EXPECT_EQ(table.out_of_range_count(), 0u);
  EXPECT_EQ(table.worst_excursion(), 0.0);
}

TEST(Table1D, LinearExtrapolationContinuesTheEndSegmentSlope) {
  const auto table = linear_table(TableExtrapolation::Linear);
  // The underlying function is 2x + 1 everywhere, so continuing the end slope
  // reproduces it exactly outside as well.
  EXPECT_NEAR(table.at(10.0), 21.0, 1e-12);
  EXPECT_NEAR(table.at(-10.0), -19.0, 1e-12);
  EXPECT_EQ(table.out_of_range_count(), 2u);
}

TEST(Table1D, NearestLowTakesTheLastBreakpointNotAfterTheQuery) {
  const Table1D table("schedule", {0.0, 1.0, 2.0}, {10.0, 20.0, 30.0},
                      TableInterpolation::Nearest_Low, TableExtrapolation::Hold);
  EXPECT_EQ(table.at(0.0), 10.0);
  EXPECT_EQ(table.at(0.999), 10.0);
  EXPECT_EQ(table.at(1.0), 20.0);
  EXPECT_EQ(table.at(1.999), 20.0);
  EXPECT_EQ(table.at(2.0), 30.0);
}

TEST(Table1D, IsBitIdenticalOnRepeatedQuerySequences) {
  const auto a = linear_table(TableExtrapolation::Hold);
  const auto b = linear_table(TableExtrapolation::Hold);
  for (int i = -50; i <= 50; ++i) {
    const double x = static_cast<double>(i) * 0.137;
    EXPECT_EQ(a.at(x), b.at(x)) << "at x = " << x;
  }
  EXPECT_EQ(a.out_of_range_count(), b.out_of_range_count());
  EXPECT_EQ(a.worst_excursion(), b.worst_excursion());
}

TEST(Table1D, ReportsAnInteriorBreakpointSoALinearisationCanSayItStraddledAKink) {
  const auto table = linear_table();
  EXPECT_TRUE(table.breakpoint_near(0.0, 1e-3));
  EXPECT_TRUE(table.breakpoint_near(1.0005, 1e-3));
  EXPECT_FALSE(table.breakpoint_near(2.5, 1e-3));
  // The ends are not interior: a difference taken there does not straddle.
  EXPECT_FALSE(table.breakpoint_near(-2.0, 1e-3));
  EXPECT_FALSE(table.breakpoint_near(4.0, 1e-3));
}

TEST(Table2D, ReproducesABilinearFunctionExactly) {
  // f(x, y) = 1 + 2x + 3y + 4xy is bilinear, so bilinear interpolation of it is
  // exact at every point of the rectangle, not merely at the corners.
  const std::vector<double> rows{0.0, 1.0, 3.0};
  const std::vector<double> columns{-1.0, 0.0, 2.0};
  Eigen::MatrixXd values(3, 3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double x = rows[static_cast<std::size_t>(i)];
      const double y = columns[static_cast<std::size_t>(j)];
      values(i, j) = 1.0 + 2.0 * x + 3.0 * y + 4.0 * x * y;
    }
  }
  const Table2D table("cl_vs_alpha_beta", rows, columns, values, TableInterpolation::Linear,
                      TableExtrapolation::Refuse);
  for (double x = 0.0; x <= 3.0; x += 0.25) {
    for (double y = -1.0; y <= 2.0; y += 0.25) {
      EXPECT_NEAR(table.at(x, y), 1.0 + 2.0 * x + 3.0 * y + 4.0 * x * y, 1e-13)
          << "at (" << x << ", " << y << ")";
    }
  }
  EXPECT_EQ(table.out_of_range_count(), 0u);
}

TEST(Table2D, RefusesAShapeMismatch) {
  Eigen::MatrixXd values(2, 3);
  values.setZero();
  EXPECT_THROW(Table2D("t", {0.0, 1.0, 2.0}, {0.0, 1.0, 2.0}, values, TableInterpolation::Linear,
                       TableExtrapolation::Hold),
               std::invalid_argument);
}

TEST(Table2D, RefusesOutOfRangeOnEitherAxis) {
  Eigen::MatrixXd values(2, 2);
  values << 1.0, 2.0, 3.0, 4.0;
  const Table2D table("t", {0.0, 1.0}, {0.0, 1.0}, values, TableInterpolation::Linear,
                      TableExtrapolation::Refuse);
  EXPECT_THROW((void)table.at(1.5, 0.5), std::out_of_range);
  EXPECT_THROW((void)table.at(0.5, 1.5), std::out_of_range);
  EXPECT_NO_THROW((void)table.at(1.0, 1.0));
}
