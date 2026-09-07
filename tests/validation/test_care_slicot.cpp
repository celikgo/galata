// SPDX-License-Identifier: Apache-2.0
// SLICOT BB01AD CAREX 2.3 and SB02MD worked results; see reference provenance.
#include "galata/synth/control.hpp"

#include "reference_table.hpp"
#include "validation_config.hpp"
#include <gtest/gtest.h>

TEST(CareSlicot, MatchesPublishedWorkedSolutionsWithinPrintedPrecision) {
  const auto reference =
      galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR, "slicot_care.csv");
  for (std::size_t i = 0; i < reference.rows.size(); ++i) {
    SCOPED_TRACE(reference.text(i, "case"));
    Eigen::Matrix2d a;
    a << 0, reference.at(i, "epsilon"), 0, 0;
    Eigen::Vector2d b;
    b << 0, 1;
    Eigen::Matrix2d q;
    q << 1, 0, 0, reference.at(i, "q2");
    const auto result = galata::synth::solve_care(a, b, q, Eigen::MatrixXd::Identity(1, 1));
    EXPECT_NEAR(result.x(0, 0), reference.at(i, "x11"), reference.at(i, "absolute_budget"));
    EXPECT_NEAR(result.x(0, 1), reference.at(i, "x12"), reference.at(i, "absolute_budget"));
    EXPECT_NEAR(result.x(1, 1), reference.at(i, "x22"), reference.at(i, "absolute_budget"));
    EXPECT_LE(result.relative_residual, result.residual_budget);
  }
}
