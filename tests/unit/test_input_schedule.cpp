// SPDX-License-Identifier: Apache-2.0
//
// Declared input histories: what they return between samples, what they return
// outside them, and what they refuse.
//
// Every expected value here is written down from the definition rather than
// read off the implementation. A zero-order hold returns the last sample not
// after t; a linear hold interpolates; neither invents a value outside the
// declared span unless the caller declared that it may.

#include "galata/sim/schedule.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

namespace {

using galata::sim::Extrapolation;
using galata::sim::HoldPolicy;
using galata::sim::InputSchedule;

std::vector<Eigen::VectorXd> scalars(const std::vector<double>& values) {
  std::vector<Eigen::VectorXd> out;
  for (const double value : values) {
    Eigen::VectorXd row(1);
    row(0) = value;
    out.push_back(row);
  }
  return out;
}

InputSchedule step_history(HoldPolicy hold, Extrapolation outside) {
  return InputSchedule({0.0, 1.0, 2.0}, scalars({10.0, 20.0, 20.0}), hold, outside);
}

}  // namespace

TEST(InputSchedule, ZeroOrderHoldsTheLastSampleNotAfterTheQuery) {
  const InputSchedule history = step_history(HoldPolicy::ZeroOrder, Extrapolation::Refuse);
  EXPECT_DOUBLE_EQ(history.at(0.0)(0), 10.0);
  EXPECT_DOUBLE_EQ(history.at(0.999)(0), 10.0);
  // ON the sample time the new value is in force: the sample says when it
  // starts, not when it ends.
  EXPECT_DOUBLE_EQ(history.at(1.0)(0), 20.0);
  EXPECT_DOUBLE_EQ(history.at(2.0)(0), 20.0);
  // A held value has zero rate everywhere it is defined. The jump is an event,
  // not a derivative, and is reported as one.
  EXPECT_DOUBLE_EQ(history.rate_at(0.5)(0), 0.0);
  ASSERT_EQ(history.discontinuities().size(), 1u);
  EXPECT_DOUBLE_EQ(history.discontinuities().front(), 1.0);
  EXPECT_DOUBLE_EQ(history.jump_at(1.0)(0), 10.0);
}

TEST(InputSchedule, LinearInterpolatesAndReportsTheSegmentSlope) {
  const InputSchedule ramp(
      {0.0, 2.0}, scalars({4.0, 10.0}), HoldPolicy::Linear, Extrapolation::Refuse);
  // (10 - 4) / 2 = 3 per second, so 4 + 3*0.5 = 5.5 at t = 0.5.
  EXPECT_DOUBLE_EQ(ramp.at(0.5)(0), 5.5);
  EXPECT_DOUBLE_EQ(ramp.at(2.0)(0), 10.0);
  EXPECT_DOUBLE_EQ(ramp.rate_at(0.5)(0), 3.0);
  // A ramp has no jumps to re-base at.
  EXPECT_TRUE(ramp.discontinuities().empty());
}

TEST(InputSchedule, OutsideTheSpanIsRefusedUnlessHoldingWasDeclared) {
  const InputSchedule strict = step_history(HoldPolicy::ZeroOrder, Extrapolation::Refuse);
  EXPECT_THROW((void)strict.at(-0.001), std::invalid_argument);
  EXPECT_THROW((void)strict.at(2.001), std::invalid_argument)
      << "a horizon longer than the declared history must be refused, not clamped: a silent "
         "clamp makes the tail of a run mean something the study does not say";

  const InputSchedule holding = step_history(HoldPolicy::ZeroOrder, Extrapolation::Hold);
  EXPECT_DOUBLE_EQ(holding.at(-5.0)(0), 10.0);
  EXPECT_DOUBLE_EQ(holding.at(50.0)(0), 20.0);
}

TEST(InputSchedule, AmbiguousOrMalformedHistoriesAreRefused) {
  // A repeated timestamp is how a step gets written by someone who has not read
  // the header. Reading it as either value picks a different trajectory.
  EXPECT_THROW(
      InputSchedule(
          {0.0, 1.0, 1.0}, scalars({1.0, 2.0, 3.0}), HoldPolicy::ZeroOrder, Extrapolation::Hold),
      std::invalid_argument);
  // Out of order.
  EXPECT_THROW(
      InputSchedule(
          {0.0, 2.0, 1.0}, scalars({1.0, 2.0, 3.0}), HoldPolicy::ZeroOrder, Extrapolation::Hold),
      std::invalid_argument);
  // Width that changes partway is not one signal.
  std::vector<Eigen::VectorXd> ragged = scalars({1.0});
  ragged.push_back(Eigen::VectorXd::Zero(3));
  EXPECT_THROW(InputSchedule({0.0, 1.0}, ragged, HoldPolicy::ZeroOrder, Extrapolation::Hold),
               std::invalid_argument);
  // Empty.
  EXPECT_THROW(InputSchedule({}, {}, HoldPolicy::ZeroOrder, Extrapolation::Hold),
               std::invalid_argument);
  // Non-finite.
  EXPECT_THROW(InputSchedule({0.0, 1.0},
                             scalars({1.0, std::numeric_limits<double>::quiet_NaN()}),
                             HoldPolicy::ZeroOrder,
                             Extrapolation::Hold),
               std::invalid_argument);
}
