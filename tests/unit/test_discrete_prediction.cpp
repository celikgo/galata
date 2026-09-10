// SPDX-License-Identifier: Apache-2.0
//
// The discrete loop's own prediction, against two references that are not the
// routine under test.
//
//  1. HAND ARITHMETIC for a scalar loop. With a = 1/2, b = 1 and k = 3/10 the
//     undelayed loop is x[k+1] = (1/5) x[k], and the one-tick-delayed loop is
//     the second-order recurrence x[k+1] = x[k]/2 - 3 x[k-1]/10 with the input
//     at tick 0 held at trim. Both are written below as the expressions they
//     are, not as decimals read off a run.
//
//  2. THE AUGMENTED STATE for a multivariable loop. A whole-period delay of d
//     is exactly d extra states holding past inputs (Astrom and Wittenmark,
//     1997, section 2.3). Powers of that block matrix reach the same states by
//     a route with no queue in it at all.

#include "galata/sim/discrete.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <stdexcept>

namespace {

using galata::model::DiscreteLinearSystem;
using galata::sim::predict_sampled_loop;

DiscreteLinearSystem scalar_plant() {
  DiscreteLinearSystem plant;
  plant.a = Eigen::MatrixXd::Constant(1, 1, 0.5);
  plant.b = Eigen::MatrixXd::Constant(1, 1, 1.0);
  plant.sample_time_s = 0.01;
  plant.state_names = {"x"};
  plant.input_names = {"u"};
  return plant;
}

TEST(DiscretePrediction, AnUndelayedScalarLoopDecaysAtItsClosedLoopPole) {
  const Eigen::MatrixXd gain = Eigen::MatrixXd::Constant(1, 1, 0.3);
  const auto prediction =
      predict_sampled_loop(scalar_plant(), gain, 0, Eigen::VectorXd::Constant(1, 1.0), 5);
  ASSERT_EQ(prediction.states.size(), 6U);
  ASSERT_EQ(prediction.applied_inputs.size(), 5U);
  double expected = 1.0;
  for (std::size_t k = 0; k < prediction.states.size(); ++k) {
    EXPECT_NEAR(prediction.states[k](0), expected, 1e-15) << "tick " << k;
    expected *= 0.5 - 0.3;
  }
}

TEST(DiscretePrediction, AOneTickDelayGivesTheHandDerivedRecurrence) {
  const Eigen::MatrixXd gain = Eigen::MatrixXd::Constant(1, 1, 0.3);
  const auto prediction =
      predict_sampled_loop(scalar_plant(), gain, 1, Eigen::VectorXd::Constant(1, 1.0), 4);
  // Tick 0 applies the trim input, so the first step is the open plant.
  const double x0 = 1.0;
  const double x1 = 0.5 * x0;
  const double x2 = 0.5 * x1 - 0.3 * x0;
  const double x3 = 0.5 * x2 - 0.3 * x1;
  const double x4 = 0.5 * x3 - 0.3 * x2;
  const double expected[] = {x0, x1, x2, x3, x4};
  ASSERT_EQ(prediction.states.size(), 5U);
  for (std::size_t k = 0; k < 5; ++k) {
    EXPECT_NEAR(prediction.states[k](0), expected[k], 1e-15) << "tick " << k;
  }
  EXPECT_EQ(prediction.applied_inputs[0](0), 0.0)
      << "before the delay line fills, the plant receives the trim input";
  EXPECT_NEAR(prediction.applied_inputs[1](0), -0.3 * x0, 1e-15)
      << "tick 1 receives the command computed at tick 0";
}

TEST(DiscretePrediction, AMultivariableDelayedLoopMatchesTheAugmentedStateMatrix) {
  DiscreteLinearSystem plant;
  plant.a.resize(2, 2);
  plant.a << 1.0, 0.02, 0.0, 0.97;
  plant.b.resize(2, 2);
  plant.b << 0.0002, 0.0, 0.02, 0.01;
  plant.sample_time_s = 0.02;
  plant.state_names = {"position", "velocity"};
  plant.input_names = {"push", "trim_tab"};
  Eigen::MatrixXd gain(2, 2);
  gain << 3.0, 1.5, -0.4, 0.2;
  constexpr int kDelay = 2;
  constexpr int kTicks = 40;
  const Eigen::Vector2d initial(1.0, -0.5);

  const auto prediction = predict_sampled_loop(plant, gain, kDelay, initial, kTicks);

  // z[k] = [x[k]; u[k-1]; u[k-2]], v[k] = u[k-2], u[k] = -K x[k].
  Eigen::MatrixXd augmented = Eigen::MatrixXd::Zero(6, 6);
  augmented.block(0, 0, 2, 2) = plant.a;
  augmented.block(0, 4, 2, 2) = plant.b;
  augmented.block(2, 0, 2, 2) = -gain;
  augmented.block(4, 2, 2, 2) = Eigen::Matrix2d::Identity();
  Eigen::VectorXd z = Eigen::VectorXd::Zero(6);
  z.head(2) = initial;
  for (int k = 0; k <= kTicks; ++k) {
    EXPECT_LT((prediction.states[static_cast<std::size_t>(k)] - z.head(2)).cwiseAbs().maxCoeff(),
              1e-13)
        << "tick " << k;
    z = augmented * z;
  }
}

TEST(DiscretePrediction, WhatCannotBePredictedIsRefused) {
  const auto plant = scalar_plant();
  const Eigen::MatrixXd gain = Eigen::MatrixXd::Constant(1, 1, 0.3);
  const Eigen::VectorXd start = Eigen::VectorXd::Constant(1, 1.0);
  EXPECT_THROW((void)predict_sampled_loop(plant, Eigen::MatrixXd::Zero(2, 1), 0, start, 3),
               std::invalid_argument)
      << "a gain of the wrong shape";
  EXPECT_THROW((void)predict_sampled_loop(plant, gain, -1, start, 3), std::invalid_argument)
      << "a negative delay";
  EXPECT_THROW((void)predict_sampled_loop(plant, gain, 0, start, 0), std::invalid_argument)
      << "no ticks";
  EXPECT_THROW((void)predict_sampled_loop(plant, gain, 0, Eigen::VectorXd::Zero(2), 3),
               std::invalid_argument)
      << "an initial state of the wrong length";
  auto untimed = plant;
  untimed.sample_time_s = 0.0;
  EXPECT_THROW((void)predict_sampled_loop(untimed, gain, 0, start, 3), std::invalid_argument)
      << "a discrete plant without a sample time is not a discrete plant";
}

}  // namespace
