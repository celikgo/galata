// SPDX-License-Identifier: Apache-2.0
//
// Linear response against the integrating-factor closed form for x'=-2x+3u.
// Reference: Hairer, Norsett & Wanner, Solving Ordinary Differential Equations I,
// 2nd revised ed., Springer, 1993, classical RK4 order and stability polynomial.
#include "galata/sim/linear.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

galata::model::LinearSystem lag() {
  galata::model::LinearSystem system;
  system.a = Eigen::MatrixXd::Constant(1, 1, -2.0);
  system.b = Eigen::MatrixXd::Constant(1, 1, 3.0);
  system.state_names = {"x"};
  system.input_names = {"u"};
  return system;
}

TEST(LinearSimulation, ForcedResponseConvergesAtFourthOrderToTheClosedForm) {
  const auto system = lag();
  const Eigen::VectorXd initial = Eigen::VectorXd::Constant(1, 0.5);
  const Eigen::VectorXd input = Eigen::VectorXd::Ones(1);
  const double exact = 1.5 - std::exp(-2.0);
  const auto coarse = galata::sim::simulate_linear(system, initial, input, 0.1, 10);
  const auto fine = galata::sim::simulate_linear(system, initial, input, 0.05, 20);
  const double coarse_error = std::abs(coarse.states.back()(0) - exact);
  const double fine_error = std::abs(fine.states.back()(0) - exact);
  EXPECT_GT(coarse_error / fine_error, 15.0);
  EXPECT_LT(coarse_error / fine_error, 20.0);
  EXPECT_LT(fine_error, 1e-6);
}

TEST(LinearSimulation, FinalStateIsRecordedWhenStrideDoesNotDivideStepCount) {
  const auto response = galata::sim::simulate_linear(
      lag(), Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), 0.01, 5, 3);
  ASSERT_EQ(response.times_s.size(), 3U);
  EXPECT_DOUBLE_EQ(response.times_s.back(), 0.05);
  EXPECT_DOUBLE_EQ(response.states.back()(0), 0.0);
}

TEST(LinearSimulation, AnAutonomousUnstableModeRetainsItsPhysicalGrowth) {
  galata::model::LinearSystem system;
  system.a = Eigen::MatrixXd::Ones(1, 1);
  system.state_names = {"x"};
  const auto response =
      galata::sim::simulate_linear(system, Eigen::VectorXd::Ones(1), Eigen::VectorXd{}, 0.01, 100);
  EXPECT_NEAR(response.states.back()(0), std::exp(1.0), 3e-10);
}

TEST(LinearSimulation, RejectsUnstableNumericalStepsAndInvalidInputs) {
  const auto system = lag();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);
  // h lambda = -4, with R(-4)=5, amplifies a physically decaying state.
  EXPECT_THROW((void)galata::sim::simulate_linear(system, zero, zero, 2.0, 1),
               std::invalid_argument);
  EXPECT_THROW((void)galata::sim::simulate_linear(system, Eigen::VectorXd::Zero(2), zero, 0.01, 1),
               std::invalid_argument);
  EXPECT_THROW((void)galata::sim::simulate_linear(
                   system, zero, zero, std::numeric_limits<double>::infinity(), 1),
               std::invalid_argument);
  EXPECT_THROW((void)galata::sim::simulate_linear(system, zero, zero, 0.01, -1),
               std::invalid_argument);
}

}  // namespace
