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

// --- declared input histories ------------------------------------------------
//
// Three references, none of them the routine under test. The lag's closed form
// per held segment, x(t+d) = x(t) e^{-2d} + 1.5 u (1 - e^{-2d}), computed with
// std::exp. The double integrator, whose solution under a held or linearly
// varying input is a polynomial of degree at most three in time, which RK4
// integrates EXACTLY, so the only tolerance is round-off and an event handled
// one step late shows up at the size of the step. And the constant-input
// overload itself, which a history that never changes must reproduce bit for
// bit.

using galata::sim::Extrapolation;
using galata::sim::HoldPolicy;
using galata::sim::InputSchedule;

InputSchedule history(std::vector<double> times,
                      std::vector<std::vector<double>> values,
                      HoldPolicy hold,
                      Extrapolation outside = Extrapolation::Hold) {
  std::vector<Eigen::VectorXd> rows;
  for (const auto& value : values) {
    rows.push_back(
        Eigen::Map<const Eigen::VectorXd>(value.data(), static_cast<Eigen::Index>(value.size())));
  }
  return InputSchedule(std::move(times), std::move(rows), hold, outside);
}

galata::model::LinearSystem double_integrator() {
  galata::model::LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(2, 2);
  system.a(0, 1) = 1.0;
  system.b = Eigen::MatrixXd::Zero(2, 1);
  system.b(1, 0) = 1.0;
  system.state_names = {"position_m", "velocity_m_s"};
  system.input_names = {"acceleration_m_s2"};
  return system;
}

TEST(LinearSimulation, AZeroOrderHistoryMatchesTheClosedFormAcrossEveryEvent) {
  const auto input = history({0.0, 0.3, 0.7}, {{1.0}, {-0.5}, {2.0}}, HoldPolicy::ZeroOrder);
  constexpr double kStep = 0.01;
  constexpr int kSteps = 100;
  const auto run =
      galata::sim::simulate_linear(lag(), Eigen::VectorXd::Constant(1, 0.5), input, kStep, kSteps);
  EXPECT_EQ(run.event_steps, (std::vector<int>{30, 70}));

  // The budget, fixed before the run: RK4's per-step relative error on this pole
  // is |e^z - R(z)|, below |z|^5 / 120 with z = -2h, accumulated over every step
  // on a state no larger than 3.
  const double z = 2.0 * kStep;
  const double budget = kSteps * std::pow(z, 5) / 120.0 * 3.0 * 2.0;
  double exact = 0.5;
  for (std::size_t k = 1; k < run.trajectory.times_s.size(); ++k) {
    const double start = run.trajectory.times_s[k - 1];
    const double held = input.at(start)(0);
    const double d = run.trajectory.times_s[k] - start;
    exact = exact * std::exp(-2.0 * d) + 1.5 * held * (1.0 - std::exp(-2.0 * d));
    EXPECT_NEAR(run.trajectory.states[k](0), exact, budget) << "t = " << run.trajectory.times_s[k];
  }
}

TEST(LinearSimulation, ADoubleIntegratorUnderAZeroOrderHistoryIsExactToRoundOff) {
  const auto input =
      history({0.0, 0.25, 0.5, 0.75}, {{2.0}, {-1.0}, {0.0}, {3.0}}, HoldPolicy::ZeroOrder);
  const auto run = galata::sim::simulate_linear(
      double_integrator(), Eigen::Vector2d(0.1, -0.2), input, 0.01, 100);
  EXPECT_EQ(run.event_steps, (std::vector<int>{25, 50, 75}));
  double position = 0.1;
  double velocity = -0.2;
  double previous = 0.0;
  for (std::size_t k = 1; k < run.trajectory.times_s.size(); ++k) {
    const double t = run.trajectory.times_s[k];
    const double held = input.at(previous)(0);
    const double d = t - previous;
    position += velocity * d + 0.5 * held * d * d;
    velocity += held * d;
    previous = t;
    EXPECT_NEAR(run.trajectory.states[k](0), position, 1e-12) << "t = " << t;
    EXPECT_NEAR(run.trajectory.states[k](1), velocity, 1e-12) << "t = " << t;
  }
}

TEST(LinearSimulation, ALinearHistoryIsEvaluatedAtEveryStageAndIsExactOnADoubleIntegrator) {
  // Breakpoints on the step lattice, so every step sees one slope and the cubic
  // solution is inside RK4's exactness.
  const auto input = history({0.0, 0.4, 1.0}, {{0.0}, {2.0}, {-1.0}}, HoldPolicy::Linear);
  const auto run =
      galata::sim::simulate_linear(double_integrator(), Eigen::Vector2d(0.0, 0.0), input, 0.02, 50);
  EXPECT_TRUE(run.event_steps.empty());
  double position = 0.0;
  double velocity = 0.0;
  double previous = 0.0;
  for (std::size_t k = 1; k < run.trajectory.times_s.size(); ++k) {
    const double t = run.trajectory.times_s[k];
    const double u0 = input.at(previous)(0);
    const double slope = input.rate_at(previous)(0);
    const double d = t - previous;
    position += velocity * d + 0.5 * u0 * d * d + slope * d * d * d / 6.0;
    velocity += u0 * d + 0.5 * slope * d * d;
    previous = t;
    EXPECT_NEAR(run.trajectory.states[k](0), position, 1e-12) << "t = " << t;
    EXPECT_NEAR(run.trajectory.states[k](1), velocity, 1e-12) << "t = " << t;
    EXPECT_NEAR(run.input_samples[k](0), input.at(t)(0), 1e-15);
  }
}

TEST(LinearSimulation, AHistoryThatHoldsOneValueReproducesTheConstantInputRunBitForBit) {
  const Eigen::VectorXd initial = Eigen::VectorXd::Constant(1, 0.5);
  const auto constant =
      galata::sim::simulate_linear(lag(), initial, Eigen::VectorXd::Constant(1, 0.7), 0.01, 200, 7);
  for (const auto& input : {history({0.0}, {{0.7}}, HoldPolicy::ZeroOrder),
                            history({-1.0, 5.0}, {{0.7}, {0.7}}, HoldPolicy::ZeroOrder)}) {
    const auto scheduled = galata::sim::simulate_linear(lag(), initial, input, 0.01, 200, 7);
    ASSERT_EQ(scheduled.trajectory.states.size(), constant.states.size());
    for (std::size_t k = 0; k < constant.states.size(); ++k) {
      EXPECT_EQ(scheduled.trajectory.states[k](0), constant.states[k](0)) << "sample " << k;
      EXPECT_EQ(scheduled.trajectory.times_s[k], constant.times_s[k]);
    }
  }
}

TEST(LinearSimulation, TheRecordedInputIsTheValueInForceAfterAnEventAtThatInstant) {
  const auto input = history({0.0, 0.3}, {{1.0}, {-4.0}}, HoldPolicy::ZeroOrder);
  const auto run = galata::sim::simulate_linear(lag(), Eigen::VectorXd::Zero(1), input, 0.1, 5);
  ASSERT_EQ(run.input_samples.size(), 6U);
  const double expected[] = {1.0, 1.0, 1.0, -4.0, -4.0, -4.0};
  for (std::size_t k = 0; k < 6; ++k) {
    EXPECT_EQ(run.input_samples[k](0), expected[k]) << "t = " << run.trajectory.times_s[k];
  }
}

TEST(LinearSimulation, WhatAHistoryCannotDefineIsRefusedBeforeAnyStep) {
  const auto system = lag();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);
  // A jump inside a step: 0.305 s is not a whole number of 0.01 s steps.
  EXPECT_THROW(
      (void)galata::sim::simulate_linear(
          system, zero, history({0.0, 0.305}, {{1.0}, {2.0}}, HoldPolicy::ZeroOrder), 0.01, 100),
      std::invalid_argument);
  // A horizon past the span under `refuse`.
  EXPECT_THROW((void)galata::sim::simulate_linear(
                   system,
                   zero,
                   history({0.0, 0.5}, {{1.0}, {2.0}}, HoldPolicy::Linear, Extrapolation::Refuse),
                   0.01,
                   100),
               std::invalid_argument);
  // The same history is admissible when the horizon stays inside it.
  EXPECT_NO_THROW((void)galata::sim::simulate_linear(
      system,
      zero,
      history({0.0, 0.5}, {{1.0}, {2.0}}, HoldPolicy::Linear, Extrapolation::Refuse),
      0.01,
      50));
  // The wrong width, and a system with nothing to drive.
  EXPECT_THROW((void)galata::sim::simulate_linear(
                   system, zero, history({0.0}, {{1.0, 2.0}}, HoldPolicy::ZeroOrder), 0.01, 10),
               std::invalid_argument);
  galata::model::LinearSystem autonomous;
  autonomous.a = Eigen::MatrixXd::Constant(1, 1, -1.0);
  autonomous.state_names = {"x"};
  EXPECT_THROW((void)galata::sim::simulate_linear(
                   autonomous, zero, history({0.0}, {{1.0}}, HoldPolicy::ZeroOrder), 0.01, 10),
               std::invalid_argument);
  EXPECT_THROW((void)galata::sim::simulate_linear(system, zero, InputSchedule{}, 0.01, 10),
               std::invalid_argument);
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
