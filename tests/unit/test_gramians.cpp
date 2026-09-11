// SPDX-License-Identifier: Apache-2.0
//
// Reachability, observability and the finite-horizon Gramians, against cases
// whose answers can be written down.
//
// WHY CLOSED FORMS AND NOT A REGRESSION LOCK. Every case here has an analytic
// answer — a first-order system's Gramian is an exponential integral, a
// decoupled pair's reachable subspace is a coordinate axis — so the reference is
// mathematics rather than a previous run of this code. A test that pinned
// whatever the routine returned today would pass forever and prove nothing.

#include "galata/analyze/gramians.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

using galata::analyze::analyse_gramians;
using galata::analyze::GramianOptions;
using galata::model::LinearSystem;

GramianOptions options_over(double horizon_s) {
  GramianOptions options;
  options.horizon_s = horizon_s;
  options.steps = 2000;
  return options;
}

// xdot = -a x + b u, y = c x.
LinearSystem first_order(double a, double b, double c) {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Constant(1, 1, -a);
  system.b = Eigen::MatrixXd::Constant(1, 1, b);
  system.c = Eigen::MatrixXd::Constant(1, 1, c);
  system.state_names = {"x"};
  system.input_names = {"u"};
  system.output_names = {"y"};
  return system;
}

// Two decoupled first-order modes, with per-mode input and output coupling that
// each test sets. `reach1`/`see1` zero means the second mode is cut off.
LinearSystem decoupled(double reach0, double reach1, double see0, double see1) {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(2, 2);
  system.a << -1.0, 0.0, 0.0, -2.0;
  system.b = Eigen::MatrixXd::Zero(2, 1);
  system.b << reach0, reach1;
  system.c = Eigen::MatrixXd::Zero(1, 2);
  system.c << see0, see1;
  system.state_names = {"slow", "fast"};
  system.input_names = {"u"};
  system.output_names = {"y"};
  return system;
}

}  // namespace

// W_c(T) = integral over [0, T] of e^{-at} b^2 e^{-at} dt = b^2 (1 - e^{-2aT}) / (2a).
// The budget is the RK4 truncation over 2000 steps of a smooth exponential,
// which is far below the eight digits asserted here; stating it as a relative
// figure rather than an absolute one keeps it meaningful if the constants change.
TEST(Gramians, AFirstOrderSystemMatchesTheClosedFormIntegral) {
  const double a = 1.5;
  const double b = 2.0;
  const double c = 3.0;
  const double horizon = 2.0;
  const auto analysis = analyse_gramians(first_order(a, b, c), options_over(horizon));

  const double expected_controllability = b * b * (1.0 - std::exp(-2.0 * a * horizon)) / (2.0 * a);
  const double expected_observability = c * c * (1.0 - std::exp(-2.0 * a * horizon)) / (2.0 * a);

  ASSERT_EQ(analysis.controllability.rows(), 1);
  EXPECT_NEAR(
      analysis.controllability(0, 0), expected_controllability, 1e-10 * expected_controllability);
  EXPECT_NEAR(
      analysis.observability_gramian(0, 0), expected_observability, 1e-10 * expected_observability);

  // One state, reachable and observable, and the T -> infinity limit exists
  // because the single eigenvalue is strictly negative.
  EXPECT_EQ(analysis.reachability.rank, 1);
  EXPECT_EQ(analysis.observability.rank, 1);
  EXPECT_TRUE(analysis.spectrum_is_strictly_stable);
  EXPECT_DOUBLE_EQ(analysis.rightmost_eigenvalue_real_part, -a);
}

// The Gramian of a strictly stable system approaches its infinite-horizon limit
// b^2/(2a) as the horizon grows. This is the property that makes the finite
// horizon a truncation rather than a different quantity, WHEN the limit exists.
TEST(Gramians, ALongHorizonApproachesTheInfiniteHorizonLimitWhenOneExists) {
  const double a = 1.5;
  const double b = 2.0;
  const double limit = b * b / (2.0 * a);
  const auto brief = analyse_gramians(first_order(a, b, 1.0), options_over(0.5));
  const auto long_run = analyse_gramians(first_order(a, b, 1.0), options_over(20.0));

  EXPECT_LT(brief.controllability(0, 0), 0.9 * limit)
      << "a half-second horizon should be visibly short of the limit";
  EXPECT_NEAR(long_run.controllability(0, 0), limit, 1e-9 * limit);
}

// A mode the input cannot reach is reported as a missing direction, and the
// direction is NAMED — which is the whole point of the analysis, per RFC-0002.
TEST(Gramians, AnUnreachableModeIsReportedByNameRatherThanAsARankDeficit) {
  const auto analysis = analyse_gramians(decoupled(1.0, 0.0, 1.0, 1.0), options_over(3.0));

  EXPECT_EQ(analysis.reachability.rank, 1);
  EXPECT_EQ(analysis.reachability.state_count, 2);
  ASSERT_EQ(analysis.reachability.missing_directions.size(), 1u);
  const auto& missing = analysis.reachability.missing_directions.front();
  ASSERT_EQ(missing.dominant_states.size(), 1u);
  EXPECT_NE(missing.dominant_states.front().find("fast"), std::string::npos)
      << missing.dominant_states.front();
  EXPECT_NEAR(std::fabs(missing.coordinates(1)), 1.0, 1e-12);
  EXPECT_NEAR(missing.coordinates(0), 0.0, 1e-12);

  // Observability is unaffected: both modes are seen.
  EXPECT_EQ(analysis.observability.rank, 2);
  EXPECT_TRUE(analysis.observability.missing_directions.empty());
}

TEST(Gramians, AnUnobservableModeIsReportedByNameToo) {
  const auto analysis = analyse_gramians(decoupled(1.0, 1.0, 1.0, 0.0), options_over(3.0));
  EXPECT_EQ(analysis.reachability.rank, 2);
  EXPECT_EQ(analysis.observability.rank, 1);
  ASSERT_EQ(analysis.observability.missing_directions.size(), 1u);
  EXPECT_NE(analysis.observability.missing_directions.front().dominant_states.front().find("fast"),
            std::string::npos);
}

// THE CASE THIS ROUTINE EXISTS FOR. A double integrator is the shape a hover
// linearisation's position and attitude chains have: the spectrum sits at the
// origin, so the infinite-horizon Gramians do not exist, and a Lyapunov solve
// would return a matrix that is not a Gramian of anything. The finite-horizon
// integral exists and is reported, with the fact stated rather than left for a
// reader to work out.
TEST(Gramians, AModelWithPolesAtTheOriginHasNoInfiniteHorizonGramianAndSaysSo) {
  LinearSystem chain;
  chain.a = Eigen::MatrixXd::Zero(2, 2);
  chain.a(0, 1) = 1.0;
  chain.b = Eigen::MatrixXd::Zero(2, 1);
  chain.b(1, 0) = 1.0;
  chain.c = Eigen::MatrixXd::Zero(1, 2);
  chain.c(0, 0) = 1.0;
  chain.state_names = {"position", "rate"};
  chain.input_names = {"acceleration"};
  chain.output_names = {"measured_position"};

  const auto analysis = analyse_gramians(chain, options_over(1.0));
  EXPECT_FALSE(analysis.spectrum_is_strictly_stable);
  EXPECT_NEAR(analysis.rightmost_eigenvalue_real_part, 0.0, 1e-12);
  EXPECT_NE(analysis.assumptions.find("do not exist for this model"), std::string::npos)
      << analysis.assumptions;

  // A double integrator is both reachable and observable, defective spectrum
  // notwithstanding — which is exactly why the rank result and the Gramian's
  // existence are separate questions.
  EXPECT_EQ(analysis.reachability.rank, 2);
  EXPECT_EQ(analysis.observability.rank, 2);

  // W_c(T) for this chain is closed form: the integral of
  // [t^2, t; t, 1] over [0, T], which is [T^3/3, T^2/2; T^2/2, T].
  const double t = 1.0;
  EXPECT_NEAR(analysis.controllability(0, 0), t * t * t / 3.0, 1e-12);
  EXPECT_NEAR(analysis.controllability(0, 1), t * t / 2.0, 1e-12);
  EXPECT_NEAR(analysis.controllability(1, 1), t, 1e-12);
}

// The Gramians are symmetric and positive semi-definite by construction. A
// singular one is expected whenever a direction is unreachable, and its
// smallest eigenvalue is then round-off rather than a small positive number —
// which is why the condition number is reported as infinite rather than as a
// large finite value nobody should trust.
TEST(Gramians, AGramianOfAnUnreachableSystemIsSingularAndItsConditionNumberSaysSo) {
  const auto analysis = analyse_gramians(decoupled(1.0, 0.0, 1.0, 1.0), options_over(3.0));
  EXPECT_EQ(analysis.controllability, analysis.controllability.transpose());
  ASSERT_EQ(analysis.controllability_eigenvalues.size(), 2u);
  EXPECT_GT(analysis.controllability_eigenvalues.front(), 0.0);
  EXPECT_NEAR(analysis.controllability_eigenvalues.back(), 0.0, 1e-14);
  EXPECT_FALSE(std::isfinite(analysis.controllability_condition_number));
}

// The rank tolerance is a declared judgement, and moving it must move the
// answer — otherwise it is not the thing deciding the rank. A mode reachable
// only through a gain of 1e-7 is retained at a floor of 1e-9 and dropped at
// 1e-5, and neither answer is wrong: they answer different questions.
TEST(Gramians, TheRankToleranceIsTheDeclaredJudgementAndMovingItMovesTheAnswer) {
  const LinearSystem barely = decoupled(1.0, 1e-7, 1.0, 1.0);
  GramianOptions strict = options_over(3.0);
  strict.rank_tolerance = 1e-9;
  GramianOptions loose = options_over(3.0);
  loose.rank_tolerance = 1e-5;

  EXPECT_EQ(analyse_gramians(barely, strict).reachability.rank, 2);
  EXPECT_EQ(analyse_gramians(barely, loose).reachability.rank, 1);
  // And the reported tolerance travels with the answer, so a reader can see
  // which question was asked.
  EXPECT_DOUBLE_EQ(analyse_gramians(barely, loose).rank_tolerance, 1e-5);
}

TEST(Gramians, TheHorizonAndTheToleranceAreRequiredToBeMeaningful) {
  const LinearSystem system = first_order(1.0, 1.0, 1.0);
  GramianOptions bad = options_over(0.0);
  EXPECT_THROW((void)analyse_gramians(system, bad), std::invalid_argument);
  bad.horizon_s = -1.0;
  EXPECT_THROW((void)analyse_gramians(system, bad), std::invalid_argument);

  GramianOptions no_steps = options_over(1.0);
  no_steps.steps = 0;
  EXPECT_THROW((void)analyse_gramians(system, no_steps), std::invalid_argument);

  GramianOptions bad_tolerance = options_over(1.0);
  bad_tolerance.rank_tolerance = 0.0;
  EXPECT_THROW((void)analyse_gramians(system, bad_tolerance), std::invalid_argument);
  bad_tolerance.rank_tolerance = 1.0;
  EXPECT_THROW((void)analyse_gramians(system, bad_tolerance), std::invalid_argument);
}

// The refusal for the horizon says WHY it is not defaulted, because a caller
// who is told only "required" will supply the first number that comes to mind.
TEST(Gramians, TheMissingHorizonRefusalExplainsWhyThereIsNoDefault) {
  try {
    (void)analyse_gramians(first_order(1.0, 1.0, 1.0), options_over(0.0));
    FAIL() << "a zero horizon must be refused";
  } catch (const std::invalid_argument& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("required rather than defaulted"), std::string::npos) << message;
    EXPECT_NE(message.find("a reader will quote"), std::string::npos) << message;
  }
}
