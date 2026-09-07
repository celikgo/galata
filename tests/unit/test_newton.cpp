// SPDX-License-Identifier: Apache-2.0
//
// Newton's method against systems whose roots are known in closed form.

#include "galata/numerics/newton.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using galata::numerics::NewtonOptions;
using galata::numerics::solve_newton;

TEST(Newton, SolvesALinearSystemInOneStep) {
  // Newton is exact on a linear system: the first step lands on the root.
  Eigen::MatrixXd a(3, 3);
  a << 2.0, 1.0, -1.0, -3.0, -1.0, 2.0, -2.0, 1.0, 2.0;
  Eigen::VectorXd b(3);
  b << 8.0, -11.0, -3.0;

  const auto residual = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd { return a * x - b; };
  const auto result = solve_newton(residual, Eigen::VectorXd::Zero(3));

  ASSERT_TRUE(result.converged) << "residual " << result.residual_norm;
  Eigen::VectorXd expected(3);
  expected << 2.0, 3.0, -1.0;
  EXPECT_TRUE(result.solution.isApprox(expected, 1e-9)) << "got " << result.solution.transpose();
  // The history shows it: one step takes the residual down by seven orders.
  //
  // Not to zero, and the reason is worth knowing. The Jacobian is computed by
  // finite differences, so it carries cancellation error of about
  // eps |f| / h. At the initial guess of zero the step floor applies, and f
  // there is |b| ~ 10, so the Jacobian entries are good to roughly 1e-9 and
  // the step inherits that. An analytic Jacobian would land on the root
  // exactly; a numerical one lands near it and the second iteration cleans up.
  EXPECT_LT(result.residual_history[1], 1e-5 * result.residual_history[0]);
  EXPECT_LT(result.residual_norm, 1e-10);
}

TEST(Newton, ConvergesQuadraticallyOnASmoothNonlinearSystem) {
  // x^2 + y^2 = 4, x y = 1. One root is near (1.932, 0.518).
  const auto residual = [](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    Eigen::VectorXd r(2);
    r(0) = v(0) * v(0) + v(1) * v(1) - 4.0;
    r(1) = v(0) * v(1) - 1.0;
    return r;
  };
  Eigen::VectorXd guess(2);
  guess << 2.0, 0.4;

  const auto result = solve_newton(residual, guess);
  ASSERT_TRUE(result.converged) << "residual " << result.residual_norm;

  // Check against the closed form: x^2 and y^2 are the roots of t^2 - 4t + 1.
  const double x2 = 2.0 + std::sqrt(3.0);
  EXPECT_NEAR(result.solution(0), std::sqrt(x2), 1e-9);
  EXPECT_NEAR(result.solution(1), 1.0 / std::sqrt(x2), 1e-9);

  // Quadratic convergence: the number of correct digits doubles. Three
  // iterations from this guess should cross from 1e-1 to below 1e-8.
  ASSERT_GE(result.residual_history.size(), 4U);
  EXPECT_LT(result.residual_history[3], 1e-8);
}

TEST(Newton, RunsAFixedNumberOfIterationsRatherThanStoppingEarly) {
  // ADR-0004: a tolerance-based exit makes the iteration count a function of
  // rounding, and with it the answer. The history length is therefore fixed.
  const auto residual = [](const Eigen::VectorXd& v) -> Eigen::VectorXd { return v; };
  NewtonOptions options;
  options.iterations = 7;
  const auto result = solve_newton(residual, Eigen::VectorXd::Constant(2, 1.0), options);
  EXPECT_EQ(result.residual_history.size(), 8U);  // one before each step, one after
  EXPECT_TRUE(result.converged);
}

TEST(Newton, ReportsFailureRatherThanReturningAWrongRoot) {
  // x^2 + 1 = 0 has no real root. Newton must say so through the residual, not
  // return the least-bad point as though it were a solution.
  const auto residual = [](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    Eigen::VectorXd r(1);
    r(0) = v(0) * v(0) + 1.0;
    return r;
  };
  const auto result = solve_newton(residual, Eigen::VectorXd::Constant(1, 1.0));
  EXPECT_FALSE(result.converged);
  EXPECT_GE(result.residual_norm, 1.0);
}

TEST(Newton, TheLineSearchRescuesABadInitialGuess) {
  // A steep exponential where an undamped full Newton step overshoots wildly.
  const auto residual = [](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    Eigen::VectorXd r(1);
    r(0) = std::exp(v(0)) - 2.0;  // root at ln 2
    return r;
  };
  // Starting far to the left, the derivative is tiny and an undamped step is
  // enormous — from x = -5 the full Newton step is nearly +300. Backtracking
  // keeps it from leaving the region entirely.
  //
  // There is a limit to this, and it is worth stating rather than discovering.
  // Farther into the exponential tail, even the shortest tested fraction can
  // overshoot: a finite backtracking budget does not establish global
  // convergence. The separate overflow-trial test checks recovery where a
  // shorter representable fraction does improve the residual.
  const auto result = solve_newton(residual, Eigen::VectorXd::Constant(1, -5.0));
  ASSERT_TRUE(result.converged) << "residual " << result.residual_norm;
  EXPECT_NEAR(result.solution(0), std::log(2.0), 1e-9);
}

TEST(Newton, ReportsTheJacobianConditionNumber) {
  // A deliberately ill-conditioned system: the second equation is nearly a
  // multiple of the first. It still has a root, but the root is poorly
  // determined, and the condition number is what says so.
  const auto residual = [](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    Eigen::VectorXd r(2);
    r(0) = v(0) + v(1) - 2.0;
    r(1) = v(0) + (1.0 + 1e-8) * v(1) - 2.0;
    return r;
  };
  const auto result = solve_newton(residual, Eigen::VectorXd::Zero(2));
  EXPECT_GT(result.jacobian_condition_number, 1e6)
      << "an ill-conditioned Jacobian was reported as well-conditioned";

  // A well-conditioned one reports a small number.
  const auto easy = [](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    Eigen::VectorXd r(2);
    r(0) = v(0) - 1.0;
    r(1) = v(1) - 2.0;
    return r;
  };
  const auto good = solve_newton(easy, Eigen::VectorXd::Zero(2));
  EXPECT_LT(good.jacobian_condition_number, 10.0);
}

TEST(Newton, SurvivesAResidualThatThrowsOutsideItsDomain) {
  // A step landing outside the model's domain is a step that was too long, not
  // a failure of the solve. The trim solver relies on this: a candidate below
  // the model's minimum airspeed throws, and backtracking must handle it.
  const auto residual = [](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    if (v(0) <= 0.0) {
      throw std::runtime_error("outside the domain");
    }
    Eigen::VectorXd r(1);
    r(0) = std::log(v(0)) - 1.0;  // root at e
    return r;
  };
  const auto result = solve_newton(residual, Eigen::VectorXd::Constant(1, 0.05));
  ASSERT_TRUE(result.converged) << "residual " << result.residual_norm;
  EXPECT_NEAR(result.solution(0), std::exp(1.0), 1e-8);
}

TEST(Newton, RejectsANonSquareSystem) {
  const auto residual = [](const Eigen::VectorXd&) -> Eigen::VectorXd {
    return Eigen::VectorXd::Zero(3);
  };
  EXPECT_THROW((void)solve_newton(residual, Eigen::VectorXd::Zero(2)), std::invalid_argument);
}

TEST(Newton, NonFiniteToleranceCannotLabelAConstantNonzeroResidualConverged) {
  const auto no_root = [](const Eigen::VectorXd&) -> Eigen::VectorXd {
    return Eigen::VectorXd::Ones(1);
  };
  for (const double bad : {0.0,
                           -1.0,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    NewtonOptions options;
    options.residual_tolerance = bad;
    EXPECT_THROW((void)solve_newton(no_root, Eigen::VectorXd::Zero(1), options),
                 std::invalid_argument);
  }
  const auto unresolved = solve_newton(no_root, Eigen::VectorXd::Zero(1));
  EXPECT_FALSE(unresolved.converged);
  EXPECT_DOUBLE_EQ(unresolved.residual_norm, 1.0);
}

TEST(Newton, ZeroJacobianPreservesIterationHistoryAndUsesResidualToDecideConvergence) {
  // For a constant residual the Jacobian is identically zero. No step can
  // improve the residual; its value alone distinguishes a root from stagnation.
  for (const double value : {0.0, 1.0}) {
    const auto constant = [value](const Eigen::VectorXd& x) -> Eigen::VectorXd {
      return Eigen::VectorXd::Constant(x.size(), value);
    };
    NewtonOptions options;
    options.iterations = 3;
    options.line_search_trials = 2;
    const Eigen::VectorXd guess = Eigen::VectorXd::Constant(2, 7.0);
    const auto result = solve_newton(constant, guess, options);
    EXPECT_EQ(result.converged, value == 0.0);
    EXPECT_TRUE((result.solution.array() == guess.array()).all());
    EXPECT_TRUE(std::isinf(result.jacobian_condition_number));
    ASSERT_EQ(result.residual_history.size(), 4U);
    for (const double norm : result.residual_history) {
      EXPECT_DOUBLE_EQ(norm, value * std::sqrt(2.0));
    }
  }
}

TEST(Newton, RejectsNonFiniteInitialGuessAndInitialResidual) {
  const auto identity = [](const Eigen::VectorXd& x) -> Eigen::VectorXd { return x; };
  for (const double bad :
       {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW((void)solve_newton(identity, Eigen::VectorXd::Constant(1, bad)),
                 std::invalid_argument);
    const auto invalid = [bad](const Eigen::VectorXd&) -> Eigen::VectorXd {
      return Eigen::VectorXd::Constant(1, bad);
    };
    EXPECT_THROW((void)solve_newton(invalid, Eigen::VectorXd::Zero(1)), std::runtime_error);
  }
}

TEST(Newton, BacktrackingRejectsOverflowingTrialsAndStillFindsTheAnalyticRoot) {
  const auto residual = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    return Eigen::VectorXd::Constant(1, std::exp(x(0)) - 2.0);
  };
  NewtonOptions options;
  options.line_search_trials = 16;
  // At -8, the full Newton step overflows exp; shorter fractions remain
  // representable. Rejected trials must not poison the accepted iterate.
  const auto result = solve_newton(residual, Eigen::VectorXd::Constant(1, -8.0), options);
  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.solution(0), std::log(2.0), 1e-9);
  EXPECT_EQ(result.residual_history.size(), static_cast<std::size_t>(options.iterations) + 1U);
}

TEST(Newton, RejectsResidualDimensionChangesDuringLineSearch) {
  const auto residual = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    if (x(0) > 0.5) {
      return Eigen::VectorXd::Zero(2);
    }
    return Eigen::VectorXd::Constant(1, x(0) - 1.0);
  };
  EXPECT_THROW((void)solve_newton(residual, Eigen::VectorXd::Zero(1)), std::runtime_error);
}

}  // namespace
