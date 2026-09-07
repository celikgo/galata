// SPDX-License-Identifier: Apache-2.0
//
// Central-difference Jacobians against analytically known ones.
//
// This is charter validation case 4 in miniature: "for a model with
// analytically known Jacobians, agreement to the documented truncation error".
// Here the models are small enough that the Jacobian can be written down, so
// the comparison is exact rather than bounded by a reference document's
// printing precision.

#include "galata/numerics/jacobian.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using galata::numerics::central_difference_jacobian;
using galata::numerics::JacobianOptions;

TEST(Jacobian, LinearFunctionIsDifferentiatedToTheCancellationLimit) {
  // f(x) = A x. The Jacobian is A everywhere and the TRUNCATION error is
  // exactly zero, because a central difference is exact on a linear function.
  //
  // It does NOT follow that the answer is exact. The floor is cancellation:
  // f(x+h) and f(x-h) are both of order |A x| and differ by only 2 |A| h, so
  // subtracting them throws away about log10(|f| / |f' h|) digits. The error
  // is about eps |f| / h, which here is 2e-16 * 5 / 3e-6 ~ 4e-10.
  //
  // This test asserts that bound rather than machine precision. Asserting
  // machine precision would be asserting something false about floating-point
  // arithmetic, and the failure would look like a bug in the difference
  // quotient when it is a property of the method.
  Eigen::MatrixXd a(3, 3);
  a << 1.5, -2.0, 0.25, 0.0, 3.0, -1.0, 4.0, 0.5, 2.5;

  const auto f = [&a](const Eigen::VectorXd& x) -> Eigen::VectorXd { return a * x; };
  Eigen::VectorXd x(3);
  x << 1.0, -2.0, 3.0;

  const auto jacobian = central_difference_jacobian(f, x);

  const double magnitude = (a * x).cwiseAbs().maxCoeff();
  const double smallest_step = jacobian.steps.minCoeff();
  const double cancellation_bound =
      10.0 * std::numeric_limits<double>::epsilon() * magnitude / smallest_step;

  EXPECT_LT((jacobian.value - a).cwiseAbs().maxCoeff(), cancellation_bound)
      << "computed:\n"
      << jacobian.value << "\nexpected:\n"
      << a << "\ncancellation bound: " << cancellation_bound;

  // Truncation, as distinct from cancellation, really is zero for a linear
  // function, and the Richardson estimate should say so.
  EXPECT_LT(jacobian.worst_relative_truncation, 1e-9);
}

TEST(Jacobian, QuadraticFunctionMatchesItsAnalyticJacobian) {
  // f(x, y) = [x^2 y, sin(x) + y^3]
  // J = [[2xy, x^2], [cos(x), 3y^2]]
  const auto f = [](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    Eigen::VectorXd out(2);
    out(0) = v(0) * v(0) * v(1);
    out(1) = std::sin(v(0)) + v(1) * v(1) * v(1);
    return out;
  };

  for (const double x : {-1.7, -0.3, 0.5, 2.4}) {
    for (const double y : {-2.1, 0.4, 1.9}) {
      Eigen::VectorXd point(2);
      point << x, y;

      Eigen::MatrixXd analytic(2, 2);
      analytic << 2.0 * x * y, x * x, std::cos(x), 3.0 * y * y;

      const auto jacobian = central_difference_jacobian(f, point);
      const double scale = analytic.cwiseAbs().maxCoeff() + 1.0;
      EXPECT_TRUE(jacobian.value.isApprox(analytic, 1e-9 * scale))
          << "at (" << x << ", " << y << ")\ncomputed:\n"
          << jacobian.value << "\nanalytic:\n"
          << analytic;
    }
  }
}

TEST(Jacobian, TruncationEstimateBoundsTheActualError) {
  // The estimate is only useful if it is not optimistic. On a function with a
  // large third derivative — where central differences are at their worst —
  // the reported estimate must still cover the real error.
  const auto f = [](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    Eigen::VectorXd out(1);
    out(0) = std::exp(3.0 * v(0));  // third derivative is 27x the value
    return out;
  };

  Eigen::VectorXd x(1);
  x << 0.8;
  const double analytic = 3.0 * std::exp(3.0 * 0.8);

  // A deliberately coarse step, so truncation dominates and the estimate has
  // something real to measure.
  JacobianOptions options;
  options.relative_step = 1e-3;
  options.absolute_step = 1e-3;

  const auto jacobian = central_difference_jacobian(f, x, options);
  const double actual_error = std::fabs(jacobian.value(0, 0) - analytic);
  const double estimated = jacobian.truncation_estimate(0, 0);

  EXPECT_GT(estimated, 0.0);
  // Richardson estimates the error of the reported (half-step) Jacobian. It
  // should be the right order and must not understate.
  EXPECT_GE(estimated * 3.0, actual_error) << "the truncation estimate " << estimated
                                           << " understates the actual error " << actual_error;
  EXPECT_LT(estimated, 100.0 * actual_error + 1e-12);
}

TEST(Jacobian, StepSizeFollowsTheDocumentedRule) {
  const auto f = [](const Eigen::VectorXd& v) -> Eigen::VectorXd { return v; };
  Eigen::VectorXd x(3);
  x << 1000.0, 1.0, 0.0;

  JacobianOptions options;
  options.relative_step = 1e-6;
  options.absolute_step = 1e-9;
  options.estimate_truncation_error = false;

  const auto jacobian = central_difference_jacobian(f, x, options);
  EXPECT_DOUBLE_EQ(jacobian.steps(0), 1e-6 * 1000.0);  // relative dominates
  EXPECT_DOUBLE_EQ(jacobian.steps(1), 1e-6 * 1.0);
  EXPECT_DOUBLE_EQ(jacobian.steps(2), 1e-9);  // the floor catches a zero state
}

TEST(Jacobian, PerComponentFloorsAreHonoured) {
  // An aircraft state mixes metres, metres per second and radians. One
  // absolute floor cannot serve all three, which is what this option is for.
  const auto f = [](const Eigen::VectorXd& v) -> Eigen::VectorXd { return v; };
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);

  JacobianOptions options;
  options.relative_step = 1e-6;
  options.absolute_step_per_component = Eigen::VectorXd(3);
  options.absolute_step_per_component << 1e-2, 1e-5, 1e-9;
  options.estimate_truncation_error = false;

  const auto jacobian = central_difference_jacobian(f, x, options);
  EXPECT_DOUBLE_EQ(jacobian.steps(0), 1e-2);
  EXPECT_DOUBLE_EQ(jacobian.steps(1), 1e-5);
  EXPECT_DOUBLE_EQ(jacobian.steps(2), 1e-9);
}

TEST(Jacobian, RefusesAStepThatVanishes) {
  // A relative step with no floor, at x = 0, perturbs by nothing. Dividing by
  // that gives infinity; refusing is better.
  const auto f = [](const Eigen::VectorXd& v) -> Eigen::VectorXd { return v; };
  Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
  JacobianOptions options;
  options.relative_step = 1e-6;
  options.absolute_step = 0.0;
  EXPECT_THROW((void)central_difference_jacobian(f, x, options), std::runtime_error);
}

TEST(Jacobian, IsRepeatableToTheBit) {
  const auto f = [](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    Eigen::VectorXd out(2);
    out(0) = std::sin(v(0)) * v(1);
    out(1) = std::exp(-v(0)) + v(1) * v(1);
    return out;
  };
  Eigen::VectorXd x(2);
  x << 0.4, -1.1;

  const auto first = central_difference_jacobian(f, x);
  const auto second = central_difference_jacobian(f, x);
  for (Eigen::Index i = 0; i < first.value.rows(); ++i) {
    for (Eigen::Index j = 0; j < first.value.cols(); ++j) {
      EXPECT_EQ(first.value(i, j), second.value(i, j));
    }
  }
}

TEST(Jacobian, RejectsNonFinitePointsAndStepOptions) {
  const auto identity = [](const Eigen::VectorXd& x) -> Eigen::VectorXd { return x; };
  for (const double bad :
       {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW((void)central_difference_jacobian(identity, Eigen::VectorXd::Constant(1, bad)),
                 std::invalid_argument);
  }
  for (const double bad :
       {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    JacobianOptions options;
    options.relative_step = bad;
    EXPECT_THROW((void)central_difference_jacobian(identity, Eigen::VectorXd::Ones(1), options),
                 std::invalid_argument);
    options = {};
    options.absolute_step = bad;
    EXPECT_THROW((void)central_difference_jacobian(identity, Eigen::VectorXd::Ones(1), options),
                 std::invalid_argument);
    options = {};
    options.absolute_step_per_component = Eigen::VectorXd::Constant(1, bad);
    EXPECT_THROW((void)central_difference_jacobian(identity, Eigen::VectorXd::Ones(1), options),
                 std::invalid_argument);
  }
}

TEST(Jacobian, RejectsNonFiniteOutputsAndUnrepresentablePerturbations) {
  const auto identity = [](const Eigen::VectorXd& x) -> Eigen::VectorXd { return x; };
  JacobianOptions options;
  options.relative_step = 2.0;
  EXPECT_THROW(
      (void)central_difference_jacobian(
          identity, Eigen::VectorXd::Constant(1, std::numeric_limits<double>::max()), options),
      std::runtime_error);
  for (const double bad :
       {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    const auto invalid = [bad](const Eigen::VectorXd&) -> Eigen::VectorXd {
      return Eigen::VectorXd::Constant(1, bad);
    };
    EXPECT_THROW((void)central_difference_jacobian(invalid, Eigen::VectorXd::Zero(1)),
                 std::runtime_error);
  }
}

TEST(Jacobian, RejectsDimensionChangesAcrossColumnsAndRichardsonEvaluations) {
  const auto changing_column = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    return Eigen::VectorXd::Zero(x(0) != 0.0 ? 1 : 2);
  };
  EXPECT_THROW((void)central_difference_jacobian(changing_column, Eigen::VectorXd::Zero(2)),
               std::runtime_error);
  JacobianOptions options;
  options.relative_step = 0.0;
  options.absolute_step = 0.01;
  const auto changing_step = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    return Eigen::VectorXd::Zero(std::abs(x(0)) > 0.007 ? 1 : 2);
  };
  EXPECT_THROW((void)central_difference_jacobian(changing_step, Eigen::VectorXd::Zero(1), options),
               std::runtime_error);
}

TEST(Jacobian, RejectsOverflowingErrorSummariesEvenWhenMatrixEntriesAreFinite) {
  // Both finite-difference slopes and their absolute discrepancy are finite,
  // but their ratio is beyond binary64. A finite matrix alone is insufficient
  // evidence that the complete numerical result can be reported safely.
  const auto separated_scales = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    const double slope = std::abs(x(0)) > 0.5 ? 5e307 : 5e-309;
    return Eigen::VectorXd::Constant(1, slope * x(0));
  };
  JacobianOptions options;
  options.relative_step = 0.0;
  options.absolute_step = 1.0;
  EXPECT_THROW(
      (void)central_difference_jacobian(separated_scales, Eigen::VectorXd::Zero(1), options),
      std::runtime_error);
}

}  // namespace
