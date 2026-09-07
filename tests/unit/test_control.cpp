// SPDX-License-Identifier: Apache-2.0
// Laub (1979), Anderson and Moore (1990): independently derived scalar and
// double-integrator CARE solutions, state-feedback identities and return difference.
#include "galata/analyze/frequency_response.hpp"
#include "galata/synth/control.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {
Eigen::MatrixXd scalar(double value) {
  return Eigen::MatrixXd::Constant(1, 1, value);
}

TEST(Care, ScalarSolutionsMatchIndependentQuadraticRoots) {
  // A quadratic solved in closed form. The budget covers small dense Schur
  // arithmetic, independently of any observed output.
  constexpr double budget = 8192.0 * std::numeric_limits<double>::epsilon();
  for (double a : {-3.0, 0.0, 2.0}) {
    for (double b : {0.25, 1.0, 3.0}) {
      for (double q : {0.5, 2.0}) {
        for (double r : {0.5, 4.0}) {
          const auto result = galata::synth::solve_care(scalar(a), scalar(b), scalar(q), scalar(r));
          const double exact = (a + std::sqrt(a * a + b * b * q / r)) * r / (b * b);
          EXPECT_NEAR(result.x(0, 0), exact, budget * std::max(1.0, exact));
          EXPECT_NEAR(result.k(0, 0), b * exact / r, budget * std::max(1.0, b * exact / r));
          EXPECT_LE(result.relative_residual, result.residual_budget);
        }
      }
    }
  }
}

TEST(Care, DoubleIntegratorHasTheClosedFormStabilisingSolution) {
  Eigen::Matrix2d a;
  a << 0, 1, 0, 0;
  Eigen::Vector2d b;
  b << 0, 1;
  const auto solution = galata::synth::solve_care(a, b, Eigen::Matrix2d::Identity(), scalar(1));
  Eigen::Matrix2d exact;
  exact << std::sqrt(3.0), 1, 1, std::sqrt(3.0);
  EXPECT_LT((solution.x - exact).norm(), 8192.0 * std::numeric_limits<double>::epsilon());
}

TEST(Care, CrossCostIsIncludedInBothEquationAndGain) {
  const auto result =
      galata::synth::solve_care(scalar(1), scalar(2), scalar(3), scalar(4), scalar(1));
  const double exact = 0.5 + std::sqrt(3.0);
  EXPECT_NEAR(result.x(0, 0), exact, 8192.0 * std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(
      result.k(0, 0), (2.0 * exact + 1.0) / 4.0, 8192.0 * std::numeric_limits<double>::epsilon());
}

TEST(Care, CoupledComplexModesAndCrossCostRecoverAnAnalyticSolutionInDifferentCoordinates) {
  // An independent matrix construction: choose X=I, Abar=J-I with J skew,
  // and Qbar=BR^-1B'+2I. Then Abar'X+XAbar-XBR^-1B'X+Qbar is identically zero
  // and Abar-BR^-1B' is strictly dissipative. N completes the square. A dense
  // state-coordinate change must preserve this answer by Xz=T'XT, Kz=KT.
  Eigen::Matrix4d skew;
  skew << 0, 2, 0.3, 0, -2, 0, 0, 0.4, -0.3, 0, 0, 4, 0, -0.4, -4, 0;
  Eigen::Matrix<double, 4, 2> b;
  b << 1, 0, 0, 1, 1, 0.5, -0.25, 0.75;
  Eigen::Matrix<double, 4, 2> n;
  n << 0.1, 0.2, -0.2, 0.3, 0.25, -0.15, 0.05, 0.4;
  Eigen::Matrix2d r;
  r << 2, 0.3, 0.3, 1;
  const Eigen::LLT<Eigen::Matrix2d> factor(r);
  const Eigen::Matrix4d a = skew - Eigen::Matrix4d::Identity() + b * factor.solve(n.transpose());
  const Eigen::Matrix4d q = b * factor.solve(b.transpose()) + 2 * Eigen::Matrix4d::Identity()
                            + n * factor.solve(n.transpose());
  const Eigen::Matrix<double, 2, 4> exact_gain = factor.solve(b.transpose() + n.transpose());
  Eigen::Matrix4d transform;
  transform << 2, 0.2, 0, 0.1, 0, 0.5, 0.1, 0, 0, 0, 1.5, 0.2, 0, 0, 0, 3;
  const Eigen::FullPivLU<Eigen::Matrix4d> basis(transform);
  const auto solution = galata::synth::solve_care(basis.solve(a * transform),
                                                  basis.solve(b),
                                                  transform.transpose() * q * transform,
                                                  r,
                                                  transform.transpose() * n);
  const Eigen::Matrix4d exact = transform.transpose() * transform;
  constexpr double budget = 65536 * std::numeric_limits<double>::epsilon();
  EXPECT_LT((solution.x - exact).norm(), budget * exact.norm());
  EXPECT_LT((solution.k - exact_gain * transform).norm(), budget * (exact_gain * transform).norm());
  ASSERT_EQ(solution.closed_loop_eigenvalues.size(), 4U);
  for (const auto pole : solution.closed_loop_eigenvalues) {
    EXPECT_LT(pole.real(), 0);
  }
}

TEST(Care, StableUncontrolledAndUnobservedModesAreAllowed) {
  // The second state is neither controlled nor penalised, but is stable. Its
  // value function is exactly zero; rejecting all rank-deficient B/Q is wrong.
  Eigen::Matrix2d a = Eigen::Matrix2d::Zero();
  a.diagonal() << 1, -2;
  Eigen::Vector2d b;
  b << 1, 0;
  Eigen::Matrix2d q = Eigen::Matrix2d::Zero();
  q(0, 0) = 1;
  const auto result = galata::synth::solve_care(a, b, q, scalar(1));
  Eigen::Matrix2d exact = Eigen::Matrix2d::Zero();
  exact(0, 0) = 1 + std::sqrt(2.0);
  EXPECT_LT((result.x - exact).norm(), 8192 * std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result.k(0, 1), 0, 8192 * std::numeric_limits<double>::epsilon());
}

TEST(Care, RejectsMalformedDimensionsNonfiniteCostsAndOverflowedTransformations) {
  EXPECT_THROW((void)galata::synth::solve_care(
                   Eigen::Matrix2d::Identity(), scalar(1), Eigen::Matrix2d::Identity(), scalar(1)),
               std::invalid_argument);
  EXPECT_THROW((void)galata::synth::solve_care(
                   scalar(1), scalar(1), scalar(1), scalar(1), Eigen::Matrix2d::Identity()),
               std::invalid_argument);
  EXPECT_THROW(
      (void)galata::synth::solve_care(
          scalar(1), scalar(1), scalar(std::numeric_limits<double>::quiet_NaN()), scalar(1)),
      std::invalid_argument);
  Eigen::Matrix2d nonsymmetric;
  nonsymmetric << 1, 0.1, 0, 1;
  EXPECT_THROW((void)galata::synth::solve_care(Eigen::Matrix2d::Identity(),
                                               Eigen::Matrix2d::Identity(),
                                               nonsymmetric,
                                               Eigen::Matrix2d::Identity()),
               std::invalid_argument);
  EXPECT_ANY_THROW((void)galata::synth::solve_care(
      scalar(1), scalar(1), scalar(1), scalar(1e-308), scalar(1e308)));
}

TEST(Care, RejectsInvalidCostsAndUnstabilisableOrUnseparatedProblems) {
  EXPECT_THROW((void)galata::synth::solve_care(scalar(1), scalar(1), scalar(1), scalar(0)),
               std::invalid_argument);
  EXPECT_THROW((void)galata::synth::solve_care(scalar(1), scalar(1), scalar(-1), scalar(1)),
               std::invalid_argument);
  EXPECT_THROW(
      (void)galata::synth::solve_care(scalar(1), scalar(1), scalar(1), scalar(1), scalar(2)),
      std::invalid_argument);
  EXPECT_THROW((void)galata::synth::solve_care(scalar(1), scalar(0), scalar(1), scalar(1)),
               std::runtime_error);
  EXPECT_THROW((void)galata::synth::solve_care(scalar(0), scalar(1), scalar(0), scalar(1)),
               std::runtime_error);
  EXPECT_THROW(
      (void)galata::synth::solve_care(
          scalar(std::numeric_limits<double>::infinity()), scalar(1), scalar(1), scalar(1)),
      std::invalid_argument);
}

TEST(Care, RepeatedAndCoupledModesSatisfyTheReturnDifferenceEquality) {
  for (int count : {2, 3, 5, 8}) {
    Eigen::MatrixXd a = -Eigen::MatrixXd::Identity(count, count);
    for (int i = 0; i + 1 < count; ++i) {
      a(i, i + 1) = 0.5;
    }
    const Eigen::MatrixXd b = Eigen::MatrixXd::Identity(count, count);
    const Eigen::MatrixXd q = Eigen::MatrixXd::Identity(count, count);
    const auto result = galata::synth::solve_care(a, b, q, q);
    for (double w : {0.01, 0.3, 1.0, 12.0}) {
      const Eigen::MatrixXcd resolvent =
          (std::complex<double>(0, w) * Eigen::MatrixXcd::Identity(count, count)
           - a.cast<std::complex<double>>())
              .partialPivLu()
              .solve(b.cast<std::complex<double>>());
      const Eigen::MatrixXcd identity = Eigen::MatrixXcd::Identity(count, count);
      const Eigen::MatrixXcd sum = identity + result.k.cast<std::complex<double>>() * resolvent;
      const Eigen::MatrixXcd expected =
          identity + resolvent.adjoint() * q.cast<std::complex<double>>() * resolvent;
      EXPECT_LT((sum.adjoint() * sum - expected).norm(),
                32768.0 * count * std::numeric_limits<double>::epsilon() * expected.norm());
    }
    const auto repeat = galata::synth::solve_care(a, b, q, q);
    EXPECT_TRUE((result.k.array() == repeat.k.array()).all());
  }
}

TEST(Lqr, ClosedLoopOutputsAccountForPlantFeedthrough) {
  galata::model::LinearSystem plant;
  plant.a = scalar(1);
  plant.b = scalar(1);
  plant.c = scalar(2);
  plant.d = scalar(3);
  plant.state_names = {"x"};
  plant.input_names = {"u"};
  plant.output_names = {"y"};
  const auto result = galata::synth::design_lqr(plant, scalar(1), scalar(1));
  EXPECT_DOUBLE_EQ(result.closed_loop.c(0, 0), 2 - 3 * result.riccati.k(0, 0));
  EXPECT_DOUBLE_EQ(result.broken_loop.c(0, 0), result.riccati.k(0, 0));
  EXPECT_DOUBLE_EQ(result.broken_loop.d(0, 0), 0);
}

TEST(Lqr, UnpenalisedUnstableModesCannotBeCalledAnOptimalDesign) {
  galata::model::LinearSystem plant;
  plant.a = scalar(1);
  plant.b = scalar(1);
  plant.state_names = {"x"};
  plant.input_names = {"u"};
  // The stabilising CARE root X=2 exists, but u=0 has cost zero and lets the
  // unobserved state diverge. CARE and an unconstrained optimal LQR design have
  // different preconditions; design_lqr requires detectability.
  EXPECT_NEAR(galata::synth::solve_care(plant.a, plant.b, scalar(0), scalar(1)).x(0, 0),
              2,
              8192 * std::numeric_limits<double>::epsilon());
  EXPECT_THROW((void)galata::synth::design_lqr(plant, scalar(0), scalar(1)), std::invalid_argument);
  // The cost completion, not Q alone, determines detectability.
  plant.a = scalar(2);
  EXPECT_THROW((void)galata::synth::design_lqr(plant, scalar(1), scalar(1), scalar(1)),
               std::invalid_argument);
  plant.a = scalar(-1);
  const auto stable = galata::synth::design_lqr(plant, scalar(0), scalar(1));
  EXPECT_NEAR(stable.riccati.k(0, 0), 0, 8192 * std::numeric_limits<double>::epsilon());
}

TEST(Lqr, DetectabilityAllowsIndirectlyObservedUnstableAndStableHiddenModes) {
  galata::model::LinearSystem plant;
  plant.a = Eigen::Matrix3d::Zero();
  plant.a << 1, 1, 0, 0, 2, 0, 0, 0, -3;
  plant.b = Eigen::Matrix3d::Identity();
  plant.state_names = {"x", "v", "hidden"};
  plant.input_names = {"ux", "uv", "uh"};
  Eigen::Matrix3d q = Eigen::Matrix3d::Zero();
  q(0, 0) = 1;
  const auto design = galata::synth::design_lqr(plant, q, Eigen::Matrix3d::Identity());
  for (const auto pole : design.riccati.closed_loop_eigenvalues) {
    EXPECT_LT(pole.real(), 0);
  }
}

TEST(Interconnection, CascadeAndFeedbackMatchIndependentScalarTransferFunctionsWithFeedthrough) {
  galata::model::LinearSystem first;
  first.a = scalar(-1);
  first.b = scalar(1);
  first.c = scalar(3);
  first.d = scalar(2);
  first.state_names = {"x"};
  first.input_names = {"u"};
  first.output_names = {"y"};
  auto second = first;
  second.a = scalar(-2);
  second.c = scalar(0.7);
  second.d = scalar(-0.2);
  const auto cascade = galata::synth::series(first, second);
  const auto closed = galata::synth::negative_feedback(cascade);
  const std::vector<double> frequencies = {0.01, 0.5, 3, 100};
  const auto open_response = galata::analyze::frequency_response(cascade, frequencies);
  const auto closed_response = galata::analyze::frequency_response(closed, frequencies);
  for (std::size_t i = 0; i < frequencies.size(); ++i) {
    const std::complex<double> s(0, frequencies[i]);
    const auto loop = (2.0 + 3.0 / (s + 1.0)) * (-0.2 + 0.7 / (s + 2.0));
    EXPECT_NEAR(std::abs(open_response.response[i](0, 0) - loop), 0, 1e-12);
    EXPECT_NEAR(std::abs(closed_response.response[i](0, 0) - loop / (1.0 + loop)), 0, 1e-12);
  }
}

TEST(Interconnection, MatrixChannelOrderAndAlgebraicFeedthroughArePreserved) {
  galata::model::LinearSystem first;
  first.a = Eigen::Matrix2d::Zero();
  first.a.diagonal() << -1, -2;
  first.b = Eigen::Matrix2d::Identity();
  first.c = Eigen::Matrix2d(2, 2);
  first.c << 1, 0.5, -0.3, 2;
  first.d = Eigen::Matrix2d(2, 2);
  first.d << 0.2, 0.3, -0.1, 0.4;
  first.state_names = {"x", "v"};
  first.input_names = {"u1", "u2"};
  first.output_names = {"y1", "y2"};
  auto second = first;
  second.a.diagonal() << -3, -4;
  second.c << 0.5, -0.2, 0.7, 1.1;
  second.d << 0.1, 0.2, 0.3, -0.1;
  const auto cascade = galata::synth::series(first, second);
  const auto closed = galata::synth::negative_feedback(cascade);
  const std::vector<double> frequencies = {0.01, 0.5, 3, 100};
  const auto open_response = galata::analyze::frequency_response(cascade, frequencies);
  const auto closed_response = galata::analyze::frequency_response(closed, frequencies);
  for (std::size_t i = 0; i < frequencies.size(); ++i) {
    const std::complex<double> s(0, frequencies[i]);
    Eigen::Vector2cd poles1, poles2;
    poles1 << 1.0 / (s + 1.0), 1.0 / (s + 2.0);
    poles2 << 1.0 / (s + 3.0), 1.0 / (s + 4.0);
    const Eigen::Matrix2cd g1 = first.d.cast<std::complex<double>>()
                                + first.c.cast<std::complex<double>>() * poles1.asDiagonal();
    const Eigen::Matrix2cd g2 = second.d.cast<std::complex<double>>()
                                + second.c.cast<std::complex<double>>() * poles2.asDiagonal();
    const Eigen::Matrix2cd loop = g2 * g1;
    const Eigen::Matrix2cd expected =
        (Eigen::Matrix2cd::Identity() + loop).partialPivLu().solve(loop);
    EXPECT_LT((open_response.response[i] - loop).norm(), 1e-12);
    EXPECT_LT((closed_response.response[i] - expected).norm(), 1e-12);
  }
  auto singular = first;
  singular.d = -Eigen::Matrix2d::Identity();
  EXPECT_THROW((void)galata::synth::negative_feedback(singular), std::invalid_argument);
  singular.d(1, 1) = 1;
  singular.d(0, 0) = -1 + std::numeric_limits<double>::epsilon();
  EXPECT_THROW((void)galata::synth::negative_feedback(singular), std::invalid_argument);
  auto scalar_controller = galata::synth::filtered_pid(1, 0, 0, 1);
  EXPECT_THROW((void)galata::synth::series(first, scalar_controller), std::invalid_argument);
  first.c = Eigen::MatrixXd::Ones(1, 2);
  first.d = Eigen::MatrixXd::Zero(1, 2);
  first.output_names = {"y"};
  EXPECT_THROW((void)galata::synth::negative_feedback(first), std::invalid_argument);
}

TEST(FilteredPid, FrequencyResponseMatchesTheDefinedTransferFunction) {
  for (double ki : {0.0, 0.7}) {
    for (double kd : {0.0, 0.2}) {
      const auto controller = galata::synth::filtered_pid(2, ki, kd, 0.03);
      const auto response = galata::analyze::frequency_response(controller, {0.01, 1, 30, 1000});
      for (std::size_t i = 0; i < response.response.size(); ++i) {
        const std::complex<double> s(0, response.frequencies_rad_s[i]);
        const auto expected = 2.0 + ki / s + kd * s / (0.03 * s + 1.0);
        EXPECT_LT(
            std::abs(response.response[i](0, 0) - expected),
            1024 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(expected)));
      }
    }
  }
  EXPECT_THROW((void)galata::synth::filtered_pid(1, 1, 1, 0), std::invalid_argument);
}
}  // namespace
