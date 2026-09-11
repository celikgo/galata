// SPDX-License-Identifier: Apache-2.0
//
// Sampled LQR: the discrete Riccati equation, the exact cost discretisation,
// and the cases F14's acceptance names — stabilisable, unstabilisable,
// undetectable and invalid-cost.
//
// TWO INDEPENDENT REFERENCES, NEITHER OF WHICH IS THE ROUTINE UNDER TEST.
//
//  1. A CLOSED FORM for the scalar problem. For a = 2, b = 1, q = 1, r = 1 the
//     DARE reduces to x^2 - 4x - 1 = 0, so x = 2 + sqrt(5) exactly, the gain is
//     (1 + sqrt(5)) / 2 and the closed-loop pole is (3 - sqrt(5)) / 2. Written
//     here as those expressions rather than as decimals.
//
//  2. VALUE ITERATION for problems with no closed form. Iterating the Riccati
//     DIFFERENCE equation from X = 0 converges to the same stabilising solution
//     by a completely different route: no symplectic matrix, no Schur
//     decomposition, no deflating subspace. Where the two agree, they agree
//     across two algorithms.
//
// AND ONE FOR THE COST. With A = 0 and B = 1 the held trajectory is
// x(s) = x0 + s u, so the interval cost integrates by hand to
//
//     Qd = q T,     Nd = q T^2 / 2,     Rd = q T^3 / 3 + r T
//
// which is the test that matters most in this file: it shows the hold produces
// a CROSS TERM out of a cost that had none, and pins its exact size.
//
// MIXED TIME DOMAINS. That `DiscreteLinearSystem` cannot be handed to a
// continuous routine is a property of the type system, not something a runtime
// test can observe — there is no expression to write, because it would not
// compile. What is checked here is the other half: that a discrete model
// without a sample time does not exist, in `test_discrete_system.cpp`.

#include "galata/synth/discrete_control.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace {

using galata::model::LinearSystem;
using galata::synth::design_sampled_lqr;
using galata::synth::discretize_cost;
using galata::synth::solve_dare;

// THE INDEPENDENT SOLVER. Riccati value iteration: X <- A'XA + Q - (A'XB + N)
// (R + B'XB)^-1 (B'XA + N'), from X = 0. A fixed iteration count, per ADR-0004,
// with no convergence test — for these small well-conditioned problems the
// count is far past where the iterate stops moving in double precision.
Eigen::MatrixXd dare_by_value_iteration(const Eigen::MatrixXd& a,
                                        const Eigen::MatrixXd& b,
                                        const Eigen::MatrixXd& q,
                                        const Eigen::MatrixXd& r,
                                        const Eigen::MatrixXd& n,
                                        int iterations = 20000) {
  Eigen::MatrixXd x = Eigen::MatrixXd::Zero(a.rows(), a.rows());
  for (int i = 0; i < iterations; ++i) {
    const Eigen::MatrixXd weighted = r + b.transpose() * x * b;
    const Eigen::MatrixXd coupling = a.transpose() * x * b + n;
    x = a.transpose() * x * a + q - coupling * weighted.llt().solve(coupling.transpose());
    x = 0.5 * (x + x.transpose()).eval();
  }
  return x;
}

LinearSystem double_integrator() {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(2, 2);
  system.a(0, 1) = 1.0;
  system.b = Eigen::MatrixXd::Zero(2, 1);
  system.b(1, 0) = 1.0;
  system.state_names = {"position_m", "velocity_m_s"};
  system.input_names = {"acceleration_m_s2"};
  return system;
}

}  // namespace

TEST(DiscreteControl, TheScalarDareMatchesItsClosedFormSolution) {
  const Eigen::MatrixXd a = Eigen::MatrixXd::Constant(1, 1, 2.0);
  const Eigen::MatrixXd b = Eigen::MatrixXd::Constant(1, 1, 1.0);
  const Eigen::MatrixXd q = Eigen::MatrixXd::Constant(1, 1, 1.0);
  const Eigen::MatrixXd r = Eigen::MatrixXd::Constant(1, 1, 1.0);

  const auto solution = solve_dare(a, b, q, r);

  const double root_five = std::sqrt(5.0);
  EXPECT_NEAR(solution.x(0, 0), 2.0 + root_five, 1e-12) << "x solves x^2 - 4x - 1 = 0";
  EXPECT_NEAR(solution.k(0, 0), 0.5 * (1.0 + root_five), 1e-12)
      << "K = (R + B'XB)^-1 B'XA, which for this problem is the golden ratio";
  EXPECT_NEAR(solution.spectral_radius, 0.5 * (3.0 - root_five), 1e-12)
      << "and the closed-loop pole lands inside the unit circle";
  EXPECT_LT(solution.spectral_radius, 1.0);
  EXPECT_LE(solution.relative_residual, solution.residual_budget)
      << "residual " << solution.relative_residual << " against budget "
      << solution.residual_budget;
}

// The scalar case cannot distinguish a correct MIMO implementation from several
// wrong ones. This one has no closed form, so the reference is the other
// algorithm.
TEST(DiscreteControl, AMultivariableDareAgreesWithValueIteration) {
  Eigen::MatrixXd a(3, 3);
  a << 1.05, 0.10, 0.00, 0.00, 0.90, 0.25, 0.10, 0.00, 1.10;
  Eigen::MatrixXd b(3, 2);
  b << 1.0, 0.0, 0.0, 0.5, 0.2, 1.0;
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(3, 3);
  q.diagonal() << 2.0, 1.0, 4.0;
  Eigen::MatrixXd r = Eigen::MatrixXd::Zero(2, 2);
  r.diagonal() << 1.0, 3.0;

  const auto solution = solve_dare(a, b, q, r);
  const Eigen::MatrixXd reference =
      dare_by_value_iteration(a, b, q, r, Eigen::MatrixXd::Zero(3, 2));

  const double relative = (solution.x - reference).norm() / std::max(reference.norm(), 1e-300);
  EXPECT_LT(relative, 1e-9) << "symplectic:\n" << solution.x << "\nvalue iteration:\n" << reference;
  EXPECT_LT(solution.spectral_radius, 1.0) << "two unstable modes must be pulled inside";
  EXPECT_LE(solution.relative_residual, solution.residual_budget);
}

// A CROSS TERM IS NOT AN OPTIONAL EXTRA. The discretised cost always has one,
// so the solver's cross-term path is the one the sampled design actually uses,
// and it is checked against value iteration on the same posed problem.
TEST(DiscreteControl, ADareWithACrossTermAgreesWithValueIteration) {
  Eigen::MatrixXd a(2, 2);
  a << 1.02, 0.30, 0.00, 0.95;
  Eigen::MatrixXd b(2, 1);
  b << 0.05, 0.40;
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2, 2);
  q.diagonal() << 3.0, 1.0;
  const Eigen::MatrixXd r = Eigen::MatrixXd::Constant(1, 1, 2.0);
  Eigen::MatrixXd n(2, 1);
  n << 0.20, -0.10;

  // The block cost must stay positive semidefinite for the problem to be posed;
  // asserted here so a failure below is about the solver, not the fixture.
  Eigen::MatrixXd block(3, 3);
  block << q, n, n.transpose(), r;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(block);
  ASSERT_GT(spectrum.eigenvalues().minCoeff(), 0.0) << "fixture block cost must be definite";

  const auto solution = solve_dare(a, b, q, r, n);
  const Eigen::MatrixXd reference = dare_by_value_iteration(a, b, q, r, n);
  const double relative = (solution.x - reference).norm() / std::max(reference.norm(), 1e-300);
  EXPECT_LT(relative, 1e-9) << "the completing-the-square reduction must give the same X";
  EXPECT_LT(solution.spectral_radius, 1.0);
}

// ============================================================================
// THE EXACT COST DISCRETISATION, AND THE CROSS TERM THE HOLD CREATES
// ============================================================================

TEST(DiscreteControl, TheDiscretisedCostMatchesTheHandIntegratedInterval) {
  // A = 0, B = 1: the held trajectory is x(s) = x0 + s u, and the interval cost
  // integrates in closed form.
  LinearSystem plant;
  plant.a = Eigen::MatrixXd::Zero(1, 1);
  plant.b = Eigen::MatrixXd::Constant(1, 1, 1.0);
  plant.state_names = {"x"};
  plant.input_names = {"u"};

  const double q = 5.0;
  const double r = 2.0;
  const double interval = 0.2;  // s
  const auto cost = discretize_cost(
      plant, Eigen::MatrixXd::Constant(1, 1, q), Eigen::MatrixXd::Constant(1, 1, r), {}, interval);

  EXPECT_NEAR(cost.q(0, 0), q * interval, 1e-14) << "Qd = q T";
  EXPECT_NEAR(cost.n(0, 0), 0.5 * q * interval * interval, 1e-14)
      << "Nd = q T^2 / 2 — A CROSS TERM OUT OF A COST THAT HAD NONE. This is the term a "
         "naive port drops, and dropping it minimises a different objective.";
  EXPECT_NEAR(cost.r(0, 0), q * interval * interval * interval / 3.0 + r * interval, 1e-14)
      << "Rd = q T^3 / 3 + r T";
  EXPECT_GT(std::abs(cost.n(0, 0)), 0.0) << "the cross term is not zero";
  EXPECT_LT(cost.symmetry_defect, 1e-12);
  EXPECT_GT(cost.minimum_block_eigenvalue, -1e-12) << "the integral stays semidefinite";
  EXPECT_NE(cost.assumptions.find("cross term"), std::string::npos)
      << "the quotable sentence warns about it: " << cost.assumptions;
}

// A COST INTEGRATED OVER A SHORT INTERVAL APPROACHES Q T, and the leading error
// is the T^2 cross term above. Checking the limit separately makes the exact
// result legible: it is not merely "close to Q T", it is Q T plus terms whose
// size is known.
TEST(DiscreteControl, TheDiscretisedCostApproachesTheRectangleRuleAsTheIntervalShrinks) {
  const auto plant = double_integrator();
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2, 2);
  q.diagonal() << 4.0, 1.0;
  const Eigen::MatrixXd r = Eigen::MatrixXd::Constant(1, 1, 0.5);

  double previous = std::numeric_limits<double>::infinity();
  for (const double interval : {0.1, 0.01, 0.001}) {
    const auto cost = discretize_cost(plant, q, r, {}, interval);
    const double error = (cost.q / interval - q).norm() / q.norm();
    EXPECT_LT(error, previous) << "the relative gap to Q must shrink with the interval";
    previous = error;
    // And the cross term shrinks like T^2 relative to Qd's T, so its relative
    // weight falls linearly — the reason a fast enough loop can get away with
    // ignoring it, and the reason a slow one cannot.
    EXPECT_LT(cost.n.norm() / cost.q.norm(), interval)
        << "at T = " << interval << " the cross term's relative weight must be below T";
  }
}

// ============================================================================
// THE WHOLE SAMPLED PATH
// ============================================================================

TEST(DiscreteControl, TheSampledDesignStabilisesTheDiscretisedPlant) {
  const auto plant = double_integrator();
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2, 2);
  q.diagonal() << 10.0, 1.0;
  const Eigen::MatrixXd r = Eigen::MatrixXd::Constant(1, 1, 0.1);
  const double interval = 0.02;  // s

  const auto design = design_sampled_lqr(plant, q, r, {}, interval);

  EXPECT_DOUBLE_EQ(design.sample_time_s, interval);
  EXPECT_DOUBLE_EQ(design.closed_loop.sample_time_s, interval)
      << "the closed loop carries the sample time it was designed at";
  EXPECT_LT(design.riccati.spectral_radius, 1.0)
      << "every sampled pole must be inside the unit circle, radius "
      << design.riccati.spectral_radius;
  EXPECT_GT(design.riccati.symplectic_separation, 0.0)
      << "and the spectrum must be separated from the circle, not merely on the right side";
  EXPECT_LE(design.riccati.relative_residual, design.riccati.residual_budget);

  // The discretised cost is the one that was solved, not the continuous one.
  EXPECT_GT(design.cost.n.norm(), 0.0) << "the hold's cross term reached the solver";
  EXPECT_EQ(design.continuous_n.norm(), 0.0) << "even though the caller declared none";

  // The closed loop is autonomous: its input authority is spent on feedback.
  EXPECT_EQ(design.closed_loop.b.norm(), 0.0);
  EXPECT_EQ(design.closed_loop.state_names, plant.state_names);

  // A - B K, recomputed here rather than read back from the design.
  const Eigen::MatrixXd expected =
      design.discretisation.system.a - design.discretisation.system.b * design.riccati.k;
  EXPECT_LT((design.closed_loop.a - expected).cwiseAbs().maxCoeff(), 1e-15);
}

// SAMPLED POLES BELONG TO ONE SAMPLE RATE. Designing the same continuous
// problem at two rates gives two different discrete pole sets, and the
// invariant that survives is the CONTINUOUS equivalent: log(lambda) / T. That is
// the numerical qualification F14 asks for beside the unit-circle test.
TEST(DiscreteControl, PolesFromTwoSampleRatesAgreeOnlyAfterMappingBackThroughTheRate) {
  const auto plant = double_integrator();
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2, 2);
  q.diagonal() << 10.0, 1.0;
  const Eigen::MatrixXd r = Eigen::MatrixXd::Constant(1, 1, 0.1);

  const auto fast = design_sampled_lqr(plant, q, r, {}, 0.002);
  const auto slow = design_sampled_lqr(plant, q, r, {}, 0.02);

  ASSERT_EQ(fast.riccati.closed_loop_eigenvalues.size(), 2u);
  ASSERT_EQ(slow.riccati.closed_loop_eigenvalues.size(), 2u);
  EXPECT_LT(fast.riccati.spectral_radius, 1.0);
  EXPECT_LT(slow.riccati.spectral_radius, 1.0);
  // The faster rate's poles sit closer to +1, because less happens per tick.
  // Comparing the two radii directly would be comparing two coordinate systems,
  // which is the mistake this test exists to name.
  EXPECT_GT(fast.riccati.spectral_radius, slow.riccati.spectral_radius)
      << "a shorter interval moves every pole toward 1, and that is not 'less stable'";

  // Mapped back through their own rates, the two designs describe similar
  // continuous dynamics. Equality is not expected: they are optimal for two
  // genuinely different objectives, since each cost was integrated over its own
  // interval. Agreement to a few percent is what says both solved the same
  // underlying problem.
  auto slowest_continuous = [](const auto& design) {
    double slowest = std::numeric_limits<double>::infinity();
    for (const auto& pole : design.riccati.closed_loop_eigenvalues) {
      slowest = std::min(slowest, -std::log(std::abs(pole)) / design.sample_time_s);
    }
    return slowest;
  };
  const double fast_rate = slowest_continuous(fast);
  const double slow_rate = slowest_continuous(slow);
  EXPECT_GT(fast_rate, 0.0);
  EXPECT_NEAR(fast_rate / slow_rate, 1.0, 0.1)
      << "continuous-equivalent decay rates " << fast_rate << " and " << slow_rate
      << " must describe the same underlying design";
}

// ============================================================================
// THE REFUSALS F14 NAMES
// ============================================================================

// UNSTABILISABLE: a mode outside the unit circle with no input reaching it. No
// gain exists, and a returned one would be a fiction.
TEST(DiscreteControl, AnUnstabilisableProblemIsRefused) {
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(2, 2);
  a.diagonal() << 0.5, 2.0;  // the second mode is outside the circle
  Eigen::MatrixXd b(2, 1);
  b << 1.0, 0.0;  // and nothing reaches it
  Eigen::MatrixXd q = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd r = Eigen::MatrixXd::Constant(1, 1, 1.0);

  EXPECT_THROW((void)solve_dare(a, b, q, r), std::exception)
      << "an uncontrollable mode at |lambda| = 2 has no stabilising solution";
}

// UNDETECTABLE: the unstable mode is reachable but unpenalised, so the
// infinite-horizon problem has no unique stabilising optimum. Stable
// unpenalised modes stay allowed, which the second half checks.
TEST(DiscreteControl, AnUnpenalisedUnstableModeIsRefusedButAStableOneIsNot) {
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(2, 2);
  a.diagonal() << 0.5, 2.0;
  const Eigen::MatrixXd b = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd r = Eigen::MatrixXd::Identity(2, 2);

  Eigen::MatrixXd unstable_unpenalised = Eigen::MatrixXd::Zero(2, 2);
  unstable_unpenalised.diagonal() << 1.0, 0.0;  // the |lambda| = 2 mode costs nothing
  EXPECT_THROW((void)solve_dare(a, b, unstable_unpenalised, r), std::exception)
      << "an unpenalised mode outside the unit circle must be refused";

  Eigen::MatrixXd stable_unpenalised = Eigen::MatrixXd::Zero(2, 2);
  stable_unpenalised.diagonal() << 0.0, 1.0;  // the |lambda| = 0.5 mode costs nothing
  EXPECT_NO_THROW({
    const auto solution = solve_dare(a, b, stable_unpenalised, r);
    EXPECT_LT(solution.spectral_radius, 1.0);
  }) << "a stable unpenalised mode is permitted, exactly as in continuous time";
}

TEST(DiscreteControl, AnInvalidCostIsRefusedByName) {
  const Eigen::MatrixXd a = Eigen::MatrixXd::Constant(1, 1, 1.5);
  const Eigen::MatrixXd b = Eigen::MatrixXd::Constant(1, 1, 1.0);
  const Eigen::MatrixXd q = Eigen::MatrixXd::Constant(1, 1, 1.0);

  EXPECT_THROW((void)solve_dare(a, b, q, Eigen::MatrixXd::Zero(1, 1)), std::invalid_argument)
      << "R must be positive definite, not merely semidefinite";
  EXPECT_THROW((void)solve_dare(a, b, q, Eigen::MatrixXd::Constant(1, 1, -1.0)),
               std::invalid_argument);
  EXPECT_THROW(
      (void)solve_dare(
          a, b, Eigen::MatrixXd::Constant(1, 1, -1.0), Eigen::MatrixXd::Constant(1, 1, 1.0)),
      std::invalid_argument)
      << "a negative Q makes the block cost indefinite";

  // Asymmetric Q is refused rather than quietly symmetrised, because the caller
  // who wrote it meant something this routine cannot guess.
  Eigen::MatrixXd asymmetric(2, 2);
  asymmetric << 1.0, 0.5, -0.5, 1.0;
  Eigen::MatrixXd a2 = Eigen::MatrixXd::Identity(2, 2) * 0.5;
  EXPECT_THROW(
      (void)solve_dare(
          a2, Eigen::MatrixXd::Identity(2, 2), asymmetric, Eigen::MatrixXd::Identity(2, 2)),
      std::invalid_argument);

  // A block cost that is indefinite because the CROSS TERM is too large, which
  // is the case a reader is most likely to build by accident.
  Eigen::MatrixXd n(1, 1);
  n << 10.0;
  EXPECT_THROW((void)solve_dare(a, b, q, Eigen::MatrixXd::Constant(1, 1, 1.0), n),
               std::invalid_argument)
      << "[Q N; N' R] with N = 10, Q = R = 1 is indefinite";
}

TEST(DiscreteControl, AnInvalidSampleTimeIsRefusedByEveryEntryPoint) {
  const auto plant = double_integrator();
  const Eigen::MatrixXd q = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd r = Eigen::MatrixXd::Constant(1, 1, 1.0);

  EXPECT_THROW((void)discretize_cost(plant, q, r, {}, 0.0), std::invalid_argument);
  EXPECT_THROW((void)discretize_cost(plant, q, r, {}, -0.01), std::invalid_argument);
  EXPECT_THROW((void)design_sampled_lqr(plant, q, r, {}, 0.0), std::invalid_argument);
  EXPECT_THROW((void)design_sampled_lqr(plant, q, r, {}, std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);

  // Mismatched weight dimensions, which is how a caller most often pairs a cost
  // with the wrong plant.
  EXPECT_THROW((void)discretize_cost(plant, Eigen::MatrixXd::Identity(3, 3), r, {}, 0.01),
               std::invalid_argument);
  EXPECT_THROW((void)discretize_cost(plant, q, Eigen::MatrixXd::Identity(2, 2), {}, 0.01),
               std::invalid_argument);
}
