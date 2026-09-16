// SPDX-License-Identifier: Apache-2.0
//
// The integration-method interface: that RK4 through it is bit-identical to
// RK4 without it, that the implicit methods converge at their stated orders,
// that they are stable where RK4 is not, and that a divergence is REFUSED with
// the offending state named instead of returned as a completed trajectory.
//
// The stiff benchmark here is the one the readiness audit measured RK4 failing:
//   dx/dt = -1000 (x - cos t) - sin t,  x(0) = 0,  exact x(t) -> cos t
// whose decaying mode is lambda = -1000. RK4's real-axis stability limit is
// h*lambda > -2.7853, so h > 2.785 ms diverges. An A-stable method does not.

#include "galata/numerics/integration_method.hpp"
#include "galata/numerics/integrator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

using galata::numerics::IntegrationMethod;
using galata::numerics::IntegrationOptions;
using galata::numerics::integrate;
using galata::numerics::integration_step;
using galata::numerics::StateBounds;
using galata::numerics::TerminationReason;

namespace {

// dx/dt = -1000 (x - cos t) - sin t. Exact solution with x(0) = 0 is
// cos(t) - exp(-1000 t), so after a few milliseconds it is cos(t) to well
// below any tolerance used here.
Eigen::VectorXd stiff_derivative(double t, const Eigen::VectorXd& x) {
  Eigen::VectorXd rate(1);
  rate(0) = -1000.0 * (x(0) - std::cos(t)) - std::sin(t);
  return rate;
}

Eigen::VectorXd decay_derivative(double /*t*/, const Eigen::VectorXd& x) {
  return Eigen::VectorXd(-2.0 * x);
}

IntegrationOptions options_for(IntegrationMethod method, double step_s, int count) {
  IntegrationOptions options;
  options.method = method;
  options.step_s = step_s;
  options.step_count = count;
  options.newton_iterations = 4;
  return options;
}

}  // namespace

// ---------------------------------------------------------------------------
// RK4 through the new interface must be the OLD RK4, bit for bit.
// ---------------------------------------------------------------------------

TEST(IntegrationMethod, Rk4ThroughTheInterfaceIsBitIdenticalToRk4Directly) {
  Eigen::VectorXd x0(2);
  x0 << 2.0, 0.0;
  const auto vdp = [](double, const Eigen::VectorXd& x) {
    Eigen::VectorXd d(2);
    d(0) = x(1);
    d(1) = (1.0 - x(0) * x(0)) * x(1) - x(0);
    return d;
  };

  const auto legacy = galata::numerics::integrate_fixed_step(vdp, x0, 0.0, 0.01, 500, 1);
  const auto through = integrate(vdp, x0, 0.0, options_for(IntegrationMethod::Rk4Fixed, 0.01, 500));

  ASSERT_TRUE(through.completed());
  ASSERT_EQ(legacy.states.size(), through.trajectory.states.size());
  for (std::size_t i = 0; i < legacy.states.size(); ++i) {
    // Exact equality, not a tolerance: ADR-0004's bit-identity tier is a claim
    // about rk4_step's summation expression, and this asserts the new path
    // reaches that same expression rather than reimplementing it.
    EXPECT_EQ(legacy.states[i](0), through.trajectory.states[i](0)) << "sample " << i;
    EXPECT_EQ(legacy.states[i](1), through.trajectory.states[i](1)) << "sample " << i;
    EXPECT_EQ(legacy.times_s[i], through.trajectory.times_s[i]) << "sample " << i;
  }
  EXPECT_EQ(through.newton_iterations_performed, 0);
}

// ---------------------------------------------------------------------------
// The stiff benchmark. This is the audit's measurement, now a gate.
// ---------------------------------------------------------------------------

TEST(StiffBenchmark, Rk4DivergesAboveItsStabilityLimit) {
  Eigen::VectorXd x0(1);
  x0 << 0.0;
  // h = 5 ms gives h*lambda = -5, outside RK4's real-axis stability region.
  const auto result = integrate(stiff_derivative, x0, 0.0, options_for(IntegrationMethod::Rk4Fixed, 0.005, 200));
  // It does not merely lose accuracy: it leaves every plausible scale. Whether
  // it ends finite or not, it is nowhere near cos(1) = 0.5403.
  const double final_value = result.trajectory.states.back()(0);
  EXPECT_GT(std::fabs(final_value), 1e10)
      << "RK4 at h*lambda = -5 was expected to diverge; it returned " << final_value;
}

TEST(StiffBenchmark, ImplicitEulerIsStableAtAStepWhereRk4Diverges) {
  Eigen::VectorXd x0(1);
  x0 << 0.0;
  const auto result =
      integrate(stiff_derivative, x0, 0.0, options_for(IntegrationMethod::ImplicitEulerFixed, 0.005, 200));
  ASSERT_TRUE(result.completed()) << result.detail;
  const double final_value = result.trajectory.states.back()(0);
  // First order at h = 5 ms on a signal of unit amplitude: a few parts in a
  // thousand is the expected accuracy, and the point of the test is that it is
  // bounded at all.
  EXPECT_NEAR(final_value, std::cos(1.0), 5e-3)
      << "implicit Euler should be stable and roughly accurate here";
  EXPECT_GT(result.newton_iterations_performed, 0);
}

TEST(StiffBenchmark, TrapezoidalIsStableAndMoreAccurateThanImplicitEuler) {
  Eigen::VectorXd x0(1);
  x0 << 0.0;
  const auto euler =
      integrate(stiff_derivative, x0, 0.0, options_for(IntegrationMethod::ImplicitEulerFixed, 0.005, 200));
  const auto trapezoid =
      integrate(stiff_derivative, x0, 0.0, options_for(IntegrationMethod::TrapezoidalFixed, 0.005, 200));
  ASSERT_TRUE(euler.completed());
  ASSERT_TRUE(trapezoid.completed());
  const double exact = std::cos(1.0);
  const double euler_error = std::fabs(euler.trajectory.states.back()(0) - exact);
  const double trapezoid_error = std::fabs(trapezoid.trajectory.states.back()(0) - exact);
  EXPECT_LT(trapezoid_error, euler_error)
      << "second order should beat first order here: trapezoid " << trapezoid_error << " vs euler "
      << euler_error;
}

TEST(StiffBenchmark, ImplicitEulerStaysStableAtAStepTwentyTimesRk4sLimit) {
  Eigen::VectorXd x0(1);
  x0 << 0.0;
  // h = 50 ms is h*lambda = -50. A-stability is a statement about the whole
  // left half-plane, so the step is limited by accuracy alone.
  const auto result =
      integrate(stiff_derivative, x0, 0.0, options_for(IntegrationMethod::ImplicitEulerFixed, 0.05, 20));
  ASSERT_TRUE(result.completed()) << result.detail;
  EXPECT_LT(std::fabs(result.trajectory.states.back()(0)), 2.0)
      << "the solution must stay bounded even where the step is far outside any explicit "
         "method's stability region";
}

// ---------------------------------------------------------------------------
// Order of convergence, measured rather than asserted from the method name.
// ---------------------------------------------------------------------------

TEST(IntegrationMethod, ImplicitMethodsConvergeAtTheirStatedOrders) {
  Eigen::VectorXd x0(1);
  x0 << 1.0;
  const double exact = std::exp(-2.0 * 1.0);

  const auto error_at = [&](IntegrationMethod method, double h) {
    const int n = static_cast<int>(std::llround(1.0 / h));
    const auto result = integrate(decay_derivative, x0, 0.0, options_for(method, h, n));
    EXPECT_TRUE(result.completed());
    return std::fabs(result.trajectory.states.back()(0) - exact);
  };

  for (const auto method :
       {IntegrationMethod::ImplicitEulerFixed, IntegrationMethod::TrapezoidalFixed}) {
    const double coarse = error_at(method, 0.01);
    const double fine = error_at(method, 0.005);
    const double observed = std::log2(coarse / fine);
    const double expected = static_cast<double>(galata::numerics::method_order(method));
    EXPECT_NEAR(observed, expected, 0.25)
        << galata::numerics::to_string(method) << " observed order " << observed << ", expected "
        << expected;
  }
}

// ---------------------------------------------------------------------------
// Divergence is refused, and the offending state is named.
// ---------------------------------------------------------------------------

TEST(StateBounds, ADivergedFiniteTrajectoryIsRefusedAndNamesTheState) {
  // dx/dt = +x diverges smoothly and stays finite for a long time. Before
  // bounds existed this was returned as a completed run.
  const auto growth = [](double, const Eigen::VectorXd& x) { return Eigen::VectorXd(x); };
  Eigen::VectorXd x0(2);
  x0 << 1.0, 1.0;

  Eigen::VectorXd limits(2);
  limits << 1.0e3, -1.0;  // second state declared unbounded
  const StateBounds bounds({"rotor_speed_rad_s", "position_north_m"}, limits);

  const auto result =
      integrate(growth, x0, 0.0, options_for(IntegrationMethod::Rk4Fixed, 0.01, 5000), nullptr, bounds);

  EXPECT_FALSE(result.completed());
  EXPECT_EQ(result.reason, TerminationReason::BoundExceeded);
  EXPECT_NE(result.detail.find("rotor_speed_rad_s"), std::string::npos)
      << "the diagnostic must name the state: " << result.detail;
  EXPECT_NE(result.detail.find("1.0000e+03"), std::string::npos)
      << "the diagnostic must quote the bound: " << result.detail;
  EXPECT_LT(result.steps_taken, 5000);
  // The trajectory kept is the part that satisfied the bound.
  for (const auto& state : result.trajectory.states) {
    EXPECT_LE(std::fabs(state(0)), 1.0e3);
  }
}

TEST(StateBounds, TheStiffDivergenceThatUsedToBeReportedAsCompletedIsNowRefused) {
  Eigen::VectorXd x0(1);
  x0 << 0.0;
  Eigen::VectorXd limits(1);
  limits << 10.0;  // the true solution never leaves [-1, 1]
  const StateBounds bounds({"x"}, limits);

  const auto result = integrate(stiff_derivative, x0, 0.0,
                                options_for(IntegrationMethod::Rk4Fixed, 0.005, 200), nullptr, bounds);
  EXPECT_FALSE(result.completed());
  EXPECT_EQ(result.reason, TerminationReason::BoundExceeded);
  EXPECT_NE(result.detail.find("'x'"), std::string::npos) << result.detail;

  // And the same problem with the same bound under an A-stable method completes.
  const auto stable = integrate(stiff_derivative, x0, 0.0,
                                options_for(IntegrationMethod::ImplicitEulerFixed, 0.005, 200),
                                nullptr, bounds);
  EXPECT_TRUE(stable.completed()) << stable.detail;
}

TEST(StateBounds, ANonFiniteStateIsNamedEvenWithNoMagnitudeDeclared) {
  const StateBounds bounds;
  Eigen::VectorXd state(2);
  state << 1.0, std::numeric_limits<double>::quiet_NaN();
  const std::string violation = bounds.violation(state);
  EXPECT_NE(violation.find("not finite"), std::string::npos) << violation;
}

TEST(StateBounds, AnEmptyBoundsObjectPreservesTheOldFiniteOnlyBehaviour) {
  Eigen::VectorXd x0(1);
  x0 << 1.0;
  const auto result = integrate(decay_derivative, x0, 0.0,
                                options_for(IntegrationMethod::Rk4Fixed, 0.01, 100), nullptr, {});
  EXPECT_TRUE(result.completed());
  EXPECT_EQ(result.reason, TerminationReason::Completed);
}

TEST(IntegrationMethod, RefusesAMalformedRequest) {
  Eigen::VectorXd x0(1);
  x0 << 1.0;
  const auto run = [&](const IntegrationOptions& options) {
    (void)integrate(decay_derivative, x0, 0.0, options);
  };
  EXPECT_THROW(run(options_for(IntegrationMethod::Rk4Fixed, 0.0, 10)), std::invalid_argument);
  EXPECT_THROW(run(options_for(IntegrationMethod::Rk4Fixed, 0.01, -1)), std::invalid_argument);
  IntegrationOptions zero_newton = options_for(IntegrationMethod::ImplicitEulerFixed, 0.01, 10);
  zero_newton.newton_iterations = 0;
  EXPECT_THROW(run(zero_newton), std::invalid_argument);
}

TEST(IntegrationMethod, RepeatedRunsAreBitIdenticalForEveryMethod) {
  Eigen::VectorXd x0(1);
  x0 << 0.0;
  for (const auto method : {IntegrationMethod::Rk4Fixed, IntegrationMethod::ImplicitEulerFixed,
                            IntegrationMethod::TrapezoidalFixed}) {
    const auto a = integrate(stiff_derivative, x0, 0.0, options_for(method, 0.002, 100));
    const auto b = integrate(stiff_derivative, x0, 0.0, options_for(method, 0.002, 100));
    ASSERT_EQ(a.trajectory.states.size(), b.trajectory.states.size());
    for (std::size_t i = 0; i < a.trajectory.states.size(); ++i) {
      EXPECT_EQ(a.trajectory.states[i](0), b.trajectory.states[i](0))
          << galata::numerics::to_string(method) << " sample " << i;
    }
    EXPECT_EQ(a.newton_iterations_performed, b.newton_iterations_performed);
  }
}
