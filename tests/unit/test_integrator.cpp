// SPDX-License-Identifier: Apache-2.0
//
// Fixed-step RK4, against problems whose exact solutions are known in closed
// form.
//
// The order-verification tests are the important ones. "It looks about right"
// is satisfied by a second-order method with a small step, and a method that
// has silently become second order because a stage coefficient is wrong will
// pass every eyeball check while costing four times the work for the accuracy
// of a much cheaper method.

#include "galata/numerics/integrator.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using galata::numerics::DerivativeFunction;
using galata::numerics::rk4_step;

Eigen::VectorXd scalar(double value) {
  Eigen::VectorXd v(1);
  v(0) = value;
  return v;
}

TEST(Rk4, IntegratesCubicsInTimeExactly) {
  // For dx/dt = f(t) with f independent of x, RK4 degenerates to Simpson's
  // rule, which is exact for polynomials up to cubic. Exactness here is a
  // strong check on the weights 1/6, 1/3, 1/3, 1/6 and on the half-step nodes:
  // almost any error in either destroys it.
  const DerivativeFunction cubic = [](double t, const Eigen::VectorXd&) {
    return scalar(4.0 * t * t * t);  // exact integral: t^4
  };

  Eigen::VectorXd state = scalar(0.0);
  const double step = 0.25;
  const int steps = 8;
  for (int k = 0; k < steps; ++k) {
    state = rk4_step(cubic, static_cast<double>(k) * step, state, step);
  }
  const double end = static_cast<double>(steps) * step;
  EXPECT_NEAR(state(0), std::pow(end, 4.0), 1e-13);
}

TEST(Rk4, IsFourthOrderOnTheExponential) {
  // dx/dt = lambda x, x(0) = 1, exact solution exp(lambda t). Halving the step
  // must cut the error by about sixteen.
  const double lambda = -1.7;
  const DerivativeFunction exponential = [lambda](double, const Eigen::VectorXd& x) {
    return Eigen::VectorXd(lambda * x);
  };
  const double end_time = 2.0;
  const double exact = std::exp(lambda * end_time);

  double previous_error = 0.0;
  for (int refinement = 0; refinement < 4; ++refinement) {
    const int steps = 20 * (1 << refinement);
    const double step = end_time / static_cast<double>(steps);
    const auto trajectory =
        galata::numerics::integrate_fixed_step(exponential, scalar(1.0), 0.0, step, steps, steps);
    const double error = std::fabs(trajectory.states.back()(0) - exact);

    if (refinement > 0) {
      const double ratio = previous_error / error;
      EXPECT_GT(ratio, 12.0) << "refinement " << refinement << ": error ratio " << ratio
                             << " is too small for a fourth-order method";
      EXPECT_LT(ratio, 20.0) << "refinement " << refinement << ": error ratio " << ratio
                             << " is suspiciously large";
    }
    previous_error = error;
  }
}

TEST(Rk4, StepSizeStudyRecoversTheMethodOrder) {
  const DerivativeFunction oscillator = [](double, const Eigen::VectorXd& x) {
    Eigen::VectorXd dx(2);
    dx(0) = x(1);
    dx(1) = -x(0);
    return dx;
  };
  Eigen::VectorXd initial(2);
  initial << 1.0, 0.0;

  const auto study = galata::numerics::step_size_study(oscillator, initial, 0.0, 0.05, 200);
  EXPECT_NEAR(study.observed_order, 4.0, 0.2)
      << "observed order " << study.observed_order << " for RK4";

  // The two Richardson estimates differ by exactly 2^4. Guards the attribution:
  // reporting the h/2 error as the h error is a factor-of-sixteen mistake about
  // one's own accuracy.
  EXPECT_NEAR(study.estimated_error_at_h / study.estimated_error_at_h_2, 16.0, 1e-9);

  // The estimate should be in the right ballpark as an error bound: the exact
  // solution at t = 10 is cos(10).
  const double exact = std::cos(10.0);
  const double actual_error = std::fabs(study.final_at_h(0) - exact);
  EXPECT_LT(actual_error, 10.0 * study.estimated_error_at_h);
  EXPECT_GT(actual_error, 0.1 * study.estimated_error_at_h);
}

TEST(Rk4, TimesAreExactMultiplesOfTheStepAndDoNotDrift) {
  // Accumulating t += h drifts; t0 + k*h does not. Over 100,000 steps the
  // difference is visible, and it lands in the argument the derivative is
  // evaluated at.
  const DerivativeFunction zero = [](double, const Eigen::VectorXd& x) {
    return Eigen::VectorXd(Eigen::VectorXd::Zero(x.size()));
  };
  const double step = 0.1;
  const int steps = 100000;
  const auto trajectory =
      galata::numerics::integrate_fixed_step(zero, scalar(0.0), 0.0, step, steps, 10000);

  for (std::size_t i = 0; i < trajectory.times_s.size(); ++i) {
    const double expected = static_cast<double>(i) * 10000.0 * step;
    EXPECT_DOUBLE_EQ(trajectory.times_s[i], expected);
  }
}

TEST(Rk4, SamplingStrideDoesNotChangeTheTrajectory) {
  // The stride must affect only what is recorded. If it affected the stepping,
  // a plot at one sample rate would disagree with the same run at another.
  const DerivativeFunction decay = [](double, const Eigen::VectorXd& x) {
    return Eigen::VectorXd(-0.9 * x);
  };
  const auto dense = galata::numerics::integrate_fixed_step(decay, scalar(3.0), 0.0, 0.01, 500, 1);
  const auto sparse =
      galata::numerics::integrate_fixed_step(decay, scalar(3.0), 0.0, 0.01, 500, 50);

  ASSERT_EQ(dense.states.size(), 501U);
  ASSERT_EQ(sparse.states.size(), 11U);
  // Bit-identical, not merely close: the same arithmetic ran in both.
  EXPECT_EQ(sparse.states.back()(0), dense.states.back()(0));
  EXPECT_EQ(sparse.states[5](0), dense.states[250](0));
}

TEST(Rk4, RepeatedRunsAreBitIdentical) {
  // ADR-0004 tier 1, at the level of a single component.
  const DerivativeFunction system = [](double t, const Eigen::VectorXd& x) {
    Eigen::VectorXd dx(2);
    dx(0) = x(1) + std::sin(t);
    dx(1) = -2.0 * x(0) - 0.3 * x(1);
    return dx;
  };
  Eigen::VectorXd initial(2);
  initial << 0.7, -0.2;

  const auto first =
      galata::numerics::integrate_fixed_step(system, initial, 0.0, 0.002, 5000, 5000);
  const auto second =
      galata::numerics::integrate_fixed_step(system, initial, 0.0, 0.002, 5000, 5000);
  EXPECT_EQ(first.states.back()(0), second.states.back()(0));
  EXPECT_EQ(first.states.back()(1), second.states.back()(1));
}

TEST(Rk4, ProjectionRunsAfterEachStepAndOnTheInitialState) {
  // The projection hook exists for quaternion renormalisation. It must apply to
  // the initial condition as well, so that a caller handing over a
  // six-digit-unit quaternion does not have that error integrated.
  int calls = 0;
  const galata::numerics::ProjectionFunction clamp = [&calls](Eigen::VectorXd& x) {
    ++calls;
    x(0) = std::fmin(x(0), 1.0);
  };
  const DerivativeFunction grow = [](double, const Eigen::VectorXd&) { return scalar(10.0); };

  const auto trajectory =
      galata::numerics::integrate_fixed_step(grow, scalar(5.0), 0.0, 0.1, 4, 1, clamp);
  EXPECT_EQ(calls, 5);  // once for the initial state, once per step
  EXPECT_DOUBLE_EQ(trajectory.states.front()(0), 1.0);
  for (const Eigen::VectorXd& state : trajectory.states) {
    EXPECT_LE(state(0), 1.0);
  }
}

TEST(Rk4, RejectsArgumentsThatWouldMakeStepCountAmbiguous) {
  const DerivativeFunction zero = [](double, const Eigen::VectorXd&) { return scalar(0.0); };
  EXPECT_THROW((void)galata::numerics::integrate_fixed_step(zero, scalar(0.0), 0.0, 0.0, 10),
               std::invalid_argument);
  EXPECT_THROW((void)galata::numerics::integrate_fixed_step(zero, scalar(0.0), 0.0, -0.1, 10),
               std::invalid_argument);
  EXPECT_THROW((void)galata::numerics::integrate_fixed_step(zero, scalar(0.0), 0.0, 0.1, -1),
               std::invalid_argument);
  EXPECT_THROW((void)galata::numerics::integrate_fixed_step(zero, scalar(0.0), 0.0, 0.1, 10, 0),
               std::invalid_argument);
}

}  // namespace
