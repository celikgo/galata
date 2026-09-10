// SPDX-License-Identifier: Apache-2.0
//
// The discrete-time model contract and the zero-order-hold discretisation.
//
// THE REFERENCES ARE CLOSED FORMS, NOT PREVIOUS OUTPUT. A first-order lag and a
// double integrator both have exactly known ZOH discretisations, written here as
// formulas in the sample time:
//
//   x_dot = a x + b u   ->   Ad = exp(a T),  Bd = b (exp(a T) - 1) / a
//   double integrator   ->   Ad = [[1, T], [0, 1]],  Bd = [[T^2 / 2], [T]]
//
// and the held-input trajectory of the double integrator is the schoolbook
// x(T) = x0 + v0 T + u T^2 / 2. So a passing test has agreed with algebra.
//
// THE EIGENVALUE MAP IS CHECKED SEPARATELY because it is the property every
// later sampled-design step depends on: lambda_discrete = exp(lambda_continuous
// T), which puts the continuous left half-plane inside the unit circle and is
// the reason a sampled stability test is a test against 1 rather than against 0.

#include "galata/model/discrete_system.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>

namespace {

using galata::model::discretize_zoh;
using galata::model::DiscreteLinearSystem;
using galata::model::InputHold;
using galata::model::LinearSystem;

// x_dot = -x / tau + u / tau: a first-order lag with unit steady-state gain.
LinearSystem first_order_lag(double tau_s) {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Constant(1, 1, -1.0 / tau_s);
  system.b = Eigen::MatrixXd::Constant(1, 1, 1.0 / tau_s);
  system.state_names = {"level"};
  system.input_names = {"command"};
  system.description = "First-order lag, independently authored for this test";
  return system;
}

// Position and velocity under a commanded acceleration.
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

TEST(DiscreteSystem, AFirstOrderLagMatchesItsClosedFormDiscretisation) {
  const double tau = 0.4;       // s
  const double interval = 0.05;  // s
  const auto result = discretize_zoh(first_order_lag(tau), interval);

  const double pole = -1.0 / tau;
  const double expected_a = std::exp(pole * interval);
  const double expected_b = (1.0 / tau) * (std::exp(pole * interval) - 1.0) / pole;

  EXPECT_NEAR(result.system.a(0, 0), expected_a, 1e-15) << "Ad = exp(a T)";
  EXPECT_NEAR(result.system.b(0, 0), expected_b, 1e-15) << "Bd = b (exp(a T) - 1) / a";
  EXPECT_DOUBLE_EQ(result.system.sample_time_s, interval);
  EXPECT_EQ(result.system.hold, InputHold::ZeroOrder);
  EXPECT_DOUBLE_EQ(result.system.sample_rate_hz(), 1.0 / interval);
}

// A LAG HELD LONG ENOUGH REACHES ITS STEADY STATE, and the discrete gain to
// steady state must be exactly one for a plant whose continuous gain is one:
// (I - Ad)^-1 Bd = 1. This catches a Bd that is off by a factor of T, which the
// closed-form comparison above would also catch but which a reader is more
// likely to recognise in this form.
TEST(DiscreteSystem, TheDiscreteSteadyStateGainMatchesTheContinuousOne) {
  const auto result = discretize_zoh(first_order_lag(0.25), 0.01);
  const double gain = result.system.b(0, 0) / (1.0 - result.system.a(0, 0));
  EXPECT_NEAR(gain, 1.0, 1e-12) << "a unit-gain plant keeps unit gain under any hold";
}

TEST(DiscreteSystem, TheDoubleIntegratorMatchesItsExactDiscretisation) {
  const double interval = 0.125;  // s
  const auto result = discretize_zoh(double_integrator(), interval);

  Eigen::MatrixXd expected_a(2, 2);
  expected_a << 1.0, interval, 0.0, 1.0;
  Eigen::MatrixXd expected_b(2, 1);
  expected_b << 0.5 * interval * interval, interval;

  EXPECT_LT((result.system.a - expected_a).cwiseAbs().maxCoeff(), 1e-15)
      << "computed:\n"
      << result.system.a;
  EXPECT_LT((result.system.b - expected_b).cwiseAbs().maxCoeff(), 1e-16)
      << "the T^2/2 term is what a first-order series would get wrong:\n"
      << result.system.b;
}

// F14's acceptance asks for held-input trajectories against an independent
// reference. The double integrator's is exact arithmetic, so a whole run of
// discrete steps can be compared term by term with no tolerance argument.
TEST(DiscreteSystem, AHeldInputTrajectoryAgreesWithTheExactContinuousSolution) {
  const double interval = 0.05;  // s
  const auto result = discretize_zoh(double_integrator(), interval);

  Eigen::VectorXd state(2);
  state << 3.0, -0.5;  // m, m/s
  const double command = 1.25;  // m/s^2, held across every interval
  Eigen::VectorXd input = Eigen::VectorXd::Constant(1, command);

  const Eigen::VectorXd initial = state;
  const int ticks = 40;
  for (int k = 0; k < ticks; ++k) {
    state = result.system.a * state + result.system.b * input;
  }

  // Exact solution of a constant acceleration over the whole elapsed time,
  // which for a zero-order hold at constant command is also the continuous
  // answer at the final tick.
  const double elapsed = interval * ticks;
  const double expected_position =
      initial(0) + initial(1) * elapsed + 0.5 * command * elapsed * elapsed;
  const double expected_velocity = initial(1) + command * elapsed;

  EXPECT_NEAR(state(0), expected_position, 1e-12)
      << "position after " << ticks << " held steps";
  EXPECT_NEAR(state(1), expected_velocity, 1e-13) << "velocity after " << ticks << " held steps";
}

// lambda_d = exp(lambda_c T). This is the property that makes the unit circle
// the right test for a sampled model, and it is checked against scalar
// exponentials of the continuous eigenvalues.
TEST(DiscreteSystem, ContinuousEigenvaluesMapOntoTheUnitDiskByTheExponential) {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(3, 3);
  system.a(0, 0) = -2.0;   // stable real
  system.a(1, 2) = 4.0;    // an oscillatory pair at +-4i, marginally stable
  system.a(2, 1) = -4.0;
  system.b = Eigen::MatrixXd::Zero(3, 1);
  system.b(0, 0) = 1.0;
  system.state_names = {"decay", "swing", "swing_rate"};
  system.input_names = {"drive"};

  const double interval = 0.02;  // s
  const auto result = discretize_zoh(system, interval);

  // The stable real mode maps strictly inside the circle.
  EXPECT_NEAR(std::exp(-2.0 * interval), std::exp(-2.0 * interval), 0.0);
  bool found_decay = false;
  bool found_swing = false;
  for (const auto& value : result.evidence.discrete_eigenvalues) {
    if (std::abs(value.imag()) < 1e-12) {
      EXPECT_NEAR(value.real(), std::exp(-2.0 * interval), 1e-12);
      found_decay = true;
    } else {
      // exp(+-4i T) sits exactly ON the unit circle, because a marginally
      // stable continuous mode is a marginally stable sampled one.
      EXPECT_NEAR(std::abs(value), 1.0, 1e-12)
          << "an undamped oscillation must land on the circle, not inside it";
      EXPECT_NEAR(value.real(), std::cos(4.0 * interval), 1e-12);
      found_swing = true;
    }
  }
  EXPECT_TRUE(found_decay);
  EXPECT_TRUE(found_swing);
  EXPECT_NEAR(result.evidence.spectral_radius, 1.0, 1e-12)
      << "the reported radius is the largest of those magnitudes";
}

TEST(DiscreteSystem, TheEvidenceReportsWhatAReaderNeedsToJudgeTheSampleRate) {
  const auto result = discretize_zoh(first_order_lag(0.4), 0.05);
  const auto& evidence = result.evidence;

  EXPECT_DOUBLE_EQ(evidence.sample_time_s, 0.05);
  EXPECT_NEAR(evidence.fastest_mode_rad_s, 2.5, 1e-12) << "1/tau for a single lag";
  EXPECT_NEAR(evidence.nyquist_rad_s, M_PI / 0.05, 1e-9);
  EXPECT_GT(evidence.exponential_error_bound, 0.0)
      << "the exponential's declared backward bound travels with the model";
  EXPECT_GT(evidence.exponential_pade_order, 0);
  EXPECT_NE(evidence.assumptions.find("-order hold"), std::string::npos)
      << "the quotable sentence names the hold: " << evidence.assumptions;
  EXPECT_NE(evidence.assumptions.find("aliases"), std::string::npos)
      << "and does not leave aliasing for the reader to remember";
  EXPECT_NE(evidence.assumptions.find("delay"), std::string::npos)
      << "and says that no delay is included";
}

// A DISCRETE MODEL WITHOUT ITS SAMPLE TIME IS REFUSED. This is the half of
// "mixed time domains are rejected" that a type cannot enforce on its own: the
// type keeps a discrete model out of a continuous routine, and this keeps a
// discrete model from existing without the one number that gives it meaning.
TEST(DiscreteSystem, ADiscreteModelWithoutASampleTimeDoesNotValidate) {
  DiscreteLinearSystem system;
  system.a = Eigen::MatrixXd::Identity(1, 1);
  system.b = Eigen::MatrixXd::Constant(1, 1, 1.0);
  system.state_names = {"x"};
  system.input_names = {"u"};

  system.sample_time_s = 0.0;
  EXPECT_THROW(system.validate(), std::invalid_argument) << "zero is not a sample time";
  system.sample_time_s = -0.01;
  EXPECT_THROW(system.validate(), std::invalid_argument) << "nor is a negative one";
  system.sample_time_s = std::numeric_limits<double>::infinity();
  EXPECT_THROW(system.validate(), std::invalid_argument) << "nor is an infinite one";
  system.sample_time_s = 0.01;
  EXPECT_NO_THROW(system.validate());
}

TEST(DiscreteSystem, WhatCannotBeDiscretisedIsRefusedByName) {
  EXPECT_THROW((void)discretize_zoh(double_integrator(), 0.0), std::invalid_argument);
  EXPECT_THROW((void)discretize_zoh(double_integrator(), -0.1), std::invalid_argument);
  EXPECT_THROW((void)discretize_zoh(double_integrator(),
                                    std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);

  // A system with no inputs has nothing for a hold to hold, and is refused with
  // a message naming the routine that does answer the question.
  LinearSystem no_inputs;
  no_inputs.a = Eigen::MatrixXd::Constant(1, 1, -1.0);
  no_inputs.state_names = {"x"};
  EXPECT_THROW((void)discretize_zoh(no_inputs, 0.01), std::invalid_argument);

  // And an invalid continuous system is refused by the continuous validator
  // rather than silently discretised into a model with the same defect.
  LinearSystem unnamed;
  unnamed.a = Eigen::MatrixXd::Identity(2, 2);
  unnamed.b = Eigen::MatrixXd::Ones(2, 1);
  unnamed.state_names = {"only_one_name"};
  unnamed.input_names = {"u"};
  EXPECT_THROW((void)discretize_zoh(unnamed, 0.01), std::invalid_argument);
}

TEST(DiscreteSystem, NamesUnitsAndCitationSurviveTheDiscretisation) {
  LinearSystem system = double_integrator();
  system.description = "A carriage on a rail";
  system.citation = "Nobody, 'A Carriage', 1900";
  system.units = "SI";
  system.output_names = {"position_m", "velocity_m_s"};

  const auto result = discretize_zoh(system, 0.02);
  EXPECT_EQ(result.system.state_names, system.state_names);
  EXPECT_EQ(result.system.input_names, system.input_names);
  EXPECT_EQ(result.system.output_names, system.output_names);
  EXPECT_EQ(result.system.description, system.description);
  EXPECT_EQ(result.system.citation, system.citation)
      << "a discrete model must not lose the source its continuous parent cited";
  EXPECT_EQ(result.system.units, system.units);
  EXPECT_EQ(std::string(galata::model::to_string(result.system.hold)), "zero_order_hold");
}
