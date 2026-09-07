// SPDX-License-Identifier: Apache-2.0
//
// Nonlinear driver against equilibrium, first-order actuator closed forms, and
// the small-perturbation limit. References: Stevens, Lewis & Johnson, Aircraft
// Control and Simulation, 3rd ed., Wiley, 2016, chapters 2-3; Hairer, Norsett &
// Wanner, Solving Ordinary Differential Equations I, Springer, 1993.
// The synthetic derivative model is an analytic test fixture, not flight data.
#include "galata/core/quaternion.hpp"
#include "galata/sim/nonlinear.hpp"

#include <gtest/gtest.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <limits>

namespace {

galata::model::Aircraft aircraft_model() {
  galata::model::Aircraft aircraft;
  aircraft.geometry = {10.0, 10.0, 1.0};
  aircraft.mass.mass_kg = 1000.0;
  aircraft.mass.inertia_cg_body_kg_m2 = Eigen::Vector3d(500.0, 600.0, 700.0).asDiagonal();
  aircraft.aero.reference_mach = 0.15;
  aircraft.aero.lift_ref = 1.0;
  aircraft.aero.drag_ref = 0.1;
  aircraft.aero.lift_alpha = 5.0;
  aircraft.aero.pitching_moment_alpha = -1.0;
  aircraft.aero.pitching_moment_pitch_rate = -5.0;
  aircraft.aero.pitching_moment_elevator = -1.0;
  aircraft.aero.side_force_beta = -0.5;
  aircraft.aero.rolling_moment_roll_rate = -0.3;
  aircraft.aero.rolling_moment_aileron = 0.1;
  return aircraft;
}

galata::trim::TrimPoint trim_for(const galata::model::Aircraft& aircraft) {
  galata::trim::LevelTrimRequest request;
  request.airspeed_m_s = 50.0;
  return galata::trim::trim_level(aircraft, request);
}

galata::sim::NonlinearRequest simulation_request() {
  galata::sim::NonlinearRequest request;
  request.step_s = 0.01;
  request.step_count = 100;
  for (int index = 0; index < 3; ++index) {
    request.actuators[static_cast<std::size_t>(index)] = {-0.5, 0.5, 10.0, 0.1};
  }
  request.actuators[3] = {0.0, 10000.0, 1e6, 0.2};
  return request;
}

TEST(NonlinearSimulation, TrimIsPreservedWhileReferencePositionTranslates) {
  const auto aircraft = aircraft_model();
  const auto trim = trim_for(aircraft);
  auto request = simulation_request();
  request.sample_stride = 17;
  // Position feedback must track the translating trim trajectory, not fight
  // the aircraft's nominal forward motion.
  request.feedback.state_names = {"p_n"};
  request.feedback.input_names = {"thrust"};
  request.feedback.k = Eigen::MatrixXd::Constant(1, 1, 1.0);
  const auto result = galata::sim::simulate_nonlinear(aircraft, trim, request);
  ASSERT_TRUE(result.completed) << result.termination_reason;
  ASSERT_FALSE(result.samples.empty());
  EXPECT_EQ(result.completed_steps, request.step_count);
  EXPECT_DOUBLE_EQ(result.samples.back().time_s, 1.0);
  const auto& final = result.samples.back();
  EXPECT_LT((final.state.velocity_body_m_s - trim.state.velocity_body_m_s).norm(), 1e-10);
  EXPECT_LT(final.state.angular_rate_body_rad_s.norm(), 1e-10);
  EXPECT_LT(galata::core::angular_distance(final.state.attitude_body_to_ned,
                                           trim.state.attitude_body_to_ned),
            1e-10);
  EXPECT_LT((final.state.position_ned_m - trim.state.position_ned_m
             - galata::core::velocity_ned(trim.state) * final.time_s)
                .norm(),
            1e-10);
  EXPECT_LT((final.controls.to_vector() - trim.controls.to_vector()).norm(), 1e-10);
  EXPECT_FALSE(result.outside_envelope_encountered);
}

TEST(NonlinearSimulation, NonStandardTemperatureTrimRemainsAnEquilibrium) {
  const auto aircraft = aircraft_model();
  galata::trim::LevelTrimRequest trim_request;
  trim_request.airspeed_m_s = 50.0;
  trim_request.delta_isa_k = 30.0;
  const auto trim = galata::trim::trim_level(aircraft, trim_request);
  const auto result = galata::sim::simulate_nonlinear(aircraft, trim, simulation_request());
  ASSERT_TRUE(result.completed) << result.termination_reason;
  ASSERT_FALSE(result.samples.empty());
  EXPECT_LT((result.samples.back().state.velocity_body_m_s - trim.state.velocity_body_m_s).norm(),
            1e-10);
  EXPECT_LT(result.samples.back().state.angular_rate_body_rad_s.norm(), 1e-10);
}

TEST(NonlinearSimulation, UnsaturatedActuatorStepConvergesToTheExponentialAtFourthOrder) {
  const auto aircraft = aircraft_model();
  const auto trim = trim_for(aircraft);
  auto request = simulation_request();
  request.command_increment.thrust_n = 100.0;
  const double exact = trim.controls.thrust_n
                       + request.command_increment.thrust_n
                             * (1.0 - std::exp(-1.0 / request.actuators[3].time_constant_s));
  double errors[3]{};
  for (int level = 0; level < 3; ++level) {
    request.step_s = 0.02 / static_cast<double>(1 << level);
    request.step_count = 50 * (1 << level);
    const auto result = galata::sim::simulate_nonlinear(aircraft, trim, request);
    ASSERT_TRUE(result.completed) << result.termination_reason;
    errors[level] = std::abs(result.samples.back().controls.thrust_n - exact);
    EXPECT_EQ(result.position_limited_steps[3], 0);
    EXPECT_EQ(result.rate_limited_steps[3], 0);
  }
  EXPECT_GT(errors[0] / errors[1], 15.0);
  EXPECT_LT(errors[0] / errors[1], 18.0);
  EXPECT_GT(errors[1] / errors[2], 15.0);
  EXPECT_LT(errors[1] / errors[2], 18.0);
}

TEST(NonlinearSimulation, PositionAndRateLimitsConstrainEveryRecordedActuatorStep) {
  const auto aircraft = aircraft_model();
  const auto trim = trim_for(aircraft);
  auto request = simulation_request();
  request.step_count = 40;
  request.actuators[0] = {-0.2, 0.2, 0.1, 0.1};
  request.command_increment.elevator_rad = 10.0;
  const auto result = galata::sim::simulate_nonlinear(aircraft, trim, request);
  ASSERT_TRUE(result.completed) << result.termination_reason;
  EXPECT_GT(result.position_limited_steps[0], 0);
  EXPECT_GT(result.rate_limited_steps[0], 0);
  for (std::size_t index = 1; index < result.samples.size(); ++index) {
    const double previous = result.samples[index - 1].controls.elevator_rad;
    const double current = result.samples[index].controls.elevator_rad;
    EXPECT_GE(current, request.actuators[0].minimum);
    EXPECT_LE(current, request.actuators[0].maximum);
    EXPECT_LE(std::abs(current - previous),
              request.actuators[0].rate_limit_per_s * request.step_s + 1e-15);
    EXPECT_TRUE(result.samples[index].position_limited[0]);
  }
  EXPECT_NEAR(result.samples[1].controls.elevator_rad - trim.controls.elevator_rad,
              request.actuators[0].rate_limit_per_s * request.step_s,
              1e-15);
}

TEST(NonlinearSimulation, SmallPerturbationsApproachTheLinearClosedLoopWithActuatorLag) {
  const auto aircraft = aircraft_model();
  const auto trim = trim_for(aircraft);
  auto request = simulation_request();
  request.step_s = 0.001;
  request.step_count = 400;
  request.feedback.state_names = {"p", "phi"};
  request.feedback.input_names = {"aileron"};
  request.feedback.k.resize(1, 2);
  request.feedback.k << 0.4, 1.0;
  request.perturbation_state_names = {"phi"};
  request.initial_perturbation = Eigen::VectorXd::Zero(1);

  // Independent analytic roll linearisation in [p,phi,delta_a]. The actuator
  // is part of the plant being compared; omitting it would compare different
  // closed loops and hide the exact integration-stage error this test targets.
  const double moment_scale = trim.dynamic_pressure_pa * aircraft.geometry.wing_area_m2
                              * aircraft.geometry.wing_span_m
                              / aircraft.mass.inertia_cg_body_kg_m2(0, 0);
  const double roll_damping = moment_scale * aircraft.aero.rolling_moment_roll_rate
                              * aircraft.geometry.wing_span_m / (2.0 * trim.airspeed_m_s);
  const double authority = moment_scale * aircraft.aero.rolling_moment_aileron;
  const double lag = request.actuators[1].time_constant_s;
  Eigen::Matrix3d closed;
  closed << roll_damping, 0.0, authority, 1.0, 0.0, 0.0, -request.feedback.k(0, 0) / lag,
      -request.feedback.k(0, 1) / lag, -1.0 / lag;
  const Eigen::Vector3d linear = (closed * 0.4).exp() * Eigen::Vector3d(0.0, 1.0, 0.0);
  double normalized_errors[2]{};
  for (int level = 0; level < 2; ++level) {
    const double perturbation = 0.02 / static_cast<double>(1 << level);
    request.initial_perturbation(0) = perturbation;
    const auto result = galata::sim::simulate_nonlinear(aircraft, trim, request);
    ASSERT_TRUE(result.completed) << result.termination_reason;
    const auto& final = result.samples.back();
    Eigen::Vector3d measured;
    measured << final.state.angular_rate_body_rad_s.x(),
        galata::core::euler_from_quaternion(final.state.attitude_body_to_ned).roll_rad,
        final.controls.aileron_rad - trim.controls.aileron_rad;
    normalized_errors[level] = (measured / perturbation - linear).norm();
    EXPECT_EQ(result.position_limited_steps[1], 0);
    EXPECT_EQ(result.rate_limited_steps[1], 0);
  }
  EXPECT_LT(normalized_errors[1], 0.4 * normalized_errors[0]);
}

TEST(NonlinearSimulation, ModelEnvelopeViolationStopsTheRunAndRemainsExplicitWhenOverridden) {
  const auto aircraft = aircraft_model();
  const auto trim = trim_for(aircraft);
  auto request = simulation_request();
  request.step_count = 1;
  request.perturbation_state_names = {"w"};
  request.initial_perturbation = Eigen::VectorXd::Constant(1, 30.0);
  const auto stopped = galata::sim::simulate_nonlinear(aircraft, trim, request);
  EXPECT_FALSE(stopped.completed);
  EXPECT_EQ(stopped.completed_steps, 0);
  EXPECT_TRUE(stopped.outside_envelope_encountered);
  EXPECT_NE(stopped.termination_reason.find("envelope"), std::string::npos);
  EXPECT_GT(stopped.max_alpha_departure_rad,
            galata::model::EnvelopeWarning::kAdvisoryAlphaLimitRad);
  request.stop_outside_envelope = false;
  const auto overridden = galata::sim::simulate_nonlinear(aircraft, trim, request);
  EXPECT_TRUE(overridden.completed) << overridden.termination_reason;
  EXPECT_TRUE(overridden.outside_envelope_encountered);
}

TEST(NonlinearSimulation, ARejectedIntermediateStagePreservesTheLastAcceptedSample) {
  const auto aircraft = aircraft_model();
  const auto trim = trim_for(aircraft);
  auto request = simulation_request();
  request.sample_stride = 17;
  request.perturbation_state_names = {"w", "q"};
  request.initial_perturbation.resize(2);
  request.initial_perturbation << trim.state.velocity_body_m_s.x() * std::tan(0.1)
                                      - trim.state.velocity_body_m_s.z(),
      3.0;
  const auto result = galata::sim::simulate_nonlinear(aircraft, trim, request);
  EXPECT_FALSE(result.completed);
  ASSERT_GT(result.completed_steps, 0);
  EXPECT_LT(result.completed_steps, request.step_count);
  ASSERT_FALSE(result.samples.empty());
  EXPECT_DOUBLE_EQ(result.samples.back().time_s,
                   static_cast<double>(result.completed_steps) * request.step_s);
  EXPECT_GE(result.termination_time_s, result.samples.back().time_s);
  EXPECT_TRUE(result.outside_envelope_encountered);
  for (const auto& sample : result.samples) {
    EXPECT_FALSE(sample.envelope.outside_advisory_envelope);
  }
}

TEST(NonlinearSimulation, InvalidActuatorsMappingsAndNonEquilibriumReferenceAreRejected) {
  const auto aircraft = aircraft_model();
  const auto trim = trim_for(aircraft);
  auto request = simulation_request();
  request.actuators[2].time_constant_s = 0.0;
  EXPECT_THROW((void)galata::sim::simulate_nonlinear(aircraft, trim, request),
               std::invalid_argument);
  request = simulation_request();
  request.actuators[0].maximum = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)galata::sim::simulate_nonlinear(aircraft, trim, request),
               std::invalid_argument);
  request = simulation_request();
  request.step_s = 1.0;
  EXPECT_THROW((void)galata::sim::simulate_nonlinear(aircraft, trim, request),
               std::invalid_argument);
  request = simulation_request();
  request.feedback.state_names = {"not_a_state"};
  EXPECT_THROW((void)galata::sim::simulate_nonlinear(aircraft, trim, request),
               std::invalid_argument);
  request = simulation_request();
  auto nontrim = trim;
  nontrim.controls.thrust_n += 100.0;
  EXPECT_THROW((void)galata::sim::simulate_nonlinear(aircraft, nontrim, request),
               std::invalid_argument);
}

}  // namespace
