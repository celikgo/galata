// SPDX-License-Identifier: Apache-2.0
//
// Flight-condition consistency against analytic force derivatives.
// Reference: Stevens, Lewis & Johnson, Aircraft Control and Simulation,
// 3rd ed., Wiley, 2016, chapters 2 and 3. The aircraft below is a synthetic
// algebraic case, not an additional validated aircraft or fitted reference.

#include "galata/core/atmosphere.hpp"
#include "galata/linearize/finite_difference.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/trim/level.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace {

galata::model::Aircraft analytic_aircraft() {
  galata::model::Aircraft aircraft;
  aircraft.geometry = {10.0, 10.0, 1.0};
  aircraft.mass.mass_kg = 1000.0;
  aircraft.mass.inertia_cg_body_kg_m2 = Eigen::Vector3d(500.0, 600.0, 700.0).asDiagonal();
  aircraft.aero.lift_ref = 1.0;
  aircraft.aero.drag_ref = 0.1;
  aircraft.aero.lift_alpha = 5.0;
  aircraft.aero.pitching_moment_alpha = -1.0;
  aircraft.aero.pitching_moment_elevator = -1.0;
  aircraft.aero.lift_elevator = 0.2;
  aircraft.aero.drag_elevator = 0.01;
  aircraft.aero.side_force_beta = -0.5;
  return aircraft;
}

TEST(FlightCondition, LinearisationRetainsTemperatureOffsetInStateAndControlDerivatives) {
  const auto aircraft = analytic_aircraft();
  for (const double offset : {-20.0, 0.0, 30.0}) {
    SCOPED_TRACE(offset);
    galata::trim::LevelTrimRequest request;
    request.airspeed_m_s = 50.0;
    request.delta_isa_k = offset;
    const auto trim = galata::trim::trim_level(aircraft, request);
    const auto linear = galata::linearize::linearize_finite_difference(aircraft, trim);

    // Sea-level pressure and temperature are defining constants. From the
    // ideal gas law and Y = rho V^2 S C_Y_beta beta / 2, beta_v = 1/V:
    // A(v,v) = rho V S C_Y_beta / (2m). No production differentiation here.
    const double density = galata::core::ussa1976::kSeaLevelPressure
                           / (galata::core::ussa1976::kSpecificGasConstant
                              * (galata::core::ussa1976::kSeaLevelTemperature + offset));
    const double force_scale = 0.5 * density * request.airspeed_m_s * request.airspeed_m_s
                               * aircraft.geometry.wing_area_m2 / aircraft.mass.mass_kg;
    const double expected_v_v = force_scale * aircraft.aero.side_force_beta / request.airspeed_m_s;
    // Z = -qS (C_D sin(alpha) + C_L cos(alpha)); controls do not change alpha.
    const double expected_w_elevator = -force_scale
                                       * (aircraft.aero.drag_elevator * std::sin(trim.alpha_rad)
                                          + aircraft.aero.lift_elevator * std::cos(trim.alpha_rad));
    // Central differences carry cancellation eps*|f|/h. With accelerations
    // O(10) and the default angle half-step 5e-7, 1e-7 is a conservative bound.
    EXPECT_NEAR(
        linear.a(galata::linearize::kVelocityV, galata::linearize::kVelocityV), expected_v_v, 1e-7);
    EXPECT_NEAR(linear.b(galata::linearize::kVelocityW, 0), expected_w_elevator, 1e-7);
    EXPECT_DOUBLE_EQ(trim.atmosphere.delta_isa_k, offset);
    EXPECT_DOUBLE_EQ(linear.trim_delta_isa_k, offset);
  }
}

TEST(FlightCondition, LateralReferenceOffsetIsRejectedBeforeReportingATrim) {
  auto aircraft = analytic_aircraft();
  aircraft.cg_to_aero_reference_m.y() = 0.1;
  galata::trim::LevelTrimRequest request;
  request.airspeed_m_s = 50.0;
  // r x F gives a nonzero rolling moment from the lift. The longitudinal
  // three-equation solve cannot balance it with its available unknowns.
  EXPECT_THROW((void)galata::trim::trim_level(aircraft, request), std::invalid_argument);
}

TEST(FlightCondition, ReturnedTrimSatisfiesAllSixAccelerationEquations) {
  auto aircraft = analytic_aircraft();
  aircraft.cg_to_aero_reference_m.x() = 0.1;
  aircraft.cg_to_aero_reference_m.z() = -0.05;
  galata::trim::LevelTrimRequest request;
  request.airspeed_m_s = 50.0;
  request.delta_isa_k = 30.0;
  const auto trim = galata::trim::trim_level(aircraft, request);
  const auto rate = aircraft.derivative(trim.state, trim.controls, request.delta_isa_k);
  EXPECT_LE(rate.segment<3>(galata::core::kVelocityU).norm(), request.residual_tolerance);
  EXPECT_LE(rate.segment<3>(galata::core::kRateP).norm(), request.residual_tolerance);
  EXPECT_GT(rate.segment<3>(galata::core::kPositionNorth).norm(), 0.0)
      << "translating position rates must not be mistaken for failed equilibrium";
}

TEST(FlightCondition, NonFiniteAndInvalidTrimRequestsAreRejected) {
  const auto aircraft = analytic_aircraft();
  for (const double bad :
       {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    galata::trim::LevelTrimRequest request;
    request.airspeed_m_s = 50.0;
    request.residual_tolerance = bad;
    EXPECT_THROW((void)galata::trim::trim_level(aircraft, request), std::invalid_argument);
    request.residual_tolerance = 1e-10;
    request.mach = bad;
    EXPECT_THROW((void)galata::trim::trim_level(aircraft, request), std::invalid_argument);
  }
  galata::trim::LevelTrimRequest request;
  request.airspeed_m_s = 50.0;
  request.use_default_guess = false;
  request.initial_thrust_n = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)galata::trim::trim_level(aircraft, request), std::invalid_argument);
}

TEST(FlightCondition, LinearisationRejectsStaleControlsAndAircraftDespiteRecordedZeroResidual) {
  const auto aircraft = analytic_aircraft();
  galata::trim::LevelTrimRequest request;
  request.airspeed_m_s = 50.0;
  auto trim = galata::trim::trim_level(aircraft, request);
  // A thrust increment produces u_dot = delta_T/m. A stored residual, even
  // paired with an unbounded stored tolerance, cannot make that an equilibrium.
  trim.controls.thrust_n += 100.0;
  trim.residual_norm = 0.0;
  trim.residual_tolerance = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, trim),
               std::invalid_argument);

  trim = galata::trim::trim_level(aircraft, request);
  auto changed_aircraft = aircraft;
  changed_aircraft.mass.mass_kg *= 2.0;
  // The same wrench now supports only half the weight. Reusing an old trim
  // after a model edit must fail even though the trim struct is untouched.
  EXPECT_THROW((void)galata::linearize::linearize_finite_difference(changed_aircraft, trim),
               std::invalid_argument);
}

TEST(FlightCondition, LinearisationRecomputesResidualAndConditionWithItsOwnExplicitBudget) {
  const auto aircraft = analytic_aircraft();
  galata::trim::LevelTrimRequest request;
  request.airspeed_m_s = 50.0;
  auto trim = galata::trim::trim_level(aircraft, request);
  const double alpha = trim.alpha_rad;
  constexpr double thrust_increment = 1e-6;  // N; exact response is delta_T/m
  trim.controls.thrust_n += thrust_increment;
  trim.residual_norm = 0.0;
  trim.residual_tolerance = 1.0;
  trim.airspeed_m_s = 999.0;
  trim.alpha_rad = 999.0;
  EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, trim),
               std::invalid_argument);
  galata::linearize::LinearisationOptions options;
  options.equilibrium_tolerance = 1e-8;
  const auto linear = galata::linearize::linearize_finite_difference(aircraft, trim, options);
  // Acceleration is O(10); 1e-13 covers its binary64 force-balance rounding.
  EXPECT_NEAR(linear.trim_residual_norm, thrust_increment / aircraft.mass.mass_kg, 1e-13);
  EXPECT_DOUBLE_EQ(linear.trim_residual_tolerance, options.equilibrium_tolerance);
  EXPECT_NEAR(linear.trim_airspeed_m_s, request.airspeed_m_s, 1e-13);
  EXPECT_NEAR(linear.trim_alpha_rad, alpha, 1e-15);
}

TEST(FlightCondition, LinearisationRejectsInvalidStateControlAndAcceptanceBudget) {
  const auto aircraft = analytic_aircraft();
  galata::trim::LevelTrimRequest request;
  request.airspeed_m_s = 50.0;
  const auto trim = galata::trim::trim_level(aircraft, request);
  for (const double bad :
       {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    auto modified = trim;
    modified.state.velocity_body_m_s.x() = bad;
    EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, modified),
                 std::invalid_argument);
    modified = trim;
    modified.controls.elevator_rad = bad;
    EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, modified),
                 std::invalid_argument);
  }
  for (const double magnitude : {0.0, 2.0}) {
    auto modified = trim;
    modified.state.attitude_body_to_ned = galata::core::Quaternion(magnitude, 0.0, 0.0, 0.0);
    EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, modified),
                 std::invalid_argument);
  }
  auto rotating = trim;
  rotating.state.angular_rate_body_rad_s.x() = 1e-6;
  EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, rotating),
               std::invalid_argument);
  for (const double bad : {0.0,
                           -1.0,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    galata::linearize::LinearisationOptions options;
    options.equilibrium_tolerance = bad;
    EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, trim, options),
                 std::invalid_argument);
  }
}

TEST(FlightCondition, AConvergedNearVerticalTrimCannotProduceAnUnsupportedEulerLinearisation) {
  auto aircraft = analytic_aircraft();
  aircraft.aero.lift_ref = 0.05;
  aircraft.aero.reference_mach = 0.588;
  galata::trim::LevelTrimRequest request;
  request.airspeed_m_s = 200.0;
  for (const double gamma : {-std::numbers::pi_v<double> / 2.0, std::numbers::pi_v<double> / 2.0}) {
    request.flight_path_angle_rad = gamma;
    const auto trim = galata::trim::trim_level(aircraft, request);
    ASSERT_LE(trim.residual_norm, request.residual_tolerance);
    ASSERT_FALSE(trim.envelope.outside_advisory_envelope);
    ASSERT_LT(std::abs(std::cos(trim.pitch_attitude_rad)),
              galata::linearize::kMinimumChartConditioning);
    EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, trim),
                 std::invalid_argument);
  }
}

TEST(FlightCondition, AValidChartDoesNotPermitPerturbationsAcrossItsUnsupportedBoundary) {
  auto aircraft = analytic_aircraft();
  aircraft.aero.lift_ref = 0.05;
  galata::trim::LevelTrimRequest request;
  request.airspeed_m_s = 200.0;
  request.flight_path_angle_rad = 1.3;
  const auto trim = galata::trim::trim_level(aircraft, request);
  EXPECT_NO_THROW((void)galata::linearize::linearize_finite_difference(aircraft, trim));
  galata::linearize::LinearisationOptions options;
  options.state_jacobian.absolute_step_per_component =
      Eigen::VectorXd::Constant(galata::linearize::kEulerStateSize, 1e-6);
  options.state_jacobian.absolute_step_per_component(galata::linearize::kPitch) = 0.2;
  EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, trim, options),
               std::invalid_argument);
  options = {};
  options.state_subset = {galata::linearize::kVelocityU, galata::linearize::kVelocityU};
  EXPECT_THROW((void)galata::linearize::linearize_finite_difference(aircraft, trim, options),
               std::invalid_argument);
}

TEST(FlightCondition, DirectModelValidationRejectsNonFiniteMassGeometryAndAerodynamics) {
  for (const double bad : {std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    auto aircraft = analytic_aircraft();
    aircraft.mass.mass_kg = bad;
    EXPECT_THROW(aircraft.mass.validate(), std::invalid_argument);
    aircraft = analytic_aircraft();
    aircraft.mass.inertia_cg_body_kg_m2(0, 0) = bad;
    EXPECT_THROW(aircraft.mass.validate(), std::invalid_argument);
    aircraft = analytic_aircraft();
    aircraft.geometry.wing_area_m2 = bad;
    EXPECT_THROW(aircraft.validate(), std::invalid_argument);
    aircraft = analytic_aircraft();
    aircraft.geometry.wing_span_m = bad;
    EXPECT_THROW(aircraft.validate(), std::invalid_argument);
    aircraft = analytic_aircraft();
    aircraft.geometry.mean_aerodynamic_chord_m = bad;
    EXPECT_THROW(aircraft.validate(), std::invalid_argument);
    aircraft = analytic_aircraft();
    aircraft.aero.pitching_moment_elevator = bad;
    EXPECT_THROW(aircraft.validate(), std::invalid_argument);
    aircraft = analytic_aircraft();
    aircraft.aero.yawing_moment_rudder = bad;
    EXPECT_THROW(aircraft.validate(), std::invalid_argument);
    aircraft = analytic_aircraft();
    aircraft.aero.reference_alpha_rad = bad;
    EXPECT_THROW(aircraft.validate(), std::invalid_argument);
    aircraft = analytic_aircraft();
    aircraft.thrust_incidence_rad = bad;
    EXPECT_THROW(aircraft.validate(), std::invalid_argument);
    aircraft = analytic_aircraft();
    aircraft.cg_to_aero_reference_m.x() = bad;
    EXPECT_THROW(aircraft.validate(), std::invalid_argument);
  }
  EXPECT_NO_THROW(analytic_aircraft().validate());
}

}  // namespace
