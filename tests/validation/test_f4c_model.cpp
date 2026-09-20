// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: the F-4C NASA CR-2144 power-approach derivative set through
// source transcription, axis/unit conversion and the complete nonlinear
// trim -> linearisation path.

#include "galata/linearize/finite_difference.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/trim/level.hpp"
#include "galata/units.hpp"

#include "reference_table.hpp"
#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>

namespace {

using galata::model::Aircraft;

constexpr double kSpeedFtS = 230.0;
constexpr double kGravity = 9.80665;
constexpr double kSlugToKg = 14.593902937206362;
constexpr double kPoundForceToNewton = 4.4482216152605;
constexpr double kMetresPerFoot = 0.3048;

Aircraft f4c() {
  return galata::model::load_aircraft(std::string(GALATA_MODELS_DIR)
                                      + "/f4c/f4c-power-approach.yaml");
}

galata::trim::TrimPoint trim_f4c(const Aircraft& aircraft) {
  galata::trim::LevelTrimRequest request;
  request.altitude_m = 0.0;
  request.airspeed_m_s = galata::units::feet_to_metres(kSpeedFtS);
  request.flight_path_angle_rad = 0.0;
  request.residual_tolerance = 1.0e-10;
  return galata::trim::trim_level(aircraft, request);
}

double source(const std::map<std::string, double>& values, const char* name) {
  return values.at(name);
}

void expect_near_source(double actual,
                        const std::map<std::string, double>& values,
                        const char* name,
                        double tolerance = 1.0e-10) {
  EXPECT_NEAR(actual, source(values, name), tolerance) << name;
}

}  // namespace

TEST(F4CModel, PublishedConditionLoadsWithDeclaredUnitsAndAxes) {
  const Aircraft aircraft = f4c();
  const auto table = galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR,
                                                     "f4c_power_approach.csv");
  const auto published = table.as_lookup("quantity", "value");
  EXPECT_NEAR(aircraft.aero.reference_alpha_rad,
              galata::units::degrees_to_radians(source(published, "reference_alpha")),
              1.0e-10);

  EXPECT_NEAR(aircraft.geometry.wing_area_m2,
              source(published, "wing_area") * kMetresPerFoot * kMetresPerFoot,
              1.0e-10);
  EXPECT_NEAR(aircraft.geometry.wing_span_m,
              source(published, "wing_span") * kMetresPerFoot,
              1.0e-10);
  EXPECT_NEAR(aircraft.geometry.mean_aerodynamic_chord_m,
              source(published, "mean_aerodynamic_chord") * kMetresPerFoot,
              1.0e-10);
  EXPECT_NEAR(aircraft.mass.mass_kg,
              source(published, "weight") * kPoundForceToNewton / kGravity,
              1.0e-8);
  EXPECT_NEAR(aircraft.mass.inertia_cg_body_kg_m2(0, 0),
              source(published, "inertia_xx") * kSlugToKg * kMetresPerFoot * kMetresPerFoot,
              1.0e-8);
  EXPECT_NEAR(aircraft.mass.inertia_cg_body_kg_m2(1, 1),
              source(published, "inertia_yy") * kSlugToKg * kMetresPerFoot * kMetresPerFoot,
              1.0e-8);
  EXPECT_NEAR(aircraft.mass.inertia_cg_body_kg_m2(2, 2),
              source(published, "inertia_zz") * kSlugToKg * kMetresPerFoot * kMetresPerFoot,
              1.0e-8);
  EXPECT_NEAR(-aircraft.mass.inertia_cg_body_kg_m2(0, 2),
              source(published, "inertia_xz") * kSlugToKg * kMetresPerFoot * kMetresPerFoot,
              1.0e-8);

  expect_near_source(aircraft.aero.lift_ref, published, "lift_ref");
  expect_near_source(aircraft.aero.drag_ref, published, "drag_ref");
  expect_near_source(aircraft.aero.lift_alpha, published, "lift_alpha");
  expect_near_source(aircraft.aero.drag_alpha, published, "drag_alpha");
  expect_near_source(aircraft.aero.pitching_moment_alpha, published, "pitching_moment_alpha");
  expect_near_source(aircraft.aero.pitching_moment_alpha_dot,
                     published,
                     "pitching_moment_alpha_dot");
  expect_near_source(aircraft.aero.pitching_moment_pitch_rate,
                     published,
                     "pitching_moment_pitch_rate");
  expect_near_source(aircraft.aero.lift_elevator, published, "lift_elevator");
  expect_near_source(aircraft.aero.drag_elevator, published, "drag_elevator");
  expect_near_source(aircraft.aero.pitching_moment_elevator,
                     published,
                     "pitching_moment_elevator");
  expect_near_source(aircraft.aero.side_force_beta, published, "side_force_beta");
  expect_near_source(aircraft.aero.side_force_aileron, published, "side_force_aileron");
  expect_near_source(aircraft.aero.side_force_rudder, published, "side_force_rudder");

  // Reconstruct the source stability-axis set from the independent fixture and
  // require the loaded body-axis moments to equal the documented rotation.
  auto source_axes = aircraft.aero;
  source_axes.rolling_moment_beta = source(published, "rolling_moment_beta");
  source_axes.yawing_moment_beta = source(published, "yawing_moment_beta");
  source_axes.rolling_moment_roll_rate = source(published, "rolling_moment_roll_rate");
  source_axes.yawing_moment_roll_rate = source(published, "yawing_moment_roll_rate");
  source_axes.rolling_moment_yaw_rate = source(published, "rolling_moment_yaw_rate");
  source_axes.yawing_moment_yaw_rate = source(published, "yawing_moment_yaw_rate");
  source_axes.rolling_moment_aileron = source(published, "rolling_moment_aileron");
  source_axes.yawing_moment_aileron = source(published, "yawing_moment_aileron");
  source_axes.rolling_moment_rudder = source(published, "rolling_moment_rudder");
  source_axes.yawing_moment_rudder = source(published, "yawing_moment_rudder");
  const auto expected_body =
      galata::model::lateral_stability_to_body(source_axes, aircraft.aero.reference_alpha_rad);
  EXPECT_NEAR(aircraft.aero.rolling_moment_beta, expected_body.rolling_moment_beta, 1.0e-11);
  EXPECT_NEAR(aircraft.aero.yawing_moment_beta, expected_body.yawing_moment_beta, 1.0e-11);
  EXPECT_NEAR(aircraft.aero.rolling_moment_roll_rate,
              expected_body.rolling_moment_roll_rate,
              1.0e-11);
  EXPECT_NEAR(aircraft.aero.yawing_moment_roll_rate,
              expected_body.yawing_moment_roll_rate,
              1.0e-11);
  EXPECT_NEAR(aircraft.aero.rolling_moment_yaw_rate,
              expected_body.rolling_moment_yaw_rate,
              1.0e-11);
  EXPECT_NEAR(aircraft.aero.yawing_moment_yaw_rate,
              expected_body.yawing_moment_yaw_rate,
              1.0e-11);
  EXPECT_NEAR(aircraft.aero.rolling_moment_aileron,
              expected_body.rolling_moment_aileron,
              1.0e-11);
  EXPECT_NEAR(aircraft.aero.yawing_moment_aileron,
              expected_body.yawing_moment_aileron,
              1.0e-11);
  EXPECT_NEAR(aircraft.aero.rolling_moment_rudder,
              expected_body.rolling_moment_rudder,
              1.0e-11);
  EXPECT_NEAR(aircraft.aero.yawing_moment_rudder,
              expected_body.yawing_moment_rudder,
              1.0e-12);
}

TEST(F4CModel, NonlinearTrimAndLinearisationAreFiniteAtThePublishedSpeed) {
  const Aircraft aircraft = f4c();
  const auto table = galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR,
                                                     "f4c_power_approach.csv");
  const auto published = table.as_lookup("quantity", "value");
  const auto trim = trim_f4c(aircraft);

  ASSERT_LT(trim.residual_norm, 1.0e-10);
  EXPECT_NEAR(trim.mach, source(published, "mach"), 0.0005);
  EXPECT_LT(std::fabs(trim.alpha_rad - aircraft.aero.reference_alpha_rad),
            galata::units::degrees_to_radians(3.0));
  EXPECT_FALSE(trim.envelope.outside_advisory_envelope);

  galata::linearize::LinearisationOptions longitudinal_options;
  longitudinal_options.state_subset = galata::linearize::longitudinal_states();
  const auto longitudinal =
      galata::linearize::linearize_finite_difference(aircraft, trim, longitudinal_options);
  galata::linearize::LinearisationOptions lateral_options;
  lateral_options.state_subset = galata::linearize::lateral_states();
  const auto lateral =
      galata::linearize::linearize_finite_difference(aircraft, trim, lateral_options);

  EXPECT_LT(longitudinal.worst_relative_truncation, 1.0e-8);
  EXPECT_LT(lateral.worst_relative_truncation, 1.0e-8);
  EXPECT_TRUE(longitudinal.a.allFinite());
  EXPECT_TRUE(longitudinal.b.allFinite());
  EXPECT_TRUE(lateral.a.allFinite());
  EXPECT_TRUE(lateral.b.allFinite());
}
